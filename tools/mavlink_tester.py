#!/usr/bin/env python3
import argparse
import os
import select
import socket
import statistics
import struct
import termios
import time


MAVLINK_V1_STX = 0xFE
SMALL_IMU = 181
SMALL_BARO = 183
ROSFLIGHT_STATUS = 191
CRC_EXTRA = {
    0: 50,
    20: 214,
    21: 159,
    22: 220,
    23: 168,
    31: 246,
    65: 118,
    111: 34,
    180: 90,
    181: 67,
    182: 218,
    183: 206,
    184: 169,
    187: 60,
    188: 249,
    189: 113,
    190: 181,
    191: 12,
    192: 134,
    193: 1,
    195: 65,
    196: 10,
    197: 221,
    199: 48,
    253: 83,
}

BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    500000: termios.B500000,
    576000: termios.B576000,
    921600: termios.B921600,
    1000000: termios.B1000000,
    2000000: termios.B2000000,
}


class MavlinkV1Parser:
    def __init__(self, validate_crc=True):
        self.buf = bytearray()
        self.validate_crc = validate_crc
        self.candidates = 0
        self.invalid_crc = 0
        self.invalid_by_msgid = {}

    def feed(self, data):
        frames = []
        self.buf.extend(data)
        while True:
            try:
                start = self.buf.index(MAVLINK_V1_STX)
            except ValueError:
                self.buf.clear()
                break

            if start:
                del self.buf[:start]

            if len(self.buf) < 6:
                break

            payload_len = self.buf[1]
            frame_len = 6 + payload_len + 2
            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            del self.buf[:frame_len]
            self.candidates += 1
            if self.validate_crc and not valid_crc(frame):
                msgid = frame[5]
                self.invalid_crc += 1
                self.invalid_by_msgid[msgid] = self.invalid_by_msgid.get(msgid, 0) + 1
                continue
            frames.append(
                {
                    "seq": frame[2],
                    "sysid": frame[3],
                    "compid": frame[4],
                    "msgid": frame[5],
                    "payload": frame[6 : 6 + payload_len],
                }
            )
        return frames


def crc_accumulate(data, crc):
    tmp = data ^ (crc & 0xFF)
    tmp = (tmp ^ ((tmp << 4) & 0xFF)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def valid_crc(frame):
    payload_len = frame[1]
    msgid = frame[5]
    extra = CRC_EXTRA.get(msgid)
    if extra is None:
        return False

    crc = 0xFFFF
    for byte in frame[1 : 6 + payload_len]:
        crc = crc_accumulate(byte, crc)
    crc = crc_accumulate(extra, crc)
    received = frame[6 + payload_len] | (frame[7 + payload_len] << 8)
    return crc == received


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive, decode, and time Voloxide Pico 2 W MAVLink telemetry."
    )
    parser.add_argument(
        "--transport",
        choices=("uart", "wifi"),
        required=True,
        help="Telemetry path to test: wired UART or Wi-Fi UDP.",
    )
    parser.add_argument("--device", default="/dev/ttyACM0", help="UART device for --transport uart.")
    parser.add_argument("--baud", type=int, default=921600, help="UART baud for --transport uart.")
    parser.add_argument("--board", help="Pico 2 W IPv4 address for --transport wifi.")
    parser.add_argument("--board-port", type=int, default=14550)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=14551)
    parser.add_argument("--hello", default="voloxide-host-hello")
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--show", type=int, default=5)
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=0.5,
        help="Discard decoded sensor frames for this long before recording statistics.",
    )
    parser.add_argument(
        "--no-crc",
        action="store_true",
        help="Decode MAVLink v1 frame candidates without checksum validation.",
    )
    parser.add_argument(
        "--diagnostics",
        action="store_true",
        help="Print parser candidate and invalid CRC counters.",
    )
    return parser.parse_args()


def configure_uart(fd, baud):
    if baud not in BAUD_RATES:
        raise SystemExit(f"unsupported baud {baud}; add it to BAUD_RATES")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = BAUD_RATES[baud]
    attrs[5] = BAUD_RATES[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def open_uart(args):
    fd = os.open(args.device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    configure_uart(fd, args.baud)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def open_udp(args):
    if not args.board:
        raise SystemExit("--board is required for --transport wifi")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.bind_port))
    sock.setblocking(False)
    sock.sendto(args.hello.encode("ascii"), (args.board, args.board_port))
    return sock


def decode_sensor_frame(frame):
    payload = frame["payload"]
    if frame["msgid"] == SMALL_IMU and len(payload) == 36:
        values = struct.unpack("<Qfffffff", payload)
        return {
            "name": "imu",
            "board_us": values[0],
            "values": values[1:],
        }
    if frame["msgid"] == SMALL_BARO and len(payload) == 12:
        altitude, pressure, temperature = struct.unpack("<fff", payload)
        return {
            "name": "baro",
            "board_us": None,
            "values": (altitude, pressure, temperature),
        }
    if frame["msgid"] == ROSFLIGHT_STATUS and len(payload) == 11:
        rc_override, num_errors, loop_time_us, armed, failsafe, offboard, error_code, control_mode = (
            struct.unpack("<HhhBBBBB", payload)
        )
        return {
            "name": "status",
            "board_us": None,
            "values": (
                armed,
                failsafe,
                rc_override,
                offboard,
                error_code,
                control_mode,
                num_errors,
                loop_time_us,
            ),
            "loop_time_us": loop_time_us,
        }
    return None


def percentile(sorted_values, pct):
    if not sorted_values:
        return None
    index = round((len(sorted_values) - 1) * pct / 100.0)
    return sorted_values[index]


def summarize(name, records):
    if not records:
        print(f"{name}: no frames")
        return

    host_deltas = [
        (records[i]["host_ns"] - records[i - 1]["host_ns"]) / 1_000_000.0
        for i in range(1, len(records))
    ]
    board_deltas = [
        (records[i]["board_us"] - records[i - 1]["board_us"]) / 1000.0
        for i in range(1, len(records))
        if records[i]["board_us"] is not None and records[i - 1]["board_us"] is not None
        and records[i]["board_us"] >= records[i - 1]["board_us"]
    ]

    print(f"{name}: frames={len(records)}")
    if host_deltas:
        ordered = sorted(host_deltas)
        rate_hz = 1000.0 / statistics.fmean(host_deltas)
        print(
            "  host interval ms: "
            f"min={ordered[0]:.3f} avg={statistics.fmean(host_deltas):.3f} "
            f"p50={percentile(ordered, 50):.3f} p90={percentile(ordered, 90):.3f} "
            f"p99={percentile(ordered, 99):.3f} max={ordered[-1]:.3f} "
            f"rate={rate_hz:.1f}Hz"
        )
    if board_deltas:
        ordered = sorted(board_deltas)
        rate_hz = 1000.0 / statistics.fmean(board_deltas)
        print(
            "  board timestamp interval ms: "
            f"min={ordered[0]:.3f} avg={statistics.fmean(board_deltas):.3f} "
            f"p50={percentile(ordered, 50):.3f} p90={percentile(ordered, 90):.3f} "
            f"p99={percentile(ordered, 99):.3f} max={ordered[-1]:.3f} "
            f"rate={rate_hz:.1f}Hz"
        )
    loop_times = [record["loop_time_us"] for record in records if "loop_time_us" in record]
    if loop_times:
        ordered = sorted(loop_times)
        print(
            "  firmware loop_time_us: "
            f"min={ordered[0]} avg={statistics.fmean(loop_times):.1f} "
            f"p50={percentile(ordered, 50)} p90={percentile(ordered, 90)} "
            f"p99={percentile(ordered, 99)} max={ordered[-1]}"
        )


def main():
    args = parse_args()
    parser = MavlinkV1Parser(validate_crc=not args.no_crc)
    records = {"imu": [], "baro": [], "status": []}
    shown = 0
    deadline = time.monotonic() + args.duration_s if args.duration_s > 0 else None

    if args.transport == "uart":
        source = open_uart(args)
        print(f"listening on {args.device} at {args.baud} baud")
    else:
        source = open_udp(args)
        print(
            f"listening on {args.bind}:{args.bind_port}; "
            f"sent hello to {args.board}:{args.board_port}"
        )

    record_after_ns = time.monotonic_ns() + int(args.warmup_s * 1_000_000_000)
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            if (
                len(records["imu"]) >= args.samples
                and len(records["baro"]) >= min(args.samples, 50)
                and len(records["status"]) >= min(args.samples, 50)
            ):
                break

            readable, _, _ = select.select([source], [], [], 0.25)
            if not readable:
                if args.transport == "wifi":
                    source.sendto(args.hello.encode("ascii"), (args.board, args.board_port))
                continue

            if args.transport == "uart":
                try:
                    data = os.read(source, 4096)
                except BlockingIOError:
                    continue
            else:
                data, _addr = source.recvfrom(4096)

            host_ns = time.monotonic_ns()
            for frame in parser.feed(data):
                decoded = decode_sensor_frame(frame)
                if decoded is None:
                    continue
                name = decoded["name"]
                if shown < args.show:
                    print(
                        f"{name} seq={frame['seq']} sys={frame['sysid']} "
                        f"comp={frame['compid']} board_us={decoded['board_us']} "
                        f"values={tuple(round(v, 4) for v in decoded['values'])}"
                    )
                    shown += 1
                if host_ns < record_after_ns:
                    continue
                records[name].append(
                    {
                        "host_ns": host_ns,
                        "board_us": decoded["board_us"],
                        "values": decoded["values"],
                        **(
                            {"loop_time_us": decoded["loop_time_us"]}
                            if "loop_time_us" in decoded
                            else {}
                        ),
                    }
                )
    finally:
        if args.transport == "uart":
            os.close(source)
        else:
            source.close()

    summarize("imu", records["imu"])
    summarize("baro", records["baro"])
    summarize("status", records["status"])
    if args.diagnostics:
        print(
            f"parser: candidates={parser.candidates} invalid_crc={parser.invalid_crc} "
            f"invalid_by_msgid={parser.invalid_by_msgid}"
        )


if __name__ == "__main__":
    main()
