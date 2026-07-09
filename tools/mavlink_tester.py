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
HEARTBEAT = 0
PARAM_REQUEST_READ = 20
PARAM_REQUEST_LIST = 21
PARAM_VALUE = 22
PARAM_SET = 23
RC_CHANNELS = 65
TIMESYNC = 111
SMALL_IMU = 181
SMALL_BARO = 183
ROSFLIGHT_OUTPUT_RAW = 190
ROSFLIGHT_CMD = 188
ROSFLIGHT_CMD_ACK = 189
ROSFLIGHT_STATUS = 191
ROSFLIGHT_VERSION = 192
ROSFLIGHT_GNSS = 197
STATUSTEXT = 253
ROSFLIGHT_CMD_SEND_VERSION = 10
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
    3000000: termios.B3000000,
    4000000: termios.B4000000,
}


class MavlinkV1Parser:
    def __init__(self, validate_crc=True, invalid_sample_limit=0):
        self.buf = bytearray()
        self.validate_crc = validate_crc
        self.candidates = 0
        self.invalid_crc = 0
        self.invalid_by_msgid = {}
        self.invalid_samples = []
        self.invalid_sample_limit = invalid_sample_limit

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
            self.candidates += 1
            if self.validate_crc and not valid_crc(frame):
                msgid = frame[5]
                self.invalid_crc += 1
                self.invalid_by_msgid[msgid] = self.invalid_by_msgid.get(msgid, 0) + 1
                if len(self.invalid_samples) < self.invalid_sample_limit:
                    self.invalid_samples.append(
                        {
                            "len": payload_len,
                            "seq": frame[2],
                            "sysid": frame[3],
                            "compid": frame[4],
                            "msgid": msgid,
                            "crc": frame[6 + payload_len]
                            | (frame[7 + payload_len] << 8),
                            "head": frame[: min(len(frame), 18)].hex(),
                        }
                    )
                try:
                    restart = frame.index(bytes([MAVLINK_V1_STX]), 1)
                except ValueError:
                    del self.buf[:frame_len]
                else:
                    del self.buf[:restart]
                continue
            del self.buf[:frame_len]
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
        description="Receive, decode, and time Veloxity Pico 2 W MAVLink telemetry."
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
    parser.add_argument("--hello", default="veloxity-host-hello")
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
    parser.add_argument(
        "--invalid-samples",
        type=int,
        default=5,
        help="When diagnostics are enabled, print up to this many invalid CRC candidate headers.",
    )
    parser.add_argument(
        "--raw-capture",
        help="Write received raw bytes to this file after warmup for offline link analysis.",
    )
    parser.add_argument(
        "--timesync-probe",
        action="store_true",
        help="Send MAVLink TIMESYNC requests and report responses. UART transport opens read/write.",
    )
    parser.add_argument(
        "--timesync-period-s",
        type=float,
        default=1.0,
        help="Seconds between TIMESYNC probes when --timesync-probe is set.",
    )
    parser.add_argument(
        "--bidirectional",
        action="store_true",
        help="Inject ground-station MAVLink frames while receiving telemetry.",
    )
    parser.add_argument(
        "--acceptance",
        action="store_true",
        help="Enable bidirectional traffic and fail if required telemetry/replies are missing.",
    )
    parser.add_argument("--gcs-sysid", type=int, default=255)
    parser.add_argument("--gcs-compid", type=int, default=190)
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--target-component", type=int, default=250)
    parser.add_argument("--heartbeat-hz", type=float, default=1.0)
    parser.add_argument("--timesync-hz", type=float, default=5.0)
    parser.add_argument(
        "--expect-imu-hz",
        type=float,
        default=None,
        help="Expected IMU telemetry rate; prints observed rate error in the summary.",
    )
    parser.add_argument(
        "--expect-rc-hz",
        type=float,
        default=None,
        help="Expected RC telemetry rate; prints observed rate error in the summary.",
    )
    parser.add_argument(
        "--expect-attitude-hz",
        type=float,
        default=None,
        help="Expected attitude telemetry rate; prints observed rate error in the summary.",
    )
    parser.add_argument(
        "--expect-output-raw-hz",
        type=float,
        default=None,
        help="Expected output_raw telemetry rate; prints observed rate error in the summary.",
    )
    parser.add_argument("--request-version-s", type=float, default=1.0)
    parser.add_argument("--request-params-s", type=float, default=2.0)
    parser.add_argument(
        "--param-flood",
        metavar="NAME",
        help="Send a burst of PARAM_SET messages for NAME while receiving telemetry.",
    )
    parser.add_argument(
        "--param-flood-count",
        type=int,
        default=300,
        help="Number of PARAM_SET messages to send when --param-flood is set.",
    )
    parser.add_argument(
        "--param-flood-value",
        type=float,
        default=0.0,
        help="First PARAM_SET value to send during a flood.",
    )
    parser.add_argument(
        "--param-flood-step",
        type=float,
        default=0.0,
        help="Value increment between flooded PARAM_SET messages.",
    )
    parser.add_argument(
        "--param-flood-type",
        choices=("real32", "int32"),
        default="real32",
        help="MAV_PARAM_TYPE for flooded PARAM_SET messages.",
    )
    parser.add_argument(
        "--param-flood-delay-s",
        type=float,
        default=0.0,
        help="Delay before starting PARAM_SET flood.",
    )
    parser.add_argument(
        "--param-flood-period-s",
        type=float,
        default=0.0,
        help="Optional pacing between PARAM_SET messages; 0 sends the burst immediately.",
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
    mode = os.O_RDWR if args.timesync_probe or args.bidirectional or args.acceptance else os.O_RDONLY
    fd = os.open(args.device, mode | os.O_NOCTTY | os.O_NONBLOCK)
    configure_uart(fd, args.baud)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def mavlink_v1_frame(seq, sysid, compid, msgid, payload):
    frame = bytearray([MAVLINK_V1_STX, len(payload), seq & 0xFF, sysid, compid, msgid])
    frame.extend(payload)
    crc = 0xFFFF
    for byte in frame[1:]:
        crc = crc_accumulate(byte, crc)
    crc = crc_accumulate(CRC_EXTRA[msgid], crc)
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)
    return bytes(frame)


def timesync_request(seq):
    payload = struct.pack("<qq", 0, time.monotonic_ns())
    return mavlink_v1_frame(seq, 255, 190, TIMESYNC, payload)


def write_transport(args, source, payload):
    if args.transport == "uart":
        os.write(source, payload)
    else:
        source.sendto(payload, (args.board, args.board_port))


def heartbeat_payload():
    return struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)


def timesync_payload():
    return struct.pack("<qq", 0, time.monotonic_ns())


def param_request_list_payload(args):
    return struct.pack("<BB", args.target_system, args.target_component)


def param_request_read_payload(args, param_name="", param_index=0):
    param_id = param_name.encode("ascii", errors="ignore")[:16].ljust(16, b"\0")
    return struct.pack("<hBB16s", param_index, args.target_system, args.target_component, param_id)


def param_set_payload(args, param_name, value):
    param_id = param_name.encode("ascii", errors="ignore")[:16].ljust(16, b"\0")
    param_type = 9 if args.param_flood_type == "real32" else 6
    if args.param_flood_type == "int32":
        value = float(int(value))
    return struct.pack(
        "<fBB16sB",
        float(value),
        args.target_system,
        args.target_component,
        param_id,
        param_type,
    )


def rosflight_cmd_payload(command):
    return struct.pack("<B", command)


class MavlinkInjector:
    def __init__(self, args, request_delay_base_s=0.0):
        self.args = args
        self.seq = 0
        now = time.monotonic()
        request_base = now + request_delay_base_s
        self.next_heartbeat = now
        self.next_timesync = now
        self.next_version = request_base + args.request_version_s
        self.next_params = request_base + args.request_params_s
        self.next_param_flood = now + args.param_flood_delay_s
        self.param_flood_sent = 0
        self.sent = {
            "heartbeat": 0,
            "timesync": 0,
            "version_cmd": 0,
            "param_request_list": 0,
            "param_request_read": 0,
            "param_set": 0,
        }

    def frame(self, msgid, payload):
        frame = mavlink_v1_frame(
            self.seq,
            self.args.gcs_sysid,
            self.args.gcs_compid,
            msgid,
            payload,
        )
        self.seq = (self.seq + 1) & 0xFF
        return frame

    def service(self, source):
        now = time.monotonic()
        if self.args.heartbeat_hz > 0 and now >= self.next_heartbeat:
            write_transport(self.args, source, self.frame(HEARTBEAT, heartbeat_payload()))
            self.sent["heartbeat"] += 1
            self.next_heartbeat = now + 1.0 / self.args.heartbeat_hz
        if self.args.timesync_hz > 0 and now >= self.next_timesync:
            write_transport(self.args, source, self.frame(TIMESYNC, timesync_payload()))
            self.sent["timesync"] += 1
            self.next_timesync = now + 1.0 / self.args.timesync_hz
        if self.args.request_version_s > 0 and now >= self.next_version:
            write_transport(
                self.args,
                source,
                self.frame(ROSFLIGHT_CMD, rosflight_cmd_payload(ROSFLIGHT_CMD_SEND_VERSION)),
            )
            self.sent["version_cmd"] += 1
            self.next_version = float("inf")
        if self.args.request_params_s > 0 and now >= self.next_params:
            write_transport(
                self.args,
                source,
                self.frame(PARAM_REQUEST_LIST, param_request_list_payload(self.args)),
            )
            write_transport(
                self.args,
                source,
                self.frame(PARAM_REQUEST_READ, param_request_read_payload(self.args, param_index=0)),
            )
            self.sent["param_request_list"] += 1
            self.sent["param_request_read"] += 1
            self.next_params = float("inf")
        if self.args.param_flood and now >= self.next_param_flood:
            while self.param_flood_sent < self.args.param_flood_count:
                value = (
                    self.args.param_flood_value
                    + self.param_flood_sent * self.args.param_flood_step
                )
                write_transport(
                    self.args,
                    source,
                    self.frame(PARAM_SET, param_set_payload(self.args, self.args.param_flood, value)),
                )
                self.sent["param_set"] += 1
                self.param_flood_sent += 1
                if self.args.param_flood_period_s > 0:
                    self.next_param_flood = now + self.args.param_flood_period_s
                    break
            else:
                self.next_param_flood = float("inf")


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
    if frame["msgid"] == HEARTBEAT and len(payload) == 9:
        custom_mode, type_, autopilot, base_mode, system_status, mavlink_version = struct.unpack(
            "<IBBBBB", payload
        )
        return {
            "name": "heartbeat",
            "board_us": None,
            "values": {
                "type": type_,
                "autopilot": autopilot,
                "base_mode": base_mode,
                "custom_mode": custom_mode,
                "system_status": system_status,
                "mavlink_version": mavlink_version,
            },
        }
    if frame["msgid"] == PARAM_VALUE and len(payload) == 25:
        value, count, index, param_id, param_type = struct.unpack("<fHH16sB", payload)
        return {
            "name": "param",
            "board_us": None,
            "values": {
                "id": param_id.split(b"\0", 1)[0].decode("ascii", errors="replace"),
                "value": value,
                "count": count,
                "index": index,
                "type": param_type,
            },
        }
    if frame["msgid"] == TIMESYNC and len(payload) == 16:
        tc1, ts1 = struct.unpack("<qq", payload)
        return {
            "name": "timesync",
            "board_us": tc1 // 1000 if tc1 > 0 else None,
            "values": {
                "tc1": tc1,
                "ts1": ts1,
            },
        }
    if frame["msgid"] == ROSFLIGHT_CMD_ACK and len(payload) == 2:
        command, success = struct.unpack("<BB", payload)
        return {
            "name": "cmd_ack",
            "board_us": None,
            "values": {
                "command": command,
                "success": success,
            },
        }
    if frame["msgid"] == ROSFLIGHT_VERSION and len(payload) == 50:
        version = payload.split(b"\0", 1)[0].decode("ascii", errors="replace")
        return {
            "name": "version",
            "board_us": None,
            "values": {
                "version": version,
            },
        }
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
    if frame["msgid"] == 31 and len(payload) == 32:
        time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed = struct.unpack(
            "<Ifffffff", payload
        )
        return {
            "name": "attitude",
            "board_us": time_boot_ms * 1000,
            "values": {
                "q": (q1, q2, q3, q4),
                "rates": (rollspeed, pitchspeed, yawspeed),
            },
        }
    if frame["msgid"] == ROSFLIGHT_OUTPUT_RAW and len(payload) == 64:
        values = struct.unpack("<Q14f", payload)
        return {
            "name": "output_raw",
            "board_us": values[0] * 1000,
            "values": values[1:],
        }
    if frame["msgid"] == RC_CHANNELS and len(payload) == 42:
        values = struct.unpack("<I18HBB", payload)
        return {
            "name": "rc",
            "board_us": values[0] * 1000,
            "values": {
                "count": values[19],
                "channels": values[1:19],
                "rssi": values[20],
            },
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
    if frame["msgid"] == ROSFLIGHT_GNSS and len(payload) == 66:
        seconds, lat, lon, rosflight_timestamp, nanos, *rest = struct.unpack(
            "<qddQi7fBB", payload
        )
        height, vel_n, vel_e, vel_d, h_acc, v_acc, s_acc, fix_type, num_sat = rest
        return {
            "name": "gnss",
            "board_us": rosflight_timestamp,
            "values": {
                "seconds": seconds,
                "nanos": nanos,
                "fix_type": fix_type,
                "num_sat": num_sat,
                "lat": lat,
                "lon": lon,
                "height": height,
                "vel": (vel_n, vel_e, vel_d),
                "acc": (h_acc, v_acc, s_acc),
            },
        }
    if frame["msgid"] == STATUSTEXT and len(payload) == 51:
        text = payload[1:].split(b"\0", 1)[0].decode("ascii", errors="replace")
        perf = parse_perf_text(text)
        if perf is not None:
            return {
                "name": "perf",
                "board_us": None,
                "values": (),
                "text": text,
                "perf": perf,
            }
        return {
            "name": "text",
            "board_us": None,
            "values": (),
            "text": text,
        }
    return None


def format_decoded_values(values):
    if isinstance(values, dict):
        parts = []
        for key, value in values.items():
            if isinstance(value, tuple):
                parts.append(
                    f"{key}=({', '.join(format_decoded_scalar(v) for v in value)})"
                )
            else:
                parts.append(f"{key}={format_decoded_scalar(value)}")
        return "{" + ", ".join(parts) + "}"
    return str(tuple(format_decoded_scalar(v) for v in values))


def format_decoded_scalar(value):
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def parse_perf_text(text):
    parts = text.split()
    try:
        if len(parts) == 9 and parts[0] == "PERF":
            return {
                "kind": "summary",
                "class": parts[1],
                "count": int(parts[2][1:]),
                "pass_us": int(parts[3][1:]),
                "comm_us": int(parts[4][1:]),
                "sensor_us": int(parts[5][1:]),
                "control_us": int(parts[6][1:]),
                "telemetry_us": int(parts[7][1:]),
                "max_us": int(parts[8][1:]),
            }
        if len(parts) == 7 and parts[0] == "PERC":
            return {
                "kind": "control_detail",
                "class": parts[1],
                "count": int(parts[2][1:]),
                "estimator_us": int(parts[3][1:]),
                "controller_us": int(parts[4][1:]),
                "mixer_us": int(parts[5][1:]),
                "pwm_us": int(parts[6][1:]),
            }
        if len(parts) == 7 and parts[0] == "PERS":
            return {
                "kind": "sensor_detail",
                "class": parts[1],
                "count": int(parts[2][1:]),
                "sensor_update_us": int(parts[3][1:]),
                "sensor_process_us": int(parts[4][1:]),
                "sensor_health_us": int(parts[5][1:]),
                "log_response_us": int(parts[6][1:]),
            }
        if len(parts) == 7 and parts[0] == "PERT":
            return {
                "kind": "board_detail",
                "class": parts[1],
                "count": int(parts[2][1:]),
                "rc_us": int(parts[3][1:]),
                "telemetry_enqueue_us": int(parts[4][1:]),
                "tx_flush_us": int(parts[5][1:]),
                "board_service_us": int(parts[6][1:]),
            }
        if len(parts) == 7 and parts[0] == "RLB":
            return {
                "kind": "release_loop_bench",
                "count": int(parts[1][1:]),
                "avg_us": int(parts[2][1:]),
                "p90_us": int(parts[3][3:]),
                "p99_us": int(parts[4][3:]),
                "max_us": int(parts[5][1:]),
                "missed_budget": int(parts[6][1:]),
            }
        if len(parts) == 8 and parts[0] == "RLC":
            return {
                "kind": "release_loop_class",
                "class": parts[1],
                "count": int(parts[2][1:]),
                "avg_us": int(parts[3][1:]),
                "p90_us": int(parts[4][3:]),
                "p99_us": int(parts[5][3:]),
                "max_us": int(parts[6][1:]),
                "missed_budget": int(parts[7][1:]),
            }
    except (ValueError, IndexError):
        return None
    return None


def percentile(sorted_values, pct):
    if not sorted_values:
        return None
    index = round((len(sorted_values) - 1) * pct / 100.0)
    return sorted_values[index]


def summarize(name, records):
    if not records:
        print(f"{name}: no frames")
        return None

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
    host_rate_hz = None
    if host_deltas:
        ordered = sorted(host_deltas)
        host_rate_hz = 1000.0 / statistics.fmean(host_deltas)
        print(
            "  host interval ms: "
            f"min={ordered[0]:.3f} avg={statistics.fmean(host_deltas):.3f} "
            f"p50={percentile(ordered, 50):.3f} p90={percentile(ordered, 90):.3f} "
            f"p99={percentile(ordered, 99):.3f} max={ordered[-1]:.3f} "
            f"rate={host_rate_hz:.1f}Hz"
        )
    if board_deltas:
        ordered = sorted(board_deltas)
        board_mean_ms = statistics.fmean(board_deltas)
        board_rate_hz = 1000.0 / board_mean_ms if board_mean_ms > 0 else None
        rate_text = f"{board_rate_hz:.1f}Hz" if board_rate_hz is not None else "n/a"
        print(
            "  board timestamp interval ms: "
            f"min={ordered[0]:.3f} avg={board_mean_ms:.3f} "
            f"p50={percentile(ordered, 50):.3f} p90={percentile(ordered, 90):.3f} "
            f"p99={percentile(ordered, 99):.3f} max={ordered[-1]:.3f} "
            f"rate={rate_text}"
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
    return host_rate_hz


def summarize_expected_rate(name, observed_hz, expected_hz):
    if expected_hz is None:
        return
    if observed_hz is None:
        print(f"{name}: expected={expected_hz:.1f}Hz observed=0.0Hz error=-100.0%")
        return
    error_pct = (observed_hz - expected_hz) / expected_hz * 100.0
    print(
        f"{name}: expected={expected_hz:.1f}Hz observed={observed_hz:.1f}Hz "
        f"error={error_pct:+.1f}%"
    )


class SequenceSummary:
    def __init__(self):
        self.total_valid = 0
        self.first = None
        self.last = None
        self.expected_next = None
        self.in_order = 0
        self.gaps = 0
        self.missing = 0
        self.backwards_or_reordered = 0
        self.duplicates = 0
        self.by_msgid = {}
        self.gap_samples = []
        self.pending_missing = set()

    def observe(self, frame):
        seq = frame["seq"]
        self.total_valid += 1
        self.by_msgid[frame["msgid"]] = self.by_msgid.get(frame["msgid"], 0) + 1
        if self.first is None:
            self.first = seq
            self.last = seq
            self.expected_next = (seq + 1) & 0xFF
            return

        delta = (seq - self.last) & 0xFF
        if seq == self.expected_next:
            self.in_order += 1
            self.last = seq
            self.expected_next = (seq + 1) & 0xFF
        elif delta == 0:
            self.duplicates += 1
            self._sample("duplicate", frame, delta)
        elif delta < 128:
            self.gaps += 1
            self.missing += delta - 1
            missing_seq = (self.last + 1) & 0xFF
            while missing_seq != seq:
                self.pending_missing.add(missing_seq)
                missing_seq = (missing_seq + 1) & 0xFF
            self._sample("gap", frame, delta)
            self.last = seq
            self.expected_next = (seq + 1) & 0xFF
        else:
            self.backwards_or_reordered += 1
            if seq in self.pending_missing:
                self.pending_missing.remove(seq)
                self.missing -= 1
            self._sample("backwards/reordered", frame, delta)

    def _sample(self, kind, frame, delta):
        if len(self.gap_samples) >= 8:
            return
        self.gap_samples.append(
            {
                "kind": kind,
                "prev": self.last,
                "seq": frame["seq"],
                "delta": delta,
                "msgid": frame["msgid"],
                "sysid": frame["sysid"],
                "compid": frame["compid"],
            }
        )


def summarize_sequences(summary):
    print(
        "valid MAVLink seq: "
        f"frames={summary.total_valid} first={summary.first} last={summary.last} "
        f"in_order_steps={summary.in_order} gaps={summary.gaps} "
        f"missing_est={summary.missing} reordered_or_backwards={summary.backwards_or_reordered} "
        f"duplicates={summary.duplicates}"
    )
    if summary.by_msgid:
        ordered = " ".join(
            f"{msgid}:{count}" for msgid, count in sorted(summary.by_msgid.items())
        )
        print(f"valid MAVLink msgids: {ordered}")
    for sample in summary.gap_samples:
        print(
            "  seq sample: "
            f"{sample['kind']} prev={sample['prev']} seq={sample['seq']} "
            f"delta={sample['delta']} msgid={sample['msgid']} "
            f"sys={sample['sysid']} comp={sample['compid']}"
        )


def summarize_text(records):
    if not records:
        print("text: no frames")
        return

    print(f"text: frames={len(records)}")
    latest_by_prefix = {}
    for record in records:
        text = record.get("text", "")
        if not text:
            continue
        prefix = text.split(maxsplit=1)[0]
        latest_by_prefix[prefix] = text
    for prefix in sorted(latest_by_prefix):
        print(f"  latest {prefix}: {latest_by_prefix[prefix]}")


def summarize_status_diagnostics(records, recent_limit=24):
    diagnostics = []
    latest_txq = None
    latest_txd = None
    latest_realtime = {}
    latest_tms_by_label = {}
    tms_counts_by_label = {}

    for record in records:
        text = record.get("text", "")
        if not text:
            continue
        if text.startswith("TXQ "):
            latest_txq = text
            diagnostics.append(text)
            continue
        if text.startswith("TXD "):
            latest_txd = text
            diagnostics.append(text)
            continue
        if text.startswith("TMS "):
            parts = text.split()
            label = parts[1] if len(parts) > 1 else "?"
            latest_tms_by_label[label] = text
            tms_counts_by_label[label] = tms_counts_by_label.get(label, 0) + 1
            diagnostics.append(text)
            continue
        if text.startswith(("RTC ", "RTI ", "RTG ")):
            latest_realtime[text.split()[0]] = text
            diagnostics.append(text)

    if (
        latest_txq is None
        and latest_txd is None
        and not latest_tms_by_label
        and not latest_realtime
    ):
        return

    print("status diagnostics:")
    if latest_txq is not None:
        print(f"  latest TXQ: {latest_txq}")
    if latest_txd is not None:
        print(f"  latest TXD: {latest_txd}")
    if latest_tms_by_label:
        for label in sorted(latest_tms_by_label):
            count = tms_counts_by_label[label]
            print(f"  latest TMS {label} ({count} frames): {latest_tms_by_label[label]}")
    for prefix in ("RTC", "RTI", "RTG"):
        if prefix in latest_realtime:
            print(f"  latest {prefix}: {latest_realtime[prefix]}")
    if diagnostics:
        print(f"  recent diagnostics ({min(len(diagnostics), recent_limit)} of {len(diagnostics)}):")
        for text in diagnostics[-recent_limit:]:
            print(f"    {text}")


def summarize_param_flood(args, records):
    if not args.param_flood:
        return

    expected_values = [
        args.param_flood_value + i * args.param_flood_step for i in range(args.param_flood_count)
    ]
    if args.param_flood_type == "int32":
        expected_values = [float(int(value)) for value in expected_values]
    expected_unique = set(expected_values)

    matching = [
        record
        for record in records["param"]
        if record["values"].get("id") == args.param_flood
    ]
    ack_values = [record["values"].get("value") for record in matching]
    matching_expected = [
        value
        for value in ack_values
        if any(abs(value - expected) <= 1e-6 for expected in expected_unique)
    ]
    print(
        "param flood: "
        f"name={args.param_flood} sent={args.param_flood_count} "
        f"matching_param_value={len(matching)} matching_expected={len(matching_expected)}"
    )
    if ack_values:
        print(
            "  ack values: "
            f"first={format_decoded_scalar(ack_values[0])} "
            f"last={format_decoded_scalar(ack_values[-1])} "
            f"unique={len(set(ack_values))}"
        )


def summarize_perf(records):
    if not records:
        print("perf: no timing diagnostic frames")
        return
    labels = {
        "I": "idle",
        "R": "rx-only",
        "S": "sensor-only",
        "U": "imu-no-control",
        "C": "control",
    }
    print(f"perf: frames={len(records)}")
    bench_rows = [
        record["perf"] for record in records if record["perf"]["kind"] == "release_loop_bench"
    ]
    if bench_rows:
        total_count = sum(row["count"] for row in bench_rows)
        if total_count:
            avg_us = sum(row["avg_us"] * row["count"] for row in bench_rows) / total_count
            max_us = max(row["max_us"] for row in bench_rows)
            p90_us = max(row["p90_us"] for row in bench_rows)
            p99_us = max(row["p99_us"] for row in bench_rows)
            missed = sum(row["missed_budget"] for row in bench_rows)
            print(
                "  release loop bench: "
                f"n={total_count} avg={avg_us:.1f}us "
                f"p90_max={p90_us}us p99_max={p99_us}us "
                f"max={max_us}us missed_budget={missed}"
            )
    for cls, label in [("C", "release loop closure"), ("N", "release loop no-control")]:
        rows = [
            record["perf"]
            for record in records
            if record["perf"]["kind"] == "release_loop_class"
            and record["perf"]["class"] == cls
        ]
        if not rows:
            continue
        total_count = sum(row["count"] for row in rows)
        if total_count:
            avg_us = sum(row["avg_us"] * row["count"] for row in rows) / total_count
            max_us = max(row["max_us"] for row in rows)
            p90_us = max(row["p90_us"] for row in rows)
            p99_us = max(row["p99_us"] for row in rows)
            missed = sum(row["missed_budget"] for row in rows)
            print(
                f"  {label}: "
                f"n={total_count} avg={avg_us:.1f}us "
                f"p90_max={p90_us}us p99_max={p99_us}us "
                f"max={max_us}us missed_budget={missed}"
            )
    for cls in ["I", "R", "S", "U", "C"]:
        rows = [
            record["perf"]
            for record in records
            if record["perf"]["kind"] == "summary" and record["perf"]["class"] == cls
        ]
        if not rows:
            continue
        total_count = sum(row["count"] for row in rows)
        if total_count == 0:
            continue

        def weighted(field):
            return sum(row[field] * row["count"] for row in rows) / total_count

        max_us = max(row["max_us"] for row in rows)
        print(
            f"  {labels[cls]}: n={total_count} "
            f"pass_avg={weighted('pass_us'):.1f}us "
            f"comm={weighted('comm_us'):.1f}us "
            f"sensor={weighted('sensor_us'):.1f}us "
            f"control={weighted('control_us'):.1f}us "
            f"telemetry={weighted('telemetry_us'):.1f}us "
            f"pass_max={max_us}us"
        )
        detail_groups = [
            (
                "control detail",
                "control_detail",
                [
                    ("estimator", "estimator_us"),
                    ("controller", "controller_us"),
                    ("mixer", "mixer_us"),
                    ("pwm", "pwm_us"),
                ],
            ),
            (
                "sensor detail",
                "sensor_detail",
                [
                    ("update", "sensor_update_us"),
                    ("process", "sensor_process_us"),
                    ("health", "sensor_health_us"),
                    ("logs", "log_response_us"),
                ],
            ),
            (
                "board/tx detail",
                "board_detail",
                [
                    ("rc", "rc_us"),
                    ("telem_enq", "telemetry_enqueue_us"),
                    ("tx_flush", "tx_flush_us"),
                    ("board", "board_service_us"),
                ],
            ),
        ]
        for title, kind, fields in detail_groups:
            detail_rows = [
                record["perf"]
                for record in records
                if record["perf"]["kind"] == kind and record["perf"]["class"] == cls
            ]
            if not detail_rows:
                continue
            detail_count = sum(row["count"] for row in detail_rows)
            if detail_count == 0:
                continue

            def detail_weighted(field):
                return sum(row[field] * row["count"] for row in detail_rows) / detail_count

            details = " ".join(
                f"{label}={detail_weighted(field):.1f}us" for label, field in fields
            )
            print(f"    {title}: {details}")


def acceptance_failures(args, parser, sequence_summary, records):
    failures = []
    if not args.no_crc and parser.invalid_crc != 0 and sequence_summary.missing != 0:
        failures.append(
            "MAVLink valid sequence gaps after CRC failures: "
            f"missing={sequence_summary.missing} invalid_crc={parser.invalid_crc} "
            f"{parser.invalid_by_msgid}"
        )
    required = {
        "imu": 100,
        "baro": 1,
        "rc": 1,
        "status": 1,
        "heartbeat": 1,
        "timesync": 1,
        "cmd_ack": 1,
        "version": 1,
        "param": 1,
    }
    for name, minimum in required.items():
        if len(records[name]) < minimum:
            failures.append(f"{name} frames below minimum: {len(records[name])} < {minimum}")

    if records["cmd_ack"]:
        matching = [
            record
            for record in records["cmd_ack"]
            if record["values"].get("command") == ROSFLIGHT_CMD_SEND_VERSION
            and record["values"].get("success") == 1
        ]
        if not matching:
            failures.append("missing successful ROSFLIGHT_CMD_SEND_VERSION ack")

    bench_rows = [
        record["perf"]
        for record in records["perf"]
        if record["perf"]["kind"] == "release_loop_bench"
    ]
    if bench_rows:
        missed = sum(row["missed_budget"] for row in bench_rows)
        if missed != 0:
            failures.append(f"release-loop budget misses: {missed}")

    text_values = [record.get("text", "") for record in records["text"] + records["perf"]]
    for prefix in ("IMDQ", "BRDQ"):
        for text in text_values:
            if text.startswith(prefix):
                parts = text.split()
                for part in parts:
                    if (
                        (part.startswith("d") or part.startswith("g"))
                        and part[1:].isdigit()
                        and int(part[1:]) != 0
                    ):
                        failures.append(f"{prefix} reported drops: {text}")

    return failures


def main():
    args = parse_args()
    if args.param_flood:
        args.bidirectional = True
    parser = MavlinkV1Parser(
        validate_crc=not args.no_crc,
        invalid_sample_limit=args.invalid_samples if args.diagnostics else 0,
    )
    sequence_summary = SequenceSummary()
    records = {
        "imu": [],
        "baro": [],
        "attitude": [],
        "output_raw": [],
        "status": [],
        "perf": [],
        "text": [],
        "rc": [],
        "gnss": [],
        "timesync": [],
        "heartbeat": [],
        "param": [],
        "cmd_ack": [],
        "version": [],
    }
    shown = 0
    deadline = time.monotonic() + args.duration_s if args.duration_s > 0 else None
    rx_bytes = 0
    first_rx_ns = None
    last_rx_ns = None
    timesync_seq = 0
    timesync_sent = 0
    next_timesync_ns = 0
    injector = (
        MavlinkInjector(args, request_delay_base_s=args.warmup_s if args.acceptance else 0.0)
        if args.bidirectional or args.acceptance
        else None
    )
    raw_capture = open(args.raw_capture, "wb") if args.raw_capture else None

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
                not args.acceptance
                and
                len(records["imu"]) >= args.samples
                and len(records["baro"]) >= min(args.samples, 50)
                and len(records["status"]) >= min(args.samples, 50)
            ):
                break

            now_ns = time.monotonic_ns()
            if args.timesync_probe and args.transport == "uart" and now_ns >= next_timesync_ns:
                try:
                    os.write(source, timesync_request(timesync_seq))
                    timesync_seq = (timesync_seq + 1) & 0xFF
                    timesync_sent += 1
                    next_timesync_ns = now_ns + int(args.timesync_period_s * 1_000_000_000)
                except BlockingIOError:
                    next_timesync_ns = now_ns + 100_000_000
            if injector is not None:
                try:
                    injector.service(source)
                except BlockingIOError:
                    pass

            readable, _, _ = select.select([source], [], [], 0.05 if injector is not None else 0.25)
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
            if host_ns >= record_after_ns:
                rx_bytes += len(data)
                first_rx_ns = first_rx_ns or host_ns
                last_rx_ns = host_ns
                if raw_capture is not None:
                    raw_capture.write(data)
            for frame in parser.feed(data):
                if host_ns >= record_after_ns:
                    sequence_summary.observe(frame)
                decoded = decode_sensor_frame(frame)
                if decoded is None:
                    continue
                name = decoded["name"]
                if shown < args.show:
                    if name == "perf" or name == "text":
                        print(
                            f"{name} seq={frame['seq']} sys={frame['sysid']} "
                            f"comp={frame['compid']} text={decoded['text']}"
                        )
                    else:
                        print(
                            f"{name} seq={frame['seq']} sys={frame['sysid']} "
                            f"comp={frame['compid']} board_us={decoded['board_us']} "
                            f"values={format_decoded_values(decoded['values'])}"
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
                        **(
                            {"text": decoded["text"], "perf": decoded["perf"]}
                            if name == "perf"
                            else {}
                        ),
                        **({"text": decoded["text"]} if name == "text" else {}),
                    }
                )
    finally:
        if raw_capture is not None:
            raw_capture.close()
        if args.transport == "uart":
            os.close(source)
        else:
            source.close()

    imu_rate_hz = summarize("imu", records["imu"])
    summarize("baro", records["baro"])
    attitude_rate_hz = summarize("attitude", records["attitude"])
    output_raw_rate_hz = summarize("output_raw", records["output_raw"])
    rc_rate_hz = summarize("rc", records["rc"])
    summarize("gnss", records["gnss"])
    summarize("timesync", records["timesync"])
    summarize("heartbeat", records["heartbeat"])
    summarize("param", records["param"])
    summarize("cmd_ack", records["cmd_ack"])
    summarize("version", records["version"])
    summarize("status", records["status"])
    summarize_text(records["text"])
    summarize_status_diagnostics(records["text"])
    summarize_param_flood(args, records)
    summarize_perf(records["perf"])
    summarize_expected_rate("imu receive rate", imu_rate_hz, args.expect_imu_hz)
    summarize_expected_rate("rc receive rate", rc_rate_hz, args.expect_rc_hz)
    summarize_expected_rate(
        "attitude receive rate", attitude_rate_hz, args.expect_attitude_hz
    )
    summarize_expected_rate(
        "output_raw receive rate", output_raw_rate_hz, args.expect_output_raw_hz
    )
    if first_rx_ns is not None and last_rx_ns is not None and last_rx_ns > first_rx_ns:
        duration_s = (last_rx_ns - first_rx_ns) / 1_000_000_000.0
        print(
            f"rx bytes: {rx_bytes} over {duration_s:.2f}s "
            f"= {rx_bytes / duration_s:.1f} B/s"
        )
    if args.diagnostics:
        print(
            f"parser: candidates={parser.candidates} invalid_crc={parser.invalid_crc} "
            f"invalid_by_msgid={parser.invalid_by_msgid}"
        )
        summarize_sequences(sequence_summary)
        for sample in parser.invalid_samples:
            print(
                "  invalid sample: "
                f"len={sample['len']} seq={sample['seq']} sys={sample['sysid']} "
                f"comp={sample['compid']} msgid={sample['msgid']} "
                f"crc=0x{sample['crc']:04x} head={sample['head']}"
            )
    if args.timesync_probe:
        print(f"timesync probe: sent={timesync_sent} responses={len(records['timesync'])}")
    if injector is not None:
        print(
            "injected: "
            + " ".join(f"{key}={value}" for key, value in injector.sent.items())
        )
    if args.acceptance:
        failures = acceptance_failures(args, parser, sequence_summary, records)
        if failures:
            print("acceptance: FAIL")
            for failure in failures:
                print(f"  {failure}")
            raise SystemExit(1)
        print("acceptance: PASS")


if __name__ == "__main__":
    main()
