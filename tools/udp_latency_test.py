#!/usr/bin/env python3
import argparse
import socket
import statistics
import struct
import time


MAGIC = b"VXL1"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure Veloxity Pico 2 W UDP echo round-trip latency."
    )
    parser.add_argument("board", help="Pico 2 W IPv4 address")
    parser.add_argument("--board-port", type=int, default=14550)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=0)
    parser.add_argument("--count", type=int, default=500)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--timeout-ms", type=float, default=250.0)
    parser.add_argument("--payload-bytes", type=int, default=32)
    return parser.parse_args()


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * pct / 100.0)
    return ordered[index]


def main():
    args = parse_args()
    board = (args.board, args.board_port)
    period = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
    timeout = args.timeout_ms / 1000.0
    payload_len = max(args.payload_bytes, len(MAGIC) + 12)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.bind_port))
    sock.settimeout(timeout)

    print(
        f"udp latency: board={board[0]}:{board[1]} count={args.count} "
        f"rate={args.rate_hz:g}Hz payload={payload_len}B",
        flush=True,
    )

    rtts_ms = []
    lost = 0
    next_send = time.perf_counter()

    for seq in range(args.count):
        now = time.perf_counter()
        if now < next_send:
            time.sleep(next_send - now)
        next_send += period

        sent_ns = time.monotonic_ns()
        packet = bytearray(payload_len)
        packet[: len(MAGIC)] = MAGIC
        struct.pack_into("!IQ", packet, len(MAGIC), seq, sent_ns)
        sock.sendto(packet, board)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                sock.settimeout(max(0.001, deadline - time.monotonic()))
                data, addr = sock.recvfrom(2048)
            except TimeoutError:
                break

            if addr[0] != board[0] or not data.startswith(MAGIC):
                continue
            if len(data) < len(MAGIC) + 12:
                continue
            rx_seq, rx_sent_ns = struct.unpack_from("!IQ", data, len(MAGIC))
            if rx_seq != seq or rx_sent_ns != sent_ns:
                continue
            rtts_ms.append((time.monotonic_ns() - sent_ns) / 1_000_000.0)
            break
        else:
            lost += 1
            continue

        if len(rtts_ms) != seq + 1 - lost:
            lost += 1

    if not rtts_ms:
        print(f"received 0/{args.count}; lost={lost}")
        return 1

    print(f"received {len(rtts_ms)}/{args.count}; lost={lost}")
    print(
        "rtt ms: "
        f"min={min(rtts_ms):.3f} "
        f"avg={statistics.fmean(rtts_ms):.3f} "
        f"p50={percentile(rtts_ms, 50):.3f} "
        f"p90={percentile(rtts_ms, 90):.3f} "
        f"p99={percentile(rtts_ms, 99):.3f} "
        f"max={max(rtts_ms):.3f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
