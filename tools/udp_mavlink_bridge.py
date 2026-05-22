#!/usr/bin/env python3
import argparse
import select
import socket
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(
        description="Minimal UDP MAVLink bridge for the Pico 2 W transport bring-up."
    )
    parser.add_argument(
        "board",
        nargs="?",
        help="Pico 2 W IPv4 address. Omit to learn it from the firmware UDP beacon.",
    )
    parser.add_argument("--board-port", type=int, default=14550)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=14550)
    parser.add_argument(
        "--hello",
        default="voloxide-host-hello",
        help="Initial datagram sent so firmware learns this host endpoint.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    board = (args.board, args.board_port) if args.board else None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.bind_port))
    sock.setblocking(False)

    if board:
        sock.sendto(args.hello.encode("ascii"), board)
        print(f"sent hello to {board[0]}:{board[1]}")
        board_label = f"{board[0]}:{board[1]}"
    else:
        board_label = "auto"

    print(f"listening on {args.bind}:{args.bind_port}, board={board_label}")
    print("type hex bytes like 'fe 09 00 ...' or text prefixed with 't:'")

    while True:
        readable, _, _ = select.select([sock, sys.stdin], [], [], 1.0)
        if not readable:
            if board is not None:
                sock.sendto(args.hello.encode("ascii"), board)
                print(f"sent hello to {board[0]}:{board[1]}")
            continue

        for source in readable:
            if source is sock:
                data, addr = sock.recvfrom(2048)
                if board is None and data == b"voloxide-pico2w-mavlink":
                    board = (addr[0], args.board_port)
                    sock.sendto(args.hello.encode("ascii"), board)
                    print(f"learned board={board[0]}:{board[1]}")
                    print(f"sent hello to {board[0]}:{board[1]}")
                    continue
                stamp = time.strftime("%H:%M:%S")
                print(f"{stamp} {addr[0]}:{addr[1]} {data.hex(' ')}")
                continue

            line = sys.stdin.readline()
            if not line:
                return
            line = line.strip()
            if not line:
                continue
            if line.startswith("t:"):
                payload = line[2:].encode("utf-8")
            else:
                payload = bytes.fromhex(line)
            if board is None:
                print("board unknown; waiting for firmware beacon")
            else:
                sock.sendto(payload, board)
                print(f"sent {len(payload)} bytes to {board[0]}:{board[1]}")


if __name__ == "__main__":
    main()
