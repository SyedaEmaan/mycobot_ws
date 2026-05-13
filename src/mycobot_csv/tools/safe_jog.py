#!/usr/bin/env python3
"""
safe_jog.py — open-loop timed jog for verify_six / forward_velocity_controller.

Publishes Float64MultiArray velocities for one or more joints. Each joint
keeps moving only for the duration needed to traverse its requested angle
at its chosen speed, then is zeroed individually. A final all-zero command
is published on completion, Ctrl+C, or exception.

Usage:
  # Single joint (positional, back-compat):
  python3 safe_jog.py <joint:1-6> <degrees> [--speed RAD_S] [--n N]

  # Multiple joints (one --move per joint):
  python3 safe_jog.py --move J:DEG[@SPEED] [--move J:DEG[@SPEED] ...] [--n N]

Examples:
  python3 safe_jog.py 1 2
  python3 safe_jog.py 3 -5 --speed 0.10
  python3 safe_jog.py --move 1:2 --move 3:-5
  python3 safe_jog.py --move 1:2 --move 3:-5@0.10 --move 5:3
"""
import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

TOPIC = "/forward_velocity_controller/commands"
DEFAULT_SPEED = 0.15  # rad/s


@dataclass
class Move:
    joint: int      # 1-based
    degrees: float
    speed: float    # rad/s, positive magnitude
    duration: float
    velocity: float  # signed rad/s


def parse_move(spec: str, n_joints: int) -> Move:
    # "J:DEG" or "J:DEG@SPEED"
    if "@" in spec:
        body, speed_s = spec.split("@", 1)
        speed = float(speed_s)
    else:
        body, speed = spec, DEFAULT_SPEED
    j_s, deg_s = body.split(":", 1)
    j, deg = int(j_s), float(deg_s)
    if not (1 <= j <= n_joints):
        raise argparse.ArgumentTypeError(f"joint must be in 1..{n_joints}, got {j}")
    if speed <= 0:
        raise argparse.ArgumentTypeError("speed must be > 0 rad/s")
    rad = math.radians(deg)
    direction = 1.0 if rad >= 0 else -1.0
    return Move(
        joint=j, degrees=deg, speed=speed,
        duration=abs(rad) / speed,
        velocity=direction * speed,
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("joint", type=int, nargs="?", help="(positional) 1-based joint")
    p.add_argument("degrees", type=float, nargs="?", help="(positional) signed deg")
    p.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                   help="rad/s for positional form (default 0.15)")
    p.add_argument("--move", action="append", default=[],
                   help="J:DEG[@SPEED] (repeatable)")
    p.add_argument("--n", type=int, default=6, help="vector length (default 6)")
    args = p.parse_args()

    moves: list[Move] = []
    if args.move:
        if args.joint is not None or args.degrees is not None:
            print("error: mixing positional joint/degrees with --move is not allowed",
                  file=sys.stderr)
            return 2
        for spec in args.move:
            moves.append(parse_move(spec, args.n))
    else:
        if args.joint is None or args.degrees is None:
            p.print_usage(sys.stderr)
            return 2
        moves.append(parse_move(
            f"{args.joint}:{args.degrees}@{args.speed}", args.n))

    used = {m.joint for m in moves}
    if len(used) != len(moves):
        print("error: same joint specified more than once", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("safe_jog")
    pub = node.create_publisher(Float64MultiArray, TOPIC, 10)

    t0 = time.monotonic()
    while pub.get_subscription_count() < 1:
        if time.monotonic() - t0 > 3.0:
            print(f"no subscriber on {TOPIC} after 3 s. Is the launch running?\n"
                  f"Check RMW_IMPLEMENTATION matches the launch (rmw_fastrtps_cpp).",
                  file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 3
        rclpy.spin_once(node, timeout_sec=0.05)

    zeros = [0.0] * args.n
    current = list(zeros)
    for m in moves:
        current[m.joint - 1] = m.velocity

    def publish(vec):
        msg = Float64MultiArray()
        msg.data = list(vec)
        pub.publish(msg)

    stopping = {"flag": False}

    def stop_and_exit(*_):
        stopping["flag"] = True
        for _i in range(3):
            publish(zeros)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, stop_and_exit)
    signal.signal(signal.SIGTERM, stop_and_exit)

    print("jogging:")
    for m in moves:
        print(f"  joint {m.joint}: {m.degrees:+.2f} deg @ {m.velocity:+.3f} rad/s "
              f"for {m.duration:.3f} s")
    print(f"total wall time: {max(m.duration for m in moves):.3f} s")

    start = time.monotonic()
    publish(current)

    # Schedule per-joint zero-times.
    deadlines = sorted(((m.duration, m.joint) for m in moves))
    next_idx = 0
    tick = 0.01  # 100 Hz tick — fine-grained but cheap.
    while next_idx < len(deadlines) and not stopping["flag"]:
        elapsed = time.monotonic() - start
        if elapsed >= deadlines[next_idx][0]:
            j = deadlines[next_idx][1]
            current[j - 1] = 0.0
            publish(current)
            next_idx += 1
        else:
            time.sleep(min(tick, deadlines[next_idx][0] - elapsed))

    for _ in range(3):
        publish(zeros)
        time.sleep(0.02)

    print("done; zeros sent.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
