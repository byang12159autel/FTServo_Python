#!/usr/bin/env python
#
# *********     Gripper Position Streaming      *********
#
# Streams position commands to an SMS/STS servo at a fixed control-loop
# rate, with a simple POSIX keyboard interface. Reads the EPROM angle
# limits (written by calibrate_range.py) to define the safe travel range.
#
# Keys:
#   a / d     jog target toward MIN / MAX by step ticks
#   s         hold at present position
#   0         snap target to MIN
#   9         snap target to MAX
#   - / +     halve / double step size
#   q         quit
#
# Contact freeze: each tick the script reads PRESENT_LOAD; when its
# magnitude exceeds --force-threshold, the goal is pinned to the present
# position (freeze_pos) so the servo stops pushing further. The user can
# still jog the target via keys; once |target - freeze_pos| exceeds
# --force-hysteresis ticks, the freeze releases and normal tracking resumes.
#
# Requires: pip install tyro
# POSIX-only (uses termios for raw keyboard input).
#
# Usage:
#   python3 keyboard_stream.py --servo-id 2 --baudrate 115200
#

import os
import sys
import time
import select
import termios
import tty
from dataclasses import dataclass

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


@dataclass
class Args:
    servo_id: int = 2
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Serial baud rate."""
    port: str = "/dev/ttyUSB0"
    """Serial port path. Windows: 'COM1', Linux: '/dev/ttyUSB0', Mac: '/dev/tty.usbserial-*'."""
    rate_hz: float = 50.0
    """Control loop rate in Hz."""
    speed: int = 1000
    """Max speed for WritePosEx (units of 0.732 rpm)."""
    acc: int = 50
    """Acceleration for WritePosEx (units of 8.7 deg/s^2). Use 0 for snappiest response."""
    step: int = 20
    """Initial step size in ticks per key press."""
    force_threshold: int = 400
    """Magnitude threshold on PRESENT_LOAD that triggers contact freeze. PRESENT_LOAD is roughly 0..1000 (PWM duty x10). Tune empirically."""
    force_hysteresis: int = 50
    """Once frozen, exit freeze when |target - contact_pos| exceeds this many ticks."""


def poll_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main(args: Args) -> None:
    portHandler = PortHandler(args.port)
    packetHandler = sms_sts(portHandler)

    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return

    if portHandler.setBaudRate(args.baudrate):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        portHandler.closePort()
        return

    min_tick, comm, err = packetHandler.read2ByteTxRx(args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L)
    if comm != COMM_SUCCESS or err != 0:
        print("Failed to read MIN_ANGLE_LIMIT from EPROM. Run calibrate_range.py first.")
        portHandler.closePort()
        return
    max_tick, comm, err = packetHandler.read2ByteTxRx(args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L)
    if comm != COMM_SUCCESS or err != 0:
        print("Failed to read MAX_ANGLE_LIMIT from EPROM. Run calibrate_range.py first.")
        portHandler.closePort()
        return

    if max_tick <= min_tick:
        print("Invalid EPROM limits (MAX=%d <= MIN=%d). Run calibrate_range.py first." % (max_tick, min_tick))
        portHandler.closePort()
        return

    # Force position mode (0). EEPROM writes require unlock; relock after.
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 0)
    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_MODE, 0)
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 1)
    if comm != COMM_SUCCESS or err != 0:
        print("Failed to set servo to position mode.")
        portHandler.closePort()
        return

    # Read current position and seed GOAL with it BEFORE enabling torque, so the
    # motor doesn't snap to a stale goal from a previous run.
    pos, comm, err = packetHandler.ReadPos(args.servo_id)
    if comm != COMM_SUCCESS or err != 0:
        print("Failed to read present position.")
        portHandler.closePort()
        return
    target = max(min_tick, min(max_tick, pos))
    packetHandler.WritePosEx(args.servo_id, target, args.speed, args.acc)

    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
    if comm != COMM_SUCCESS or err != 0:
        print("Failed to enable torque.")
        portHandler.closePort()
        return
    step = args.step
    period = 1.0 / args.rate_hz

    in_contact = False
    freeze_pos = 0

    print("Range: [%d, %d]   start_target=%d" % (min_tick, max_tick, target))
    print("Force: threshold=%d  hysteresis=%d ticks" % (args.force_threshold, args.force_hysteresis))
    print("Keys: a/d jog | s hold | 0 MIN | 9 MAX | -/+ step | q quit")

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        running = True
        while running:
            t0 = time.monotonic()

            key = poll_key()
            if key is not None:
                if key == 'q':
                    running = False
                elif key == 'a':
                    target = max(min_tick, target - step)
                elif key == 'd':
                    target = min(max_tick, target + step)
                elif key == 's':
                    cur, c, e = packetHandler.ReadPos(args.servo_id)
                    if c == COMM_SUCCESS and e == 0:
                        target = max(min_tick, min(max_tick, cur))
                elif key == '0':
                    target = min_tick
                elif key == '9':
                    target = max_tick
                elif key == '-':
                    step = max(1, step // 2)
                elif key == '+' or key == '=':
                    step = min(max_tick - min_tick, max(1, step * 2))

            load_raw, c, e = packetHandler.read2ByteTxRx(args.servo_id, SMS_STS_PRESENT_LOAD_L)
            load_mag = (load_raw & 0x3FF) if (c == COMM_SUCCESS and e == 0) else 0

            if not in_contact and load_mag > args.force_threshold:
                cur, c, e = packetHandler.ReadPos(args.servo_id)
                if c == COMM_SUCCESS and e == 0:
                    freeze_pos = cur
                    in_contact = True

            if in_contact and abs(target - freeze_pos) > args.force_hysteresis:
                in_contact = False

            commanded = freeze_pos if in_contact else target
            packetHandler.WritePosEx(args.servo_id, commanded, args.speed, args.acc)

            tag = "[CONTACT]" if in_contact else "         "
            sys.stdout.write("\rtarget=%4d  cmd=%4d  load=%4d  step=%4d  %s " % (target, commanded, load_mag, step, tag))
            sys.stdout.flush()

            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        sys.stdout.write("\n")
        sys.stdout.flush()
        packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
        print("Torque disabled. Closing port.")
        portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
