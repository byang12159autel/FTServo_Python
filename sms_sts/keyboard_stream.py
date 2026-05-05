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
# Force regulation: pair --max-torque (caps PID output) with --add-close
# (extends the goal past the firmware angle limit). The position controller
# saturates at MAX_TORQUE_LIMIT against any compliant resistance, applying
# constant compression force. No host-side contact logic — the cap is the
# only force regulator.
#
# Requires: pip install tyro
# POSIX-only (uses termios for raw keyboard input).
#
# Usage:
#   python3 keyboard_stream.py --servo-id 2 --baudrate 115200
#

import csv
import os
import sys
import time
import select
import termios
import tty
from dataclasses import dataclass
from typing import Optional

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


@dataclass
class Args:
    servo_id: int = 1
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Serial baud rate."""
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    """Serial port path. Defaults to the stable by-id symlink for the CH340 adapter."""
    rate_hz: float = 50.0
    """Control loop rate in Hz."""
    speed: int = 1000
    """Max speed for WritePosEx (units of 0.732 rpm)."""
    acc: int = 50
    """Acceleration for WritePosEx (units of 8.7 deg/s^2). Use 0 for snappiest response."""
    step_size: int = 20
    """Initial jog step size in ticks per 'a'/'d' key press. '+' doubles, '-' halves at runtime."""
    max_torque: Optional[int] = None
    """If set, write MAX_TORQUE_LIMIT (reg 16-17, 0..1000) to EEPROM at startup. Caps how hard the position controller can push. Persists across power cycles."""
    add_close: int = 0
    """Add this many ticks to the closed-side limit. The position goal can then be set past the firmware angle limit, creating persistent position error so the PID saturates at MAX_TORQUE_LIMIT — useful for compliant grippers that need constant compression force. Pair with --max-torque."""
    log: Optional[str] = None
    """If set, write a CSV log (t,target,present_pos,present_load,present_current_mA,present_temp_C) for plot_stream.py."""


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

    if args.add_close < 0:
        print("--add-close must be >= 0. Got %d." % args.add_close)
        portHandler.closePort()
        return
    effective_max = max_tick + args.add_close

    # Force position mode (0). EEPROM writes require unlock; relock after.
    # If --max-torque was given, also write MAX_TORQUE_LIMIT in the same window.
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 0)
    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_MODE, 0)
    if args.max_torque is not None:
        if not (0 <= args.max_torque <= 1000):
            print("--max-torque must be in [0, 1000]. Got %d." % args.max_torque)
            packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 1)
            portHandler.closePort()
            return
        packetHandler.write2ByteTxRx(args.servo_id, 16, args.max_torque)   # reg 16-17
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 1)
    if comm != COMM_SUCCESS or err != 0:
        print("Failed to set servo to position mode.")
        portHandler.closePort()
        return
    if args.max_torque is not None:
        readback, c, e = packetHandler.read2ByteTxRx(args.servo_id, 16)
        print("MAX_TORQUE_LIMIT set to %d (read back %d, %d%% of stall torque)." % (
            args.max_torque, readback, readback // 10))

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
    step_size = args.step_size
    period = 1.0 / args.rate_hz

    if args.add_close > 0:
        print("Range: [%d, %d]   add_close=%d → effective MAX goal=%d   start_target=%d" % (
            min_tick, max_tick, args.add_close, effective_max, target))
        print("(Firmware will physically gate motion at MAX=%d; commanding past it creates persistent" % max_tick)
        print(" position error → PID saturates at MAX_TORQUE_LIMIT for constant-force compression.)")
    else:
        print("Range: [%d, %d]   start_target=%d" % (min_tick, max_tick, target))
    print("Keys: a/d jog | s hold | 0 MIN | 9 MAX | -/+ step | q quit")

    log_file = None
    log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t", "target", "present_pos", "present_load", "present_current_mA", "present_temp_C"])
        print("Logging to %s" % args.log)

    t_start = time.monotonic()

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
                    target = max(min_tick, target - step_size)
                elif key == 'd':
                    target = min(effective_max, target + step_size)
                elif key == 's':
                    cur, c, e = packetHandler.ReadPos(args.servo_id)
                    if c == COMM_SUCCESS and e == 0:
                        target = max(min_tick, min(effective_max, cur))
                elif key == '0':
                    target = min_tick
                elif key == '9':
                    target = effective_max
                elif key == '-':
                    step_size = max(1, step_size // 2)
                elif key == '+' or key == '=':
                    step_size = min(effective_max - min_tick, max(1, step_size * 2))

            data, c, e = packetHandler.readTxRx(args.servo_id, SMS_STS_PRESENT_POSITION_L, 6)
            read_ok = (c == COMM_SUCCESS and e == 0 and len(data) == 6)
            if read_ok:
                pos_word = packetHandler.scs_makeword(data[0], data[1])
                present_pos = packetHandler.scs_tohost(pos_word, 15)
                load_word = packetHandler.scs_makeword(data[4], data[5])
                load_signed = packetHandler.scs_tohost(load_word, 10)
            else:
                present_pos = float('nan')
                load_signed = 0

            curr_raw, cc, ce = packetHandler.read2ByteTxRx(args.servo_id, SMS_STS_PRESENT_CURRENT_L)
            curr_ok = (cc == COMM_SUCCESS and ce == 0)
            current_mA = packetHandler.scs_tohost(curr_raw, 15) * 6.5 if curr_ok else 0.0

            temp_raw, tc, te = packetHandler.read1ByteTxRx(args.servo_id, SMS_STS_PRESENT_TEMPERATURE)
            temp_ok = (tc == COMM_SUCCESS and te == 0)
            temp_c = temp_raw if temp_ok else 0

            packetHandler.SyncWritePosEx(args.servo_id, target, args.speed, args.acc)
            packetHandler.groupSyncWrite.txPacket()
            packetHandler.groupSyncWrite.clearParam()

            if log_writer is not None:
                log_writer.writerow([
                    "%.4f" % (t0 - t_start),
                    target,
                    present_pos if read_ok else "nan",
                    load_signed if read_ok else "nan",
                    "%.0f" % current_mA if curr_ok else "nan",
                    temp_c if temp_ok else "nan",
                ])

            sys.stdout.write("\rtarget_pos=%4d  pos=%4d  load=%+5d  curr=%+6.0fmA  temp=%2dC  step_size=%4d  " % (
                target, present_pos if read_ok else -1, load_signed, current_mA, temp_c, step_size))
            sys.stdout.flush()

            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        sys.stdout.write("\n")
        sys.stdout.flush()
        if log_file is not None:
            log_file.close()
        packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
        print("Torque disabled. Closing port.")
        portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
