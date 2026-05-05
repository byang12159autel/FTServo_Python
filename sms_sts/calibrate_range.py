#!/usr/bin/env python
#
# *********     Gripper Range Calibration      *********
#
# Hand-jog calibration for SMS/STS servos used as a robot gripper.
# Disables torque so the user can move the gripper by hand to fully-OPEN
# and fully-CLOSED positions, then writes those ticks as min/max angle
# limits to the servo's EPROM so the firmware refuses to drive past them.
#
# Pass both --min-tick and --max-tick to skip the hand-jog capture and write
# the supplied values directly (useful when you already know the limits, e.g.
# from a previous read_spec.py readout).
#
# Requires: pip install tyro
#
# Usage:
#   python3 calibrate_range.py --servo-id 2 --baudrate 115200
#   python3 calibrate_range.py --servo-id 1 --min-tick 1500 --max-tick 2700
#

import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

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
    min_range: int = 50
    """Reject calibration if |CLOSED_TICK - OPEN_TICK| is below this many ticks."""
    min_tick: int | None = None
    """Manual MIN_ANGLE_LIMIT (ticks). If both --min-tick and --max-tick are set, the hand-jog capture is skipped."""
    max_tick: int | None = None
    """Manual MAX_ANGLE_LIMIT (ticks). If both --min-tick and --max-tick are set, the hand-jog capture is skipped."""


def check(comm_result, error, packetHandler, where):
    if comm_result != COMM_SUCCESS:
        print("[%s] %s" % (where, packetHandler.getTxRxResult(comm_result)))
        return False
    if error != 0:
        print("[%s] %s" % (where, packetHandler.getRxPacketError(error)))
        return False
    return True


def capture(packetHandler, scs_id, label):
    print("Move the gripper to the fully-%s position by hand, then press Enter..." % label)
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    last_pos = None
    try:
        tty.setcbreak(fd)
        while True:
            pos, comm, err = packetHandler.ReadPos(scs_id)
            if check(comm, err, packetHandler, "read %s" % label):
                last_pos = pos
                sys.stdout.write("\r  live %s pos = %4d   " % (label, pos))
                sys.stdout.flush()
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1)
                if ch in ("\r", "\n"):
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        sys.stdout.write("\n")
        sys.stdout.flush()
    if last_pos is None:
        return None
    print("  recorded %s_TICK = %d" % (label, last_pos))
    return last_pos


def main(args: Args) -> None:
    if (args.min_tick is None) != (args.max_tick is None):
        print("--min-tick and --max-tick must be supplied together (or neither).")
        return
    if args.min_tick is not None and args.min_tick >= args.max_tick:
        print("--min-tick (%d) must be < --max-tick (%d)." % (args.min_tick, args.max_tick))
        return

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

    # Disable torque so the user can hand-jog the gripper.
    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
    if not check(comm, err, packetHandler, "disable torque"):
        portHandler.closePort()
        return

    # Zero any existing OFS so captured positions are raw encoder ticks. We may
    # write a new OFS later if the gripper's range crosses the encoder wrap.
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 0)
    comm, err = packetHandler.write2ByteTxRx(args.servo_id, SMS_STS_OFS_L, 0)
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 1)
    if not check(comm, err, packetHandler, "zero OFS"):
        portHandler.closePort()
        return

    if args.min_tick is not None:
        open_tick, closed_tick = args.min_tick, args.max_tick
        print("Manual mode: skipping hand-jog capture.")
    else:
        open_tick = capture(packetHandler, args.servo_id, "OPEN")
        if open_tick is None:
            portHandler.closePort()
            return
        closed_tick = capture(packetHandler, args.servo_id, "CLOSED")
        if closed_tick is None:
            portHandler.closePort()
            return

    if abs(closed_tick - open_tick) < args.min_range:
        print("Range too narrow (%d < %d ticks). Aborting." % (abs(closed_tick - open_tick), args.min_range))
        portHandler.closePort()
        return

    min_tick = min(open_tick, closed_tick)
    max_tick = max(open_tick, closed_tick)

    if abs(closed_tick - open_tick) > 2048:
        print("WARNING: open/closed differ by >2048 ticks; the gripper likely crosses")
        print("the encoder wraparound (raw=0). Position-mode commands near MIN/MAX")
        print("may overshoot through 0 and stall. Consider rotating the servo horn.")

    print("")
    print("Will write to EPROM:")
    print("  MIN_ANGLE_LIMIT (reg 9)  = %d" % min_tick)
    print("  MAX_ANGLE_LIMIT (reg 11) = %d" % max_tick)
    print("  (OPEN_TICK=%d, CLOSED_TICK=%d)" % (open_tick, closed_tick))
    if input("Confirm? [y/N] ").strip().lower() != "y":
        print("Aborted.")
        portHandler.closePort()
        return

    mode_before, comm, err = packetHandler.read1ByteTxRx(args.servo_id, SMS_STS_MODE)
    if not check(comm, err, packetHandler, "read MODE pre"):
        portHandler.closePort()
        return
    print("Initial MODE register = %d (0=position, 1=wheel/CR, 2=PWM, 3=step)" % mode_before)

    comm, err = packetHandler.unLockEprom(args.servo_id)
    if not check(comm, err, packetHandler, "unlock EPROM"):
        portHandler.closePort()
        return

    comm, err = packetHandler.write2ByteTxRx(args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L, min_tick)
    if not check(comm, err, packetHandler, "write MIN_ANGLE"):
        portHandler.closePort()
        return

    comm, err = packetHandler.write2ByteTxRx(args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L, max_tick)
    if not check(comm, err, packetHandler, "write MAX_ANGLE"):
        portHandler.closePort()
        return

    # Force position mode (0) so subsequent WritePosEx commands track to a target.
    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_MODE, 0)
    if not check(comm, err, packetHandler, "write MODE"):
        portHandler.closePort()
        return

    comm, err = packetHandler.LockEprom(args.servo_id)
    if not check(comm, err, packetHandler, "lock EPROM"):
        portHandler.closePort()
        return

    min_back, comm, err = packetHandler.read2ByteTxRx(args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L)
    if not check(comm, err, packetHandler, "read MIN_ANGLE"):
        portHandler.closePort()
        return
    max_back, comm, err = packetHandler.read2ByteTxRx(args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L)
    if not check(comm, err, packetHandler, "read MAX_ANGLE"):
        portHandler.closePort()
        return

    mode_back, comm, err = packetHandler.read1ByteTxRx(args.servo_id, SMS_STS_MODE)
    if not check(comm, err, packetHandler, "read MODE post"):
        portHandler.closePort()
        return

    print("Verified: MIN=%d, MAX=%d, MODE=%d" % (min_back, max_back, mode_back))
    if min_back != min_tick or max_back != max_tick:
        print("MISMATCH between written and read-back angle limits.")
        portHandler.closePort()
        return
    if mode_back != 0:
        print("MODE register did not stick at 0 (got %d). Aborting before sweep — the" % mode_back)
        print("motor would spin in wheel/step mode instead of tracking position.")
        portHandler.closePort()
        return

    # Seed GOAL with current position BEFORE enabling torque, so the motor
    # doesn't snap to a stale goal left in SRAM from a previous run.
    pos, comm, err = packetHandler.ReadPos(args.servo_id)
    if not check(comm, err, packetHandler, "read pos pre-torque"):
        portHandler.closePort()
        return
    seed = max(min_tick, min(max_tick, pos))
    packetHandler.WritePosEx(args.servo_id, seed, 50, 20)

    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
    if not check(comm, err, packetHandler, "enable torque"):
        portHandler.closePort()
        return

    near_wrap = abs(closed_tick - open_tick) > 2048 or min_tick < 100 or max_tick > 3995
    # if near_wrap:
    #     print("Skipping sweep: gripper extents are within 100 ticks of the encoder")
    #     print("wrap (raw=0 or 4095). Position-mode overshoot near MIN/MAX would")
    #     print("cross 0 and stall the motor. Rotate the servo horn so both extents")
    #     print("are well clear of raw=0/4095, then re-run calibration.")
    # else:
    print("Sweeping to MIN, then MAX for visual confirmation...")
    for label, target in (("MIN", min_tick), ("MAX", max_tick)):
        packetHandler.WritePosEx(args.servo_id, target, 50, 20)
        t_end = time.monotonic() + 3.0
        next_tick = time.monotonic()
        while time.monotonic() < t_end:
            pos, comm, err = packetHandler.ReadPos(args.servo_id)
            if comm == COMM_SUCCESS and err == 0:
                print("  -> %s target=%4d  pos=%4d" % (label, target, pos))
            next_tick += 0.1
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)

    print("")
    print("Done. OPEN_TICK=%d  CLOSED_TICK=%d" % (open_tick, closed_tick))
    portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
