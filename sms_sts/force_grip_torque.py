#!/usr/bin/env python
#
# *********     Torque-capped Gripper Control (position mode only)     *********
#
# A simpler alternative to force_grip.py. Instead of switching into PWM mode for
# the grip hold, this stays in MODE 0 (position) the whole time and regulates
# grip force with the live TORQUE_LIMIT register (addr 48, SRAM — no EEPROM
# unlock, written as cheaply as GOAL_POSITION).
#
# To grip: cap TORQUE_LIMIT to the desired force, then command GOAL_POSITION
# PAST the object (close_tick). The position controller pushes until it reaches
# the torque ceiling and stalls there — a constant-force hold with the firmware's
# own current loop doing the work. To release: raise TORQUE_LIMIT back to full
# and command open_tick.
#
# Why prefer this over PWM mode (force_grip.py):
#   - No mode switching → no torque-off/unlock/lock/torque-on dance per grip.
#   - Angle limits stay in force (PWM mode ignores them), so no host-side
#     position-bounds safety loop is needed.
#   - Firmware PROTECTION_CURRENT (addr 28) still guards against overcurrent.
# Tradeoff: this is a force *cap* in a position loop, not a pure open-loop force
# source. For clamping an object that is exactly what you want; for very soft
# compliant holds at a known duty, PWM mode (force_grip.py) is the better fit.
#
# Verified on an STS3215 (model 10504): TORQUE_LIMIT is live-writable in RAM and
# MODE 0 honors it — at cap 120 the gripper stalled before the goal drawing
# |current|≈6; at cap 990 it reached the goal drawing |current|≈487.
#
# Usage:
#   python3 force_grip_torque.py --servo-id 21 --baudrate 1000000 \
#       --open-tick 2367 --close-tick 3023 --grip-force 300
#
# Keys (after launch):
#   c — close and grip at --grip-force
#   o — open (full torque)
#   + / - — raise / lower grip force live (±50, only meaningful while gripping)
#   q — quit (open, torque off, exit)
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


MODE_POSITION = 0

STATE_IDLE_OPEN = "IDLE_OPEN"
STATE_CLOSING = "CLOSING"
STATE_GRIPPING = "GRIPPING"
STATE_OPENING = "OPENING"


@dataclass
class Args:
    servo_id: int = 1
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Serial baud rate."""
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    """Serial port path."""
    open_tick: int | None = None
    """Open position (ticks). Defaults to MIN_ANGLE_LIMIT read from servo."""
    close_tick: int | None = None
    """Closed position (ticks). Defaults to MAX_ANGLE_LIMIT read from servo."""
    speed: int = 1000
    """Speed for position-mode movement."""
    acc: int = 50
    """Trapezoidal acceleration for position-mode movement."""
    grip_force: int = 300
    """TORQUE_LIMIT (0..1000) applied while gripping. Lower = gentler clamp."""
    move_force: int | None = None
    """TORQUE_LIMIT (0..1000) while opening/moving. Defaults to MAX_TORQUE_LIMIT from servo."""
    max_temp: int = 60
    """Halve grip force if PRESENT_TEMPERATURE exceeds this (degC)."""
    rate_hz: int = 50
    """Telemetry/control loop rate (Hz)."""


def ok(comm, err):
    return comm == COMM_SUCCESS and err == 0


def main(args: Args) -> None:
    ph = PortHandler(args.port)
    pk = sms_sts(ph)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        print("Failed to open port / set baud.")
        return

    # Confirm the servo is in position mode; this script never leaves it.
    mode, c, e = pk.read1ByteTxRx(args.servo_id, SMS_STS_MODE)
    if not ok(c, e):
        print("No response from servo %d." % args.servo_id)
        ph.closePort(); return
    if mode != MODE_POSITION:
        print("Servo is in MODE %d, not position (0). Set it with read_spec/calibrate first." % mode)
        ph.closePort(); return

    if args.open_tick is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L)
        if not ok(c, e):
            print("Failed to read MIN_ANGLE_LIMIT."); ph.closePort(); return
        args.open_tick = v
    if args.close_tick is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L)
        if not ok(c, e):
            print("Failed to read MAX_ANGLE_LIMIT."); ph.closePort(); return
        args.close_tick = v
    if args.move_force is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MAX_TORQUE_LIMIT_L)
        args.move_force = v if ok(c, e) else 1000

    grip_force = max(0, min(1000, args.grip_force))

    print("Servo %d  port %s  baud %d" % (args.servo_id, args.port, args.baudrate))
    print("Range: open_tick=%d  close_tick=%d" % (args.open_tick, args.close_tick))
    print("Grip force=%d   Move force=%d   Max temp=%d C" % (grip_force, args.move_force, args.max_temp))
    print("Keys: c=close+grip  o=open  +/-=adjust force  q=quit\n")

    # Start open at full torque.
    pk.WriteTorqueLimit(args.servo_id, args.move_force)
    pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
    pk.WritePosEx(args.servo_id, args.open_tick, args.speed, args.acc)

    state = STATE_OPENING
    period = 1.0 / args.rate_hz
    backed_off = False
    tick = 0

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_t = time.monotonic()
        while True:
            tick += 1
            pos, speed, c, e = pk.ReadPosSpeed(args.servo_id)
            pos_ok = ok(c, e)
            load, lc, le = pk.ReadLoad(args.servo_id)
            load_ok = ok(lc, le)
            cur, cc, ce = pk.ReadCurrent(args.servo_id)
            cur_ok = ok(cc, ce)

            temp = None
            if tick % 10 == 0:
                t, tc, te = pk.read1ByteTxRx(args.servo_id, SMS_STS_PRESENT_TEMPERATURE)
                if ok(tc, te):
                    temp = t

            ch = None
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1).lower()

            if ch == "q":
                break
            elif ch == "c" and state in (STATE_IDLE_OPEN, STATE_OPENING):
                pk.WriteTorqueLimit(args.servo_id, grip_force)
                pk.WritePosEx(args.servo_id, args.close_tick, args.speed, args.acc)
                state = STATE_CLOSING
            elif ch == "o" and state in (STATE_CLOSING, STATE_GRIPPING):
                pk.WriteTorqueLimit(args.servo_id, args.move_force)
                pk.WritePosEx(args.servo_id, args.open_tick, args.speed, args.acc)
                state = STATE_OPENING
                backed_off = False
            elif ch in ("+", "=", "-", "_"):
                grip_force = max(0, min(1000, grip_force + (50 if ch in ("+", "=") else -50)))
                if state in (STATE_CLOSING, STATE_GRIPPING) and not backed_off:
                    pk.WriteTorqueLimit(args.servo_id, grip_force)

            # CLOSING → GRIPPING once motion stops (stalled at torque cap or arrived).
            if state == STATE_CLOSING and pos_ok and abs(speed) < 10 and tick > 5:
                state = STATE_GRIPPING
            elif state == STATE_OPENING and pos_ok and abs(pos - args.open_tick) < 8:
                state = STATE_IDLE_OPEN

            # Thermal back-off while holding.
            if state == STATE_GRIPPING and temp is not None:
                if temp > args.max_temp and not backed_off:
                    pk.WriteTorqueLimit(args.servo_id, grip_force // 2)
                    backed_off = True
                elif temp <= args.max_temp - 5 and backed_off:
                    pk.WriteTorqueLimit(args.servo_id, grip_force)
                    backed_off = False

            sys.stdout.write(
                "\r%-10s pos=%4d spd=%+5d load=%+5d cur=%+5d force=%4d temp=%sC %s   " % (
                    state, pos if pos_ok else -1, speed if pos_ok else 0,
                    load if load_ok else 0, cur if cur_ok else 0, grip_force,
                    str(temp) if temp is not None else "??",
                    "(BACKED OFF)" if backed_off else "            "))
            sys.stdout.flush()

            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        sys.stdout.write("\n"); sys.stdout.flush()
        try:
            pk.WriteTorqueLimit(args.servo_id, args.move_force)
            pk.WritePosEx(args.servo_id, args.open_tick, args.speed, args.acc)
            time.sleep(0.5)
            pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
        except Exception as ex:
            print("Cleanup error: %s" % ex)
        ph.closePort()
        print("Done.")


if __name__ == "__main__":
    main(tyro.cli(Args))
