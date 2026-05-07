#!/usr/bin/env python
#
# *********     Force-regulated Gripper Control      *********
#
# Position mode for movement, PWM mode for grip force regulation.
#
# State machine:
#   IDLE_OPEN ─'c'─▶ CLOSING ─load>thresh─▶ GRIPPING ─'r'/'o'─▶ OPENING ─▶ IDLE_OPEN
#   any state + 'q' → safe release + open + exit
#
# Movement (CLOSING / OPENING) runs in MODE 0 so the firmware's trapezoidal
# profile and angle limits apply. CLOSING transitions to GRIPPING when EITHER:
#   - |PRESENT_LOAD| > --contact-load for N reads  (sharp/hard contact)
#   - |PRESENT_SPEED| < --stall-speed for N reads  (gripper stopped — soft
#     compression complete, mechanical stop reached, or arrived at close_tick)
# Stall detection is the reliable signal for soft grippers, where load ramps
# up smoothly instead of spiking on contact and position creeps indefinitely
# under compression. GRIPPING then writes --hold-pwm duty in MODE 3 for
# constant-force hold. Releasing switches back to MODE 0 and seeds
# GOAL_POSITION with the current encoder reading so the position controller
# doesn't snap to a stale goal.
#
# Mode switching follows the calibrate_range.py pattern:
#   torque off → unLockEprom → write MODE → LockEprom → torque on
#
# Usage:
#   python3 force_grip.py --servo-id 1
#   python3 force_grip.py --servo-id 1 --hold-pwm 600 --contact-load 300
#
# Keys (after launch):
#   c — close and grip
#   o — open
#   r — release (drop grip but keep gripper where it is)
#   q — quit (release, open, exit)
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
MODE_PWM = 3

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
    approach_speed: int = 1000
    """Speed for position-mode movement."""
    approach_acc: int = 50
    """Trapezoidal acceleration for position-mode movement."""
    contact_load: int = 400
    """|PRESENT_LOAD| threshold (0..1000) that triggers transition to PWM grip."""
    hold_pwm: int = 800
    """PWM duty for grip hold (0..1000). 800 = 80% of stall torque (~64 kg·cm here)."""
    hold_acc: int = 100
    """Ramp rate to/from hold PWM (in PWM mode)."""
    max_temp: int = 60
    """Back off PWM to half if PRESENT_TEMPERATURE exceeds this (degC)."""
    rate_hz: int = 50
    """Control loop rate (Hz). Each iteration does 2-3 register reads."""
    contact_debounce: int = 3
    """Require this many consecutive |load|>contact_load reads before transitioning to GRIPPING. Filters single-packet corruption."""
    safety_debounce: int = 3
    """Require this many consecutive out-of-bounds position reads before tripping SAFETY in GRIPPING. Filters single-packet corruption."""
    contact_settle_ticks: int = 10
    """Ignore contact AND stall triggers for this many ticks after entering CLOSING (lets trapezoidal profile spin up)."""
    stall_speed: int = 10
    """|PRESENT_SPEED| below this counts as 'stopped' (units ≈ 0.732 RPM/LSB; 10 ≈ 7 RPM)."""
    stall_debounce: int = 5
    """Consecutive 'stopped' reads required to transition CLOSING→GRIPPING. Stall is the primary trigger for soft grippers."""


def ok(comm, err):
    return comm == COMM_SUCCESS and err == 0


def set_mode(pk, sid, new_mode):
    """Switch MODE register. EEPROM, so disable torque + unlock around the write."""
    pk.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)
    pk.unLockEprom(sid)
    comm, err = pk.write1ByteTxRx(sid, SMS_STS_MODE, new_mode)
    pk.LockEprom(sid)
    if not ok(comm, err):
        print("\n[set_mode %d] failed comm=%d err=%d" % (new_mode, comm, err))
        return False
    return True


def seed_pos_goal(pk, sid):
    """Read PRESENT_POSITION and write it as GOAL_POSITION before re-enabling torque,
    so position mode doesn't snap to a stale goal left in SRAM."""
    pos, comm, err = pk.ReadPos(sid)
    if not ok(comm, err):
        print("\n[seed_pos_goal] read failed")
        return None
    pk.WritePosEx(sid, pos, 50, 20)
    return pos


def enter_position_mode(pk, sid):
    if not set_mode(pk, sid, MODE_POSITION):
        return False
    if seed_pos_goal(pk, sid) is None:
        return False
    pk.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 1)
    return True


def main(args: Args) -> None:
    ph = PortHandler(args.port)
    pk = sms_sts(ph)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        print("Failed to open port / set baud.")
        return

    if args.open_tick is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L)
        if not ok(c, e):
            print("Failed to read MIN_ANGLE_LIMIT.")
            ph.closePort(); return
        args.open_tick = v
    if args.close_tick is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L)
        if not ok(c, e):
            print("Failed to read MAX_ANGLE_LIMIT.")
            ph.closePort(); return
        args.close_tick = v

    print("Servo %d  port %s  baud %d" % (args.servo_id, args.port, args.baudrate))
    print("Range: open_tick=%d  close_tick=%d" % (args.open_tick, args.close_tick))
    print("Contact: |load|>%d   Hold PWM: %d   Max temp: %d C" % (
        args.contact_load, args.hold_pwm, args.max_temp))
    print("Keys: c=close+grip  o=open  r=release  q=quit\n")

    if not enter_position_mode(pk, args.servo_id):
        ph.closePort(); return

    state = STATE_IDLE_OPEN
    period = 1.0 / args.rate_hz
    backed_off = False    # true while temp back-off is active
    tick = 0
    contact_streak = 0    # consecutive ticks with |load|>threshold while CLOSING
    stall_streak = 0      # consecutive ticks with |speed|<threshold while CLOSING
    safety_streak = 0     # consecutive ticks with out-of-bounds pos while GRIPPING
    closing_tick = 0      # tick count since entering CLOSING (for settle period)

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_t = time.monotonic()
        while True:
            tick += 1
            # ReadPosSpeed: pos+speed in one 4-byte read (same wire cost as one read)
            pos, speed, c, e = pk.ReadPosSpeed(args.servo_id)
            pos_ok = ok(c, e)
            speed_ok = pos_ok    # same packet — succeeds or fails together

            load_raw, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_PRESENT_LOAD_L)
            load_ok = ok(c, e)
            # PRESENT_LOAD: bit 10 = direction sign, bits 0..9 = magnitude (0..1023).
            load = pk.scs_tohost(load_raw, 10) if load_ok else 0

            # Temp is slow to change; poll every 10 ticks (~5 Hz at 50 Hz loop).
            temp = None
            if tick % 10 == 0:
                t, c, e = pk.read1ByteTxRx(args.servo_id, SMS_STS_PRESENT_TEMPERATURE)
                if ok(c, e):
                    temp = t

            ch = None
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1).lower()

            # Keyboard-driven transitions
            if ch == "q":
                break
            elif ch == "c" and state in (STATE_IDLE_OPEN, STATE_OPENING):
                pk.WritePosEx(args.servo_id, args.close_tick,
                              args.approach_speed, args.approach_acc)
                state = STATE_CLOSING
                closing_tick = 0
                contact_streak = 0
                stall_streak = 0
            elif ch == "o" and state in (STATE_IDLE_OPEN, STATE_CLOSING):
                pk.WritePosEx(args.servo_id, args.open_tick,
                              args.approach_speed, args.approach_acc)
                state = STATE_OPENING
            elif ch in ("o", "r") and state == STATE_GRIPPING:
                pk.WriteSpec(args.servo_id, 0, args.hold_acc)   # ramp PWM down
                time.sleep(0.05)
                if not enter_position_mode(pk, args.servo_id):
                    break
                backed_off = False
                if ch == "o":
                    pk.WritePosEx(args.servo_id, args.open_tick,
                                  args.approach_speed, args.approach_acc)
                    state = STATE_OPENING
                else:
                    state = STATE_IDLE_OPEN

            # Sensor-driven transitions
            if state == STATE_CLOSING:
                closing_tick += 1
                past_settle = closing_tick > args.contact_settle_ticks
                contact_now = (load_ok and abs(load) > args.contact_load)
                stalled_now = (speed_ok and abs(speed) < args.stall_speed)
                contact_streak = contact_streak + 1 if past_settle and contact_now else 0
                stall_streak = stall_streak + 1 if past_settle and stalled_now else 0
                # Transition on contact (sharp impact) OR stall (gripper stopped —
                # soft compression done, hard stop, or arrived at close_tick).
                # Stall is the reliable trigger for soft grippers.
                if (contact_streak >= args.contact_debounce
                        or stall_streak >= args.stall_debounce):
                    if not set_mode(pk, args.servo_id, MODE_PWM):
                        break
                    pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
                    pk.WriteSpec(args.servo_id, args.hold_pwm, args.hold_acc)
                    state = STATE_GRIPPING
                    safety_streak = 0
            elif state == STATE_OPENING and pos_ok and abs(pos - args.open_tick) < 5:
                state = STATE_IDLE_OPEN

            # Safety: PWM mode ignores angle limits, enforce them in host.
            # Debounce so a single corrupt position read doesn't trip.
            if state == STATE_GRIPPING and pos_ok:
                lo, hi = min(args.open_tick, args.close_tick) - 50, max(args.open_tick, args.close_tick) + 50
                if pos < lo or pos > hi:
                    safety_streak += 1
                else:
                    safety_streak = 0
                if safety_streak >= args.safety_debounce:
                    sys.stdout.write("\n[SAFETY] pos %d outside [%d,%d] for %d reads, killing PWM\n" % (
                        pos, lo, hi, safety_streak))
                    pk.WriteSpec(args.servo_id, 0, args.hold_acc)
                    enter_position_mode(pk, args.servo_id)
                    state = STATE_IDLE_OPEN
                    backed_off = False
                    safety_streak = 0

            # Thermal back-off in GRIPPING
            if state == STATE_GRIPPING and temp is not None:
                if temp > args.max_temp and not backed_off:
                    pk.WriteSpec(args.servo_id, args.hold_pwm // 2, args.hold_acc)
                    backed_off = True
                elif temp <= args.max_temp - 5 and backed_off:
                    pk.WriteSpec(args.servo_id, args.hold_pwm, args.hold_acc)
                    backed_off = False

            sys.stdout.write("\r%-10s pos=%4d  spd=%+5d  load=%+5d  temp=%sC  %s    " % (
                state, pos if pos_ok else -1,
                speed if speed_ok else 0,
                load if load_ok else 0,
                str(temp) if temp is not None else "??",
                "(BACKED OFF)" if backed_off else "            "))
            sys.stdout.flush()

            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()    # we fell behind; resync
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        sys.stdout.write("\n")
        sys.stdout.flush()
        # Safe shutdown
        try:
            if state == STATE_GRIPPING:
                pk.WriteSpec(args.servo_id, 0, args.hold_acc)
                time.sleep(0.05)
                enter_position_mode(pk, args.servo_id)
            pk.WritePosEx(args.servo_id, args.open_tick,
                          args.approach_speed, args.approach_acc)
            time.sleep(0.5)
            pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
        except Exception as ex:
            print("Cleanup error: %s" % ex)
        ph.closePort()
        print("Done.")


if __name__ == "__main__":
    main(tyro.cli(Args))
