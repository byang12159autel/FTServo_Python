#!/usr/bin/env python
#
# *********     One-shot close→hold→release test      *********
#
# Non-interactive sequence to verify the close→hold transition without the
# full force_grip.py state machine. Runs:
#
#   1. open gripper (position mode)
#   2. close gripper (position mode), monitor for contact OR stall
#   3. switch to PWM mode at --hold-pwm, hold for --hold-seconds
#   4. release: PWM=0, back to position mode, command open
#   5. exit cleanly with torque off
#
# The close phase uses the same trigger logic as force_grip.py:
#   - contact: |PRESENT_LOAD| > --contact-load for N consecutive reads
#   - stall:   |PRESENT_SPEED| < --stall-speed  for N consecutive reads
# Whichever fires first → switch to PWM hold.
#
# Usage:
#   python3 test_close_hold.py --servo-id 1 --baudrate 1000000
#   python3 test_close_hold.py --hold-pwm 400 --hold-seconds 5
#

import os
import sys
import time
from dataclasses import dataclass, field

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


MODE_POSITION = 0
MODE_PWM = 3


@dataclass
class Args:
    servo_id: int = 1
    """Servo ID on the bus."""
    baudrate: int = 1000000
    """Serial baud rate. Default assumes you've run set_baud.py."""
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    """Serial port path."""
    open_tick: int | None = None
    """Open position. Defaults to MIN_ANGLE_LIMIT read from servo."""
    close_tick: int | None = None
    """Closed position. Defaults to MAX_ANGLE_LIMIT read from servo."""
    approach_speed: int = 1000
    """Speed for position-mode movement."""
    approach_acc: int = 50
    """Trapezoidal acceleration for position-mode movement."""
    contact_load: int = 400
    """|load| threshold for sharp-contact trigger (0..1000)."""
    stall_speed: int = 10
    """|speed| threshold for stall trigger (≈ 0.732 RPM/LSB)."""
    contact_debounce: int = 3
    """Consecutive contact reads required to trigger PWM hold."""
    stall_debounce: int = 5
    """Consecutive stall reads required to trigger PWM hold."""
    contact_settle_ticks: int = 10
    """Skip contact/stall checks for first N ticks after issuing close command."""
    hold_pwm: int = 400
    """PWM duty for hold phase (0..1000). Ignored if --hold-pwm-sweep is set."""
    hold_pwm_sweep: list[int] = field(default_factory=list)
    """If non-empty, sweep over these PWM duties (e.g. --hold-pwm-sweep 200 400 600 800). Each duty is held for --hold-seconds and a per-duty average is logged."""
    hold_seconds: float = 3.0
    """How long to hold each duty before moving to the next (or releasing)."""
    rate_hz: int = 50
    """Polling rate during close + hold phases."""
    close_timeout_s: float = 5.0
    """Bail out of close phase if neither contact nor stall fires by this time."""


def ok(c, e):
    return c == COMM_SUCCESS and e == 0


def set_mode(pk, sid, new_mode):
    pk.write1ByteTxRx(sid, SMS_STS_TORQUE_ENABLE, 0)
    pk.unLockEprom(sid)
    c, e = pk.write1ByteTxRx(sid, SMS_STS_MODE, new_mode)
    pk.LockEprom(sid)
    return ok(c, e)


def main(args: Args) -> None:
    ph = PortHandler(args.port)
    pk = sms_sts(ph)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        print("Failed to open port / set baud."); return

    if args.open_tick is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L)
        if not ok(c, e): print("read MIN_ANGLE failed"); ph.closePort(); return
        args.open_tick = v
    if args.close_tick is None:
        v, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L)
        if not ok(c, e): print("read MAX_ANGLE failed"); ph.closePort(); return
        args.close_tick = v

    print("Servo %d  open=%d  close=%d  hold_pwm=%d  hold=%.1fs" % (
        args.servo_id, args.open_tick, args.close_tick, args.hold_pwm, args.hold_seconds))

    period = 1.0 / args.rate_hz

    try:
        # --- Phase 1: ensure position mode, open the gripper ---
        if not set_mode(pk, args.servo_id, MODE_POSITION):
            print("set_mode(POSITION) failed"); return
        pos, c, e = pk.ReadPos(args.servo_id)
        if not ok(c, e): print("ReadPos failed"); return
        pk.WritePosEx(args.servo_id, pos, 50, 20)        # seed goal = current pos
        pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
        print("\n[1] Opening to %d..." % args.open_tick)
        pk.WritePosEx(args.servo_id, args.open_tick, args.approach_speed, args.approach_acc)
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            pos, c, e = pk.ReadPos(args.servo_id)
            if ok(c, e) and abs(pos - args.open_tick) < 10:
                break
            time.sleep(period)
        print("    pos=%d (target=%d)" % (pos, args.open_tick))

        # --- Phase 2: close, watch for contact OR stall ---
        print("\n[2] Closing to %d, watching contact (load>%d) & stall (|spd|<%d)..." % (
            args.close_tick, args.contact_load, args.stall_speed))
        pk.WritePosEx(args.servo_id, args.close_tick, args.approach_speed, args.approach_acc)
        contact_streak = stall_streak = 0
        closing_tick = 0
        t_start = time.monotonic()
        trigger = None
        while time.monotonic() - t_start < args.close_timeout_s:
            closing_tick += 1
            pos, speed, c, e = pk.ReadPosSpeed(args.servo_id)
            ps_ok = ok(c, e)
            load_raw, c, e = pk.read2ByteTxRx(args.servo_id, SMS_STS_PRESENT_LOAD_L)
            load_ok = ok(c, e)
            load = pk.scs_tohost(load_raw, 10) if load_ok else 0

            past_settle = closing_tick > args.contact_settle_ticks
            if past_settle and load_ok and abs(load) > args.contact_load:
                contact_streak += 1
            else:
                contact_streak = 0
            if past_settle and ps_ok and abs(speed) < args.stall_speed:
                stall_streak += 1
            else:
                stall_streak = 0

            print("    t=%.2fs pos=%4d spd=%+5d load=%+5d  c=%d s=%d" % (
                time.monotonic() - t_start, pos if ps_ok else -1,
                speed if ps_ok else 0, load if load_ok else 0,
                contact_streak, stall_streak))

            if contact_streak >= args.contact_debounce:
                trigger = "contact"; break
            if stall_streak >= args.stall_debounce:
                trigger = "stall"; break
            time.sleep(period)
        if trigger is None:
            print("    no trigger within %.1fs — bailing out." % args.close_timeout_s)
            return
        print("    → triggered by %s at pos=%d" % (trigger, pos))

        # --- Phase 3: switch to PWM mode, hold (single or sweep) ---
        sweep = args.hold_pwm_sweep if args.hold_pwm_sweep else [args.hold_pwm]
        print("\n[3] Switching to PWM mode, sweep=%s, %.1fs each..." % (sweep, args.hold_seconds))
        if not set_mode(pk, args.servo_id, MODE_PWM):
            print("set_mode(PWM) failed"); return
        pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)

        results = []     # (duty, end_pos, avg_load, avg_curr_ma, avg_temp)
        for duty in sweep:
            print("\n    duty=%d" % duty)
            pk.WriteSpec(args.servo_id, duty, 100)
            samples = []
            t_end = time.monotonic() + args.hold_seconds
            while time.monotonic() < t_end:
                pos, c, e = pk.ReadPos(args.servo_id)
                load_raw, _, _ = pk.read2ByteTxRx(args.servo_id, SMS_STS_PRESENT_LOAD_L)
                curr_raw, _, _ = pk.read2ByteTxRx(args.servo_id, SMS_STS_PRESENT_CURRENT_L)
                temp, _, _ = pk.read1ByteTxRx(args.servo_id, SMS_STS_PRESENT_TEMPERATURE)
                load = pk.scs_tohost(load_raw, 10)
                current = pk.scs_tohost(curr_raw, 15)
                current_ma = current * 6.5
                pos = pos if ok(c, e) else -1
                samples.append((pos, load, current_ma, temp))
                print("      t=%.2fs pos=%4d load=%+5d curr=%+5d (%5.0f mA) temp=%dC" % (
                    t_end - time.monotonic(), pos, load, current, current_ma, temp))
                time.sleep(0.25)
            # Average over the last half (let it stabilize)
            half = samples[len(samples) // 2:]
            avg_load = sum(s[1] for s in half) / len(half)
            avg_curr = sum(s[2] for s in half) / len(half)
            avg_temp = sum(s[3] for s in half) / len(half)
            end_pos = samples[-1][0]
            results.append((duty, end_pos, avg_load, avg_curr, avg_temp))

        print("\n    sweep summary (averaged over last half of each hold):")
        print("    %6s %8s %8s %12s %8s" % ("duty", "end_pos", "avg_load", "avg_curr_mA", "avg_temp"))
        for duty, end_pos, avg_load, avg_curr, avg_temp in results:
            print("    %6d %8d %+8.0f %+12.0f %8.1f" % (duty, end_pos, avg_load, avg_curr, avg_temp))

        # --- Phase 4: release, return to open ---
        print("\n[4] Releasing PWM, back to position mode, opening...")
        pk.WriteSpec(args.servo_id, 0, 100)
        time.sleep(0.05)
        if not set_mode(pk, args.servo_id, MODE_POSITION):
            print("set_mode(POSITION) failed"); return
        pos, c, e = pk.ReadPos(args.servo_id)
        if ok(c, e):
            pk.WritePosEx(args.servo_id, pos, 50, 20)    # seed goal = current pos
        pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
        pk.WritePosEx(args.servo_id, args.open_tick, args.approach_speed, args.approach_acc)
        time.sleep(2.0)
        pos, c, e = pk.ReadPos(args.servo_id)
        print("    final pos=%d (target=%d)" % (pos if ok(c, e) else -1, args.open_tick))

    finally:
        pk.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
        ph.closePort()
        print("\nDone.")


if __name__ == "__main__":
    main(tyro.cli(Args))
