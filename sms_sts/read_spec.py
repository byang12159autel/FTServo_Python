#!/usr/bin/env python
#
# *********     SMS/STS Servo Diagnostics      *********
#
# Reads identity, EEPROM limits, position-controller tuning, current SRAM
# state, and live telemetry from a Feetech SMS/STS servo.
#
# Register addresses (Feetech SMS/STS memory table; many are NOT exposed by
# name in the SDK so we read them by raw address):
#
#   EEPROM (read-only)
#     0-1  Firmware version (Major.Minor)
#     3-4  Model number              (2B)
#
#   EEPROM (read-write)
#     5    ID
#     6    Baud rate index           (0=1M, 1=0.5M, ..., 4=115200)
#     9-10 Min angle limit           (2B)
#    11-12 Max angle limit           (2B)
#    13    Max temperature limit     (degC)
#    14    Max input voltage         (0.1V)
#    15    Min input voltage         (0.1V)
#    16-17 Max torque limit          (2B, 0..1000 = 0..100% of stall torque)
#    21    Position P gain           (1B, default ~32)
#    22    Position D gain           (1B, default ~32)
#    23    Position I gain           (1B, default ~0)
#    24-25 Punch / min startup force (2B)
#    26    CW dead band              (ticks)
#    27    CCW dead band             (ticks)
#    28-29 Protection current        (2B, raw)
#    36    Overload torque           (% of max torque)
#    33    Mode (0=pos, 1=wheel, 2=open-loop wheel, 3=PWM)
#
#   SRAM
#    40    Torque enable
#    41    ACC
#    42-43 Goal position             (signed, 15-bit two's complement)
#    46-47 Goal speed                (signed)
#    55    EEPROM lock
#    56-57 Present position          (signed)
#    58-59 Present speed             (signed)
#    60-61 Present load              (signed; magnitude ~ load %)
#    62    Present voltage           (0.1V)
#    63    Present temperature       (degC)
#    66    Moving flag
#    69-70 Present current           (signed, ~6.5 mA / LSB)
#
# Usage:
#   python3 read_spec.py --servo-id 1 --baudrate 115200
#

import os
import sys
from dataclasses import dataclass

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


# EEPROM read-only
REG_FIRMWARE_L = 0
REG_MODEL_L = 3

# EEPROM read-write (identity / limits)
REG_RETURN_DELAY = 7
REG_STATUS_RETURN_LVL = 8
REG_MAX_TEMP = 13
REG_MAX_VOLT = 14
REG_MIN_VOLT = 15
REG_MAX_TORQUE_L = 16

# Position controller
REG_POS_KP = 21
REG_POS_KD = 22
REG_POS_KI = 23
REG_PUNCH_L = 24
REG_CW_DEAD = 26
REG_CCW_DEAD = 27
REG_PROTECT_CURRENT_L = 28
REG_OVERLOAD_TORQUE = 36


@dataclass
class Args:
    servo_id: int = 1
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Serial baud rate."""
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    """Serial port path. Defaults to the stable by-id symlink for the CH340 adapter."""


def read1(packetHandler, sid, addr, name, suffix=""):
    val, comm, err = packetHandler.read1ByteTxRx(sid, addr)
    if comm != COMM_SUCCESS or err != 0:
        print("  %-22s [addr %2d]  read failed (comm=%d err=%d)" % (name, addr, comm, err))
        return None
    print("  %-22s [addr %2d]  %d%s" % (name, addr, val, suffix))
    return val


def read2(packetHandler, sid, addr, name, signed=False, suffix=""):
    val, comm, err = packetHandler.read2ByteTxRx(sid, addr)
    if comm != COMM_SUCCESS or err != 0:
        print("  %-22s [addr %2d]  read failed (comm=%d err=%d)" % (name, addr, comm, err))
        return None
    if signed:
        val = packetHandler.scs_tohost(val, 15)
    print("  %-22s [addr %2d]  %d%s" % (name, addr, val, suffix))
    return val


def main(args: Args) -> None:
    portHandler = PortHandler(args.port)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print("Failed to open port.")
        return
    if not portHandler.setBaudRate(args.baudrate):
        print("Failed to set baud %d." % args.baudrate)
        portHandler.closePort()
        return

    print("Servo ID %d @ %s @ %d baud" % (args.servo_id, args.port, args.baudrate))

    print("\nIdentity (EEPROM read-only):")
    read2(packetHandler, args.servo_id, REG_FIRMWARE_L, "FIRMWARE")
    read2(packetHandler, args.servo_id, REG_MODEL_L, "MODEL")
    read1(packetHandler, args.servo_id, SMS_STS_ID, "ID")
    read1(packetHandler, args.servo_id, SMS_STS_BAUD_RATE, "BAUD_RATE (index)")

    print("\nLimits & protection (EEPROM):")
    read2(packetHandler, args.servo_id, SMS_STS_MIN_ANGLE_LIMIT_L, "MIN_ANGLE", signed=True, suffix=" ticks")
    read2(packetHandler, args.servo_id, SMS_STS_MAX_ANGLE_LIMIT_L, "MAX_ANGLE", signed=True, suffix=" ticks")
    read1(packetHandler, args.servo_id, REG_MAX_TEMP, "MAX_TEMPERATURE", suffix=" C")
    read1(packetHandler, args.servo_id, REG_MAX_VOLT, "MAX_VOLTAGE", suffix=" (0.1 V)")
    read1(packetHandler, args.servo_id, REG_MIN_VOLT, "MIN_VOLTAGE", suffix=" (0.1 V)")
    tlim = read2(packetHandler, args.servo_id, REG_MAX_TORQUE_L, "MAX_TORQUE_LIMIT", suffix=" /1000")
    read2(packetHandler, args.servo_id, REG_PROTECT_CURRENT_L, "PROTECT_CURRENT")
    read1(packetHandler, args.servo_id, REG_OVERLOAD_TORQUE, "OVERLOAD_TORQUE", suffix=" %")

    print("\nPosition controller (EEPROM):")
    kp = read1(packetHandler, args.servo_id, REG_POS_KP, "POS_KP")
    kd = read1(packetHandler, args.servo_id, REG_POS_KD, "POS_KD")
    ki = read1(packetHandler, args.servo_id, REG_POS_KI, "POS_KI")
    read2(packetHandler, args.servo_id, REG_PUNCH_L, "PUNCH (min force)")
    read1(packetHandler, args.servo_id, REG_CW_DEAD, "CW_DEAD", suffix=" ticks")
    read1(packetHandler, args.servo_id, REG_CCW_DEAD, "CCW_DEAD", suffix=" ticks")

    print("\nCurrent SRAM state:")
    read1(packetHandler, args.servo_id, SMS_STS_MODE, "MODE (0=pos)")
    read1(packetHandler, args.servo_id, SMS_STS_TORQUE_ENABLE, "TORQUE_ENABLE")
    read1(packetHandler, args.servo_id, SMS_STS_ACC, "ACC")
    read2(packetHandler, args.servo_id, SMS_STS_GOAL_SPEED_L, "GOAL_SPEED", signed=True)
    read2(packetHandler, args.servo_id, SMS_STS_GOAL_POSITION_L, "GOAL_POSITION", signed=True)
    read1(packetHandler, args.servo_id, SMS_STS_LOCK, "EEPROM_LOCK")

    print("\nLive telemetry (SRAM read-only):")
    read2(packetHandler, args.servo_id, SMS_STS_PRESENT_POSITION_L, "PRESENT_POSITION", signed=True)
    read2(packetHandler, args.servo_id, SMS_STS_PRESENT_SPEED_L, "PRESENT_SPEED", signed=True)
    read2(packetHandler, args.servo_id, SMS_STS_PRESENT_LOAD_L, "PRESENT_LOAD", signed=True)
    read1(packetHandler, args.servo_id, SMS_STS_PRESENT_VOLTAGE, "PRESENT_VOLTAGE", suffix=" (0.1 V)")
    read1(packetHandler, args.servo_id, SMS_STS_PRESENT_TEMPERATURE, "PRESENT_TEMP", suffix=" C")
    read1(packetHandler, args.servo_id, SMS_STS_MOVING, "MOVING")
    read2(packetHandler, args.servo_id, SMS_STS_PRESENT_CURRENT_L, "PRESENT_CURRENT", signed=True, suffix=" (~6.5 mA/LSB)")

    print("\nFeetech SMS/STS factory defaults are typically Kp=32 Kd=32 Ki=0,")
    print("MAX_TORQUE_LIMIT=1000 (=100%).")
    if kp is not None and kp < 16:
        print("--> Kp=%d looks low; bumping toward 32 should reduce lag." % kp)
    if tlim is not None and tlim < 1000:
        print("--> MAX_TORQUE_LIMIT=%d (<1000); peak torque is capped to %.0f%% of stall." % (tlim, tlim / 10))

    portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
