#!/usr/bin/env python
#
# *********     Position Controller Diagnostics      *********
#
# Reads the SMS/STS position-loop tuning registers so we can sanity-check
# why keyboard_stream.py is lagging the commanded position.
#
# Registers (Feetech SMS/STS convention; not exposed by name in the SDK):
#   21  Position P gain   (1B, default ~32)
#   22  Position D gain   (1B, default ~32)
#   23  Position I gain   (1B, default ~0)
#   24  Min startup force (2B, "Punch")
#   41  ACC default       (1B, units 8.7 deg/s^2; 0 = max accel)
#   46  GOAL_SPEED        (2B signed, units depend on model)
#
# Usage:
#   python3 read_pid.py --servo-id 2 --baudrate 115200
#

import os
import sys
from dataclasses import dataclass

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


REG_POS_KP = 21
REG_POS_KD = 22
REG_POS_KI = 23
REG_PUNCH_L = 24
REG_PUNCH_H = 25


@dataclass
class Args:
    servo_id: int = 2
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Serial baud rate."""
    port: str = "/dev/ttyUSB0"
    """Serial port path."""


def read1(packetHandler, sid, addr, name):
    val, comm, err = packetHandler.read1ByteTxRx(sid, addr)
    if comm != COMM_SUCCESS or err != 0:
        print("  %-20s [addr %2d]  read failed (comm=%d err=%d)" % (name, addr, comm, err))
        return None
    print("  %-20s [addr %2d]  %d" % (name, addr, val))
    return val


def read2(packetHandler, sid, addr, name, signed=False):
    val, comm, err = packetHandler.read2ByteTxRx(sid, addr)
    if comm != COMM_SUCCESS or err != 0:
        print("  %-20s [addr %2d]  read failed (comm=%d err=%d)" % (name, addr, comm, err))
        return None
    if signed:
        val = packetHandler.scs_tohost(val, 15)
    print("  %-20s [addr %2d]  %d" % (name, addr, val))
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

    print("Servo ID %d @ %d baud" % (args.servo_id, args.baudrate))
    print("\nPosition controller:")
    kp = read1(packetHandler, args.servo_id, REG_POS_KP, "POS_KP")
    kd = read1(packetHandler, args.servo_id, REG_POS_KD, "POS_KD")
    ki = read1(packetHandler, args.servo_id, REG_POS_KI, "POS_KI")
    read2(packetHandler, args.servo_id, REG_PUNCH_L, "Punch (min force)")

    print("\nCurrent SRAM state:")
    read1(packetHandler, args.servo_id, SMS_STS_MODE, "MODE (0=pos)")
    read1(packetHandler, args.servo_id, SMS_STS_TORQUE_ENABLE, "TORQUE_ENABLE")
    read1(packetHandler, args.servo_id, SMS_STS_ACC, "ACC")
    read2(packetHandler, args.servo_id, SMS_STS_GOAL_SPEED_L, "GOAL_SPEED", signed=True)
    read2(packetHandler, args.servo_id, SMS_STS_GOAL_POSITION_L, "GOAL_POSITION", signed=True)

    print("\nFeetech SMS/STS factory defaults are typically Kp=32 Kd=32 Ki=0.")
    print("If Kp is much lower than 32, that alone can produce big tracking lag.")
    if kp is not None and kp < 16:
        print("--> Kp=%d looks low; bumping toward 32 should reduce lag." % kp)

    portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
