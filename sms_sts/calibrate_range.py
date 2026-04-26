#!/usr/bin/env python
#
# *********     Gripper Range Calibration      *********
#
# Hand-jog calibration for SMS/STS servos used as a robot gripper.
# Disables torque so the user can move the gripper by hand to fully-OPEN
# and fully-CLOSED positions, then writes those ticks as min/max angle
# limits to the servo's EPROM so the firmware refuses to drive past them.
#
# Requires: pip install tyro
#
# Usage:
#   python3 calibrate_range.py --servo-id 2 --baudrate 115200
#

import sys
import time
from dataclasses import dataclass

import tyro

sys.path.append("..")
from scservo_sdk import *                      # Uses FTServo SDK library


@dataclass
class Args:
    servo_id: int = 2
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Serial baud rate."""
    port: str = "/dev/ttyUSB0"
    """Serial port path. Windows: 'COM1', Linux: '/dev/ttyUSB0', Mac: '/dev/tty.usbserial-*'."""
    min_range: int = 50
    """Reject calibration if |CLOSED_TICK - OPEN_TICK| is below this many ticks."""


def check(comm_result, error, packetHandler, where):
    if comm_result != COMM_SUCCESS:
        print("[%s] %s" % (where, packetHandler.getTxRxResult(comm_result)))
        return False
    if error != 0:
        print("[%s] %s" % (where, packetHandler.getRxPacketError(error)))
        return False
    return True


def capture(packetHandler, scs_id, label):
    input("Move the gripper to the fully-%s position by hand, then press Enter..." % label)
    pos, comm, err = packetHandler.ReadPos(scs_id)
    if not check(comm, err, packetHandler, "read %s" % label):
        return None
    print("  recorded %s_TICK = %d" % (label, pos))
    return pos


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

    # Disable torque so the user can hand-jog the gripper.
    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 0)
    if not check(comm, err, packetHandler, "disable torque"):
        portHandler.closePort()
        return

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

    if min_tick < 50 or max_tick > 4045:
        print("WARNING: a recorded tick is near 0 or 4095. The gripper may have")
        print("crossed the single-turn wraparound boundary; calibration may be wrong.")

    print("")
    print("Will write to EPROM:")
    print("  MIN_ANGLE_LIMIT (reg 9)  = %d" % min_tick)
    print("  MAX_ANGLE_LIMIT (reg 11) = %d" % max_tick)
    print("  (OPEN_TICK=%d, CLOSED_TICK=%d)" % (open_tick, closed_tick))
    if input("Confirm? [y/N] ").strip().lower() != "y":
        print("Aborted.")
        portHandler.closePort()
        return

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

    print("Verified: MIN=%d, MAX=%d" % (min_back, max_back))
    if min_back != min_tick or max_back != max_tick:
        print("MISMATCH between written and read-back values.")
        portHandler.closePort()
        return

    comm, err = packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_TORQUE_ENABLE, 1)
    if not check(comm, err, packetHandler, "enable torque"):
        portHandler.closePort()
        return

    print("Sweeping to MIN, then MAX for visual confirmation...")
    packetHandler.WritePosEx(args.servo_id, min_tick, 200, 50)
    time.sleep(2.0)
    packetHandler.WritePosEx(args.servo_id, max_tick, 200, 50)
    time.sleep(2.0)

    print("")
    print("Done. OPEN_TICK=%d  CLOSED_TICK=%d" % (open_tick, closed_tick))
    portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
