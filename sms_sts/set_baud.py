#!/usr/bin/env python
#
# *********     Servo Baudrate Setter      *********
#
# One-shot tool to change an SMS/STS servo's serial baudrate. Writes the
# baud code to register SMS_STS_BAUD_RATE (addr 6) in EPROM, then verifies
# by reading at the new baud.
#
# Baud codes (sms_sts.py:9):
#   0 = 1 Mbps      4 = 115200
#   1 = 500 kbps    5 = 76800
#   2 = 250 kbps    6 = 57600
#   3 = 128 kbps    7 = 38400
#
# Requires: pip install tyro
#
# Usage:
#   python3 set_baud.py --servo-id 1 --baudrate 115200 --new-code 0
#

import os
import sys
import time
from dataclasses import dataclass

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


BAUD_CODE_TO_RATE = {
    0: 1000000,
    1: 500000,
    2: 250000,
    3: 128000,
    4: 115200,
    5: 76800,
    6: 57600,
    7: 38400,
}


@dataclass
class Args:
    servo_id: int = 1
    """Servo ID on the bus."""
    baudrate: int = 115200
    """Current baudrate the servo is operating at."""
    new_code: int = 0
    """New baud code: 0=1M, 1=500k, 2=250k, 3=128k, 4=115200, 5=76800, 6=57600, 7=38400."""
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    """Serial port path. Defaults to the stable by-id symlink for the CH340 adapter."""


def main(args: Args) -> None:
    if args.new_code not in BAUD_CODE_TO_RATE:
        print("Invalid --new-code %d. Must be 0..7." % args.new_code)
        return
    new_baud = BAUD_CODE_TO_RATE[args.new_code]

    portHandler = PortHandler(args.port)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print("Failed to open port.")
        return
    if not portHandler.setBaudRate(args.baudrate):
        print("Failed to set baud %d." % args.baudrate)
        portHandler.closePort()
        return

    pos, comm, err = packetHandler.ReadPos(args.servo_id)
    if comm != COMM_SUCCESS or err != 0:
        print("No response at %d. Wrong baud or wrong ID?" % args.baudrate)
        portHandler.closePort()
        return
    print("Servo responded at %d. Present pos = %d." % (args.baudrate, pos))

    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 0)
    # TxOnly: the servo switches baud before its status packet would arrive,
    # so a TxRx write would time out waiting for a reply at the old baud.
    packetHandler.write1ByteTxOnly(args.servo_id, SMS_STS_BAUD_RATE, args.new_code)
    time.sleep(0.05)

    portHandler.setBaudRate(new_baud)
    packetHandler.write1ByteTxRx(args.servo_id, SMS_STS_LOCK, 1)

    pos, comm, err = packetHandler.ReadPos(args.servo_id)
    if comm != COMM_SUCCESS or err != 0:
        print("Verification failed: no response at %d." % new_baud)
        portHandler.closePort()
        return
    print("Verified at %d. Present pos = %d." % (new_baud, pos))
    print("Now update --baudrate %d in calibrate_range.py and keyboard_stream.py." % new_baud)
    portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
