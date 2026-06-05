#!/usr/bin/env python
#
# *********     Servo ID Setter      *********
#
# One-shot tool to change an SMS/STS servo's bus ID. Unlocks EPROM, writes the
# new ID to register SMS_STS_ID (addr 5), re-locks EPROM at the new ID, then
# verifies by reading position at the new ID.
#
# The servo answers to the new ID the instant the write lands, so the lock-back
# and verification must target the new ID, not the old one.
#
# Only one servo (the one being renumbered) should be on the bus unless you are
# certain of its current ID -- otherwise you risk assigning the same ID twice.
#
# Requires: pip install tyro
#
# Usage:
#   python3 set_id.py --old-id 1 --new-id 2 --baudrate 115200
#

import os
import sys
from dataclasses import dataclass

import tyro

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


@dataclass
class Args:
    old_id: int = 1
    """Current servo ID on the bus."""
    new_id: int = 2
    """New servo ID to assign (0..253; 254 is the broadcast ID and is reserved)."""
    baudrate: int = 115200
    """Baudrate the servo is operating at."""
    port: str = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    """Serial port path. Defaults to the stable by-id symlink for the CH340 adapter."""


def main(args: Args) -> None:
    if not 0 <= args.new_id <= 253:
        print("Invalid --new-id %d. Must be 0..253 (254 is broadcast)." % args.new_id)
        return
    if args.old_id == args.new_id:
        print("--old-id and --new-id are the same (%d). Nothing to do." % args.old_id)
        return

    portHandler = PortHandler(args.port)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print("Failed to open port.")
        return
    if not portHandler.setBaudRate(args.baudrate):
        print("Failed to set baud %d." % args.baudrate)
        portHandler.closePort()
        return

    pos, comm, err = packetHandler.ReadPos(args.old_id)
    if comm != COMM_SUCCESS or err != 0:
        print("No response at ID %d. Wrong ID or wrong baud?" % args.old_id)
        portHandler.closePort()
        return
    print("Servo responded at ID %d. Present pos = %d." % (args.old_id, pos))

    # Guard against clobbering an existing servo at the target ID.
    pos2, comm2, err2 = packetHandler.ReadPos(args.new_id)
    if comm2 == COMM_SUCCESS and err2 == 0:
        print("A servo already responds at ID %d. Aborting to avoid a clash." % args.new_id)
        portHandler.closePort()
        return

    packetHandler.unLockEprom(args.old_id)
    comm, err = packetHandler.write1ByteTxRx(args.old_id, SMS_STS_ID, args.new_id)
    if comm != COMM_SUCCESS:
        print("ID write failed (comm=%d). EPROM may still be locked." % comm)
        portHandler.closePort()
        return
    # From here on the servo answers to the new ID.
    packetHandler.LockEprom(args.new_id)

    pos, comm, err = packetHandler.ReadPos(args.new_id)
    if comm != COMM_SUCCESS or err != 0:
        print("Verification failed: no response at ID %d." % args.new_id)
        portHandler.closePort()
        return
    print("Verified at ID %d. Present pos = %d." % (args.new_id, pos))
    portHandler.closePort()


if __name__ == "__main__":
    main(tyro.cli(Args))
