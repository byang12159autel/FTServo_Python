#!/usr/bin/env python
#
# *********     Ping / Sweep Example      *********
#
# Default: ping ID 2 at 115200.
# With --sweep: scan a range of IDs across common baudrates to find the servo.
#
#   python3 ping.py                       # ping ID 2 @ 115200
#   python3 ping.py --id 5 --baud 500000  # ping a specific ID/baud
#   python3 ping.py --sweep               # scan IDs 0..253 across common bauds
#   python3 ping.py --sweep --id-max 20   # scan IDs 0..20 only (faster)
#

import argparse
import sys

sys.path.append("..")
from scservo_sdk import *                      # Uses FTServo SDK library


COMMON_BAUDRATES = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]


def try_ping(packetHandler, scs_id):
    model, comm, err = packetHandler.ping(scs_id)
    if comm == COMM_SUCCESS and err == 0:
        return model
    return None


def sweep(port, id_min, id_max, baudrates):
    portHandler = PortHandler(port)
    if not portHandler.openPort():
        print("Failed to open the port %s" % port)
        return
    packetHandler = sms_sts(portHandler)

    found = []
    try:
        for baud in baudrates:
            if not portHandler.setBaudRate(baud):
                print("  (could not set baudrate %d, skipping)" % baud)
                continue
            print("Scanning IDs %d..%d @ %d baud..." % (id_min, id_max, baud))
            for scs_id in range(id_min, id_max + 1):
                model = try_ping(packetHandler, scs_id)
                if model is not None:
                    print("  FOUND: ID=%d  baud=%d  model=%d" % (scs_id, baud, model))
                    found.append((scs_id, baud, model))
    finally:
        portHandler.closePort()

    print("")
    if found:
        print("Sweep complete. %d servo(s) found:" % len(found))
        for scs_id, baud, model in found:
            print("  ID=%d  baud=%d  model=%d" % (scs_id, baud, model))
    else:
        print("Sweep complete. No servos responded.")
        print("Check: power, data wiring (TX/RX), USB adapter, and port name.")


def single_ping(port, scs_id, baud):
    portHandler = PortHandler(port)
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return

    packetHandler = sms_sts(portHandler)

    if portHandler.setBaudRate(baud):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        portHandler.closePort()
        return

    model, comm, err = packetHandler.ping(scs_id)
    if comm != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(comm))
    else:
        print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (scs_id, model))
    if err != 0:
        print("%s" % packetHandler.getRxPacketError(err))

    portHandler.closePort()


def main():
    p = argparse.ArgumentParser(description="Ping or sweep FTServo bus")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--id", type=int, default=2, help="servo ID for single ping")
    p.add_argument("--baud", type=int, default=115200, help="baudrate for single ping")
    p.add_argument("--sweep", action="store_true",
                   help="scan IDs across common baudrates")
    p.add_argument("--id-min", type=int, default=0)
    p.add_argument("--id-max", type=int, default=253)
    p.add_argument("--bauds", type=str, default=None,
                   help="comma-separated baudrates for sweep "
                        "(default: %s)" % ",".join(str(b) for b in COMMON_BAUDRATES))
    args = p.parse_args()

    if args.sweep:
        bauds = ([int(b) for b in args.bauds.split(",")]
                 if args.bauds else COMMON_BAUDRATES)
        sweep(args.port, args.id_min, args.id_max, bauds)
    else:
        single_ping(args.port, args.id, args.baud)


if __name__ == "__main__":
    main()
