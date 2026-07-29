#!/usr/bin/env python
#
# *********     Ping / Sweep Example      *********
#
# Default: ping ID 1 at 115200.
# With --sweep: scan a range of IDs across common baudrates to find the servo.
#
#   python3 ping.py                       # ping ID 1 @ 115200
#   python3 ping.py --id 5 --baud 500000  # ping a specific ID/baud
#   python3 ping.py --sweep               # scan IDs 0..253 across common bauds
#   python3 ping.py --sweep --id-max 20   # scan IDs 0..20 only (faster)
#

import argparse
import os
import sys

import serial

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from scservo_sdk import *                      # Uses FTServo SDK library


COMMON_BAUDRATES = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]

# Consecutive port-level errors tolerated before declaring the adapter dead and
# abandoning the sweep. A live bus produces zero of these; an unplugged adapter
# produces one per ping, so a small bound keeps us from printing 254 identical
# failures.
MAX_CONSECUTIVE_PORT_ERRORS = 10


def try_ping(packetHandler, scs_id):
    """Ping one ID.

    Returns (model, comm). model is the model number on a clean reply, else None.
    comm is the raw SDK result so the caller can tell "this ID is simply not on the
    bus" (COMM_RX_TIMEOUT) apart from "the transmit itself failed" (COMM_TX_FAIL),
    which means the adapter is gone rather than the ID being absent.
    """
    model, comm, err = packetHandler.ping(scs_id)
    if comm == COMM_SUCCESS and err == 0:
        return model, comm
    return None, comm


def sweep(port, id_min, id_max, baudrates):
    portHandler = PortHandler(port)
    if not portHandler.openPort():
        print("Failed to open the port %s" % port)
        if portHandler.last_error:
            print("  reason: %s" % portHandler.last_error)
        return
    packetHandler = sms_sts(portHandler)

    found = []
    aborted = False
    try:
        for baud in baudrates:
            if not portHandler.setBaudRate(baud):
                print("  (could not set baudrate %d, skipping)" % baud)
                continue
            print("Scanning IDs %d..%d @ %d baud..." % (id_min, id_max, baud))

            port_errors = 0
            for scs_id in range(id_min, id_max + 1):
                try:
                    model, comm = try_ping(packetHandler, scs_id)
                except (serial.SerialException, OSError) as e:
                    # The port handler absorbs transient framing/break conditions,
                    # so anything escaping to here is unexpected. Keep the ID/baud
                    # context and whatever was already found.
                    comm = COMM_TX_FAIL
                    model = None
                    print("  port error at ID=%d baud=%d: %s" % (scs_id, baud, e))

                # A failed transmit is the port's fault, not the ID's. Absent IDs
                # come back as COMM_RX_TIMEOUT, which is the normal scan outcome.
                if comm == COMM_TX_FAIL:
                    port_errors += 1
                    if port_errors >= MAX_CONSECUTIVE_PORT_ERRORS:
                        print("  giving up on %s after %d consecutive transmit "
                              "failures (adapter unplugged?)" % (port, port_errors))
                        if portHandler.last_error:
                            print("  last port error: %s" % portHandler.last_error)
                        aborted = True
                        break
                    continue

                port_errors = 0
                if model is not None:
                    print("  FOUND: ID=%d  baud=%d  model=%d" % (scs_id, baud, model))
                    found.append((scs_id, baud, model))

            if aborted:
                break
    except KeyboardInterrupt:
        print("\nInterrupted; reporting what was found so far.")
    finally:
        portHandler.closePort()

    print("")
    status = "Sweep aborted early" if aborted else "Sweep complete"
    if found:
        print("%s. %d servo(s) found:" % (status, len(found)))
        for scs_id, baud, model in found:
            print("  ID=%d  baud=%d  model=%d" % (scs_id, baud, model))
    else:
        print("%s. No servos responded." % status)
        print("Check: power, data wiring (TX/RX), USB adapter, and port name.")


def single_ping(port, scs_id, baud):
    portHandler = PortHandler(port)
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        if portHandler.last_error:
            print("  reason: %s" % portHandler.last_error)
        return

    packetHandler = sms_sts(portHandler)

    if portHandler.setBaudRate(baud):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        if portHandler.last_error:
            print("  reason: %s" % portHandler.last_error)
        portHandler.closePort()
        return

    try:
        model, comm, err = packetHandler.ping(scs_id)
    except (serial.SerialException, OSError) as e:
        print("Port error while pinging ID %d: %s" % (scs_id, e))
        portHandler.closePort()
        return

    if comm != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(comm))
    else:
        print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (scs_id, model))
    if err != 0:
        print("%s" % packetHandler.getRxPacketError(err))

    portHandler.closePort()


def main():
    p = argparse.ArgumentParser(description="Ping or sweep FTServo bus")
    p.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
                   help="serial port (defaults to the stable by-id symlink for the CH340 adapter)")
    p.add_argument("--id", type=int, default=1, help="servo ID for single ping")
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
