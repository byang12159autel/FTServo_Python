#!/usr/bin/env python

import time
import serial
import sys
import platform

DEFAULT_BAUDRATE = 1000000
LATENCY_TIMER = 50 

class PortHandler(object):
    def __init__(self, port_name):
        self.is_open = False
        self.baudrate = DEFAULT_BAUDRATE
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

        self.is_using = False
        self.port_name = port_name
        self.ser = None

        # Reason for the most recent port-level failure, for callers that want to
        # report *why* openPort() returned False or a read came back empty.
        self.last_error = ""

    def openPort(self):
        return self.setBaudRate(self.baudrate)

    def closePort(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError) as e:
                self.last_error = str(e)
        self.is_open = False

    def clearPort(self):
        if self.ser is None:
            return
        try:
            self.ser.flush()
        except (serial.SerialException, OSError) as e:
            self.last_error = str(e)

    def setPortName(self, port_name):
        self.port_name = port_name

    def getPortName(self):
        return self.port_name

    def setBaudRate(self, baudrate):
        baud = self.getCFlagBaud(baudrate)

        if baud <= 0:
            # self.setupPort(38400)
            # self.baudrate = baudrate
            # Record the reason so callers don't report a stale last_error.
            self.last_error = "unsupported baudrate %s" % baudrate
            return False  # TODO: setCustomBaudrate(baudrate)
        else:
            self.baudrate = baudrate
            return self.setupPort(baud)

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        if self.ser is None:
            return 0
        try:
            return self.ser.in_waiting
        except (serial.SerialException, OSError) as e:
            self.last_error = str(e)
            return 0

    def readPort(self, length):
        # A CH340/CH341 adapter can momentarily report the line as readable while
        # returning zero bytes -- typically a framing/break condition seen when
        # listening at a baud rate the bus is not actually running at, which is
        # routine during a baudrate sweep. pyserial escalates that to a
        # SerialException ("device reports readiness to read but returned no
        # data"), which would otherwise abort a long scan mid-run.
        #
        # Degrade it to "no bytes available" instead: rxPacket() already treats an
        # empty read as a pending packet and gives up via isPacketTimeout(), so the
        # caller gets a clean COMM_RX_TIMEOUT. A genuinely unplugged adapter still
        # surfaces as COMM_TX_FAIL through writePort() on the next transmit.
        if self.ser is None:
            return b"" if sys.version_info > (3, 0) else []

        try:
            data = self.ser.read(length)
        except (serial.SerialException, OSError) as e:
            self.last_error = str(e)
            data = b""

        if sys.version_info > (3, 0):
            return data
        else:
            return [ord(ch) for ch in data]

    def writePort(self, packet):
        # Report a short write rather than raising, so txPacket() converts it into
        # COMM_TX_FAIL through its existing written-length check.
        if self.ser is None:
            return 0

        try:
            return self.ser.write(packet)
        except (serial.SerialException, OSError) as e:
            self.last_error = str(e)
            return 0

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + LATENCY_TIMER

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()

        # openPort()/setBaudRate() are documented to return a bool, and every caller
        # branches on it. A missing, busy or permission-denied port must therefore
        # come back as False instead of raising past the caller's error branch.
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                # parity = serial.PARITY_ODD,
                # stopbits = serial.STOPBITS_TWO,
                bytesize=serial.EIGHTBITS,
                timeout=0
            )
        except (serial.SerialException, OSError, ValueError) as e:
            self.last_error = str(e)
            self.ser = None
            self.is_open = False
            return False

        self.is_open = True

        try:
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError) as e:
            self.last_error = str(e)

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def getCFlagBaud(self, baudrate):
        if baudrate in [4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 250000, 500000, 1000000]:
            return baudrate
        else:
            return -1          