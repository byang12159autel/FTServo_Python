This is source code from [official feetech repository](https://github.com/ftservo/FTServo_Python).

## Structure

```
root dirrectory
     |---scservo_sdk 
     |---sms_sts
     |---scscl
     |---hls
```
The 'scscl' 'sms_sts' 'hls' directories contain examples of using the library.

The source code of the library is located in the `scservo_sdk` directory.

The 'scsservo_sdk' directory contains the original archive with the source code of the library from the developer.

## Usage

Tested on Linux Raspbian GNU/Linux 9.13 (stretch).
Python version Python 3.5.3

### Method 1. Clone repositry

```bash
$ cd /usr/src/
$ sudo git clone https://github.com/ftservo/FTServo_Python.git
$ sudo chown -R pi FTServo_Python
$ cd FTServo_Python
$ pixi install            # creates the .pixi environment

# Basic Ping Test
$ python3 sms_sts/ping.py
# Keyboard Position Control
$ pixi run python3 sms_sts/keyboard_stream.py --servo-id 2 --baudrate 115200
```

## SMS/STS Control Modes

The SMS/STS servos support four control modes, selected by writing a value to register `SMS_STS_MODE` (address 33):

### Mode 0 — Position (Servo) Mode (default)
Closed-loop position control over the 0–4095 tick range (one full revolution).
- API: `WritePosEx(id, position, speed, acc)`, `RegWritePosEx(...)` + `RegAction()`, `SyncWritePosEx(...)`
- The servo accelerates at `acc * 8.7 deg/s²` to a max speed of `speed * 0.732 rpm`, then stops at the target tick.
- Examples: `sms_sts/write.py`, `sms_sts/reg_write.py`, `sms_sts/read_write.py`, `sms_sts/sync_write.py`

### Mode 1 — Wheel (Constant-Speed) Mode
Closed-loop continuous-rotation velocity control with no position limit.
- API: `WheelMode(id)` to enter, then `WriteSpec(id, speed, acc)` to command signed speed (negative = reverse). Send speed=0 to stop.
- Example: `sms_sts/wheel.py`

### Mode 2 — Open-Loop Wheel Mode
Continuous-rotation velocity control without position feedback. Lower torque accuracy than mode 1, but useful when encoder feedback is unreliable.
- Enter by writing `2` to register 33: `packetHandler.write1ByteTxRx(id, SMS_STS_MODE, 2)`
- Drive with `WriteSpec(id, speed, acc)`.

### Mode 3 — Step (PWM) Mode
Direct PWM/open-loop drive. The "speed" field is interpreted as a signed PWM duty value rather than an RPM target.
- Enter by writing `3` to register 33: `packetHandler.write1ByteTxRx(id, SMS_STS_MODE, 3)`
- Drive with `WriteSpec(id, pwm, acc)`.

To return to position mode from any other mode, write `0` to register 33.

### Method 2. Install pip package

Copy the sample file to any location convenient for you. In the example I use '/home/pi/FeetechTestFiles'

```
$ pip install ftservo-python-sdk
$ cd /home/pi/FeetechTestFiles/sms_sts
$ python3 ping.py
Succeeded to open the port
Succeeded to change the baudrate
[ID:001] ping Succeeded. SCServo model number : 1540
```

