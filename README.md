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

### Quickstart

```bash
$ sudo git clone https://github.com/ftservo/FTServo_Python.git
$ sudo chown -R pi FTServo_Python
$ cd FTServo_Python
$ pixi install            # creates the .pixi environment

# Quick Tests: 
# Basic Ping Test
$ pixi run python3 sms_sts/ping.py
# Calibrate Max/Min range travel
$ pixi run python3 sms_sts/calibrate_range.py --servo-id 2 --baudrate 115200
# Keyboard Position Control
$ pixi run python3 sms_sts/keyboard_stream.py --servo-id 2 --baudrate 115200
```

### Full Gripper Setup

Increase Baudrate 115200 -> 1000000. The WritePosEx packet is 14 bytes. 1 Mbps drops every byte's wire time ~9×. 
```bash
pixi run python3 sms_sts/set_baud.py

pixi run python3 sms_sts/keyboard_stream.py --servo-id 2 --baudrate 1000000 --rate-hz 200 --acc 200 
```

Internally the position control runs a trapezoidal velocity profile for each new goal. Tune --acc and --speed for the trapezoid. 



## SMS/STS Control Modes

The SMS/STS servos support four control modes, selected by writing a value to register `SMS_STS_MODE` (address 33):

### Mode 0 — Position (Servo) Mode (default)
Closed-loop position control over the 0–4095 tick range (one full revolution).
- API: `WritePosEx(id, position, speed, acc)`, `RegWritePosEx(...)` + `RegAction()`, `SyncWritePosEx(...)`
- Internally runs a **trapezoidal velocity profile** for each new goal:
  1. Accelerate at `acc * 8.7 deg/s²` from 0
  2. Cruise at `speed * 0.732 rpm`
  3. Decelerate at `acc` back to 0, landing on the goal tick
- Edge cases:
  - **Triangular profile**: if the move is too short to reach cruise speed, the servo decelerates before the flat-top — automatic, no flat-top segment.
  - **`acc=0`**: bypasses the ramp entirely. The servo applies max internal effort to drive position error to zero — closer to a "snap" than a trapezoid. Snappier for small jogs, but can shock heavy loads.
  - **`speed=0`**: no speed cap; the servo uses its internal max.
  - **`GOAL_TIME` register (44–45)**: alternative timed-move mode some firmwares support (specify duration instead of speed). `WritePosEx` writes 0 here, so this SDK is always in speed mode.
- Mental model: `acc` shapes the *corners*, `speed` shapes the *middle*.
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

## Debug Notes

### Servo stops moving but still responds — power cycle fixes it

Most likely cause: an **overload/error latch with auto-torque-off**. SMS/STS firmware monitors load, current, voltage, and temperature; when a threshold is exceeded long enough it latches a fault bit and, on most units, drops `TORQUE_ENABLE` to 0 automatically. After that:

- The servo keeps responding to packets (so it isn't "dead" on the bus).
- It accepts your `WritePosEx` writes — register updates fine.
- But it doesn't move, because torque is off and the error is latched.
- Some firmwares refuse to re-enable torque until the fault is cleared.

A power cycle clears the volatile error latch and brings torque back. `keyboard_stream.py` re-enables torque at startup (line 114).

Why this is the prime suspect for the gripper script:
- The contact-freeze logic triggers on `PRESENT_LOAD > 400`. If the gripper hits a stop or freeze reacts late, the servo can briefly push hard enough to latch overload before the freeze pins it.
- If `WritePosEx` is replaced with `writeTxOnly` (or even with `TxRx` when the error byte isn't checked), a latched fault is invisible from the host.

**Other plausible causes:**
1. **UART parser desync** — if the script is killed mid-packet, the servo's RX state machine can be waiting on bytes that never come. Power cycle resets the parser.
2. **EEPROM stuck unlocked** — if code unlocked EEPROM and crashed before relocking. Doesn't usually stop motion, but worth a relock.
3. **Host-side serial buffer wedged** — FTDI/CH340 driver state. Power-cycling the *adapter* (not the servo) helps here; servo power cycle alone won't.

**What to do:**
- Keep `WritePosEx` (the `TxRx` version) and check the returned `err`. The SDK has `getRxPacketError(err)` for human-readable output.
- Periodically read `SMS_STS_PRESENT_TEMPERATURE` (reg 63) and `SMS_STS_PRESENT_CURRENT_L` (reg 69) to spot drift before it latches.
- If torque was auto-disabled, re-write `SMS_STS_TORQUE_ENABLE = 1` to clear soft state on most units before reaching for the power switch.
- Lower `--force-threshold` or raise `--rate-hz` so contact freeze engages before overload.

### Other Installation Option: Install pip package

Copy the sample file to any location convenient for you. In the example I use '/home/pi/FeetechTestFiles'

```
$ pip install ftservo-python-sdk
$ cd /home/pi/FeetechTestFiles/sms_sts
$ python3 ping.py
Succeeded to open the port
Succeeded to change the baudrate
[ID:001] ping Succeeded. SCServo model number : 1540
```

