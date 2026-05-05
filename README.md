This is source code from [official feetech repository](https://github.com/ftservo/FTServo_Python).

TODO: Central script to write ID and baudrate to EPROM

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
# Basic Ping Test, use sweep flag to search for ID & Baudrate
$ pixi run python3 sms_sts/ping.py --sweep

# Calibrate Max/Min range travel
$ pixi run python3 sms_sts/calibrate_range.py --servo-id 2 --baudrate 115200

# Keyboard Position Control
$ pixi run python3 sms_sts/keyboard_stream.py --servo-id 2 --baudrate 115200
```

### Full Gripper Setup

Increase Baudrate 115200 -> 1000000. The WritePosEx packet is 14 bytes. 1 Mbps drops every byte's wire time ~9×. 
```bash
# Sets baudrate to 1000000
pixi run python3 sms_sts/set_baud.py

# Rerun Calibration
pixi run python3 sms_sts/calibrate_range.py --servo-id 2 --baudrate 1000000

# Deadtime 20ms
pixi run python3 sms_sts/keyboard_stream.py --servo-id 2 --baudrate 1000000 --rate-hz 200 --acc 200 

# soft squeeze (cap at 90% stall torque)                                                       
pixi run python3 sms_sts/keyboard_stream.py --baudrate 1000000 --max-torque 900 
```

Internally the position control runs a trapezoidal velocity profile for each new goal. Tune default --acc 50 and --speed 60 for the trapezoid. 

#### Notes:
- Watch calibration range. Current no fix for 0 wrap-around (0 -> 4096) except for hardware remount
- MAX_TORQUE_LIMIT is normalized. The register holds a 0..1000 value that represents percent × 10 of that motor's stall torque, not an absolute number. Max torque = 100 for 80kg.cm servo is 8kg.cm



## Hardware List
- Open Arm Components
- Fleetech Servo

- M4 x 12mm | x4
- M4 Nut | x4
- M2.5 x 16mm | x2
- M2.5 x 6mm | x3
- M4x4x6 Brass Insert | x2
- M3 x 6mm | x4 
- M3 x 20mm | x4
- M3 Nut | x4
- PCB Stand
- M2.5 x 6mm countersunk| x2
- M3 x6 self tapping countersunk| x2

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

## MAX_TORQUE_LIMIT — Position-Mode Force Cap

`MAX_TORQUE_LIMIT` (register 16-17, 2 bytes, range 0..1000) caps the duty cycle the position controller is allowed to output. It is a **clamp**, not a fault trigger — the control loop keeps running, the H-bridge just sees a smaller duty.

Inside the firmware's main loop (running at ~1-2 kHz):

```
       ┌─────────────────┐    desired_duty
goal──▶│ Position PID    │────────┐
pos ──▶│ (Kp, Kd, Ki)    │        │
       └─────────────────┘        ▼
                            ┌──────────────────┐
                            │ clamp to ±cap    │ ← MAX_TORQUE_LIMIT
                            │ (saturating)     │
                            └────────┬─────────┘
                                     │ final_duty
                                     ▼
                              ┌────────────┐
                              │  H-bridge  │
                              └────────────┘
```

The PID computes `desired_duty` from position error every cycle. The firmware then clamps it to `[−MAX_TORQUE_LIMIT, +MAX_TORQUE_LIMIT]` before driving the H-bridge. If the PID wants 800 PWM but the cap is 200, only 200 reaches the motor.

### Use case: force-regulated grip

For soft grippers and other compliant mechanisms, the cleanest pattern is:

1. Write a low cap (e.g. 200 = 20% of stall torque) to `MAX_TORQUE_LIMIT`
2. Stay in position mode (mode 0) — no mode switching needed
3. Command `WritePosEx` to a position past the object (or past `close_tick`)
4. The PID drives toward the goal, hits the cap, and applies constant capped torque indefinitely

This gives you the same constant-force behavior as PWM mode (mode 3) but stays inside the firmware's safety envelope. The angle-limit gate, overload protection, and current protection all work normally — none of which is reliably true in mode 3.

### Scale is normalized per motor

The 0..1000 register value is **percent × 10 of *that* motor's stall torque**, not an absolute torque value. The firmware doesn't know the motor's rated kg·cm. So:

| Cap | % stall | 80 kg·cm motor | 40 kg·cm motor | 20 kg·cm motor |
|---|---|---|---|---|
| 1000 | 100% | 80 kg·cm | 40 kg·cm | 20 kg·cm |
| 200 | 20% | 16 kg·cm | 8 kg·cm | 4 kg·cm |
| 100 | 10% | 8 kg·cm | 4 kg·cm | 2 kg·cm |

Across a fleet of mixed servos: same `MAX_TORQUE_LIMIT` value gives the same *fraction* of capacity, not the same absolute force.

### Register lives in EEPROM

Writes require the unlock-write-lock dance (`unLockEprom` → `write2ByteTxRx(id, 16, value)` → `LockEprom`). The value persists across power cycles. `keyboard_stream.py` exposes this as `--max-torque <0..1000>` and handles the EEPROM ritual for you.

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

