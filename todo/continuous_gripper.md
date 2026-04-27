# Gripper Continuous Position Control — Implementation Plan

Target: drive an SMS/STS servo as a robot gripper using closed-loop position control over a calibrated range, with grip force feedback.

## 1. Calibrate the safe travel range (one-time)
- [ ] Disable torque and hand-jog the gripper to the fully-open position; record tick value via `ReadPos(id)` as `OPEN_TICK`.
- [ ] Hand-jog to fully-closed (against mechanical stop, no payload); record tick value as `CLOSED_TICK`.
- [ ] Write `OPEN_TICK` to `SMS_STS_MIN_ANGLE_LIMIT_L` (reg 9) and `CLOSED_TICK` to `SMS_STS_MAX_ANGLE_LIMIT_L` (reg 11).
- [ ] Wrap the EPROM writes with `unLockEprom(id)` / `LockEprom(id)`.
- [ ] Verify limits by commanding positions outside the range and confirming the servo refuses.

## 2. Confirm Mode 0 (Position mode)
- [ ] Read register 33 (`SMS_STS_MODE`); confirm it is `0`. If not, write `0` and re-lock EPROM.
- [ ] Confirm `WritePosEx(id, OPEN_TICK, speed, acc)` and `WritePosEx(id, CLOSED_TICK, speed, acc)` move the gripper end-to-end.

## 3. Streaming position control loop
- [ ] Pick a loop rate (start at 50 Hz; raise to 100–200 Hz if smoother tracking is needed).
- [ ] If running >100 Hz or sharing the bus with other servos, raise the baud to `SMS_STS_1M` and switch to `SyncWritePosEx`.
- [ ] In each tick: compute desired tick from robot state (clamped to [OPEN_TICK, CLOSED_TICK]) and call `WritePosEx(id, target, speed, acc)`.
- [ ] Tune `speed` so the servo always reaches the target before the next command is sent (no chunky stop-start motion).
- [ ] Tune `acc`: start with `acc=50`; use `acc=0` for snappiest response, higher values to smooth direction changes.

## 4. Close the loop on grip force
- [ ] Each tick (or every N ticks), read load via `read2ByteTxRx(id, SMS_STS_PRESENT_LOAD_L)` — or current via `SMS_STS_PRESENT_CURRENT_L` on newer STS units.
- [ ] Determine `FORCE_THRESHOLD` empirically: command a closing motion onto a representative payload and log load values until the desired clamping force is reached.
- [ ] When `load > FORCE_THRESHOLD`, freeze the goal at the current position: read `ReadPos(id)` and re-issue `WritePosEx(id, present_pos, 0, 0)` to hold without further advance.
- [ ] Add a release condition: when the commanded target moves back below the contact position by more than a small hysteresis, resume normal position tracking.
- [ ] Add a stall timeout: if load stays above threshold for longer than expected, log a warning (possible jam or dropped object).

## 5. Safety and robustness
- [ ] On startup, ping the servo and abort if it does not respond.
- [ ] Handle `scs_comm_result != COMM_SUCCESS` in the control loop (retry once, then surface an error).
- [ ] Monitor `SMS_STS_PRESENT_TEMPERATURE` (reg 63) periodically; back off if it climbs above a safe threshold.
- [ ] On shutdown, command the gripper to a known-safe position before closing the port.

## 6. Open questions
- [ ] Does the gripper's full travel exceed one revolution (4096 ticks)? If yes, multi-turn config is needed instead of single-turn position mode.
- [ ] Is a single SMS/STS unit sufficient, or will the gripper share a bus with other servos that need their own loop rates?
