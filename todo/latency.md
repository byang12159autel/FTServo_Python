# Reducing key→motion latency in `keyboard_stream.py`

Current default loop runs at 115200 baud, 50 Hz, with `WritePosEx` waiting for an ACK each tick. End-to-end latency from keypress to torque is ~25–30 ms. Stacking the changes below brings it to ~1–2 ms.

Start with #2 (baud) and #1 (TxOnly) — they dominate.

## 1. Serial round-trip on every write *(biggest hidden cost)*
`WritePosEx` calls `writeTxRx` (sms_sts.py:68), which waits for a 6-byte status packet back. At 115200 baud that's ~0.5 ms tx + ~0.5 ms rx + servo think time ≈ 1–2 ms per command.

**Fix:** switch the goal write to `writeTxOnly` — fire-and-forget. Either edit the SDK or replicate `WritePosEx`'s packet inline:
```python
packetHandler.writeTxOnly(id, SMS_STS_ACC, 7, [acc, pos_lo, pos_hi, 0, 0, spd_lo, spd_hi])
```
Saves the response leg entirely.

## 2. Baudrate *(also big)*
115200 → 1 Mbps drops every byte's wire time ~9×. The `WritePosEx` packet is 14 bytes, so ~1.2 ms → ~0.14 ms.

**Fix:** run `sms_sts/set_baud.py --servo-id 2 --current-baud 115200 --new-code 0`, then pass `--baudrate 1000000` to `keyboard_stream.py` and `calibrate_range.py`.

## 3. Acceleration ramp
`acc=50` × 8.7 deg/s² ≈ 435 deg/s². Going from 0 to your goal speed eats time before motion looks like motion.

**Fix:** pass `--acc 0` for instant ramp.

## 4. Goal-speed cap
`--speed 1000` ≈ 732 rpm. If the move is large, raising `--speed` (try 3000–4000) gets you there faster. For tiny jogs it doesn't matter.

## 5. Control-loop period
`--rate-hz 50` = 20 ms tick. A keypress can sit up to 20 ms before the next write goes out.

**Fix:** `--rate-hz 200` (or 500). Combined with #1+#2 you've got plenty of bus headroom.

## 6. Loop ordering
Right now each tick does: poll key → read load → write goal (keyboard_stream.py:145–179). A fresh keypress waits for a load read RTT before its command goes out.

**Fix:** reorder to poll key → write goal → read load. Saves ~1 read-RTT (~2 ms at 115200).

## 7. Combine the reads
`PRESENT_POSITION` (56–57), `PRESENT_SPEED` (58–59), `PRESENT_LOAD` (60–61) are contiguous. One `readTxRx(id, 56, 6)` instead of separate calls halves read-side overhead — useful when you also call `ReadPos` on the `s` key.

---

# Reverting the servo baudrate

Same script, run it backwards — pass the current high baud as `--current-baud` and the code for the target baud as `--new-code`:

```
python3 sms_sts/set_baud.py --servo-id 2 --current-baud 1000000 --new-code 4
```

Code map (sms_sts.py:9–16):
| code | baud   |
|------|--------|
| 0    | 1 M    |
| 1    | 500 k  |
| 2    | 250 k  |
| 3    | 128 k  |
| 4    | 115200 |
| 5    | 76800  |
| 6    | 57600  |
| 7    | 38400  |

## Recovery: unknown current baud
If you lose track of what baud the servo is at (failed write, power glitch), brute-force scan: try `--current-baud` of `1000000`, `500000`, `250000`, `115200`, … until the "Servo responded" line prints, then flip from there.
