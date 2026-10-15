# 2026-10-15: first measurement session on the real controller (firmware v1.0)

`session1.py` (moves A–H, 2° shorts) was stopped at 355° by its own guard (planning error, target past the guard limit);
`session1b.py` finished it (5°/10° shorts, STOP from full speed, back to 342°). `*.csv` = event log via `host/rotorlog.py`.

`session1.csv` was recorded with the rotorlog version before 34a97b1: the last record of some LG replies is cut by one
digit (rotctld limits raw replies to 127 chars) - edges near the end of the file show up as fake stop/slow events.

| PWM | speed |
|---|---|
| ~96 (Hamlib speed 15) | ~2.0°/s |
| ~112 (speed 30) | ~2.8°/s |
| ~176 (speed 60) | ~3.5–3.7°/s |
| 253 (goto) | 4.3–4.6°/s |

All gotos landed exactly on target with no coasting. STOP at full speed: 0.5–1° of coasting. Run → first pulse 0.5–1.0 s.
Through rotctld with the collector running, one query took ~0.6 s; S itself reached the controller without delay.

## Firmware 1.1 speed check

`speedtest11.py`: `M 16 31` (VR3000 = 3.0°/s requested) ran at **2.95°/s** (PWM 128, n=34, 2.85–3.13); the goto 5° ahead landed exactly on 362.0.

## Range ends (firmware 1.1)

`limits11.py`, all 8 checks passed:
- At both ends, the controller refused `AZ395` / `AZ-35` (GE 32), and rotctld refused `P 395` / `P -35` (RPRT -21). Nothing moved.
- A full-speed manual run (`M 16 100` / `M 8 100`) from 5° before the end slowed down at once (PWM ~120). It cut at PWM 77 exactly at 390.0 / -30.0, with no coasting past the end.
- A second manual push at the end was logged as a command, but the motor did not start.

The final `P 342` from -30 went the short way to raw -18 (heading 342): correct, but it ended on the overlap turn.

### Run 2 (`limits11b.py`): the full test in one clean run, 11/11 passed

CCW end first (from raw -18), then CW, then back to raw 342, unwound. The results are the same as run 1:
- Both ends: targets beyond were refused by the controller (GE 32) and by rotctld (RPRT -21).
- Full-speed manual runs slowed down from PWM ~120 and cut at PWM 77 exactly at -30.0 / 390.0.
- A second push did not move the motor.
- The gotos to -25 and 385 landed exactly. -30 to 385 (415 deg) took 93 s.

The collector lost 420 of about 1500 events, all during the long full-speed moves. Three clients shared rotctld at 9600 baud (test script, collector, hamcam), and pulses plus samples come at about 20/s. Every state event at both ends was recorded.

## Firmware 1.2 and 1.3 precision tests

- `precision12.*` (1.2): landings on 120 reported 119.7 from below and 120.1 from above, every time. Both are clamp
  limits: the rotator stopped before the expected edge. `coast_fit.py` gives the real coasting: about 50-75 ms x the speed
  estimate at a landing, 177-223 ms x speed at a STOP from steady full speed. The fit led to 1.3 (registers i and h).
- `precision13.*` (1.3): all 12 landings on 120 report 120.0, from both directions. The rest estimates fall inside the
  stretch where the rotator stopped. The fractional targets 117.7, 125.6, 114.2 and 120.0 report exactly; 123.3 reports
  123.1, and 120.5 from 120.0 stays (a 0.5 move cannot land closer). STOP from full speed: 1.01-1.22 deg = 226-273 ms
  x speed (1.2 test: 177-223). PD e792: the contact is closed for 0.792 of each degree.

## Firmware 1.5 full test

- `precision15.*`: the same sequence as for 1.3. All landings on 120 report 120.0 (once 120.1 from above).
  117.7, 123.3, 114.2 and 120.0 report exactly; 125.6 reports 125.8; 120.5 from 120.0 stays. STOP and PD e791 match 1.3.
  h = 225, set under 1.4, survived the flash: the delayed settings write works on the real board.
- `live15.csv` + `live_check.py`: live AR samples (~4/s) against the time-interpolated edge timeline of the log, on
  neighbouring edges. 1.5 live value: mean 0.06, median 0.04, 90 % 0.12. The edge-only value (1.4): mean 0.35, 90 % 0.73.
  104/104 samples lie inside the bracketing edges.
- `hamcam_live.py`: 350 -> 10 -> 350 through north while polling hamcam's telemetry at 5 Hz. The azimuth moves
  smoothly (e.g. 351.0, 351.5, 352.6 ...), and both rests are exact (raw 10.0 / -10.0). The overlap tag lags up to ~1 s
  around north (telemetry asks w AR once a second).

- `hamcam_live2.py` (hamcam v0.0.35): the same move with every sample checked: the overlap tag switched in the same
  telemetry sample as the azimuth crossed north, both ways; 0 wrong of 100 samples (v0.0.34: up to ~1 s late).
