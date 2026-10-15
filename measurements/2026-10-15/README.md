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
