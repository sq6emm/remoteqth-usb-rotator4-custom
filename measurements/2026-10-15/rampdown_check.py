"""Largest PWM drop rate while driven (PWM > 0 before and after) in an event log, over windows of >= 200 ms.
1.7 must stay at the soft stop ramp (register 6: 1000 ms = 255 PWM/s)."""
import csv, sys
for f in sys.argv[1:]:
    rows = [r for r in csv.DictReader(open(f)) if r["type"] in ("sample", "slow", "edge", "run")]
    pts = [(int(r["t_ms"]), int(r["pwm"])) for r in rows]
    worst = 0
    for i, (t0, p0) in enumerate(pts):
        if p0 == 0: continue
        for t1, p1 in pts[i + 1:]:
            if p1 == 0: break
            if t1 - t0 >= 200:
                worst = max(worst, (p0 - p1) / ((t1 - t0) / 1000)); break
    print(f"{f}: largest PWM drop while driven {worst:.0f} PWM/s")
