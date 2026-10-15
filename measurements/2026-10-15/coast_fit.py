"""Coasting at the cut, from the 1.2 event log: speed and position at the cut from the edges before it, the edges crossed
while coasting after it, and the stretch where the rotator came to rest -> bounds on the real coasting distance."""
import csv, sys
F = 0.79
rows = list(csv.DictReader(open(sys.argv[1])))
def E(c): n = c // 2; return n + (F if c % 2 else 0)
ev = []
for r in rows:
    if r["pos_deg"] == "": continue
    c = round(float(r["pos_deg"]) * 2); ev.append((int(r["t_ms"]), r["type"], r["dir"], c, int(r["pwm"]), r["coast"]))
out = []
for i, (t, ty, d, c, p, co) in enumerate(ev):
    if ty not in ("cut", "stop"): continue
    edges = [e for e in ev[max(0, i - 40):i] if e[1] == "edge"]
    if len(edges) < 2: continue
    dirn = 1 if edges[-1][2] == "cw" else -1
    pos = lambda e: E(e[3]) if dirn > 0 else E(e[3] + 1)
    (t1, *_), (t2, *_) = edges[-2], edges[-1]
    p1, p2 = pos(edges[-2]), pos(edges[-1])
    if t2 - t1 <= 0 or t2 - t1 > 2000: continue
    v = abs(p2 - p1) / ((t2 - t1) / 1000)
    pe = p2 + dirn * v * (t - t2) / 1000
    nxt = [e for e in ev[i + 1:i + 12] if e[1] == "edge"]
    after = [e for e in nxt if e[0] - t < 1500]
    last_c = after[-1][3] if after else edges[-1][3]
    lo, hi = E(last_c), E(last_c + 1)
    # coast bounds: rest lies in [lo, hi]
    cmin = max(0.0, (lo - pe) * dirn if dirn > 0 else (pe - hi)) if False else None
    rest_lo, rest_hi = (lo, hi)
    coast_lo = max(0.0, (rest_lo - pe) if dirn > 0 else (pe - rest_hi))
    coast_hi = (rest_hi - pe) if dirn > 0 else (pe - rest_lo)
    out.append((t, ty, dirn, p, v, pe, len(after), coast_lo, coast_hi))
    print(f"{ty:4} {'CW ' if dirn > 0 else 'CCW'} pwm {p:3d} v {v:4.2f} deg/s at {pe:7.2f}: {len(after)} edge(s) while coasting, "
          f"rest in [{rest_lo:.2f}, {rest_hi:.2f}] -> coast {coast_lo:.2f} .. {coast_hi:.2f} deg = {coast_lo / v * 1000:4.0f} .. {coast_hi / v * 1000:4.0f} ms x v"
          f"   (model 180 ms: {v * 0.18:.2f})")
