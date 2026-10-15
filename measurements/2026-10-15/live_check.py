"""Live position (1.5) vs the edge timeline of the event log: between two logged edges the rotator moves almost evenly,
so the time-interpolated edge positions are a good reference. Compares the live AR samples, and the edge-only value
(what 1.4 reported), against it.  live_check.py live15.csv precision15.csv"""
import csv, sys, bisect
F = 0.79
def E(c): n = c // 2; return n + (F if c % 2 else 0)
live = list(csv.DictReader(open(sys.argv[1])))
sync = next(r for r in live if r["src"] == "sync")
off = float(sync["host_t"]) - int(sync["value"]) / 1000 - 0.05       # host time of controller ms 0 (half a query late)
edges = []
for r in csv.DictReader(open(sys.argv[2])):
    if r["type"] != "edge" or r["pos_deg"] == "": continue
    c = round(float(r["pos_deg"]) * 2); d = 1 if r["dir"] == "cw" else -1
    edges.append((int(r["t_ms"]) / 1000 + off, E(c) if d > 0 else E(c + 1), d))
ts = [e[0] for e in edges]
dl, de, n, inside = [], [], 0, 0
for r in live:
    if r["src"] != "AR": continue
    t, v = float(r["host_t"]) - 0.05, float(r["value"])                # the reply describes ~half a query earlier
    i = bisect.bisect_right(ts, t)
    if i == 0 or i >= len(edges): continue
    (t0, p0, d0), (t1, p1, d1) = edges[i - 1], edges[i]
    if d0 != d1 or t1 - t0 > 1.5 or t - t0 > 1.5 or abs(p1 - p0) > 0.8: continue   # two neighbouring edges of one run
    ref = p0 + (p1 - p0) * (t - t0) / (t1 - t0)
    dl.append(abs(v - ref)); de.append(abs(p0 - ref)); n += 1
    lo, hi = min(p0, p1) - 0.15, max(p0, p1) + 0.15
    inside += lo <= v <= hi
if n:
    dl.sort(); de.sort()
    q = lambda a, f: a[min(len(a) - 1, int(f * len(a)))]
    print(f"{n} live samples between edges of a run: |live - edge timeline| mean {sum(dl)/n:.2f} median {q(dl,.5):.2f} 90% {q(dl,.9):.2f} max {dl[-1]:.2f}")
    print(f"   edge-only value (1.4):                 mean {sum(de)/n:.2f} median {q(de,.5):.2f} 90% {q(de,.9):.2f} max {de[-1]:.2f}")
    print(f"   inside the bracketing edges (+-0.15): {inside}/{n}")
ham = [(float(r["host_t"]), float(r["value"])) for r in live if r["src"] == "hamcam"]
steps = [abs(b[1] - a[1]) for a, b in zip(ham, ham[1:]) if 0 < abs(b[1] - a[1]) < 10]
if steps: print(f"hamcam telemetry: {len(ham)} samples, changing steps mean {sum(steps)/len(steps):.2f} deg, max {max(steps):.2f}")
