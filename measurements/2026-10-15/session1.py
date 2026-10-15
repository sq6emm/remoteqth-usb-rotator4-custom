"""Measurement session 1 (2026-10-15): moves between 290 and 350 deg; rotorlog.py records the event log in parallel."""
import os, socket, sys, time
s = socket.create_connection((os.environ.get("ROTCTLD_HOST", "192.168.14.199"), 4533), 5); f = s.makefile("rwb", buffering=0)
LO, HI = 288, 352
def q(c):
    f.write((c + "\n").encode()); out = []
    while True:
        l = f.readline().decode().strip()
        if not l: continue
        out.append(l)
        if c.startswith("w ") or l.startswith("RPRT"): return out[-1]
def ar(): return float(q("w AR")[2:])
def gs(): return int(q("w GS")[2:])
def ge(): return int(q("w GE")[2:])
T0 = time.time()
def log(*a): print(f"{time.time()-T0:7.2f}", *a, flush=True)
def guard():
    a = ar(); e = ge()
    if not LO <= a <= HI or e:
        log("SAFETY", a, "GE", e, "-> S", q("S")); raise SystemExit(1)
    return a
def wait_idle(tmax=60):
    t = time.time()
    while time.time() - t < tmax:
        a = guard()
        if not gs() & 2: time.sleep(1.5); return guard()
        time.sleep(0.3)
    log("TIMEOUT -> S", q("S")); raise SystemExit(1)
def goto(x, tag):
    a = guard(); log(f"{tag}: P {x} from {a}", q(f"P {x} 0")); b = wait_idle(); log(f"{tag}: landed {b} (err {b - x:+.1f})")
def manual(d, speed, secs, tag):
    a = guard(); log(f"{tag}: M {d} {speed} from {a}", q(f"M {d} {speed}"))
    t = time.time()
    while time.time() - t < secs: a = guard(); time.sleep(0.3)
    x = round(a - 5) if d == 8 else round(a + 5)          # soft landing a few degrees ahead
    log(f"{tag}: P {x} to land", q(f"P {x} 0")); b = wait_idle(); log(f"{tag}: landed {b} (err {b - x:+.1f})")
def stop_at(x, after, tag):
    a = guard(); log(f"{tag}: P {x} from {a}", q(f"P {x} 0")); time.sleep(after)
    a = guard(); log(f"{tag}: S at {a}", q("S")); b = wait_idle(); log(f"{tag}: rest {b} (coast {abs(b - a):.1f} deg after S)")
try:
    log("start", ar(), "GE", ge(), q("w PD"))
    goto(302, "A long CCW"); goto(342, "B long CW")
    manual(8, 15, 8, "C manual CCW speed 15"); manual(16, 15, 8, "D manual CW speed 15")
    manual(8, 30, 8, "E manual CCW speed 30"); manual(16, 30, 8, "F manual CW speed 30")
    manual(8, 60, 6, "G manual CCW speed 60"); manual(16, 60, 6, "H manual CW speed 60")
    a = round(ar())
    for i, st in enumerate([2, -2, 5, -5, 10, -10]): goto(a + st if st > 0 else a, f"I short {abs(st)}")
    stop_at(305, 4, "J STOP CCW full speed"); goto(342, "K back")
    log("end", ar(), "GE", ge(), q("w PD"))
except BaseException as e:
    if not isinstance(e, SystemExit): log("ERR", repr(e), "-> S", q("S"))
    raise
