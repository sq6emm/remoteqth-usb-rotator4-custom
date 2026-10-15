"""Firmware 1.1 speed check (2026-10-15): M 16 31 (VR3000 = 3.0 deg/s) for 6 s from 342, then a goto 5 deg ahead."""
import socket, time
s = socket.create_connection(("192.168.14.199", 4533), 5); f = s.makefile("rwb", buffering=0)
LO, HI = 330, 375
def q(c):
    f.write((c + "\n").encode()); out = []
    while True:
        l = f.readline().decode().strip()
        if not l: continue
        out.append(l)
        if c.startswith("w ") or l.startswith("RPRT"): return out[-1]
T0 = time.time()
def log(*a): print(f"{time.time()-T0:7.2f}", *a, flush=True)
def guard():
    a = float(q("w AR")[2:]); e = int(q("w GE")[2:])
    if not LO <= a <= HI or e: log("SAFETY", a, "GE", e, "-> S", q("S")); raise SystemExit(1)
    return a
try:
    a = guard(); log("start", a, q("w VE"))
    log("M 16 31", q("M 16 31")); t = time.time()
    while time.time() - t < 6: a = guard(); time.sleep(0.3)
    x = round(a + 5); log(f"P {x} to land from {a}", q(f"P {x} 0")); t = time.time()
    while time.time() - t < 30:
        a = guard()
        if not int(q("w GS")[2:]) & 2: break
        time.sleep(0.3)
    else: log("TIMEOUT -> S", q("S")); raise SystemExit(1)
    time.sleep(1.5); log("landed", guard(), "target", x, q("w PD"))
except BaseException as e:
    if not isinstance(e, SystemExit): log("ERR", repr(e), "-> S", q("S"))
    raise
