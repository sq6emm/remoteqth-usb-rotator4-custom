"""Firmware 1.5 full test: the precision sequence of precision12/13.py plus live position sampling (AR ~4/s, hamcam telemetry ~3/s) (2026-10-15): repeat landings on 120 from both sides, fractional targets, one STOP at
full speed. Backstop: S if the position leaves -20..175 or an error appears. Polls slowly so the collector keeps up."""
import socket, time, threading, json, urllib.request
SAMPLES = open("live15.csv", "w"); SAMPLES.write("host_t,src,value\n"); RUN = True
def sampler():
    c = socket.create_connection(("192.168.14.199", 4533), 5); g = c.makefile("rwb", buffering=0)
    def qq(cmd):
        g.write((cmd + "\n").encode())
        while True:
            l = g.readline().decode().strip()
            if l: return l
    r = qq("w LG"); SAMPLES.write(f"{time.time():.3f},sync,{r.split('@')[1].rstrip(':')}\n")   # controller ms <-> host time
    while RUN:
        r = qq("w AR"); SAMPLES.write(f"{time.time():.3f},AR,{r[2:]}\n"); time.sleep(0.2)
def telem():
    while RUN:
        try:
            d = json.load(urllib.request.urlopen("https://sq6emm.hamradio.pl/camera/telemetry/state", timeout=3))
            SAMPLES.write(f"{time.time():.3f},hamcam,{d['az']}\n")
        except Exception: pass
        time.sleep(0.3)
threading.Thread(target=sampler, daemon=True).start(); threading.Thread(target=telem, daemon=True).start()
H = ("192.168.14.199", 4533); LO, HI = -20, 175
s = socket.create_connection(H, 5); f = s.makefile("rwb", buffering=0)
def q(c):
    f.write((c + "\n").encode())
    while True:
        l = f.readline().decode().strip()
        if l and (c.startswith("w ") or l.startswith("RPRT")): return l
def stop():
    try:
        c = socket.create_connection(H, 5); c.sendall(b"S\n"); r = c.recv(64); c.close(); return r.decode().strip()
    except Exception as e: return repr(e)
T0 = time.time()
def log(*a): print(f"{time.time()-T0:7.2f}", *a, flush=True)
def ar(): return float(q("w AR")[2:])
def guard():
    a = ar(); e = int(q("w GE")[2:])
    if not LO <= a <= HI or e: log("SAFETY", a, "GE", e, "-> S", stop()); raise SystemExit(1)
    return a
def wait_rest(tmax=150):
    t = time.time()
    while time.time() - t < tmax:
        a = guard()
        if time.time() - t > 1 and not int(q("w GS")[2:]) & 2: break
        time.sleep(0.5)
    else: log("TIMEOUT -> S", stop()); raise SystemExit(1)
    time.sleep(2.0)                                    # rest estimate (coasting over)
    return guard()
res = []
def goto(x, tag=""):
    a0 = guard(); q(f"P {x} 0"); b = wait_rest()
    d = "CW " if x > a0 else "CCW"
    res.append((x, d, a0, b)); log(f"{tag:10} {a0:7.1f} -> {x:6.1f} {d}: rest {b:6.1f}  ({b - x:+.1f})  IP {q('w IP')}")
try:
    log("start", ar(), q("w VE"), q("w PD"))
    goto(120, "approach")
    for _ in range(3):
        goto(110, "from below"); goto(120, "120 CW"); goto(130, "from above"); goto(120, "120 CCW")
    for x in (120.5, 123.3, 117.7, 125.6, 114.2, 120.0): goto(x, "fraction")
    a0 = guard(); q("P 165 0"); time.sleep(8); a1 = guard(); log("S at", a1, stop()); b = wait_rest(); log(f"STOP from full speed: S near {a1}, rest {b}")
    log("PD", q("w PD"))
    goto(350, "home")
except BaseException as e:
    if not isinstance(e, SystemExit): log("ERR", repr(e), "-> S", stop())
    raise
finally:
    RUN = False; time.sleep(0.5); SAMPLES.close()
    cw = [b - x for x, d, a, b in res if d == "CW " and x == 120]; ccw = [b - x for x, d, a, b in res if d == "CCW" and x == 120]
    print("\n120 from below:", ["%+.1f" % v for v in cw], " from above:", ["%+.1f" % v for v in ccw])
