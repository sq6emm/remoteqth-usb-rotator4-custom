"""Range-end test (firmware 1.1, 2026-10-15): both ends of -30..390 - refused targets beyond, full-speed manual run
into the end, a second push at the end. Backstop: S if the position leaves -31.5..391.5 or an unexpected error."""
import socket, time
s = socket.create_connection(("192.168.14.199", 4533), 5); f = s.makefile("rwb", buffering=0)
LO, HI = -31.5, 391.5
def send(c): f.write((c + "\n").encode())             # raw set commands: the controller sends no reply
def stop():                                            # backstop STOP on a fresh connection (the old one may be broken)
    try:
        c = socket.create_connection(("192.168.14.199", 4533), 5); c.sendall(b"S\n"); r = c.recv(64); c.close(); return r.decode().strip()
    except Exception as e: return repr(e)
def q(c):
    f.write((c + "\n").encode())
    while True:
        l = f.readline().decode().strip()
        if l and (c.startswith("w ") or l.startswith("RPRT")): return l
T0 = time.time(); res = []
def log(*a): print(f"{time.time()-T0:7.2f}", *a, flush=True)
def ar(): return float(q("w AR")[2:])
def guard():
    a = ar(); e = int(q("w GE")[2:]) & ~32                 # 32 = target refused (expected here)
    if not LO <= a <= HI or e: log("SAFETY", a, "GE", e, "-> S", q("S")); raise SystemExit(1)
    return a
def wait_idle(tmax):
    t = time.time(); mx, mn = -999, 999
    while time.time() - t < tmax:
        a = guard(); mx, mn = max(mx, a), min(mn, a)
        if time.time() - t > 1 and not int(q("w GS")[2:]) & 2: break
        time.sleep(0.3)
    else: log("TIMEOUT -> S", q("S")); raise SystemExit(1)
    time.sleep(1.5); a = guard(); return a, max(mx, a), min(mn, a)
def check(name, ok, info): res.append((name, ok)); log(("PASS " if ok else "FAIL ") + name, info)
def refused(cmd_raw, cmd_hl):
    a0 = guard()
    send(f"w {cmd_raw}"); time.sleep(2); a1 = guard(); ge = int(q("w GE")[2:])
    check(f"controller refuses {cmd_raw}", a1 == a0 and ge & 32, f"pos {a0} -> {a1}, GE {ge}")
    r = q(cmd_hl); time.sleep(2); a2 = guard()
    check(f"rotctld refuses '{cmd_hl}'", r != "RPRT 0" and a2 == a0, f"{r}, pos {a2}")
def end(dirn, target_near, limit):
    a, _, _ = (log(f"P {target_near}", q(f"P {target_near} 0")), wait_idle(150))[1]
    log("near the end:", a)
    refused(f"AZ{limit + 5 * dirn}", f"P {limit + 5 * dirn} 0")
    log(f"M {16 if dirn > 0 else 8} 100 (full speed into the end)", q(f"M {16 if dirn > 0 else 8} 100"))
    a, mx, mn = wait_idle(60)
    far = mx if dirn > 0 else mn
    check(f"full-speed manual run stops at {limit}", abs(a - limit) <= 0.5 and (far <= limit + 0.5 if dirn > 0 else far >= limit - 0.5), f"rest {a}, furthest seen {far}, PD {q('w PD')}")
    log(f"M {16 if dirn > 0 else 8} 50 again at the end", q(f"M {16 if dirn > 0 else 8} 50")); time.sleep(3); b = guard()
    mv = int(q("w GS")[2:]) & 2
    if mv: log("still moving -> S", q("S"))
    check("second push at the end does not move", b == a and not mv, f"pos {a} -> {b}")
try:
    log("start", ar(), q("w VE"), "GE", q("w GE"))
    end(+1, 385, 390)
    end(-1, -25, -30)
    log("back: P 342", q("P 342 0")); a, _, _ = wait_idle(150); log("back at", a, "GE", q("w GE"))
except BaseException as e:
    if not isinstance(e, SystemExit): log("ERR", repr(e), "-> S", stop())
    raise
finally:
    print("\nSUMMARY:", ", ".join(f"{n}: {'PASS' if ok else 'FAIL'}" for n, ok in res))
