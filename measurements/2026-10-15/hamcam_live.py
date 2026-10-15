import json, socket, threading, time, urllib.request
H = ("192.168.14.199", 4533); out = []; RUN = True
def telem():
    while RUN:
        try:
            d = json.load(urllib.request.urlopen("https://sq6emm.hamradio.pl/camera/telemetry/state", timeout=3))
            out.append((time.time(), d["az"], d["overlap"]))
        except Exception as e: out.append((time.time(), None, repr(e)))
        time.sleep(0.2)
def q(f, c):
    f.write((c + "\n").encode())
    while True:
        l = f.readline().decode().strip()
        if l and (c.startswith("w ") or l.startswith("RPRT")): return l
def stop():
    c = socket.create_connection(H, 5); c.sendall(b"S\n"); r = c.recv(64); c.close(); return r
s = socket.create_connection(H, 5); f = s.makefile("rwb", buffering=0)
threading.Thread(target=telem, daemon=True).start(); time.sleep(1)
try:
    for tgt in (10, 350):
        t0 = time.time(); print(f"P {tgt}", q(f, f"P {tgt} 0"))
        while time.time() - t0 < 30:
            a = float(q(f, "w AR")[2:]); e = int(q(f, "w GE")[2:])
            if not -20 <= a <= 25 or e: print("SAFETY", a, e, stop()); raise SystemExit(1)
            if time.time() - t0 > 1 and not int(q(f, "w GS")[2:]) & 2: break
            time.sleep(0.3)
        else: print("TIMEOUT", stop()); raise SystemExit(1)
        time.sleep(3); print("  rest", q(f, "w AR"))
except BaseException as e:
    if not isinstance(e, SystemExit): print("ERR", e, stop())
    raise
finally:
    RUN = False; time.sleep(0.4)
    t0 = out[0][0]; last = None
    for t, az, ov in out:
        if (az, ov) != last: print(f"  hamcam t+{t - t0:5.1f}s az {az} overlap {ov}"); last = (az, ov)
    vals = [a for _, a, _ in out if a is not None]
    steps = [abs((b - a + 180) % 360 - 180) for a, b in zip(vals, vals[1:]) if a != b]
    print(f"hamcam: {len(out)} samples, {len(steps)} changes, step mean {sum(steps)/max(1,len(steps)):.2f} max {max(steps or [0]):.2f} deg")
