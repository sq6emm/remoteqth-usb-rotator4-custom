"""Calibration test: excursions in both directions, back to Sky Tower each time; the tower's place in a clean frame
(template match against the calibration frame, sub-pixel) vs where the reported position says it must be."""
import socket, subprocess, time, cv2, numpy as np
H = ("192.168.14.199", 4533); LO, HI = 40, 215; PXDEG = 34.9
REF = cv2.imread("/s/raw0.jpg", cv2.IMREAD_GRAYSCALE).astype(np.float32)   # taken at rotator 122.0 before CW7 = 122.4 now
X_REF, R_REF = 1264.5, 122.4                    # Sky Tower centre in that frame, and the reading for that pointing now
TX0, TY0, TW, TH = 1235, 820, 70, 90            # template: Sky Tower + CZK mast
TPL = REF[TY0:TY0 + TH, TX0:TX0 + TW]
s = socket.create_connection(H, 5); f = s.makefile("rwb", buffering=0)
def q(c):
    f.write((c + "\n").encode())
    while True:
        l = f.readline().decode().strip()
        if l and (c.startswith("w ") or l.startswith("RPRT")): return l
def stop():
    c = socket.create_connection(H, 5); c.sendall(b"S\n"); r = c.recv(64); c.close(); return r
def goto(x):
    q(f"P {x} 0"); t = time.time()
    while time.time() - t < 120:
        a = float(q("w AR")[2:]); e = int(q("w GE")[2:])
        if not LO <= a <= HI or e: print("SAFETY", a, e, stop()); raise SystemExit(1)
        if time.time() - t > 1 and not int(q("w GS")[2:]) & 2: break
        time.sleep(0.3)
    else: print("TIMEOUT", stop()); raise SystemExit(1)
    time.sleep(2.5); return float(q("w AR")[2:])
def tower_x():
    xs = []
    for i in range(2):
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-rtsp_transport", "tcp", "-i", "rtsp://127.0.0.1:8557/main",
                        "-frames:v", "1", "-q:v", "2", f"/s/ct.jpg"], check=True)
        g = cv2.imread("/s/ct.jpg", cv2.IMREAD_GRAYSCALE).astype(np.float32)
        y0, y1, x0, x1 = TY0 - 40, TY0 + TH + 40, 1000, 1560
        r = cv2.matchTemplate(g[y0:y1, x0:x1], TPL, cv2.TM_CCOEFF_NORMED)
        _, mx, _, (lx, ly) = cv2.minMaxLoc(r)
        dx = 0.0
        if 0 < lx < r.shape[1] - 1:                   # sub-pixel: parabola through the peak
            a, b, c = r[ly, lx - 1], r[ly, lx], r[ly, lx + 1]; dx = 0.5 * (a - c) / (a - 2 * b + c)
        xs.append(x0 + lx + dx + (X_REF - TX0)); time.sleep(0.7)
    return sum(xs) / len(xs), mx
res = []
try:
    print("start", q("w AR"), q("w VE"))
    for exc, tag in ((100, "CCW 22 -> back CW"), (150, "CW 28 -> back CCW"), (60, "CCW 62 -> back CW"),
                     (200, "CW 78 -> back CCW"), (111.3, "CCW 11 -> back CW"), (127.6, "CW 5 -> back CCW"), (90, "CCW 32 -> back CW"), (165, "CW 43 -> back CCW")):
        goto(exc); r = goto(122.4)
        x, score = tower_x()
        exp = X_REF + (R_REF - r) * PXDEG              # where the tower must be if the reported position is right
        err = (x - exp) / PXDEG
        res.append(err)
        print(f"{tag:20} rest {r:6.1f}: tower at x {x:7.1f} (expected {exp:7.1f}, match {score:.2f}) -> position error {err:+.2f} deg", flush=True)
except BaseException as e:
    if not isinstance(e, SystemExit): print("ERR", repr(e), stop())
    raise
finally:
    if res: print(f"\n{len(res)} returns: error mean {sum(res)/len(res):+.2f}, spread {max(res)-min(res):.2f}, max |error| {max(abs(v) for v in res):.2f} deg")
