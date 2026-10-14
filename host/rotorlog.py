#!/usr/bin/env python3
"""Read the SQ6EMM rotor event log (LG) and characterise the motor.

Collect (Ctrl-C to stop) - it only reads, it never moves the rotator:
  rotorlog.py collect --rotctld 192.168.14.199:4533 -o run1.csv     # through rotctld (raw 'w' command)
  rotorlog.py collect --serial /dev/rotator -o run1.csv             # direct, rotctld stopped
Analyse:
  rotorlog.py analyse run1.csv

Event fields (firmware core.h): k = type | dir << 4 | level << 6 | coast << 7, c = position in 0.5 deg counts
(E_CMD: target in 0.1 deg), p = PWM at that moment.
"""
import argparse, csv, os, socket, sys, termios, time

TYPES = {1: "edge", 2: "start", 3: "run", 4: "slow", 5: "soft", 6: "cut", 7: "idle", 8: "sample", 9: "stop",
         10: "stall", 11: "timeout", 12: "limit", 13: "hold", 14: "cmd"}
FIELDS = ["seq", "t_ms", "type", "dir", "level", "coast", "pos_deg", "target_deg", "pwm"]


class RotctldLink:
    def __init__(self, hostport):
        host, port = hostport.rsplit(":", 1)
        self.s = socket.create_connection((host, int(port)), 5)
        self.f = self.s.makefile("rwb", buffering=0)

    def query(self, cmd):
        self.f.write(f"w {cmd}\n".encode())
        deadline = time.time() + 3
        while time.time() < deadline:
            line = self.f.readline().decode(errors="replace").strip()
            if line.startswith(cmd[:2]) and "@" in line:
                return line
            if line.startswith("RPRT") and line != "RPRT 0":
                raise IOError(line)
        raise IOError("no reply")


class SerialLink:
    def __init__(self, dev):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0                       # raw
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B9600
        a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 2
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        time.sleep(2.5)                              # opening the port resets the controller (DTR) - wait for the boot
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = b""

    def query(self, cmd):
        os.write(self.fd, (cmd + "\r").encode())
        deadline = time.time() + 2
        while time.time() < deadline:
            if b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                line = line.decode(errors="replace").strip()
                if line.startswith(cmd[:2]):
                    return line
                continue
            self.buf += os.read(self.fd, 256)
        raise IOError("no reply")


def parse(line):
    """LG<first>@<now>:t,c,p,k;... -> first, now, [(t, c, p, k)]"""
    head, _, body = line.partition(":")
    first, _, now = head[2:].partition("@")
    recs = [tuple(int(x) for x in r.split(",")) for r in body.split(";") if r]
    return int(first), int(now), recs


def row(seq, t, c, p, k):
    ty = k & 15
    d = (k >> 4) & 3
    return {"seq": seq, "t_ms": t, "type": TYPES.get(ty, str(ty)), "dir": {1: "cw", 2: "ccw"}.get(d, ""),
            "level": (k >> 6) & 1 if ty == 1 else "", "coast": (k >> 7) & 1 if ty == 1 else "",
            "pos_deg": "" if ty == 14 else c / 2, "target_deg": c / 10 if ty == 14 else "", "pwm": p}


def collect(args):
    link = RotctldLink(args.rotctld) if args.rotctld else SerialLink(args.serial)
    nxt, now, _ = parse(link.query("LG"))             # the current head: start from now, skip the old history
    out = open(args.output, "w", newline="")
    w = csv.DictWriter(out, FIELDS); w.writeheader()
    lost = n = 0; t_q = time.time(); q = 0
    t_end = time.time() + args.duration if args.duration else None
    try:
        while t_end is None or time.time() < t_end:
            first, now, recs = parse(link.query(f"LG{nxt}"))
            q += 1
            if first != nxt:
                gap = (first - nxt) & 0xFFFF
                lost += gap
                print(f"!! {gap} events lost (read faster)", file=sys.stderr)
            for i, r in enumerate(recs):
                rr = row((first + i) & 0xFFFF, *r)
                w.writerow(rr); n += 1
                if rr["type"] not in ("edge", "sample"):
                    print(f"{rr['t_ms'] / 1000:10.3f} {rr['type']:7} {rr['dir']:3} pos {rr['pos_deg']} target {rr['target_deg']} pwm {rr['pwm']}")
            out.flush()
            nxt = (first + len(recs)) & 0xFFFF
            if len(recs) < 6:
                time.sleep(args.interval)
            if time.time() - t_q > 10:
                print(f"   ({q / (time.time() - t_q):.1f} queries/s, {n} events, {lost} lost)", file=sys.stderr)
                t_q = time.time(); q = 0
    except KeyboardInterrupt:
        pass
    print(f"{n} events written to {args.output}, {lost} lost")


def analyse(args):
    rows = list(csv.DictReader(open(args.file)))
    for r in rows:
        r["t"] = int(r["t_ms"]) / 1000
        r["pwm"] = int(r["pwm"])
    # split into moves at each "start"
    moves, cur = [], None
    for r in rows:
        if r["type"] == "start":
            cur = {"rows": []}; moves.append(cur)
        if cur is not None:
            cur["rows"].append(r)
    speed_by_pwm = {}
    print(f"{'move':>4} {'dir':>4} {'from':>7} {'to':>7} {'1st edge after':>16} {'PWM then':>8} {'top deg/s':>9} "
          f"{'coast deg':>9} {'coast s':>7}")
    for i, m in enumerate(moves):
        rs = m["rows"]
        t_run = next((r["t"] for r in rs if r["type"] == "run"), None)
        edges = [r for r in rs if r["type"] == "edge" and r["coast"] == "0"]
        coast = [r for r in rs if r["type"] == "edge" and r["coast"] == "1"]
        cut = next((r for r in rs if r["type"] in ("cut", "stop", "limit")), None)
        first_edge = edges[0] if edges else None
        delay = f"{first_edge['t'] - t_run:.2f} s" if first_edge and t_run else "-"
        # speed from same-level edges (one full degree, removes the 77/23 % contact duty asymmetry)
        top = 0.0
        for a, b in zip(edges, edges[2:]):
            dt = b["t"] - a["t"]
            if dt <= 0 or a["level"] != b["level"] or abs(a["pwm"] - b["pwm"]) > 8:
                continue                              # only steady PWM: during the ramps the speed lags the PWM
            v = 1.0 / dt
            top = max(top, v)
            pwm = round((a["pwm"] + b["pwm"]) / 2 / 16) * 16
            speed_by_pwm.setdefault(pwm, []).append(v)
        c_deg = len(coast) * 0.5
        c_s = coast[-1]["t"] - cut["t"] if coast and cut else 0
        d = rs[0]["dir"]
        p0 = next((r["pos_deg"] for r in rs if r["pos_deg"]), "?")
        p1 = next((r["pos_deg"] for r in reversed(rs) if r["pos_deg"]), "?")
        print(f"{i:4} {d:>4} {p0:>7} {p1:>7} {delay:>16} {first_edge['pwm'] if first_edge else '-':>8} {top:9.2f} {c_deg:9.1f} {c_s:7.2f}")
    print("(1st edge after = from 'run' to the first pulse: soft-start ramp + break-away + the rest of the degree)")
    print("\nspeed by PWM (full-degree periods at steady PWM):")
    for pwm in sorted(speed_by_pwm):
        v = speed_by_pwm[pwm]
        print(f"  PWM {pwm:3d}: {sum(v) / len(v):5.2f} deg/s  (n={len(v)}, min {min(v):.2f}, max {max(v):.2f})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("collect")
    g = c.add_mutually_exclusive_group(required=True)
    g.add_argument("--rotctld", help="host:port of rotctld (-m 204)")
    g.add_argument("--serial", help="serial device (rotctld must be stopped)")
    c.add_argument("-o", "--output", required=True)
    c.add_argument("--interval", type=float, default=0.2, help="pause when caught up, s")
    c.add_argument("--duration", type=float, help="stop after this many seconds (default: Ctrl-C)")
    a = sub.add_parser("analyse")
    a.add_argument("file")
    args = ap.parse_args()
    collect(args) if args.cmd == "collect" else analyse(args)


if __name__ == "__main__":
    main()
