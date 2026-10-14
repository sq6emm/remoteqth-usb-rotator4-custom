#!/bin/sh
# Hamlib integration: real rotctld (EasycommIII, -m 204) talking to the simulated controller over a pty.
set -e
cd "$(dirname "$0")/.."
mkdir -p build-host
g++ -std=c++17 -O2 -o build-host/simrotor host/simrotor.cpp sq6emm_rotor/core.cpp sq6emm_rotor/proto.cpp
echo "Hamlib: $(rotctl --version)"
rotctl -l | grep -i easycomm

./build-host/simrotor 91 > build-host/pty.txt 2> build-host/sim.log &
SIM=$!
sleep 0.5
PTY=$(cat build-host/pty.txt)
rotctld -m 204 -r "$PTY" -s 9600 -T 127.0.0.1 -t 4533 -C min_az=-30,max_az=390 > build-host/rotctld.log 2>&1 &
RCD=$!
sleep 1
R() { rotctl -m 2 -r 127.0.0.1:4533 "$@"; }
fail=0
ok() { echo "PASS $1"; }
bad() { echo "FAIL $1"; fail=1; }
waitpos() {   # waitpos <target> <timeout s>
  i=0
  while [ $i -lt $(($2 * 2)) ]; do
    a=$(R p | head -1)
    if awk -v a="$a" -v t="$1" 'BEGIN { d = a - t; if (d < 0) d = -d; exit !(d <= 0.3) }'; then
      sleep 1.5; b=$(R p | head -1)                 # settled: the same reading 1.5 s later
      [ "$a" = "$b" ] && { echo "$a"; return 0; }
    fi
    sleep 0.5; i=$((i + 1))
  done
  echo "$a"; return 1
}

a=$(R p | head -1); awk -v a="$a" 'BEGIN { exit !(a == 91) }' && ok "p at start: $a" || bad "p at start: $a"
v=$(R _ ); echo "info: $v"
R P 120 0
t0=$(date +%s)
a=$(waitpos 120 30) && ok "P 120 -> p $a after $(( $(date +%s) - t0 )) s" || bad "P 120 -> p $a"
R P 350 0
a=$(waitpos 350 60) && ok "P 350 (heading) -> p $a" || bad "P 350 -> p $a"
sleep 1; grep -q "fw  -10.0" build-host/sim.log && ok "P 350 from 120 went CCW across north (raw -10)" || bad "raw -10 not seen"
R P 10 0
a=$(waitpos 10 60) && ok "P 10 across north -> p $a" || bad "P 10 -> p $a"
R P 300 0
a=$(waitpos 300 120) && ok "P 300 -> p $a (the only candidate from 10: raw 300)" || bad "P 300 -> p $a"
R P 20 0
a=$(waitpos 20 60) && ok "P 20 -> p $a" || bad "P 20 -> p $a"
sleep 1; grep -q "fw  380.0" build-host/sim.log && ok "300 -> 20 went CW across north (raw 380, 80 deg instead of 280)" || bad "raw 380 not seen"
R P 200 0; sleep 4
R S
sleep 0.3
p1=$(R p | head -1); sleep 2; p2=$(R p | head -1)
[ "$p1" = "$p2" ] && ok "S stops at once ($p1)" || bad "S: $p1 -> $p2"
R M 16 50; sleep 3
a1=$(R p | head -1); sleep 1; a2=$(R p | head -1)
awk -v a="$a1" -v b="$a2" 'BEGIN { exit !(b > a) }' && ok "M 16 50 (right = VR4900) moves CW: $a1 -> $a2" || bad "M 16 50: $a1 -> $a2"
R S; sleep 1.5
R M 8 50; sleep 3
a1=$(R p | head -1); sleep 1; a2=$(R p | head -1)
awk -v a="$a1" -v b="$a2" 'BEGIN { exit !(b < a) }' && ok "M 8 50 (left = VL4900) moves CCW: $a1 -> $a2" || bad "M 8 50: $a1 -> $a2"
R S
R P -20 0
a=$(waitpos 340 200) && ok "P -20 from raw 380 (400 deg, needs min_az=-30) -> p $a (reported as 0..360)" || bad "P -20 -> p $a"
sleep 1; grep -q "fw  -20.0" build-host/sim.log && ok "P -20 went to raw -20" || bad "raw -20 not seen"
viol=$(tail -1 build-host/sim.log | sed 's/.*viol //')
[ "$viol" = "0" ] && ok "no hardware-rule violations" || bad "violations: $viol"
kill $RCD $SIM 2>/dev/null || true
echo "--- rotctld log tail"; tail -5 build-host/rotctld.log
echo "--- sim log tail"; tail -3 build-host/sim.log
[ $fail = 0 ] && echo "ALL PASS" || { echo "SOME FAILED"; exit 1; }
