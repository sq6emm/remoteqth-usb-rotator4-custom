#!/bin/sh
# Event log through Hamlib: rotorlog.py polls LG via rotctld 'w' while rotctl moves the simulated rotator.
set -e
cd "$(dirname "$0")/.."
mkdir -p build-host
g++ -std=c++17 -O2 -o build-host/simrotor host/simrotor.cpp sq6emm_rotor/core.cpp sq6emm_rotor/proto.cpp
./build-host/simrotor 91 > build-host/pty.txt 2> build-host/sim.log &
SIM=$!; sleep 0.5
rotctld -m 204 -r "$(cat build-host/pty.txt)" -s 9600 -T 127.0.0.1 -t 4533 -C min_az=-30,max_az=390 > build-host/rotctld.log 2>&1 &
RCD=$!; sleep 1
python3 host/rotorlog.py collect --rotctld 127.0.0.1:4533 -o build-host/log.csv --duration 75 > build-host/collect.txt 2>&1 &
COL=$!; sleep 1
R() { rotctl -m 2 -r 127.0.0.1:4533 "$@"; }
R P 92 0; sleep 4          # 1 deg
R P 100 0; sleep 7         # 8 deg
R P 150 0; sleep 16        # 50 deg
R P 140 0; sleep 7         # reversal-free 10 deg back
R M 16 30; sleep 6; R S; sleep 3   # slow manual move (VR2900) + STOP
R M 16 99; sleep 6; R S; sleep 3   # fast manual move + STOP
wait $COL
kill $RCD $SIM 2>/dev/null || true
cat build-host/collect.txt
python3 host/rotorlog.py analyse build-host/log.csv
