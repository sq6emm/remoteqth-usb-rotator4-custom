# sq6emm-rotor

Firmware written from scratch for the SQ6EMM azimuth rotator controller. It speaks **Easycomm III** only (Hamlib `rotctld -m 204`).

- **Controller:** RemoteQTH Rotator Interface 4.0.3 (ATmega328P at 16 MHz, Optiboot).
- **Rotator:** SPID RAU. Reed pulses come from the motor side, 1 pulse per degree. Both edges are counted, so the resolution is 0.5°.
- **Size:** about 16 kB of flash and 1.5 kB of RAM (including the 512-byte event log), against 27 kB for the K3NG-based ver 6.

## What it does

- **Range −30° … 390° (420°).** It overlaps on both sides of north. For a heading (0–360) the firmware picks the turn that is closest to the current position. A pass across north therefore never swings the long way.
- **Always a soft start.** PWM ramps from 0 to 255 over 2 s.
- **Always a soft stop:**
  - At the target, the last 3° slow down from PWM 200 to the floor of 64.
  - The ends of the range slow down over their last 5°.
  - Button release, a reversal and the end of `ML`/`MR` ramp down over 1 s.
- **Explicit STOP (`SA`)** cuts the motor current at once. This is the only hard stop.
- **Relays never switch under current:** PWM is 0 for 30 ms before and after a relay change. The opposite direction is energised only after the coasting has ended.
- **Pulse input:**
  - It is sampled at 2 kHz by a timer interrupt, and a level counts after 3 ms of stability. EMI spikes and reed chatter cannot count.
  - Pulses with the motor off are coasting in the last direction (the slew drive cannot be back-driven), so they are counted that way.
- **Stall guard:** if the motor is driven but no pulse arrives for 1.5 s (armed after 3 s), it stops with a jam error. There is also a 150 s move timeout.
- **Position storage:** the position is saved to EEPROM 1 s after the last pulse. The watchdog (2 s) drops the relays and PWM if the loop ever hangs.
- **Display:** the LCD shows the azimuth, an overlap marker, target, speed, errors and the preset. The CW, CCW and START buttons work as before, and so does the preset encoder (its knob is defective, see the ver 6 notes).

## Easycomm III

Commands are separated by space, CR or LF. A `;` is ignored, because Hamlib sends `GS\n;`. Queries are answered as `<cmd><value>\r\n`. Set commands get no reply.

| Command | Meaning |
|---|---|
| `AZ` | Query azimuth → `AZ123.5`. Gives 0–359.5, or raw −30…390 with register 9. |
| `AZ<x>` | Go to. For 0 ≤ x ≤ 360 it is a heading, and the turn closest to the current position is chosen. Any other value is an explicit raw position in the overlap, for example `AZ-20` or `AZ385`. A target outside the range is refused (`GE` bit 32). |
| `EL`, `EL<x>` | Returns `EL0.0`; setting it is ignored (azimuth-only rotator). |
| `SA` (`SE`) | STOP: immediate. `SE` is ignored. |
| `ML`, `MR` | Run CCW / CW to the range end, until a stop command. |
| `VL<v>`, `VR<v>` | Run CCW / CW at v mdeg/s, 0–9900, mapped onto the floor…MaxSpeed PWM. `0` is a soft stop. This is what Hamlib `M 8/16 <speed>` sends. |
| `GS` | Status: 1 idle, 2 moving, 4 pointing (idle on target), 8 error. |
| `GE` | Errors: 1 pulse-line chatter, 2 jam/stall, 8 timeout, 16 no stored position, 32 target out of range. A good move clears 2/8/32. `RESET` clears all. |
| `CR<r>` / `CR <r>` | Read register → `CR<r>,<value>`. |
| `CW<r>,<v>` | Write a register (stored in EEPROM). Invalid values are ignored. |
| `VE` | Version → `VESQ6EMM-rotor-1.0`. |
| `IP` | Inputs: bit 1 CW button, 2 CCW, 4 START, 8 pulse line level. |
| `AN` | Raw ADC of encoder channels A6,A2. |
| `RESET` | Stop and clear the errors (Hamlib `R`). |
| `PARK` | Go to the park position (register f). |
| `AR` | SQ6EMM extension: raw position, for example `AR375.5` (which turn). |
| `PD` | SQ6EMM extension: diagnostics, `PDa<accepted>,r<rejected>,c<coasting>,h<reversal holds>,s<stalls>,t<timeouts>,p<pwm>,f<Hz>`. |
| `LG`, `LG<seq>` | SQ6EMM extension: event log. `LG` gives `LG<next seq>@<ms>:`. `LG<seq>` returns up to 6 records from seq: `LG<first>@<ms>:t,c,p,k;…` (see below). |

### Registers

| Reg | Meaning | Default |
|---|---|---|
| 0 | MaxSpeed (PWM 1–255) | 253 |
| 1 | PWM floor: end of the slow-down. Lower values stall the motor from standstill at 490 Hz. | 64 |
| 2 | PWM frequency, Hz. Timer2 steps only: 30, 61, 122, 245, **490**, 980, 1953, 3921, 7812, 31372, 62500. Above 980 only with register e. | 490 |
| 3 / 4 | Range CCW / CW end, degrees. The span must be at least 360. | −30 / 390 |
| 5 / 6 | Soft start / soft stop ramp, ms (full scale) | 2000 / 1000 |
| 7 | **Position.** Read it, or write it to calibrate. A value of 0–359.9 keeps the current turn; any other value is raw. | — |
| 8 | Slow-down zone before the target, degrees | 3.0 |
| 9 | `AZ` reply: 0 gives 0–360, 1 gives raw | 0 |
| a | One correction move after an overshoot of 1° or more | 0 |
| b | Stall guard | 1 |
| c | Range ends enforced | 1 |
| d | Unstick: not implemented, reads 0 | 0 |
| e | Allow PWM above 980 Hz (needs hardware changes, see below) | 0 |
| f | Park position, degrees | 0 |

## PWM frequency and the hardware

The schematic (RemoteQTH 4.0.3 sheet 2, "Optional DC PWM") shows what drives the motor:
- **Q4** is a PSMN5R6-100PS, a standard-level MOSFET with Ciss of about 8 nF, on the low side of the motor supply.
- Its gate is driven straight from D11 through 510 Ω. There is no gate driver, so the gate swings only 0–5 V.
- **D2**, the freewheel diode, is a 1N4007, which has slow reverse recovery.

Each switching edge takes roughly 10 µs. Estimated switching loss in Q4 for a 24 V / 2 A motor (it scales with V × I):

| Frequency | Q4 loss | Verdict |
|---|---|---|
| 245–490 Hz | ~0.1–0.15 W | fine |
| 980 Hz | ~0.3 W | fine |
| 1.95 kHz | ~0.6 W | warm without a heatsink |
| 3.9 kHz | ~1.2 W | too hot without a heatsink |
| 31 kHz or more | — | the gate never fully switches |

That is why frequencies above 980 Hz need register e. **Silent PWM (31 kHz)** needs two changes:
- a gate driver (for example MCP1407 or TC4427), or a logic-level MOSFET with a smaller gate resistor;
- a Schottky diode in place of D2 (for example SB560 or MBR1060).

Changing the frequency also changes the motor's low-speed behaviour. Check register 1 (the floor) again after any change.

D11 is OC2A, so only the Timer2 prescaler steps exist. The other timer's pins (D9/D10) are taken by the CCW relay and the brake, so arbitrary frequencies are not possible.

## EEPROM and moving from the K3NG firmware

The firmware's own data lives at byte 128 (settings, with CRC) and byte 192 (position, with CRC). The K3NG data in bytes 0…~80 is left alone.

- **First start after K3NG ver 6:** the position is taken over from K3NG's `last_azimuth` at byte 9 (float). No recalibration is needed.
- **Every save** also writes K3NG's `last_azimuth`. Flashing ver 6 back therefore still finds the right position. Ver 6 only knows 0..360, and its counter stops at −15/375, so flash it back only while the rotator is inside 0..360.
- **No stored position:** the start position is 0 with `GE` bit 16. Calibrate with `CW7,<deg>`.

## Event log (motor characterisation)

A 64-entry ring buffer in RAM records the following, each with a millisecond timestamp, the position and the PWM:
- every accepted pulse edge (level, and whether it came while coasting);
- a sample every 100 ms while the motor is driven, if nothing else was logged;
- each state change: command (with target), relay on, ramp start, slow-down entered, soft stop, cut, relay off, explicit STOP, stall, timeout, range end, coasting hold.

Record fields `t,c,p,k`:
- `t`: milliseconds since power-up.
- `c`: position in 0.5° counts. For a command record it is the target in 0.1°.
- `p`: PWM.
- `k`: `type | dir << 4 | level << 6 | coast << 7`. Types: 1 edge, 2 start, 3 run, 4 slow, 5 soft, 6 cut, 7 idle, 8 sample, 9 stop, 10 stall, 11 timeout, 12 range end, 13 hold, 14 command. Direction: 1 CW, 2 CCW.

`host/rotorlog.py` collects the log into a CSV and analyses it. It **only reads**; it never moves the rotator.

```
python3 host/rotorlog.py collect --rotctld 192.168.14.199:4533 -o run1.csv   # through rotctld's raw 'w', hamcam keeps running
python3 host/rotorlog.py collect --serial /dev/rotator -o run1.csv           # direct (rotctld stopped; opening the port resets the MCU)
python3 host/rotorlog.py analyse run1.csv
```

The analysis gives, per move:
- the time from the ramp start to the first pulse, and the PWM at that moment (break-away);
- the top speed;
- the coasting distance and time after the cut or STOP.

It also builds a speed-versus-PWM table from full-degree periods at steady PWM, which avoids the 77/23 % contact duty asymmetry. Through rotctld the collector manages about 2.7 queries/s × 6 records, which keeps up with a full-speed move (about 12 events/s) with margin. Tested: `host/log_test.sh`.

## Hamlib

```
rotctld -m 204 -r /dev/rotator -s 9600 -C min_az=-30,max_az=390
```

Without the `-C` option Hamlib allows only 0–360. Headings still go the short way across north, and only an explicit `P -20` or `P 385` needs it.

`p` returns decimals (0.5° steps), so hamcam's `ROT_DECIMAL` workaround is not needed. The old K3NG raw commands (`w \?AZ`, `w \A…`, `w X1..X4`) no longer exist:

| Old K3NG command | Replacement |
|---|---|
| `\?AZ` | `AR` |
| `\A` | `CW7,<deg>` |
| speeds | `CW0,<pwm>` |
| `\?PD` | `PD` |

## Build and test (Docker)

```
# AVR build (Arduino AVR core 1.8.3, LiquidCrystal 1.0.7) - check the size stays below 30720 bytes
docker run --rm -v $PWD:/w -w /w hamcam-arduino:1.8.3 arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --output-dir /w/build ./sq6emm_rotor
# host tests: core + protocol against a simulated rotator (motor inertia/stiction, reed duty 77 %, chatter, EMI)
docker build -t hamcam-rotorhost:dev -f docker/Dockerfile.host docker
docker run --rm -v $PWD:/w -w /w hamcam-rotorhost:dev sh -c 'g++ -std=c++17 -O2 -Wall -o build-host/test host/test.cpp sq6emm_rotor/core.cpp sq6emm_rotor/proto.cpp && ./build-host/test'
# Hamlib integration: real rotctld -m 204 against the simulated controller on a pty
docker run --rm -v $PWD:/w -w /w hamcam-rotorhost:dev sh host/hamlib_test.sh
# event log through rotctld: rotorlog.py collects while rotctl moves the simulated rotator, then analyses
docker run --rm -v $PWD:/w -w /w hamcam-rotorhost:dev sh host/log_test.sh
```

The simulator also checks the hardware rules:
- no relay switching under current;
- no reversal while the motor still turns;
- no PWM jump on start;
- no hard cut from high PWM except an explicit STOP.

## Flashing

Use the same procedure as for ver 6 (see `k3ng-remoteqth-custom/SQ6EMM-CHANGES.md` §6). Keep `SQ6EMM-k3ng-2020.08.26.02-ver6.hex` ready as the fallback. rotctld must change from `-m 603` to `-m 204`.

## License

GPL-3.0.
