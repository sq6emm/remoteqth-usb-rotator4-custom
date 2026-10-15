# sq6emm-rotor

Firmware written from scratch for the SQ6EMM azimuth rotator controller. It speaks **Easycomm III** only (Hamlib `rotctld -m 204`).

- **Controller:** RemoteQTH Rotator Interface 4.0.3 (ATmega328P at 16 MHz, Optiboot).
- **Rotator:** SPID RAU. Reed pulses come from the motor side, 1 pulse per degree: the contact is closed for the first 0.79° of each degree and open for the rest. Both edges are counted at their real places, and between edges the position is interpolated from the speed (see *Precision*).
- **Size:** about 22.2 kB of flash and 1.50 kB of RAM (event log 48 entries since 1.7) (including the 512-byte event log), against 27 kB for the K3NG-based ver 6. The stack has at least 320 bytes to spare (measured in simavr from the ELF's .data + .bss).

## What it does

- **Range −30° … 390° (420°).** It overlaps on both sides of north. For a heading (0–360) the firmware picks the turn that is closest to the current position. A pass across north therefore never swings the long way.
- **Always a soft start.** PWM ramps from 0 to 255 over 2 s.
- **Always a soft stop:**
  - At the target, the last 3° slow down from PWM 200 to the floor of 64.
  - The ends of the range slow down over their last 5°.
  - Button release, a reversal and the end of `ML`/`MR` ramp down over 1 s.
- **Explicit STOP (`SA`)** cuts the motor current at once. This is the only hard stop, apart from the stall guard (nothing is turning then). If a new target lands on the current position while running fast, the motor soft-stops and comes back.
- **Relays never switch under current:** PWM is 0 for 30 ms before and after a relay change. The opposite direction is energised only after the coasting has ended.
- **Pulse input:**
  - It is sampled at 2 kHz by a timer interrupt, and a level counts after 3 ms of stability. EMI spikes and reed chatter cannot count.
  - Pulses with the motor off are coasting in the last direction (the slew drive cannot be back-driven), so they are counted that way.
- **Stall guard:** if the motor is driven but no pulse arrives for 1.5 s at full PWM, it stops with a jam error. The limit is armed after 3 s and is scaled by 253/PWM up to 8 s, because at slow speed the contact stays closed longer.
- **Timeouts:** a goto stops 150 s after its last command; manual runs stop after 15 min.
- **Commands during a move:** a manual command (`ML`/`MR`/`VL`/`VR`/button) against the current direction soft-stops first, waits for the coasting to end, then reverses. The range ends keep working.
- **Position storage:** a "move in progress" record is written when the motor starts, and the position is saved 1 s after the last pulse. A reset during a move (FT232 DTR, watchdog, brown-out) comes back with `GE` 64, "Pos uncertain". The watchdog (2 s) drops the relays and PWM if the loop ever hangs.
- **Never moves by itself:** nothing starts after power-up or a reset, and a button held or shorted at power-up is ignored until it has been released once.
- **Display:** the LCD shows the azimuth, an overlap marker, target, speed, errors and the preset. The CW, CCW and START buttons work as before, and so does the preset encoder (its knob is defective, see the ver 6 notes).

## Precision (1.2 – 1.7)

The SPID pulse contact gives two edges per degree, but not half a degree apart: it closes at the start of each degree
and opens 0.79° later (measured on 2026-10-15: 182 ms closed, 48 ms open at full speed). Up to 1.1 every edge counted
as 0.5°, regardless of direction. Depending on the direction a move arrived from, the reported position could be up
to 0.79° off, plus the coasting after the cut that no edge saw.

1.2 gets the most out of this contact:
- **Every edge at its real place.** The closing edge is n.0, the opening edge n + 0.8 (register g). Which edge is
  reported depends on the side the rotator crossed it from.
- **Between edges** the position is the last edge plus speed × time. The speed comes from the stretch just crossed.
- **The cut anticipates the coasting.** A landing uses speed estimate × 65 ms (register i), a STOP or soft stop speed ×
  200 ms (register h). Both are fitted to the real event log of 2026-10-15: while slowing down the estimate runs high,
  so the landing factor is smaller. The rotator coasts onto the target instead of past it.
- **While moving (1.5)** `AZ`, `AR` and the LCD show the interpolated position: driven, the last edge plus speed ×
  time; coasting, an even slow-down onto the expected rest point. Simulator: mean 0.08° from the truth, at most 0.35°
  once the speed is known (edge value: mean 0.35°, at most 0.79°). It never steps back against the travel. Control
  and the EEPROM keep the edge-based value.
- **At rest** the position is the estimated stopping point, to 0.1°. It is always kept between the two edges the
  rotator is known to be between.
- **Short moves:** a move too short for a speed reading only starts if it can end closer to the target than staying.
- **Calibration (`CW7`)** keeps fractions. The edge grid moves by `t − position`, whole degrees in the counter and the
  rest in a stored offset. Up to 1.1 a calibration snapped to the 0.5° grid.

1.3 also changes how the PWM may drop. It never falls faster than the soft-stop ramp, even when a target appears close
while running fast: then the rotator overruns, soft-stops and comes back. Up to 1.2 the slow-down zone applied at once
(253 → 68 in one step). A direct cut is now allowed up to PWM 110, the range the real landings always cut in.

In the simulator fitted to the real log (contact 0.79; motor response 0.1 s; coasting = inertia + worm friction), over
80 random gotos in both directions:
- reported vs. true position: within 0.09° (1.1: 0.79°);
- true position vs. target: mean 0.06°, at most 0.44° (1.1: mean 0.45°, at most 0.82°).

The worst case is a deliberate "stay". Steps of 1° or less still land within 0.5°: in the soft start the speed is
not known well enough yet.

The event log still records
counts (0.5° units): `rotorlog.py` shows them as nominal degrees.

### Robustness (1.7, from the independent review)

- **Slowing down follows register 6** at the configured rate. Up to 1.6 the target and range-end slow-down dropped
  about 1 PWM per main-loop pass (≈1000 PWM/s) whatever register 6 said. The slow-down now also starts early enough
  for the configured ramp at the present speed. The range end holds even with ramp_down 10000 (simulator: 390.06).
- **Self-healing counter.** Even stretch = contact closed is checked at every edge. A missed edge (for example a
  glitch in the boot read) is counted at the next edge, and `PD` field `y` counts these repairs.
- **Interference bursts** of 3 ms or more produced two edges closer than any real stretch (at least 45 ms). Such a pair
  is undone (`PD` field `z`); before, it counted +1.0° and gave an absurd speed.
- **"Position uncertain" (`GE` 64) survives every reset until `CW7`.** Before, it survived only one boot.
- **The calibration offset is stored with every position slot.** A torn settings write can no longer shift the edge
  grid silently.
- **Stored positions outside the range are rejected**, so the controller comes up with "no position".
- **The interrupt state is read as one snapshot per pass.** A pulse arriving between separate reads could make a
  landing cut early.
- **No relay click without motion:** a short move, once started, runs to its first edge.

## Easycomm III

Commands are separated by space, CR or LF. A `;` is ignored, because Hamlib sends `GS\n;`. Queries are answered as `<cmd><value>\r\n`. Set commands get no reply.

| Command | Meaning |
|---|---|
| `AZ` | Query azimuth → `AZ123.5`. Gives 0–359.5, or raw −30…390 with register 9. |
| `AZ<x>` | Go to. For 0 ≤ x ≤ 360 it is a heading, and the turn closest to the current position is chosen. Any other value is an explicit raw position in the overlap, for example `AZ-20` or `AZ385`. A target outside the range is refused (`GE` bit 32). |
| `EL`, `EL<x>` | Returns `EL0.0`; setting it is ignored (azimuth-only rotator). |
| `SA` (`SE`) | STOP: immediate. `SE` is ignored. |
| `ML`, `MR` | Run CCW / CW to the range end, until a stop command. |
| `VL<v>`, `VR<v>` | Run CCW / CW at v mdeg/s (0–9900), in real speed: the measured speed curve (below) picks the PWM. The motor runs at about 2.0–4.4°/s, so slower requests run at ~2°/s and faster ones at full speed. `0` is the slowest speed (Hamlib speed 1). This is what Hamlib `M 8/16 <speed>` sends (`VL`/`VR` = (speed − 1) × 100, so speed 21–45 is the useful range); it is stopped with `SA` (Hamlib `S`). |
| `GS` | Status: 1 idle, 2 moving, 4 pointing (idle on target), 8 error. |
| `GE` | Errors: 1 pulse-line chatter, 2 jam/stall, 8 timeout, 16 no stored position, 32 target out of range, 64 position uncertain (reset during a move). A good move clears 2/8/32. `RESET` clears 1/2/8/32. Only `CW7` clears 16 and 64. |
| `CR<r>` / `CR <r>` | Read register → `CR<r>,<value>`. |
| `CW<r>,<v>` | Write a register (stored in EEPROM). Invalid values are ignored. Registers 3, 4, 7 and c are refused while the rotator moves. |
| `VE` | Version → `VESQ6EMM-rotor-1.7`. |
| `IP` | Inputs: bit 1 CW button, 2 CCW, 4 START, 8 pulse line level. |
| `AN` | Raw ADC of encoder channels A6,A2. |
| `RESET` | Stop and clear the errors (Hamlib `R`). |
| `PARK` | Go to the park position (register f). |
| `AR` | SQ6EMM extension: raw position, for example `AR375.5` (which turn). |
| `PD` | SQ6EMM extension: diagnostics, `PDa<accepted>,r<rejected>,c<coasting>,h<reversal holds>,s<stalls>,t<timeouts>,p<pwm>,f<Hz>,e<contact closed share, per mille>` (`e-` until enough steady pulses were seen). |
| `LG`, `LG<seq>` | SQ6EMM extension: event log. `LG` gives `LG<next seq>@<ms>:`. `LG<seq>` returns up to 6 records from seq: `LG<first>@<ms>:t,c,p,k;…` (see below), only whole records and never more than 118 characters, because rotctld cuts raw replies at 127. A `+` at the end means more records may be waiting. |

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
| g | Where the reed contact opens within a degree, degrees (0.1–0.9). Compare with `PD` field `e` (per mille). | 0.8 |
| h | Coasting after a STOP or soft stop (steady speed): distance = speed × this, ms (0–1000). Real STOPs from full speed: 177–273 ms | 225 |
| i | Coasting after a landing cut (slowing down, so the speed estimate runs high): distance = speed estimate × this, ms. Real landings: 52–75 ms | 65 |

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

The firmware's own data:
- **Byte 128:** settings, with magic, version and CRC. Every field is range-checked on load; anything out of range gives the defaults.
- **Bytes 256…895:** a ring of 128 position slots (count, sequence, flags, CRC). Each save writes the next slot, which spreads the wear: about 6 million moves before the 100k-cycle rating of any cell. A brown-out during a write leaves the previous slot valid. A slot flagged "moving" is written when the motor starts; finding it as the newest slot at power-up gives `GE` 64.

The K3NG data in bytes 0…66 is left alone, except as described below.

- **First start after K3NG ver 6:** the position is taken over from K3NG's `last_azimuth` at byte 9 (float; offset verified with avr-gcc). No recalibration is needed.
- **The K3NG mirror:** `last_azimuth` is refreshed at most every 10 minutes, and right after a `CW7`. Flashing ver 6 back therefore finds the position, but it can be up to 10 min old. Before flashing back, send `CW7,<current position>` and wait 2 s. Ver 6 only knows 0..360, and its counter stops at −15/375, so flash it back only while the rotator is inside 0..360.
- **No stored position:** the start position is 0 with `GE` bit 16, and nothing is saved until you calibrate with `CW7,<deg>`.

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

### Speed curve

`VL`/`VR` speeds go through a table of steady speed per PWM, measured on the real rotator at 490 Hz on 2026-10-15 (`measurements/2026-10-15`): PWM 96 → 2.0°/s, 112 → 2.8°/s, 176 → 3.6°/s, 253 → 4.4°/s, linear in between (`SPEED_PWM` / `SPEED_MDEG` in `config.h`). MaxSpeed (register 0) and the floor (register 1) still limit the result. Re-measure with `host/rotorlog.py` after changing the PWM frequency or the mechanics. The simulators use the same measured curve (`host/motor.h`).

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
# AVR level: the compiled .elf in simavr (ATmega328P 16 MHz) with a motor model on the real pins
docker build -t hamcam-rotorsimavr:dev -f docker/Dockerfile.simavr docker
docker run --rm -v $PWD:/w -w /w hamcam-rotorsimavr:dev sh -c 'g++ -std=c++17 -O2 -I/usr/include/simavr -o build-host/avrtest host/avrtest.cpp -lsimavr -lsimavrparts -lelf && ./build-host/avrtest' 
```

The AVR-level test (`host/avrtest.cpp`) covers what the host simulator cannot:
- the Timer2 PWM registers (490 and 980 Hz, read from the registers, because simavr does not drive OC2A in PWM modes);
- the 2 kHz sampling interrupt on PD2;
- serial reply latency (8.5 ms);
- relay and brake pins, and the CW button on A5;
- the LCD contents (HD44780 model);
- EEPROM: takeover from a K3NG image, the position block and the K3NG mirror;
- the watchdog being enabled;
- stack depth: at least 310 bytes never used;
- a 1.1 EEPROM (settings version 1, position slots in 0.5° counts) taken over;
- a reset during a move (`GE64`, nothing moves by itself);
- a torn position write (the previous slot is used);
- a settings image with a valid CRC but impossible values (defaults);
- buttons held at power-up;
- a long `LG` reply during a move (non-blocking serial output, at most 118 characters).

Both simulators check the hardware rules:
- no relay switching under current;
- no reversal while the motor still turns;
- no PWM jump on start;
- no hard cut from high PWM except an explicit STOP.

## Flashing

Settings converted from an older image, or defaults after an unreadable one, are written only 8 s after boot (1.4).
Right after a flash, avrdude closing the port and rotctld opening it reset the MCU (DTR); 1.3 lost its first-boot
settings write that way. A register value stored by an older version (e.g. `h` = 200 from 1.3) is kept. A new default
only applies to a fresh image, or after `CW<r>,<value>`.

Use the same procedure as for ver 6 (see `k3ng-remoteqth-custom/SQ6EMM-CHANGES.md` §6). Keep `SQ6EMM-k3ng-2020.08.26.02-ver6.hex` ready as the fallback. rotctld must change from `-m 603` to `-m 204`.

## License

GPL-3.0.
