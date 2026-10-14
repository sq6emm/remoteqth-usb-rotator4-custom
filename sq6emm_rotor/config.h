// SQ6EMM rotor controller - build-time configuration
// Hardware: RemoteQTH Rotator Interface 4.0.3 (ATmega328P 16 MHz, Optiboot), SPID RAU azimuth rotator with
// motor-side reed pulses (1 pulse per degree, both edges counted = 0.5 deg).
#pragma once

#define FW_VERSION "SQ6EMM-rotor-1.0"

// ---- pins (RemoteQTH 4.0.3, see docs/Rotator-Interface-4.0.3.pdf) ----
#define PIN_PULSE      2    // PD2, PULSE-CCW line from the rotator (10k pull-up, 1k/100n/1k + BAV99); contact closed = low
#define PIN_RELAY_CW   8    // Q3 -> RL2 (CW), also the CW button LED
#define PIN_RELAY_CCW  9    // Q2 -> RL1 (CCW), also the CCW button LED
#define PIN_BRAKE     10    // Q1 -> RL3 (brake release), also the START button LED
#define PIN_PWM       11    // OC2A -> 510R -> Q4 gate (PSMN5R6-100PS, low side of the motor supply, D2 1N4007 freewheel)
#define PIN_BTN_START A3    // buttons to ground
#define PIN_BTN_CCW   A4
#define PIN_BTN_CW    A5
#define ENC_ADC_A      6    // preset encoder channel A on A6 (analog-only pin -> sampled by the ADC interrupt)
#define ENC_ADC_B      2    // channel B on A2
#define LCD_RS 12
#define LCD_EN 13
#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7

#define SERIAL_BAUD 9600    // Hamlib EasycommIII: 9600..19200 8N1

// ---- fixed timing (ms unless noted) ----
#define PULSE_SAMPLE_HZ    2000   // the pulse line is sampled by a Timer1 interrupt at this rate ...
#define PULSE_STABLE_N        6   // ... and a new level counts after this many equal samples (3 ms): EMI spikes and
                                  // reed chatter never last that long, the shortest real phase is ~50 ms (0.23 deg)
#define RELAY_SETTLE_MS      30   // relays switch only without motor current: PWM is 0 this long before / after
#define QUIET_MS            300   // never energize the opposite direction while coasting pulses still arrive:
#define QUIET_MAX_MS       2000   // no edge for max(QUIET_MS, 2 x the last edge intervals), at most QUIET_MAX_MS,
#define COAST_MIN_MS        500   // and at least COAST_MIN_MS after the motor current was switched off
#define STALL_ARM_MS       3000   // stall guard armed after this long ...
#define STALL_NO_PULSE_MS  1500   // ... then: driven but no pulse for this long = jam
#define MOVE_TIMEOUT_MS  150000UL // a single move never takes longer (420 deg at ~4.4 deg/s is ~100 s)
#define BRAKE_HOLD_MS      3000   // brake released (RL3 / START LED) until this long after the motor stopped
#define SAVE_AFTER_MS      1000   // position saved to EEPROM this long after the last pulse while idle
#define TOLERANCE_T           4   // stop when the remaining distance is <= 0.4 deg (counts are 0.5 deg)
#define LIMIT_ZONE_T         50   // manual moves / range ends: slow down over the last 5 deg
#define SLOW_PWM_CAP        200   // slow-down ramps start at most at this PWM
#define OVERSHOOT_T          10   // overshoot correction (register a) when off target by >= 1.0 deg
#define POS_MARGIN_T        150   // the counter keeps counting 15 deg beyond the range (never loses pulses)

// ---- encoder (preset knob) ----
#define ENC_LOW          300      // ADC hysteresis thresholds (external AREF from the board divider)
#define ENC_HIGH         700
#define ENC_STABLE         5      // samples (~1 ms) a level must be stable
#define ENC_TRANSITIONS    4      // quadrature transitions per detent
#define ENC_REVERSE        0
#define ENC_FAST_MS      150      // 5 clicks within this time ...
#define ENC_FAST_STEP     10      // ... count 10 deg per click
#define PRESET_SHOW_MS  5000

// ---- defaults of the configuration registers (Easycomm III CR/CW, stored in EEPROM) ----
#define DEF_PWM_MAX      253      // 0: MaxSpeed (PWM 0..255)
#define DEF_PWM_FLOOR     64      // 1: slow-down end PWM (lower stalls the motor from standstill at 490 Hz)
#define DEF_PWM_FREQ     490      // 2: PWM frequency in Hz (Timer2 steps only)
#define DEF_AZ_MIN      -300      // 3: range CCW end, 0.1 deg (raw: 0 = north on the first turn)
#define DEF_AZ_MAX      3900      // 4: range CW end, 0.1 deg
#define DEF_RAMP_UP     2000      // 5: soft start, ms from 0 to PWM 255
#define DEF_RAMP_DOWN   1000      // 6: soft stop (reversal, button release, ML/MR stop), ms from 255 to 0
#define DEF_SLOW_ZONE     30      // 8: slow-down zone before the target, 0.1 deg
#define DEF_REPORT_RAW     0      // 9: AZ reply 0 = 0..359.9, 1 = raw (-30..390)
#define DEF_OVERSHOOT      0      // a: one correction move after an overshoot
#define DEF_JAM            1      // b: stall guard
#define DEF_ENDPOINTS      1      // c: range ends enforced
#define DEF_FAST_PWM       0      // e: allow PWM > 980 Hz (needs a gate driver + Schottky freewheel diode)
#define DEF_PARK           0      // f: park azimuth, 0.1 deg
#define PWM_SAFE_MAX_HZ  980      // without register e
