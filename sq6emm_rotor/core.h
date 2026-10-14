// Rotor core: position counting, motion control, configuration registers. No Arduino dependencies.
// Angles are integers in 0.1 deg ("tenths"); the pulse counter counts 0.5 deg steps (1 count = 5 tenths).
// "Raw" positions run over the whole range (default -30.0 .. 390.0 deg); the azimuth is raw modulo 360.
#pragma once
#include <stdint.h>
#include "config.h"
#include "port.h"

struct Settings {             // register
  uint8_t  pwm_max;           // 0
  uint8_t  pwm_floor;         // 1
  uint16_t pwm_freq;          // 2
  int16_t  az_min;            // 3 (tenths)
  int16_t  az_max;            // 4 (tenths)
  uint16_t ramp_up;           // 5 (ms)
  uint16_t ramp_down;         // 6 (ms)
  uint16_t slow_zone;         // 8 (tenths)
  uint8_t  report_raw;        // 9
  uint8_t  overshoot;         // a
  uint8_t  jam;               // b
  uint8_t  endpoints;         // c
  uint8_t  fast_pwm;          // e
  int16_t  park;              // f (tenths)
};

// GS status bits (Easycomm III)
#define ST_IDLE     1
#define ST_MOVING   2
#define ST_POINTING 4
#define ST_ERROR    8
// GE error bits: 1..4 Easycomm III, 8.. SQ6EMM
#define ER_SENSOR   1     // pulse line bouncing far beyond normal (many rejected edges)
#define ER_JAM      2     // driven but no pulses (stall guard)
#define ER_HOMING   4     // not used (no home switch)
#define ER_TIMEOUT  8     // move took longer than MOVE_TIMEOUT_MS
#define ER_NOPOS   16     // no stored position at power-up (set it with CW7)
#define ER_LIMIT   32     // a target outside the range was refused

class Rotor {
public:
  Settings s;

  // ---- shared with the pulse interrupt ----
  volatile int16_t cnt = 0;          // position in 0.5 deg counts
  volatile int8_t  drive_dir = 0;    // relays: +1 CW, -1 CCW, 0 off
  volatile int8_t  last_dir = 0;     // direction of the last drive (coasting pulses count this way)
  volatile uint32_t last_edge_ms = 0;
  volatile uint8_t  last_level = 1;  // debounced level; idle: contact open, pull-up -> high
  volatile uint8_t  pend_n = 0;      // samples the line has differed from last_level
  volatile uint16_t edge_dt[2] = {0, 0};   // the last two intervals between accepted edges (ms)
  volatile uint32_t n_acc = 0, n_rej = 0, n_coast = 0;
  uint16_t n_holds = 0, n_stalls = 0, n_timeouts = 0;

  uint8_t errors = 0;
  uint8_t pwm_now = 0;
  uint16_t pwm_hz = DEF_PWM_FREQ;

  void defaults();
  void begin(uint32_t now_ms);                       // after settings and position are loaded
  void sample(uint8_t level, uint32_t now_ms);       // pulse line sample (timer ISR, PULSE_SAMPLE_HZ)
  void tick(uint32_t now_ms);                        // call every main-loop pass

  int16_t pos() const;                               // raw position, tenths (atomic copy)
  int16_t az() const;                                // 0..3599
  int16_t target() const { return tgt; }
  int8_t  dir() const { return mdir; }
  bool    moving() const { return st != S_IDLE || want; }   // includes a move waiting for the coasting to end
  uint8_t status() const;

  // commands
  bool goto_az(int16_t t);       // 0..3600: the turn closest to the current position
  bool goto_raw(int16_t t);      // explicit raw position; false if outside the range
  void manual(int8_t d);         // run towards a range end until soft_stop / stop
  void soft_stop();              // ramp down (all normal stops)
  void stop_now();               // explicit STOP: motor current off immediately
  void set_speed(uint8_t pwm) { speed = pwm; }   // VL/VR velocity cap (0 = s.pwm_max)
  void set_position(int16_t t);  // calibration: 0..3599 keeps the current turn, other values are raw
  void clear_errors() { errors = 0; }

  // registers ('0'..'9', 'a'..'f'); text in/out, angles with one decimal
  bool reg_read(char r, char *out) const;
  bool reg_write(char r, const char *val);
  bool settings_dirty = false;

  // position persistence: the hardware layer saves when due()
  bool save_due(uint32_t now_ms) const;
  void saved() { saved_cnt = cnt_copy(); has_saved = true; }
  int16_t cnt_copy() const;

private:
  enum { S_IDLE, S_RELAY_ON, S_RUN, S_SOFT, S_RELAY_OFF };
  enum { M_NONE, M_TARGET, M_MANUAL };
  uint8_t st = S_IDLE, mode = M_NONE, speed = 0;
  int8_t mdir = 0;                  // direction of the current / pending move
  int16_t tgt = 0;                  // raw target (tenths)
  bool want = false;                // a request waits for the motor to stop / coast out
  bool corrected = false, holding = false, brake_on = false;
  uint32_t t_state = 0, t_move = 0, t_tick = 0, t_idle = 0;
  uint16_t ramp_acc = 0;            // PWM * 256 while ramping
  int16_t saved_cnt = 0; bool has_saved = false;
  uint32_t rej_window_ms = 0; uint16_t rej_in_window = 0;

  int16_t lo() const { return s.az_min; }
  int16_t hi() const { return s.az_max; }
  int16_t dist_to_limit() const;
  bool coasting(uint32_t now, uint32_t le) const;
  uint8_t cap() const;
  void start_request(int8_t d);
  void set_state(uint8_t ns, uint32_t now) { st = ns; t_state = now; }
};

// helpers shared with the protocol
int16_t parse_tenths(const char *p, bool *ok);
char *fmt_tenths(char *out, int16_t v);
char *fmt_u32(char *out, uint32_t v);
