// Rotor core: position counting, motion control, configuration registers. No Arduino dependencies.
// Angles are integers in 0.1 deg ("tenths"). The reed contact of the SPID RAU closes at the start of each degree and opens
// again at edge_frac (measured 0.79): the counter counts the stretches between edges (even = closed, n..n+frac; odd =
// open, n+frac..n+1), and the position is the edge last crossed - n.0 or n.frac, whichever side it was crossed from.
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
  uint8_t  edge_frac;         // g (tenths): where the contact opens within a degree (1.2)
  uint16_t coast_ms;          // h (ms): coasting after the cut = speed x this (1.2)
  int8_t   pos_off;           // tenths, -5..4: where the edge grid sits within a degree, set by CW7 (1.2)
  uint16_t coast_land_ms;     // i (ms): coasting after a landing cut = the speed estimate x this (1.2; new fields last)
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
#define ER_UNCERT  64     // power-up after a reset during a move: the stored position may be off (set it with CW7)

// Event log (characterisation of the real motor: start delay, speed per PWM, coasting).
// k = type | dir << 4 (1 CW, 2 CCW, 0 relays off) | level << 6 (edges) | coast << 7 (edge with the motor off)
// c = position in 0.5 deg counts, except E_CMD: the new target in 0.1 deg
struct LogRec { uint32_t t; int16_t c; uint8_t p; uint8_t k; };
enum { E_EDGE = 1, E_START, E_RUN, E_SLOW, E_SOFT, E_CUT, E_IDLE, E_SAMPLE, E_STOP, E_STALL, E_TIMEOUT, E_LIMIT,
       E_HOLD, E_CMD };

class Rotor {
public:
  Settings s;

  // ---- shared with the pulse interrupt ----
  volatile int16_t cnt = 0;          // contact stretch: 2n = closed (n .. n+frac), 2n+1 = open (n+frac .. n+1)
  volatile int16_t pt = 0;           // position (tenths): the edge last crossed, or the calibrated / loaded value
  volatile uint32_t dsum[2] = {0, 0};      // contact duty at steady drive: [1] closed stretches (ms), [0] open ones
  volatile uint16_t dn[2] = {0, 0};
  volatile uint8_t run_edges = 0;    // edges while driven in this run (speed is known from the 3rd)
  volatile int8_t  drive_dir = 0;    // relays: +1 CW, -1 CCW, 0 off
  volatile int8_t  last_dir = 0;     // direction of the last drive (coasting pulses count this way)
  volatile uint32_t last_edge_ms = 0;
  volatile uint8_t  last_level = 1;  // debounced level; idle: contact open, pull-up -> high
  volatile uint8_t  pend_n = 0;      // samples the line has differed from last_level
  volatile uint16_t edge_dt[2] = {0, 0};   // the last two intervals between accepted edges (ms)
  volatile uint32_t n_acc = 0, n_rej = 0, n_coast = 0;
  volatile uint16_t n_resync = 0, n_burst = 0;   // counter parity healed after a missed edge / edge pairs undone as a burst
  uint16_t n_holds = 0, n_stalls = 0, n_timeouts = 0;

  LogRec logbuf[LOG_N];
  volatile uint16_t log_seq = 0;                     // events written so far (the next sequence number, wraps)
  volatile bool log_full = false;                    // the ring has wrapped at least once
  // copies up to max records starting at sequence 'from' (or the oldest kept); returns the count, *first = seq
  uint8_t log_read(uint16_t from, LogRec *out, uint8_t max, uint16_t *first) const;

  uint8_t errors = 0;
  uint8_t pwm_now = 0;
  uint16_t pwm_hz = DEF_PWM_FREQ;

  void defaults();
  void begin(uint32_t now_ms);                       // after settings and position are loaded
  void sample(uint8_t level, uint32_t now_ms);       // pulse line sample (timer ISR, PULSE_SAMPLE_HZ)
  void tick(uint32_t now_ms);                        // call every main-loop pass

  int16_t pos() const;                               // raw position, tenths (atomic copy)
  void set_raw(int16_t t);                           // position t at rest; the stretch follows from t and the contact
  int16_t edge_t(int16_t c) const;                   // tenths of the edge at the start of stretch c
  int16_t duty_permille() const;                     // measured closed share of a degree (-1: no data yet)
  int16_t az() const;                                // 0..3599
  int16_t pos_live(uint32_t now) const;              // for the replies / LCD: interpolated while moving and coasting
  int16_t az_live(uint32_t now) const;               // the same, 0..3599
  int16_t target() const { return tgt; }
  int8_t  dir() const { return mdir; }
  bool    moving() const { return st != S_IDLE || want; }   // includes a move waiting for the coasting to end
  bool    driving() const { return st != S_IDLE; }          // relays energised or about to be
  bool    manual_mode() const { return mode == M_MANUAL; }
  uint8_t status() const;

  // commands
  bool goto_az(int16_t t);       // 0..3600: the turn closest to the current position
  bool goto_raw(int16_t t);      // explicit raw position; false if outside the range
  void manual(int8_t d);         // run towards a range end until soft_stop / stop
  void soft_stop();              // ramp down (all normal stops)
  void stop_now();               // explicit STOP: motor current off immediately
  void set_speed(uint8_t pwm) { speed = pwm; }   // VL/VR velocity cap (0 = s.pwm_max)
  uint8_t pwm_for_speed(uint32_t mdeg) const;   // VL/VR mdeg/s -> PWM (measured curve, within floor..MaxSpeed)
  void set_position(int16_t t);  // calibration: 0..3599 keeps the current turn, other values are raw
  void clear_errors() { errors &= (ER_NOPOS | ER_UNCERT); }   // position errors only go away with CW7

  // registers ('0'..'9', 'a'..'g'); text in/out, angles with one decimal
  bool reg_read(char r, char *out) const;
  bool reg_write(char r, const char *val);
  bool settings_dirty = false;

  // position persistence: the hardware layer saves when due()
  bool save_due(uint32_t now_ms, bool force) const;    // idle, coasting over, and the position changed (or force)
  void saved(int16_t written) { saved_pt = written; has_saved = true; }
  bool calibrated_now = false;                       // set by CW7: the hardware layer refreshes the K3NG mirror
  int16_t cnt_copy() const;

private:
  enum { S_IDLE, S_RELAY_ON, S_RUN, S_SOFT, S_RELAY_OFF };
  enum { M_NONE, M_TARGET, M_MANUAL };
  uint8_t st = S_IDLE, mode = M_NONE, speed = 0;
  int8_t mdir = 0;                  // direction the relays drive (set at relay-on only)
  int8_t man_dir = 0;               // direction requested by ML/MR/VL/VR/buttons
  volatile int16_t clamp_lo = 0, clamp_hi = 0;   // counter limits in counts (read by the ISR)
  void set_clamps();
  int16_t tgt = 0;                  // raw target (tenths)
  bool want = false;                // a request waits for the motor to stop / coast out
  bool corrected = false, holding = false, brake_on = false;
  uint32_t t_state = 0, t_move = 0, t_tick = 0, t_idle = 0, t_cmd = 0;
  uint16_t ramp_acc = 0;            // PWM * 256 while ramping
  int16_t saved_pt = 0; bool has_saved = false;
  uint8_t pwm_edge = 0;             // PWM at the previous edge (duty measurement only at steady PWM)
  int16_t stretch(int16_t t, uint8_t level) const;
  // one consistent copy of what the pulse interrupt owns (an edge between separate reads mixed old and new values)
  struct Snap { int16_t c, p; uint16_t a; uint8_t n; int8_t d; uint32_t le; };
  Snap snap(uint32_t now) const;
  int16_t tol(const Snap &n, int8_t d) const;      // stop tolerance in direction d (tenths), when the speed is not known
  int16_t next_edge(int8_t d, int16_t c, int16_t p) const;
  int16_t speed_tps(const Snap &n, uint32_t now) const;                  // tenths/s over the stretch just crossed, 0 = unknown
  int16_t pos_est(const Snap &n, int8_t d, int16_t v, uint32_t now) const;   // last edge + travel since (before the next)
  // the state before the last accepted edge: a second edge sooner than EDGE_MIN_MS undoes both (EMI burst)
  volatile int16_t u_cnt = 0, u_pt = 0; volatile uint16_t u_dt0 = 0, u_dt1 = 0; volatile uint32_t u_le = 0;
  volatile uint8_t u_re = 0; volatile bool u_ok = false;
  void mark_cut(uint32_t now, uint16_t coast_ms);   // motor current off: where, how fast, how it will coast
  int16_t est_cut = 0, v_cut = 0; int8_t dir_cut = 0; bool est_pending = false; uint16_t coast_cut = 0;
  uint32_t t_cut = 0;
  int16_t in_stretch(int16_t e) const;   // clamp an estimate into the stretch the rotator is known to be in
  void finish_rest();               // apply a pending rest estimate now
  uint32_t rej_window_ms = 0; uint16_t rej_in_window = 0;

  int16_t lo() const { return s.az_min; }
  int16_t hi() const { return s.az_max; }
  int16_t dist_to_limit() const;
  bool coasting(uint32_t now, uint32_t le) const;
  uint32_t last_edge(uint32_t now) const;   // last pulse time, never later than now
  uint8_t cap() const;
  void start_request(int8_t d);
  bool slow_logged = false;
  volatile uint32_t t_log = 0;
  void log_isr(uint8_t type, uint32_t now, int16_t c, uint8_t flags);   // interrupts already off
  void log_ev(uint8_t type, uint32_t now, int16_t c = 0x7FFF);          // main loop (c default: position)
  void set_state(uint8_t ns, uint32_t now);
};

bool settings_ok(const Settings &s);

// helpers shared with the protocol
int16_t parse_tenths(const char *p, bool *ok);
char *fmt_tenths(char *out, int16_t v);
char *fmt_u32(char *out, uint32_t v);
