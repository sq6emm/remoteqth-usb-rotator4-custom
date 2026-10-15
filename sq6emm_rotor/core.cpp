#include "core.h"

static int16_t iabs16(int16_t v) { return v < 0 ? -v : v; }
static uint8_t umin8(uint8_t a, uint8_t b) { return a < b ? a : b; }

// counts (0.5 deg) <-> tenths, rounding to the nearest count
static int16_t t2c(int16_t t) { return t >= 0 ? (t + 2) / 5 : -((-t + 2) / 5); }

void Rotor::defaults() {
  s.pwm_max = DEF_PWM_MAX;   s.pwm_floor = DEF_PWM_FLOOR; s.pwm_freq = DEF_PWM_FREQ;
  s.az_min = DEF_AZ_MIN;     s.az_max = DEF_AZ_MAX;
  s.ramp_up = DEF_RAMP_UP;   s.ramp_down = DEF_RAMP_DOWN; s.slow_zone = DEF_SLOW_ZONE;
  s.report_raw = DEF_REPORT_RAW; s.overshoot = DEF_OVERSHOOT; s.jam = DEF_JAM; s.endpoints = DEF_ENDPOINTS;
  s.fast_pwm = DEF_FAST_PWM; s.park = DEF_PARK; s.edge_frac = DEF_EDGE_FRAC; s.coast_ms = DEF_COAST_MS; s.pos_off = 0;
}

void Rotor::set_clamps() {
  int16_t lo_c = t2c(s.az_min - POS_MARGIN_T), hi_c = t2c(s.az_max + POS_MARGIN_T);
  ATOMIC_BEGIN clamp_lo = lo_c; clamp_hi = hi_c; ATOMIC_END
}

bool settings_ok(const Settings &s) {
  return s.pwm_max >= 1 && s.pwm_floor <= s.pwm_max && s.pwm_freq >= 30 && s.pwm_freq <= 62500 &&
         (s.pwm_freq <= PWM_SAFE_MAX_HZ || s.fast_pwm == 1) &&
         s.az_min >= -3600 && s.az_min <= 0 && s.az_max >= 3600 && s.az_max <= 7200 && s.az_max - s.az_min >= 3600 &&
         s.ramp_up >= 100 && s.ramp_up <= 10000 && s.ramp_down >= 100 && s.ramp_down <= 10000 && s.slow_zone <= 300 &&
         s.report_raw <= 1 && s.overshoot <= 1 && s.jam <= 1 && s.endpoints <= 1 && s.fast_pwm <= 1 &&
         s.park >= s.az_min && s.park <= s.az_max && s.edge_frac >= 1 && s.edge_frac <= 9 && s.coast_ms <= 1000 && s.pos_off >= -5 && s.pos_off <= 4;
}

void Rotor::begin(uint32_t now) {
  hw_pwm(0); hw_relay(0); hw_brake(false);
  if (!settings_ok(s)) defaults();
  set_clamps();
  if (s.pwm_freq > PWM_SAFE_MAX_HZ && !s.fast_pwm) s.pwm_freq = DEF_PWM_FREQ;
  pwm_hz = hw_pwm_freq(s.pwm_freq);
  t_tick = t_idle = rej_window_ms = now;
  saved_pt = pos(); has_saved = true;
}

// ---------------------------------------------------------------- pulse input (interrupt context)
// The reed line is sampled at PULSE_SAMPLE_HZ; a level change counts once it has been seen PULSE_STABLE_N times in a
// row. A change that reverts earlier is chatter or an EMI spike (counted in n_rej). Every accepted level change is
// edge between two contact stretches; the position becomes that edge's place (n.0 or n.frac). The slew drive cannot be
// back-driven, so changes with the motor off are coasting in the last direction.
void Rotor::sample(uint8_t level, uint32_t now_ms) {
  if (level == last_level) {
    if (pend_n) { n_rej++; pend_n = 0; }
    return;
  }
  if (++pend_n < PULSE_STABLE_N) return;
  pend_n = 0;
  uint32_t dt = now_ms - last_edge_ms;
  edge_dt[1] = edge_dt[0]; edge_dt[0] = dt > 65535UL ? 65535 : (uint16_t)dt;
  last_level = level; last_edge_ms = now_ms; n_acc++;
  int8_t d = drive_dir;
  uint8_t coast = 0;
  if (!d) { d = last_dir; n_coast++; coast = 1; }
  if (d > 0) { if (cnt < clamp_hi) { cnt++; pt = edge_t(cnt); } }
  else if (d < 0) { if (cnt > clamp_lo) { pt = edge_t(cnt); cnt--; } }
  if (!coast && pwm_now == pwm_edge && dt < 4000) { dsum[level] += dt; dn[level]++; }   // a rising edge ends a closed stretch
  if (!coast && run_edges < 255) run_edges++;
  pwm_edge = pwm_now;
  log_isr(E_EDGE, now_ms, cnt, (level << 6) | (coast << 7));
}

// ---------------------------------------------------------------- event log
void Rotor::log_isr(uint8_t type, uint32_t now, int16_t c, uint8_t flags) {
  int8_t d = drive_dir ? drive_dir : (flags & 0x80 ? last_dir : 0);
  LogRec &r = logbuf[log_seq % LOG_N];
  if ((int32_t)(now - t_log) < 0) now = t_log;     // main-loop events carry the pass start time: keep the log monotonic
  r.t = now; r.c = c; r.p = pwm_now;
  r.k = type | (d > 0 ? 0x10 : (d < 0 ? 0x20 : 0)) | flags;
  if (++log_seq == LOG_N) log_full = true;
  t_log = now;
}

void Rotor::log_ev(uint8_t type, uint32_t now, int16_t c) {
  ATOMIC_BEGIN log_isr(type, now, c == 0x7FFF ? cnt : c, 0); ATOMIC_END
}

uint8_t Rotor::log_read(uint16_t from, LogRec *out, uint8_t max, uint16_t *first) const {
  uint8_t n = 0;
  ATOMIC_BEGIN
    uint16_t head = log_seq, oldest = log_full ? (uint16_t)(head - LOG_N) : 0;
    if ((uint16_t)(head - from) > (uint16_t)(head - oldest)) from = oldest;   // too old (or from the future)
    *first = from;
    while (n < max && from != head) { out[n++] = logbuf[from % LOG_N]; from++; }
  ATOMIC_END
  return n;
}

static const uint8_t state_event[] = {E_IDLE, E_START, E_RUN, E_SOFT, E_CUT};   // S_IDLE .. S_RELAY_OFF

void Rotor::set_state(uint8_t ns, uint32_t now) {
  st = ns; t_state = now;
  if (ns == S_RUN) slow_logged = false;
  log_ev(state_event[ns], now);
}

// The main loop takes 'now' once per pass; a pulse later in the same pass carries a newer timestamp. Clamp it to
// 'now', otherwise now - last_edge wraps to ~4e9 ms (false jam, "quiet" right after a pulse, early saves).
uint32_t Rotor::last_edge(uint32_t now) const {
  uint32_t le; ATOMIC_BEGIN le = last_edge_ms; ATOMIC_END
  return (int32_t)(now - le) < 0 ? now : le;
}

int16_t Rotor::cnt_copy() const { int16_t c; ATOMIC_BEGIN c = cnt; ATOMIC_END return c; }
int16_t Rotor::pos() const { int16_t p; ATOMIC_BEGIN p = pt; ATOMIC_END return p; }

int16_t Rotor::edge_t(int16_t c) const {
  int16_t n = c >= 0 ? c / 2 : -((1 - c) / 2);     // floor(c / 2)
  return n * 10 + ((c - 2 * n) ? s.edge_frac : 0) + s.pos_off;
}

// the stretch with the contact 'level' (0 = closed = even) nearest to the position t
int16_t Rotor::stretch(int16_t t, uint8_t level) const {
  int16_t u = t - s.pos_off;                       // in the grid's own frame
  int16_t n = u >= 0 ? u / 10 : -((9 - u) / 10);  // floor(u / 10)
  int16_t c = 2 * n + (u - n * 10 >= s.edge_frac ? 1 : 0);
  if ((c & 1) != (level & 1)) c += (t - edge_t(c) < edge_t(c + 1) - t) ? -1 : 1;
  return c;
}

void Rotor::set_raw(int16_t t) {
  int16_t c = stretch(t, last_level);
  ATOMIC_BEGIN pt = t; cnt = c; ATOMIC_END
}

int16_t Rotor::next_edge(int8_t d, int16_t c, int16_t p) const {
  int16_t e;
  if (d > 0) { e = edge_t(c + 1); if (e <= p) e = edge_t(c + 2); }
  else { e = edge_t(c); if (e >= p) e = edge_t(c - 1); }
  return e;
}

// without a speed: half the way to the next edge (0.4 before a wide stretch, 0.1 before the narrow open one) -
// stopping here is then never further from the target than stopping at the next edge
int16_t Rotor::tol(int8_t d) const {
  int16_t c, p; ATOMIC_BEGIN c = cnt; p = pt; ATOMIC_END
  int16_t g = d > 0 ? next_edge(d, c, p) - p : p - next_edge(d, c, p);
  return g > 1 ? g / 2 : 0;
}

// speed over the stretch just crossed (the latest one follows the slow-down best), tenths/s; 0 = unknown
int16_t Rotor::speed_tps(uint32_t now, uint32_t le) const {
  uint16_t a; uint8_t n; int16_t c, p; int8_t d;
  ATOMIC_BEGIN a = edge_dt[0]; n = run_edges; c = cnt; p = pt; d = drive_dir; ATOMIC_END
  if (n < 2 || !a || a > 2000 || now - le > 1500) return 0;
  int16_t len = d > 0 ? p - edge_t(c - 1) : edge_t(c + 2) - p;
  return len <= 0 ? 0 : (int16_t)((uint32_t)len * 1000 / a);
}

// between edges the position is the last edge plus speed x time - but never at the next edge, which has not come yet
int16_t Rotor::pos_est(int8_t d, int16_t v, uint32_t now, uint32_t le) const {
  int16_t c, p; ATOMIC_BEGIN c = cnt; p = pt; ATOMIC_END
  if (!v || !d) return p;
  int16_t g = d > 0 ? next_edge(d, c, p) - p : p - next_edge(d, c, p);
  int32_t x = (int32_t)v * (int32_t)(now - le) / 1000;
  if (x > g - 1) x = g - 1;
  if (x < 0) x = 0;
  return p + d * (int16_t)x;
}

void Rotor::mark_cut(uint32_t now) {
  uint32_t le = last_edge(now);
  int8_t d = drive_dir;
  v_cut = d ? speed_tps(now, le) : 0;
  est_cut = pos_est(d, v_cut, now, le); dir_cut = d; est_pending = d != 0;
}

int16_t Rotor::duty_permille() const {
  uint32_t s1, s0; uint16_t n1, n0;
  ATOMIC_BEGIN s1 = dsum[1]; s0 = dsum[0]; n1 = dn[1]; n0 = dn[0]; ATOMIC_END
  if (n1 < 8 || n0 < 8) return -1;
  uint32_t m1 = s1 / n1, m0 = s0 / n0;
  return (int16_t)(m1 * 1000 / (m1 + m0));
}
int16_t Rotor::az() const { int16_t a = pos() % 3600; return a < 0 ? a + 3600 : a; }

uint8_t Rotor::status() const {
  uint8_t r = 0;
  if (st == S_IDLE && !want) {
    r |= ST_IDLE;
    if (mode == M_TARGET && iabs16(tgt - pos()) < OVERSHOOT_T) r |= ST_POINTING;
  } else {
    r |= ST_MOVING;
  }
  if (errors) r |= ST_ERROR;
  return r;
}

// ---------------------------------------------------------------- commands
bool Rotor::goto_raw(int16_t t) {
  if (s.endpoints && (t < lo() || t > hi())) { errors |= ER_LIMIT; return false; }
  tgt = t; mode = M_TARGET; want = true; corrected = false; t_cmd = t_tick;
  errors &= ~(ER_JAM | ER_TIMEOUT | ER_LIMIT);
  log_ev(E_CMD, t_tick, t);
  return true;
}

bool Rotor::goto_az(int16_t t) {
  t %= 3600; if (t < 0) t += 3600;
  int16_t p = pos(), best = 0; bool found = false;
  for (int8_t k = -1; k <= 1; k++) {                 // the same heading on the previous / this / the next turn
    int16_t c = t + k * 3600;
    if (c < lo() || c > hi()) continue;
    if (!found || iabs16(c - p) < iabs16(best - p)) { best = c; found = true; }
  }
  if (!found) { errors |= ER_LIMIT; return false; }
  return goto_raw(best);
}

void Rotor::manual(int8_t d) {
  mode = M_MANUAL; man_dir = d; want = true; t_cmd = t_tick;   // mdir changes only when the relays do
  errors &= ~(ER_JAM | ER_TIMEOUT | ER_LIMIT);
  log_ev(E_CMD, t_tick, d > 0 ? s.az_max : s.az_min);
}

void Rotor::soft_stop() {
  want = false; mode = M_NONE; holding = false;
  if (st == S_RUN || st == S_RELAY_ON) set_state(S_SOFT, t_tick);
}

void Rotor::stop_now() {
  want = false; mode = M_NONE; holding = false;
  if (pwm_now) mark_cut(t_tick);
  hw_pwm(0); pwm_now = 0; ramp_acc = 0;
  log_ev(E_STOP, t_tick);
  if (st != S_IDLE && st != S_RELAY_OFF) set_state(S_RELAY_OFF, t_tick);
}

void Rotor::set_position(int16_t t) {
  int16_t p = pos();
  if (t >= 0 && t < 3600 && !(errors & ER_NOPOS)) {   // keep the current turn (unknown position: take t as it is)
    int16_t best = t;
    for (int8_t k = -1; k <= 1; k += 2) {
      int16_t c = t + k * 3600;
      if (c < s.az_min - POS_MARGIN_T || c > s.az_max + POS_MARGIN_T) continue;   // only turns inside the range
      if (iabs16(c - p) < iabs16(best - p)) best = c;
    }
    t = best;
  }
  // the edges are physical: the antenna is where pos() says within its stretch, so the calibration moves the whole
  // grid by t - pos(), whole degrees in the counter and the rest in pos_off (a fraction a 0.5 deg grid could not hold)
  int16_t o = (int16_t)((t - p + s.pos_off) % 10);
  if (o < 0) o += 10;
  if (o >= 5) o -= 10;
  if (o != s.pos_off) { s.pos_off = (int8_t)o; settings_dirty = true; }
  set_raw(t);
  errors &= ~(ER_NOPOS | ER_UNCERT);
  calibrated_now = true;
  if (mode == M_TARGET && st == S_IDLE) mode = M_NONE;   // the old target no longer means anything
}

// At slow speed the reed contact stays closed for ~0.77 deg, so "no pulse for 300 ms" alone does not mean stopped:
// wait for twice the last edge intervals (the motor only slows down while coasting) and a minimum coast time.
bool Rotor::coasting(uint32_t now, uint32_t le) const {
  if (!n_acc) return false;
  if (now - t_idle < COAST_MIN_MS) return true;
  uint16_t a, b; ATOMIC_BEGIN a = edge_dt[0]; b = edge_dt[1]; ATOMIC_END
  uint32_t q = 2UL * (a > b ? a : b);
  if (q < QUIET_MS) q = QUIET_MS;
  if (q > QUIET_MAX_MS) q = QUIET_MAX_MS;
  return (now - le) < q;
}

int16_t Rotor::dist_to_limit() const {
  int16_t p = pos();
  return mdir > 0 ? hi() - p : p - lo();
}

uint8_t Rotor::pwm_for_speed(uint32_t v) const {
  static const uint8_t P[SPEED_N] = SPEED_PWM;
  static const uint16_t V[SPEED_N] = SPEED_MDEG;
  uint16_t p;
  if (v <= V[0]) p = P[0];
  else if (v >= V[SPEED_N - 1]) p = 255;
  else {
    uint8_t i = 1;
    while (v > V[i]) i++;
    p = P[i - 1] + (uint16_t)((uint32_t)(P[i] - P[i - 1]) * (v - V[i - 1]) / (V[i] - V[i - 1]));
  }
  if (p > s.pwm_max) p = s.pwm_max;
  if (p < s.pwm_floor) p = s.pwm_floor;
  return p ? p : 1;
}

uint8_t Rotor::cap() const {
  return (speed && speed < s.pwm_max) ? speed : s.pwm_max;
}

// PWM for a remaining distance d inside a slow-down zone z: from min(cap, SLOW_PWM_CAP) down to the floor
static uint8_t slow_pwm(int16_t d, int16_t z, uint8_t c, uint8_t floor) {
  uint8_t top = umin8(c, SLOW_PWM_CAP);
  if (top <= floor) return top;
  if (d <= 0) return floor;
  return floor + (uint8_t)((uint32_t)(top - floor) * (uint32_t)d / (uint32_t)z);
}

bool Rotor::save_due(uint32_t now, bool force) const {
  if (st != S_IDLE || (errors & ER_NOPOS)) return false;   // an unknown position is never stored as if it were valid
  if ((now - last_edge(now)) < SAVE_AFTER_MS) return false;
  return force || !has_saved || pos() != saved_pt || calibrated_now;
}

// ---------------------------------------------------------------- motion state machine
void Rotor::tick(uint32_t now) {
  uint32_t dt = now - t_tick;
  if (dt > 100) dt = 100;
  t_tick = now;
  int16_t p = pos();
  uint32_t le = last_edge(now);

  // contact chatter far beyond normal bounce -> sensor error (latched until RESET)
  if (now - rej_window_ms >= 1000) {
    uint32_t r; ATOMIC_BEGIN r = n_rej; ATOMIC_END
    if ((uint16_t)r - rej_in_window > 200) errors |= ER_SENSOR;
    rej_in_window = (uint16_t)r; rej_window_ms = now;
  }

  if (st != S_IDLE) {
    uint32_t tl; ATOMIC_BEGIN tl = t_log; ATOMIC_END
    if (now - tl >= LOG_SAMPLE_MS) log_ev(E_SAMPLE, now);
  }

  // direction a pending request needs (0 = nothing to do)
  int8_t need = 0;
  if (want) {
    if (mode == M_TARGET) {
      int16_t diff = tgt - p;
      int8_t dd = diff > 0 ? 1 : -1;
      int16_t c, q; ATOMIC_BEGIN c = cnt; q = pt; ATOMIC_END
      int16_t land = iabs16(next_edge(dd, c, q) - q) + SHORT_COAST_T;   // a short move ends about here (no speed yet)
      int16_t ad = iabs16(diff);
      need = (ad <= tol(dd) || ad <= iabs16(land - ad)) ? 0 : dd;      // move only if it ends closer than staying
    } else if (mode == M_MANUAL) {
      int16_t dl = man_dir > 0 ? hi() - p : p - lo();
      need = (s.endpoints && dl <= tol(man_dir)) ? 0 : man_dir;
    }
  }

  switch (st) {
  case S_IDLE:
    if (want) {
      if (!need) { want = false; break; }          // already there
      if (need != last_dir && coasting(now, le)) {   // still coasting the other way
        if (!holding) { n_holds++; holding = true; log_ev(E_HOLD, now); }
        break;
      }
      holding = false; want = false;
      mdir = need;
      hw_brake(true); brake_on = true;
      ATOMIC_BEGIN drive_dir = need; last_dir = need; run_edges = 0; ATOMIC_END
      est_pending = false;
      hw_relay(need);
      ramp_acc = 0; pwm_now = 0;
      set_state(S_RELAY_ON, now);
      break;
    }
    if (est_pending && (now - t_idle) >= COAST_MIN_MS && (now - le) >= QUIET_MS) {   // at rest: where between the edges
      est_pending = false;
      int16_t e = est_cut + dir_cut * (int16_t)((int32_t)v_cut * s.coast_ms / 1000);
      ATOMIC_BEGIN
        int16_t lo_e = edge_t(cnt), hi_e = edge_t(cnt + 1);
        if (e < lo_e) e = lo_e;
        if (e > hi_e) e = hi_e;
        if (pt == lo_e && e == hi_e) e = hi_e - 1;  // still inside: the upper edge has not been crossed
        if (pt == hi_e && e == lo_e) e = lo_e + 1;
        pt = e;
      ATOMIC_END
    }
    if (brake_on && (now - t_idle) >= BRAKE_HOLD_MS) { hw_brake(false); brake_on = false; }
    // one correction move after an overshoot (register a), once the coasting is over
    if (s.overshoot && mode == M_TARGET && !corrected && (now - le) > 500 && (now - t_idle) > 500 &&
        iabs16(tgt - p) >= OVERSHOOT_T) {
      corrected = true; want = true;
    }
    break;

  case S_RELAY_ON:                                 // relay contacts settle without current
    if (now - t_state >= RELAY_SETTLE_MS) { set_state(S_RUN, now); t_move = now; t_cmd = now; }
    break;

  case S_RUN: {
    uint32_t since_pulse = (int32_t)(le - t_move) > 0 ? now - le : now - t_move;
    uint32_t stall_ms = (uint32_t)STALL_NO_PULSE_MS * 253 / (pwm_now > 32 ? pwm_now : 32);   // slow = longer gaps
    if (stall_ms > STALL_MAX_MS) stall_ms = STALL_MAX_MS;
    if (stall_ms < STALL_NO_PULSE_MS) stall_ms = STALL_NO_PULSE_MS;
    if (s.jam && (now - t_move) > STALL_ARM_MS && since_pulse > stall_ms) {
      errors |= ER_JAM; n_stalls++;
      log_ev(E_STALL, now);
      stop_now();                                  // nothing turns, so there is nothing to ramp down
      break;
    }
    if ((now - t_cmd) > (mode == M_MANUAL ? MANUAL_TIMEOUT_MS : MOVE_TIMEOUT_MS)) {
      errors |= ER_TIMEOUT; n_timeouts++;
      log_ev(E_TIMEOUT, now);
      soft_stop();
      break;
    }
    if (want) {
      if (need == mdir) want = false;              // same direction: just follow the new target (tracking)
      else if (need == 0 && mode == M_TARGET) want = false;   // target right here: the target check below stops
      else if (need == 0) { mode = M_NONE; want = false; set_state(S_SOFT, now); break; }  // manual, that end reached
      else { set_state(S_SOFT, now); break; }      // reversal: soft stop first, IDLE starts the other way
    }
    uint8_t c = cap(), w = c;
    int16_t v = speed_tps(now, le);               // known from the 3rd edge of the run: then the cut anticipates
    int16_t pe = pos_est(mdir, v, now, le);       // the position between edges and the coasting after the cut
    int16_t coast = (int16_t)((int32_t)v * s.coast_ms / 1000);
    int16_t close = v ? 0 : tol(mdir);
    if (mode == M_TARGET) {
      int16_t d = (tgt - pe) * mdir - coast;      // remaining distance in the direction of travel, after coasting
      if (d <= close) {                            // reached (or passed): the next edge would not be closer
        if (pwm_now > (uint16_t)s.pwm_floor + SOFT_CUT_MARGIN) {   // still fast (target moved onto us): soft stop,
          want = true;                                             // then come back once the coasting is over
          set_state(S_SOFT, now);
          break;
        }
        mark_cut(now);
        hw_pwm(0); pwm_now = 0; ramp_acc = 0;      // normal case: the slow-down left us at the floor PWM
        set_state(S_RELAY_OFF, now);
        break;
      }
      if (s.slow_zone && d < (int16_t)s.slow_zone) w = umin8(w, slow_pwm(d, s.slow_zone, c, s.pwm_floor));
    } else if (mode != M_MANUAL) {                 // no mode left (cannot happen): stop softly
      set_state(S_SOFT, now);
      break;
    }
    if (s.endpoints) {
      int16_t dl = (mdir > 0 ? hi() - pe : pe - lo()) - coast;
      if (dl <= close) {
        log_ev(E_LIMIT, now);
        mode = M_NONE; want = false;
        if (pwm_now > (uint16_t)s.pwm_floor + SOFT_CUT_MARGIN) { set_state(S_SOFT, now); break; }
        mark_cut(now);
        hw_pwm(0); pwm_now = 0; ramp_acc = 0;
        set_state(S_RELAY_OFF, now);
        break;
      }
      if (dl < LIMIT_ZONE_T) w = umin8(w, slow_pwm(dl, LIMIT_ZONE_T, c, s.pwm_floor));
    }
    uint32_t acc = ramp_acc + dt * 65280UL / s.ramp_up;   // soft start: PWM 0 -> 255 in ramp_up ms
    if (acc > 65280UL) acc = 65280UL;
    ramp_acc = (uint16_t)acc;
    uint8_t r = ramp_acc >> 8;
    if (w < r) { r = w; ramp_acc = (uint16_t)r << 8; }   // distance-based slow-down applies at once
    if (w < c && !slow_logged) { slow_logged = true; log_ev(E_SLOW, now); }
    if (r != pwm_now) { pwm_now = r; hw_pwm(r); }
    break;
  }

  case S_SOFT: {                                   // soft stop: ramp down to the floor, then cut
    uint32_t dec = dt * 65280UL / s.ramp_down;
    ramp_acc = ramp_acc > dec ? ramp_acc - dec : 0;
    uint8_t r = ramp_acc >> 8;
    if (r <= s.pwm_floor) {
      mark_cut(now);
      hw_pwm(0); pwm_now = 0; ramp_acc = 0;
      set_state(S_RELAY_OFF, now);
    } else if (r != pwm_now) {
      pwm_now = r; hw_pwm(r);
    }
    break;
  }

  case S_RELAY_OFF:                                // current is off: release the relay after it decayed
    if (now - t_state >= RELAY_SETTLE_MS) {
      hw_relay(0);
      ATOMIC_BEGIN drive_dir = 0; ATOMIC_END
      set_state(S_IDLE, now); t_idle = now;
    }
    break;
  }
}

// ---------------------------------------------------------------- registers
static bool parse_int(const char *p, int32_t *v) {
  bool neg = false; int32_t x = 0; uint8_t n = 0;
  if (*p == '-') { neg = true; p++; } else if (*p == '+') p++;
  while (*p >= '0' && *p <= '9') { x = x * 10 + (*p++ - '0'); if (++n > 7) return false; }
  if (!n || *p) return false;
  *v = neg ? -x : x;
  return true;
}

int16_t parse_tenths(const char *p, bool *ok) {
  bool neg = false; int32_t x = 0; uint8_t n = 0, dec = 0; int8_t frac = -1, second = 0;
  *ok = false;
  if (*p == '-') { neg = true; p++; } else if (*p == '+') p++;
  for (; *p; p++) {
    if (*p >= '0' && *p <= '9') {
      if (frac < 0) { x = x * 10 + (*p - '0'); if (++n > 5) return 0; }
      else if (dec == 0) { frac = *p - '0'; dec = 1; }
      else if (dec == 1) { second = *p - '0'; dec = 2; }
    } else if (*p == '.' && frac < 0) {
      frac = 0;
    } else {
      return 0;
    }
  }
  if (!n && dec == 0) return 0;
  if (frac < 0) frac = 0;
  x = x * 10 + frac + (second >= 5 ? 1 : 0);
  if (x > 32000) return 0;
  *ok = true;
  return neg ? -x : x;
}

char *fmt_u32(char *out, uint32_t v) {
  char b[11]; uint8_t i = 0;
  do { b[i++] = '0' + v % 10; v /= 10; } while (v);
  while (i) *out++ = b[--i];
  *out = 0;
  return out;
}

char *fmt_tenths(char *out, int16_t v) {
  if (v < 0) { *out++ = '-'; v = -v; }
  out = fmt_u32(out, (uint32_t)(v / 10));
  *out++ = '.'; *out++ = '0' + v % 10; *out = 0;
  return out;
}

bool Rotor::reg_read(char r, char *out) const {
  switch (r) {
  case '0': fmt_u32(out, s.pwm_max); break;
  case '1': fmt_u32(out, s.pwm_floor); break;
  case '2': fmt_u32(out, pwm_hz); break;
  case '3': fmt_tenths(out, s.az_min); break;
  case '4': fmt_tenths(out, s.az_max); break;
  case '5': fmt_u32(out, s.ramp_up); break;
  case '6': fmt_u32(out, s.ramp_down); break;
  case '7': fmt_tenths(out, pos()); break;
  case '8': fmt_tenths(out, s.slow_zone); break;
  case '9': fmt_u32(out, s.report_raw); break;
  case 'a': fmt_u32(out, s.overshoot); break;
  case 'b': fmt_u32(out, s.jam); break;
  case 'c': fmt_u32(out, s.endpoints); break;
  case 'd': fmt_u32(out, 0); break;              // unstick: not implemented
  case 'e': fmt_u32(out, s.fast_pwm); break;
  case 'f': fmt_tenths(out, s.park); break;
  case 'g': fmt_tenths(out, s.edge_frac); break;
  case 'h': fmt_u32(out, s.coast_ms); break;
  default: return false;
  }
  return true;
}

bool Rotor::reg_write(char r, const char *val) {
  bool ok; int32_t v = 0; int16_t t = 0;
  if (moving() && (r == '3' || r == '4' || r == '7' || r == 'c' || r == 'g' || r == 'h')) return false;   // range / position only at rest
  bool angle = (r == '3' || r == '4' || r == '7' || r == '8' || r == 'f' || r == 'g');
  if (angle) { t = parse_tenths(val, &ok); if (!ok) return false; }
  else if (!parse_int(val, &v)) return false;
  bool flag = (v == 0 || v == 1);
  switch (r) {
  case '0': if (v < 1 || v > 255 || v < s.pwm_floor) return false; s.pwm_max = v; break;
  case '1': if (v < 0 || v > 255 || v > s.pwm_max) return false; s.pwm_floor = v; break;
  case '2':
    if (v < 31 || v > 62500) return false;
    if (v > PWM_SAFE_MAX_HZ && !s.fast_pwm) return false;     // Q4 has no gate driver, D2 is a slow 1N4007
    s.pwm_freq = v; pwm_hz = hw_pwm_freq(s.pwm_freq);
    break;
  case '3': if (t < -3600 || t > 0 || s.az_max - t < 3600 || s.park < t) return false; s.az_min = t; set_clamps(); break;
  case '4': if (t < 3600 || t > 7200 || t - s.az_min < 3600 || s.park > t) return false; s.az_max = t; set_clamps(); break;
  case '5': if (v < 100 || v > 10000) return false; s.ramp_up = v; break;
  case '6': if (v < 100 || v > 10000) return false; s.ramp_down = v; break;
  case '7': if (t < s.az_min - POS_MARGIN_T || t > s.az_max + POS_MARGIN_T) return false; set_position(t); return true;
  case '8': if (t < 0 || t > 300) return false; s.slow_zone = t; break;
  case '9': if (!flag) return false; s.report_raw = v; break;
  case 'a': if (!flag) return false; s.overshoot = v; break;
  case 'b': if (!flag) return false; s.jam = v; break;
  case 'c': if (!flag) return false; s.endpoints = v; break;
  case 'e':
    if (!flag) return false;
    s.fast_pwm = v;
    if (!v && s.pwm_freq > PWM_SAFE_MAX_HZ) { s.pwm_freq = DEF_PWM_FREQ; pwm_hz = hw_pwm_freq(s.pwm_freq); }
    break;
  case 'f': if (t < s.az_min || t > s.az_max) return false; s.park = t; break;
  case 'g': if (t < 1 || t > 9) return false; s.edge_frac = t; set_raw(pos()); break;
  case 'h': if (v < 0 || v > 1000) return false; s.coast_ms = v; break;
  default: return false;
  }
  settings_dirty = true;
  return true;
}
