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
  s.fast_pwm = DEF_FAST_PWM; s.park = DEF_PARK;
}

void Rotor::begin(uint32_t now) {
  hw_pwm(0); hw_relay(0); hw_brake(false);
  if (s.pwm_freq > PWM_SAFE_MAX_HZ && !s.fast_pwm) s.pwm_freq = DEF_PWM_FREQ;
  pwm_hz = hw_pwm_freq(s.pwm_freq);
  t_tick = t_idle = rej_window_ms = now;
  saved_cnt = cnt; has_saved = true;
}

// ---------------------------------------------------------------- pulse input (interrupt context)
// The reed line is sampled at PULSE_SAMPLE_HZ; a level change counts once it has been seen PULSE_STABLE_N times in a
// row. A change that reverts earlier is chatter or an EMI spike (counted in n_rej). Every accepted level change is
// half a degree. The slew drive cannot be back-driven, so changes with the motor off are coasting in the last
// direction.
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
  if (!d) { d = last_dir; n_coast++; }
  if (d > 0) { if (cnt < t2c(s.az_max + POS_MARGIN_T)) cnt++; }
  else if (d < 0) { if (cnt > t2c(s.az_min - POS_MARGIN_T)) cnt--; }
}

int16_t Rotor::cnt_copy() const { int16_t c; ATOMIC_BEGIN c = cnt; ATOMIC_END return c; }
int16_t Rotor::pos() const { return cnt_copy() * 5; }
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
  tgt = t; mode = M_TARGET; want = true; corrected = false;
  errors &= ~(ER_JAM | ER_TIMEOUT | ER_LIMIT);
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
  mode = M_MANUAL; mdir = d; want = true;
  errors &= ~(ER_JAM | ER_TIMEOUT | ER_LIMIT);
}

void Rotor::soft_stop() {
  want = false; mode = M_NONE;
  if (st == S_RUN || st == S_RELAY_ON) set_state(S_SOFT, t_tick);
}

void Rotor::stop_now() {
  want = false; mode = M_NONE;
  hw_pwm(0); pwm_now = 0; ramp_acc = 0;
  if (st != S_IDLE && st != S_RELAY_OFF) set_state(S_RELAY_OFF, t_tick);
}

void Rotor::set_position(int16_t t) {
  int16_t p = pos();
  if (t >= 0 && t < 3600) {                          // keep the current turn
    if (iabs16(t + 3600 - p) < iabs16(t - p)) t += 3600;
    else if (iabs16(t - 3600 - p) < iabs16(t - p)) t -= 3600;
  }
  int16_t c = t2c(t);
  ATOMIC_BEGIN cnt = c; ATOMIC_END
  errors &= ~ER_NOPOS;
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

bool Rotor::save_due(uint32_t now) const {
  if (st != S_IDLE) return false;
  uint32_t le; ATOMIC_BEGIN le = last_edge_ms; ATOMIC_END
  if ((now - le) < SAVE_AFTER_MS) return false;
  return !has_saved || cnt_copy() != saved_cnt;
}

// ---------------------------------------------------------------- motion state machine
void Rotor::tick(uint32_t now) {
  uint32_t dt = now - t_tick;
  if (dt > 100) dt = 100;
  t_tick = now;
  int16_t p = pos();
  uint32_t le; ATOMIC_BEGIN le = last_edge_ms; ATOMIC_END

  // contact chatter far beyond normal bounce -> sensor error (latched until RESET)
  if (now - rej_window_ms >= 1000) {
    uint32_t r; ATOMIC_BEGIN r = n_rej; ATOMIC_END
    if ((uint16_t)r - rej_in_window > 200) errors |= ER_SENSOR;
    rej_in_window = (uint16_t)r; rej_window_ms = now;
  }

  // direction a pending request needs (0 = nothing to do)
  int8_t need = 0;
  if (want) {
    if (mode == M_TARGET) {
      int16_t diff = tgt - p;
      need = iabs16(diff) <= TOLERANCE_T ? 0 : (diff > 0 ? 1 : -1);
    } else if (mode == M_MANUAL) {
      int16_t dl = mdir > 0 ? hi() - p : p - lo();
      need = (s.endpoints && dl <= TOLERANCE_T) ? 0 : mdir;
    }
  }

  switch (st) {
  case S_IDLE:
    if (want) {
      if (!need) { want = false; break; }          // already there
      if (need != last_dir && coasting(now, le)) {   // still coasting the other way
        if (!holding) { n_holds++; holding = true; }
        break;
      }
      holding = false; want = false;
      mdir = need;
      hw_brake(true); brake_on = true;
      ATOMIC_BEGIN drive_dir = need; last_dir = need; ATOMIC_END
      hw_relay(need);
      ramp_acc = 0; pwm_now = 0;
      set_state(S_RELAY_ON, now);
      break;
    }
    if (brake_on && (now - t_idle) >= BRAKE_HOLD_MS) { hw_brake(false); brake_on = false; }
    // one correction move after an overshoot (register a), once the coasting is over
    if (s.overshoot && mode == M_TARGET && !corrected && (now - le) > 500 && (now - t_idle) > 500 &&
        iabs16(tgt - p) >= OVERSHOOT_T) {
      corrected = true; want = true;
    }
    break;

  case S_RELAY_ON:                                 // relay contacts settle without current
    if (now - t_state >= RELAY_SETTLE_MS) { set_state(S_RUN, now); t_move = now; }
    break;

  case S_RUN: {
    uint32_t since_pulse = now - (le > t_move ? le : t_move);
    if (s.jam && (now - t_move) > STALL_ARM_MS && since_pulse > STALL_NO_PULSE_MS) {
      errors |= ER_JAM; n_stalls++;
      stop_now();                                  // nothing turns, so there is nothing to ramp down
      break;
    }
    if ((now - t_move) > MOVE_TIMEOUT_MS) {
      errors |= ER_TIMEOUT; n_timeouts++;
      soft_stop();
      break;
    }
    if (want) {
      if (need == mdir) want = false;              // same direction: just follow the new target (tracking)
      else if (need == 0) want = false;            // new target is right here: the target check below stops
      else { set_state(S_SOFT, now); break; }      // reversal: soft stop first, IDLE starts the other way
    }
    uint8_t c = cap(), w = c;
    if (mode == M_TARGET) {
      int16_t d = (tgt - p) * mdir;               // remaining distance in the direction of travel
      if (d <= TOLERANCE_T) {                      // reached (or passed): the slow-down left us at the floor PWM
        hw_pwm(0); pwm_now = 0; ramp_acc = 0;
        set_state(S_RELAY_OFF, now);
        break;
      }
      if (s.slow_zone && d < (int16_t)s.slow_zone) w = umin8(w, slow_pwm(d, s.slow_zone, c, s.pwm_floor));
    } else if (mode != M_MANUAL) {                 // no mode left (cannot happen): stop softly
      set_state(S_SOFT, now);
      break;
    }
    if (s.endpoints) {
      int16_t dl = dist_to_limit();
      if (dl <= TOLERANCE_T) {
        hw_pwm(0); pwm_now = 0; ramp_acc = 0; mode = M_NONE;
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
    if (r != pwm_now) { pwm_now = r; hw_pwm(r); }
    break;
  }

  case S_SOFT: {                                   // soft stop: ramp down to the floor, then cut
    uint32_t dec = dt * 65280UL / s.ramp_down;
    ramp_acc = ramp_acc > dec ? ramp_acc - dec : 0;
    uint8_t r = ramp_acc >> 8;
    if (r <= s.pwm_floor) {
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
  default: return false;
  }
  return true;
}

bool Rotor::reg_write(char r, const char *val) {
  bool ok; int32_t v = 0; int16_t t = 0;
  bool angle = (r == '3' || r == '4' || r == '7' || r == '8' || r == 'f');
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
  case '3': if (t < -3600 || t > 0 || s.az_max - t < 3600) return false; s.az_min = t; break;
  case '4': if (t < 3600 || t > 7200 || t - s.az_min < 3600) return false; s.az_max = t; break;
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
  default: return false;
  }
  settings_dirty = true;
  return true;
}
