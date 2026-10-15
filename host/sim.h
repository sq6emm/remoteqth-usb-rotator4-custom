// Host simulation of the controller hardware + SPID RAU: motor with inertia and stiction, reed contact with
// 77 % closed duty per degree, contact chatter and EMI spikes, the 2 kHz sampling interrupt, and checks of the hardware rules the firmware must keep.
#pragma once
#include "motor.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <random>
#include "../sq6emm_rotor/core.h"
#include "../sq6emm_rotor/proto.h"

struct Sim;
extern Sim *g_sim;

struct Sim {
  Rotor rot;
  Proto proto{rot};
  // time
  uint64_t us = 0;
  uint32_t ms() const { return (uint32_t)(us / 1000); }
  // motor / outputs
  double theta = 0, omega = 0;        // true antenna azimuth (deg, continuous) and speed (deg/s)
  int8_t relay = 0; uint8_t pwm = 0; bool brake = false; uint16_t hz = 490;
  bool jam = false;
  double u_start = 0.24, tau = 0.30, tau_coast = 0.08;   // speed per PWM: motor.h (measured)
  // contact
  double duty = 0.77;
  double chatter_prob = 0.0;           // probability of chatter after a real edge
  double glitch_per_s = 0.0;           // EMI spikes per second (20 us)
  uint64_t chatter_until = 0, next_flip = 0; bool chatter_flip = false;
  uint64_t glitch_until = 0;
  uint8_t obs_level = 1;
  uint64_t next_sample = 0;
  // rule checks
  int violations = 0; std::string last_violation;
  int max_up_step = 0; uint8_t last_pwm = 0; uint64_t last_pwm_us = 0;
  bool allow_hard_cut = false;         // set around explicit STOP
  uint64_t hard_cut_until = 0;
  bool dir_mismatch = false;         // an "SA" (explicit STOP) was received just now
  std::string rxline;
  std::mt19937 rng{12345};
  double rnd() { return std::uniform_real_distribution<double>(0, 1)(rng); }

  void violate(const char *m) {
    violations++; last_violation = m;
    fprintf(stderr, "  VIOLATION at %.3f s: %s (theta %.2f omega %.2f pwm %d relay %d)\n", us / 1e6, m, theta, omega, pwm, relay);
  }

  Sim(double start_deg) {
    g_sim = this;
    theta = start_deg;
    rot.defaults();
    rot.cnt = (int16_t)lround(start_deg * 2);
    obs_level = rot.last_level = base_level();
    rot.begin(ms());
  }

  uint8_t base_level() const {          // contact closed (line low) for the first 77 % of each degree
    double f = theta - floor(theta);
    return f < duty ? 0 : 1;
  }

  void step_physics(double dt) {
    double u = pwm / 255.0;
    if (jam) { omega = 0; return; }
    if (relay && pwm) {
      if (std::fabs(omega) < 0.02 && u < u_start) { omega = 0; return; }   // stiction
      double target = relay * motor_speed(pwm);
      omega += (target - omega) * dt / tau;
    } else {
      omega -= omega * dt / tau_coast;
      if (std::fabs(omega) < 0.01) omega = 0;
    }
    theta += omega * dt;
  }

  void step(uint64_t dt_us) {
    us += dt_us;
    uint8_t before = base_level();
    step_physics(dt_us / 1e6);
    uint8_t now_base = base_level();
    if (now_base != before && chatter_prob > 0 && rnd() < chatter_prob) {
      chatter_until = us + 200 + (uint64_t)(rnd() * 1300); next_flip = us + 50; chatter_flip = false;
    }
    if (us < chatter_until && us >= next_flip) { chatter_flip = !chatter_flip; next_flip = us + 60 + (uint64_t)(rnd() * 240); }
    if (us >= chatter_until) chatter_flip = false;
    if (glitch_per_s > 0 && rnd() < glitch_per_s * dt_us / 1e6) glitch_until = us + 20;
    uint8_t lvl = now_base ^ (chatter_flip ? 1 : 0) ^ (us < glitch_until ? 1 : 0);
    obs_level = lvl;
    if (us >= next_sample) { next_sample += 1000000 / PULSE_SAMPLE_HZ; rot.sample(lvl, ms()); }   // Timer1 ISR
  }

  // run for t milliseconds: physics every 50 us, firmware loop every 1 ms
  void run(uint32_t t_ms) {
    for (uint32_t i = 0; i < t_ms; i++) {
      uint32_t now = ms();                 // like loop(): 'now' is taken first, pulses arrive while the pass runs
      for (int k = 0; k < 20; k++) step(50);
      rot.tick(now);
      if (relay && rot.dir() != relay) { if (!dir_mismatch) violate("core direction differs from the energised relay"); dir_mismatch = true; }
      if (rot.save_due(now, false)) rot.saved(rot.cnt_copy());
    }
  }
  template <class F> bool run_until(F cond, uint32_t max_ms) {
    for (uint32_t i = 0; i < max_ms; i++) { run(1); if (cond()) return true; }
    return false;
  }

  const char *feed(char c) {
    if (c == '\r' || c == '\n') {
      if (rxline.find("SA") != std::string::npos) hard_cut_until = us + 50000;
      rxline.clear();
    } else rxline += c;
    return proto.feed(c, ms());
  }
  std::string send(const std::string &s) {
    std::string out;
    for (char c : s) { const char *r = feed(c); if (r) out += r; }
    return out;
  }

  double fw_pos() const { return rot.pos() / 10.0; }
};

// ---- hardware hooks used by the core (defined once: #define SIM_HW_IMPL before including) ----
#ifdef SIM_HW_IMPL
void hw_relay(int8_t d) {
  Sim &s = *g_sim;
  if (d != s.relay) {
    if (s.pwm) s.violate("relay switched with motor current");
    if (d && d != s.relay && std::fabs(s.omega) > 0.05 && (s.omega > 0) != (d > 0)) s.violate("reverse while the motor still turns");
  }
  s.relay = d;
}
void hw_pwm(uint8_t d) {
  Sim &s = *g_sim;
  if (d > s.last_pwm) {
    uint64_t dt = s.us - s.last_pwm_us;
    int allowed = (int)(dt / 1000.0 * 255.0 / s.rot.s.ramp_up) + 2;
    if (d - s.last_pwm > allowed) s.violate("PWM jump (no soft start)");
    if (d - s.last_pwm > s.max_up_step) s.max_up_step = d - s.last_pwm;
  }
  if (d == 0 && s.last_pwm > 110 && !s.allow_hard_cut && s.us >= s.hard_cut_until) s.violate("hard cut from high PWM (no soft stop)");
  if (d && !s.relay) s.violate("PWM without a relay");
  s.pwm = d; s.last_pwm = d; s.last_pwm_us = s.us;
}
void hw_brake(bool r) { g_sim->brake = r; }
uint16_t hw_pwm_freq(uint16_t hz) {
  static const uint16_t steps[] = {30, 61, 122, 245, 490, 980, 1953, 3921, 7812, 31372, 62500};
  uint16_t r = 30;
  for (uint16_t s : steps) if (s <= hz) r = s;
  g_sim->hz = r;
  return r;
}
#endif
