// Host tests: firmware core + Easycomm III interpreter against the simulated rotator.
#define SIM_HW_IMPL
#include "sim.h"
#include <cstring>
#include <vector>

Sim *g_sim = nullptr;
static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static bool idle(Sim &s) { return !s.rot.moving() && s.omega == 0; }
static void settle(Sim &s, uint32_t max_ms = 200000) { s.run_until([&] { return idle(s); }, max_ms); s.run(800); }

static void test_goto_basic() {
  printf("goto: 91 -> 120 -> 91.5, soft start, exact stop, no overshoot\n");
  Sim s(91.0);
  s.send("AZ120.0 EL0.0\n");
  uint32_t t0 = s.ms();
  double maxpos = 0;
  s.run_until([&] { if (s.theta > maxpos) maxpos = s.theta; return idle(s) && s.ms() - t0 > 100; }, 60000);
  s.run(800);
  printf("  took %.1f s, true %.2f fw %.1f, max %.2f, max PWM step %d\n", (s.ms() - t0) / 1000.0, s.theta, s.fw_pos(), maxpos, s.max_up_step);
  CHECK(std::fabs(s.fw_pos() - 120.0) <= 0.5, "fw position %.1f", s.fw_pos());
  CHECK(std::fabs(s.theta - 120.0) < 1.0, "true position %.2f", s.theta);
  CHECK(std::fabs(s.fw_pos() - s.theta) < 0.8, "fw %.1f vs true %.2f", s.fw_pos(), s.theta);
  CHECK(maxpos < 121.0, "overshoot to %.2f", maxpos);
  CHECK(s.send("GS\n") == "GS5\r\n", "GS after reaching = idle+pointing, got %s", s.send("GS\n").c_str());
  s.send("AZ91.5\n"); settle(s);
  CHECK(std::fabs(s.fw_pos() - 91.5) <= 0.5, "back to 91.5: %.1f", s.fw_pos());
  CHECK(s.violations == 0, "violations: %d (%s)", s.violations, s.last_violation.c_str());
}

static void test_overlap() {
  printf("overlap: shortest way across north inside -30..390\n");
  {
    Sim s(10.0);
    s.send("AZ350\n"); settle(s);
    printf("  10 -> 350: raw %.1f\n", s.fw_pos());
    CHECK(std::fabs(s.fw_pos() - (-10.0)) <= 0.5, "10 -> 350 should end at raw -10, got %.1f", s.fw_pos());
    CHECK(s.send("AZ\n").rfind("AZ3", 0) == 0 && std::fabs(s.theta + 10.0) <= 0.4, "AZ reply %s (true %.2f)", s.send("AZ\n").c_str(), s.theta);
    CHECK(s.send("AR\n").rfind("AR-", 0) == 0, "AR reply %s", s.send("AR\n").c_str());
    s.send("AZ20\n"); settle(s);
    CHECK(std::fabs(s.fw_pos() - 20.0) <= 0.5, "-10 -> 20: %.1f", s.fw_pos());
    CHECK(s.violations == 0, "violations %d", s.violations);
  }
  {
    Sim s(350.0);
    s.send("AZ10\n"); settle(s);
    printf("  350 -> 10: raw %.1f\n", s.fw_pos());
    CHECK(std::fabs(s.fw_pos() - 370.0) <= 0.5, "350 -> 10 should end at raw 370, got %.1f", s.fw_pos());
    s.send("AZ100\n"); settle(s);                    // from 370 only one candidate: 100 (460 is outside)
    CHECK(std::fabs(s.fw_pos() - 100.0) <= 0.5, "370 -> 100: %.1f", s.fw_pos());
  }
  {
    Sim s(380.0);
    s.send("AZ5\n"); settle(s);                      // 365 is closer than 5
    CHECK(std::fabs(s.fw_pos() - 365.0) <= 0.5, "380 -> 5 should stay on the 2nd turn (365), got %.1f", s.fw_pos());
    s.send("AZ395\n"); s.run(500);                    // outside the range: refused
    CHECK(!s.rot.moving() && (s.rot.errors & ER_LIMIT), "AZ395 must be refused (err %d)", s.rot.errors);
    s.send("AZ385.0\n"); settle(s);                  // explicit raw position in the overlap
    CHECK(std::fabs(s.fw_pos() - 385.0) <= 0.5, "AZ385 raw: %.1f", s.fw_pos());
    CHECK(s.send("GE\n") == "GE0\r\n", "errors cleared by the next good move: %s", s.send("GE\n").c_str());
  }
}

static void test_reversal() {
  printf("reversal: CW to 200, at 150 a new target 100 -> soft stop, wait, CCW\n");
  Sim s(91.0);
  s.send("AZ200\n");
  s.run_until([&] { return s.theta >= 150; }, 60000);
  s.send("AZ100\n");
  settle(s);
  printf("  end raw %.1f true %.2f, holds %u\n", s.fw_pos(), s.theta, s.rot.n_holds);
  CHECK(std::fabs(s.fw_pos() - 100.0) <= 0.5, "end %.1f", s.fw_pos());
  CHECK(std::fabs(s.fw_pos() - s.theta) < 0.8, "fw %.1f vs true %.2f after reversal", s.fw_pos(), s.theta);
  CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
}

static void test_tracking() {
  printf("tracking: target moves 0.8 deg/s, a new AZ every 2 s (the rotor is faster, so it waits between updates)\n");
  Sim s(100.0);
  double tgt = 104;
  int stops = 0; bool was_moving = false; double maxerr = 0;
  for (int i = 0; i < 40; i++) {
    char b[32]; snprintf(b, sizeof b, "AZ%.1f\n", tgt); s.send(b);
    for (int k = 0; k < 2000; k++) {
      s.run(1);
      bool m = s.rot.moving();
      if (was_moving && !m) stops++;
      was_moving = m;
      double now_tgt = tgt + 0.8 * k / 1000.0;
      if (i > 3) maxerr = std::max(maxerr, std::fabs(now_tgt - s.theta));
    }
    tgt += 1.6;
  }
  printf("  stops %d, max lag %.2f deg\n", stops, maxerr);
  CHECK(maxerr < 4.0, "lag %.2f", maxerr);
  CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
}

static void test_stop_and_manual() {
  printf("STOP (SA SE) immediate; ML runs to -30 and stops softly at the range end\n");
  Sim s(200.0);
  s.send("AZ300\n"); s.run(5000);
  CHECK(s.pwm > 200, "running at %d", s.pwm);
  s.allow_hard_cut = true;
  s.send("SA SE \n"); s.run(2);
  CHECK(s.pwm == 0, "STOP must cut the PWM at once (pwm %d)", s.pwm);
  s.allow_hard_cut = false;
  settle(s);
  Sim m(20.0);
  m.send("ML\n");
  settle(m);
  printf("  ML end: raw %.1f true %.2f\n", m.fw_pos(), m.theta);
  CHECK(m.fw_pos() >= -30.5 && m.fw_pos() <= -29.0, "ML end %.1f", m.fw_pos());
  CHECK(m.violations == 0, "violations %d (%s)", m.violations, m.last_violation.c_str());
  m.send("VR5000\n"); m.run(3000);
  CHECK(m.rot.moving() && m.rot.dir() > 0, "VR5000 moves CW");
  m.allow_hard_cut = true; m.send("SA SE \n"); settle(m); m.allow_hard_cut = false;   // Hamlib S during M
  CHECK(!m.rot.moving(), "SA stops a VR run");
  CHECK(m.violations == 0, "violations %d (%s)", m.violations, m.last_violation.c_str());
}

static void test_short_moves() {
  printf("short moves: 1 deg and 0.5 deg steps arrive exactly\n");
  Sim s(150.0);
  double expect = 150;
  for (double d : {1.0, 1.0, -1.0, 0.5, 2.0, -3.0, 5.0}) {
    expect += d;
    char b[32]; snprintf(b, sizeof b, "AZ%.1f\n", expect); s.send(b);
    settle(s);
    CHECK(std::fabs(s.theta - expect) <= 0.5 && std::fabs(s.fw_pos() - s.theta) <= 0.3, "step %+.1f: expect %.1f got fw %.1f true %.2f", d, expect, s.fw_pos(), s.theta);
  }
  CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
}

static void test_jam() {
  printf("jam: rotator blocked -> stall guard stops within ~5 s, GE2\n");
  Sim s(100.0);
  s.jam = true; s.allow_hard_cut = true;
  s.send("AZ200\n");
  bool stopped = s.run_until([&] { return !s.rot.moving(); }, 10000);
  CHECK(stopped && s.ms() < 6000, "stopped %d after %u ms", stopped, s.ms());
  CHECK(s.send("GE\n") == "GE2\r\n", "GE %s", s.send("GE\n").c_str());
  CHECK((s.rot.status() & ST_ERROR), "status error bit");
  s.send("RESET\n");
  CHECK(s.send("GE\n") == "GE0\r\n", "RESET clears errors");
}

static void test_noise() {
  printf("noise: contact chatter + EMI spikes over 2 x 360 deg -> no counting error\n");
  Sim s(-25.0);
  s.chatter_prob = 0.5; s.glitch_per_s = 30;
  s.send("AZ385.0\n"); settle(s);                      // 410 deg CW
  double e1 = s.fw_pos() - s.theta;
  s.send("AZ-20.0\n"); settle(s);                      // 405 deg back
  double e2 = s.fw_pos() - s.theta;
  printf("  errors after 410 deg CW: %+.2f, after 405 deg CCW: %+.2f, accepted %u rejected %u\n", e1, e2, s.rot.n_acc, s.rot.n_rej);
  CHECK(std::fabs(e1) < 0.9 && std::fabs(e2) < 0.9, "counting error %.2f / %.2f", e1, e2);
  CHECK(!(s.rot.errors & ER_SENSOR), "normal chatter must not raise a sensor error");
}

static void test_protocol() {
  printf("protocol: Hamlib EasycommIII strings\n");
  Sim s(91.0);
  CHECK(s.send("AZ\n") == "AZ91.0\r\n", "AZ query: %s", s.send("AZ\n").c_str());
  CHECK(s.send("EL\n") == "EL0.0\r\n", "EL query");
  CHECK(s.send("AZ EL\n") == "AZ91.0 EL0.0\r\n", "combined query: %s", s.send("AZ EL\n").c_str());
  CHECK(s.send("VE\n;") == "VE" FW_VERSION "\r\n", "VE with the trailing ';'");
  CHECK(s.send("GS\n;") == "GS1\r\n", "GS idle: %s", s.send("GS\n;").c_str());
  CHECK(s.send("CR 0\n;") == "CR0,253\r\n", "CR 0: %s", s.send("CR 0\n;").c_str());
  CHECK(s.send("CR2\n") == "CR2,490\r\n", "CR2 (PWM Hz): %s", s.send("CR2\n").c_str());
  CHECK(s.send("CW2,3921\n;") == "" && s.send("CR2\n") == "CR2,490\r\n", "3.9 kHz refused without register e");
  s.send("CW2,980\n"); CHECK(s.hz == 980 && s.send("CR2\n") == "CR2,980\r\n", "980 Hz accepted");
  s.send("CW2,500\n"); CHECK(s.hz == 490, "500 -> 490 Hz step (got %u)", s.hz);
  s.send("CWe,1\n"); s.send("CW2,31372\n"); CHECK(s.hz == 31372, "31 kHz with register e");
  s.send("CWe,0\n"); CHECK(s.hz == 490, "register e off falls back to 490 (got %u)", s.hz);
  s.send("CW7,88.5\n"); CHECK(s.send("AZ\n") == "AZ88.5\r\n", "CW7 sets the position: %s", s.send("AZ\n").c_str());
  CHECK(s.send("CR7\n") == "CR7,88.5\r\n", "CR7");
  s.send("CW9,1\n"); s.send("CW7,-12.5\n"); CHECK(s.send("AZ\n") == "AZ-12.5\r\n", "raw reporting: %s", s.send("AZ\n").c_str());
  s.send("CW9,0\n"); CHECK(s.send("AZ\n") == "AZ347.5\r\n", "normal reporting: %s", s.send("AZ\n").c_str());
  s.send("CW7,200.0\n"); CHECK(s.send("AR\n") == "AR200.0\r\n", "CW7 never picks a turn outside the range (from -12.5: 200, not -160): %s", s.send("AR\n").c_str());
  s.send("CW7,350.0\n"); CHECK(s.send("AR\n") == "AR-10.0\r\n" || s.send("AR\n") == "AR350.0\r\n", "CW7 350 from 200: %s", s.send("AR\n").c_str());
  s.send("CW7,-12.5\n");
  CHECK(s.send("CR3\n") == "CR3,-30.0\r\n" && s.send("CR4\n") == "CR4,390.0\r\n", "range registers");
  CHECK(s.send("CW4,350\n") == "" && s.send("CR4\n") == "CR4,390.0\r\n", "range < 360 refused");
  CHECK(s.send("CRz\n") == "CRz,?\r\n", "unknown register");
  CHECK(s.send("UP000 XXX DN000 XXX\n") == "", "Easycomm I extras ignored");
  CHECK(s.send("garbage!!\r\n") == "", "garbage ignored");
  std::string longline(200, 'A'); CHECK(s.send(longline + "\n") == "" && s.send("AZ\n") == "AZ347.5\r\n", "overlong line dropped");
  CHECK(s.send("PD\n").rfind("PDa", 0) == 0, "PD diagnostics: %s", s.send("PD\n").c_str());
  CHECK(s.send("AZ12.34\n") == "" , "set has no reply");
  s.run(100);
  CHECK(s.rot.target() == 123, "AZ12.34 -> target 12.3 (got %d)", s.rot.target());
  s.allow_hard_cut = true; s.send("SA\n");
}

static void test_settings_limits() {
  printf("registers: speed cap, floor, ramp\n");
  Sim s(50.0);
  s.send("CW0,150\n");
  s.send("AZ90\n"); s.run(8000);
  CHECK(s.pwm <= 150, "max speed register caps PWM (pwm %d)", s.pwm);
  settle(s);
  s.send("CW5,4000\n");                                   // slower ramp
  uint32_t t0 = s.ms(); s.send("AZ120\n");
  s.run_until([&] { return s.pwm >= 140; }, 10000);
  printf("  ramp to 140 with ramp_up 4000: %.2f s\n", (s.ms() - t0) / 1000.0);
  CHECK(s.ms() - t0 > 2000, "ramp register");
  settle(s);
  CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
}

static void test_full_range() {
  printf("full range: -30 -> 390 in one move (timeout must not hit)\n");
  Sim s(-30.0);
  uint32_t t0 = s.ms();
  s.send("AZ390.0\n"); settle(s);
  printf("  -30 -> 390 took %.1f s, end %.1f\n", (s.ms() - t0) / 1000.0, s.fw_pos());
  CHECK(std::fabs(s.fw_pos() - 390.0) <= 0.5 && s.rot.errors == 0, "end %.1f err %d", s.fw_pos(), s.rot.errors);
}

static void test_endurance() {
  printf("endurance: 60 random moves with chatter + 300 EMI spikes/s, then back to the start\n");
  Sim s(91.0);
  s.chatter_prob = 0.7; s.glitch_per_s = 300;
  std::mt19937 r(7);
  double total = 0, maxerr = 0;
  for (int i = 0; i < 60; i++) {
    double t = std::uniform_int_distribution<int>(-290, 3890)(r) / 10.0;
    char b[32]; snprintf(b, sizeof b, "AZ%.1f\n", t);
    if (t < 0 || t > 360) s.send(b); else { snprintf(b, sizeof b, "AZ%.1f\n", t); s.send(b); }
    double p0 = s.theta;
    settle(s);
    total += std::fabs(s.theta - p0);
    maxerr = std::max(maxerr, std::fabs(s.fw_pos() - s.theta));
  }
  s.send("AZ91.0\n"); settle(s);
  printf("  %.0f deg turned, max |fw - true| %.2f, end true %.2f fw %.1f, rejected %u, sensor err %d\n", total, maxerr, s.theta, s.fw_pos(), s.rot.n_rej, !!(s.rot.errors & ER_SENSOR));
  CHECK(maxerr < 0.9, "max error %.2f", maxerr);
  CHECK(std::fabs(s.theta - 91.0) < 0.9, "end %.2f", s.theta);
  CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
}

// read the whole event log through the protocol, like the PC tool does
struct Ev { uint16_t seq; uint32_t t; int c, p, k; };
static size_t g_lg_longest = 0;                     // longest LG reply seen (rotctld limit)
static std::vector<Ev> read_log(Sim &s, uint16_t &next, int *lost = nullptr) {
  std::vector<Ev> v;
  for (int guard = 0; guard < 100; guard++) {
    std::string r = s.send("LG" + std::to_string(next) + "\n");
    g_lg_longest = std::max(g_lg_longest, r.size());
    unsigned first; unsigned long now; int off = 0;
    if (sscanf(r.c_str(), "LG%u@%lu:%n", &first, &now, &off) < 2 || !off) break;
    if (lost && first != next) *lost += (uint16_t)(first - next);
    const char *q = r.c_str() + off; int n = 0;
    uint16_t seq = first;
    Ev e; int used;
    while (sscanf(q, "%u,%d,%d,%d;%n", &e.t, &e.c, &e.p, &e.k, &used) == 4) { e.seq = seq++; v.push_back(e); q += used; n++; }
    next = seq;
    if (!n) break;
  }
  return v;
}

static void test_log() {
  printf("event log: LG through the protocol, one move from 100 to 103 (read during the move)\n");
  Sim s(100.0);
  uint16_t next = 0; int lost = 0;
  std::vector<Ev> all;
  s.send("AZ103\n");
  for (int i = 0; i < 40; i++) { s.run(250); auto v = read_log(s, next, &lost); all.insert(all.end(), v.begin(), v.end()); }
  int edges = 0, coast = 0, samples = 0; std::string seqs;
  for (auto &e : all) {
    int ty = e.k & 15;
    if (ty == E_EDGE) { edges++; if (e.k & 0x80) coast++; }
    else if (ty == E_SAMPLE) samples++;
    else seqs += std::to_string(ty) + " ";
  }
  printf("  %zu events, %d edges (%d coasting), %d samples, others: %s lost %d\n", all.size(), edges, coast, samples, seqs.c_str(), lost);
  CHECK(lost == 0, "events lost while polling every 250 ms");
  CHECK(seqs == "14 2 3 4 6 7 ", "state events CMD START RUN SLOW CUT IDLE, got %s", seqs.c_str());
  CHECK(edges == 5 || edges == 6, "3 deg = 5 or 6 edges (the last one may be left to the coasting), got %d", edges);
  CHECK(samples > 5, "PWM samples while driven: %d", samples);
  bool mono = true; for (size_t i = 1; i < all.size(); i++) if (all[i].t < all[i - 1].t || all[i].seq != (uint16_t)(all[i - 1].seq + 1)) mono = false;
  CHECK(mono, "time and sequence increase");
  CHECK(all.size() && all[0].c == 1030 && (all[0].k & 15) == E_CMD, "first event = command with the target 103.0");
  // overflow: a long move without reading, then ask from 0 -> the oldest kept entries, gap reported
  Sim s2(0.0);
  s2.send("AZ200\n"); settle(s2);
  uint16_t n2 = 0; int lost2 = 0;
  auto v2 = read_log(s2, n2, &lost2);
  printf("  long unread move: %u events written, %zu kept, gap %d\n", s2.rot.log_seq, v2.size(), lost2);
  CHECK(v2.size() == LOG_N && lost2 == (int)s2.rot.log_seq - LOG_N, "ring keeps the last %d", LOG_N);
  CHECK(s2.send("LG" + std::to_string(n2) + "\n").find(':') + 3 == s2.send("LG" + std::to_string(n2) + "\n").size(), "caught up: no records");
}

static void test_review() {
  printf("review findings: opposite manual command during a move, targets at the current position, low speed, long tracking\n");
  for (const char *c : {"ML\n", "VL4900\n"}) {
    Sim s(91.0);
    s.send("AZ200\n");
    s.run_until([&] { return s.theta >= 120; }, 60000);
    s.send(c);
    s.run_until([&] { return !s.rot.moving() && s.omega == 0; }, 160000);
    s.run(1000);
    printf("  AZ200 then %.*s at 120: end true %.2f fw %.1f\n", 6, c, s.theta, s.fw_pos());
    CHECK(s.fw_pos() >= -30.5 && s.fw_pos() <= -29.0 && std::fabs(s.fw_pos() - s.theta) < 0.9, "%.*s during a CW goto reverses and stops at -30 (true %.2f fw %.1f)", 2, c, s.theta, s.fw_pos());
    CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
  }
  {
    Sim s(91.0);
    s.send("AZ200\n");
    s.run_until([&] { return s.theta >= 150; }, 60000);
    s.send("AZ150.2\n");                               // a target right here while running at full PWM
    settle(s);
    printf("  AZ150.2 at full speed: end true %.2f fw %.1f\n", s.theta, s.fw_pos());
    CHECK(s.violations == 0, "no hard cut: violations %d (%s)", s.violations, s.last_violation.c_str());
    CHECK(std::fabs(s.fw_pos() - 150.2) <= 0.5, "ends on 150.2 (fw %.1f)", s.fw_pos());
  }
  {
    Sim s(91.0);
    s.send("AZ200\n"); s.run(6000);
    CHECK(s.send("CW7,10\n") == "" && std::fabs(s.fw_pos() - s.theta) < 0.9, "CW7 refused while moving (fw %.1f true %.2f)", s.fw_pos(), s.theta);
    s.send("CW4,370\n"); CHECK(s.send("CR4\n") == "CR4,390.0\r\n", "CW4 refused while moving");
    settle(s);
    CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
  }
  {
    Sim s(100.0);
    s.send("VR0100\n"); s.run(30000);                  // Hamlib M 16 2: slowest speeds
    printf("  VR0100 30 s: moved %.2f deg, pwm %d, errors %d\n", s.theta - 100, s.pwm, s.rot.errors);
    CHECK(!(s.rot.errors & ER_JAM) && s.rot.moving(), "slow manual run: no false jam (err %d)", s.rot.errors);
    s.send("VR0000\n"); s.run(3000);
    CHECK(s.rot.moving(), "VR0000 = slowest speed (Hamlib M 16 1), not stop");
    s.allow_hard_cut = true; s.send("SA\n"); settle(s);
  }
  {
    Sim s(10.0);                                        // 200 s of tracking with the target always ahead
    double tgt = 12;
    for (int i = 0; i < 100; i++) { char b[32]; snprintf(b, sizeof b, "AZ%.1f\n", tgt); s.send(b); s.run(2000); tgt += 9.0; if (tgt > 350) break; }
    CHECK(!(s.rot.errors & ER_TIMEOUT), "continuous tracking does not hit the move timeout (err %d)", s.rot.errors);
  }
}

static void test_v11() {
  printf("1.1: VL/VR in real deg/s (measured speed curve), LG replies within the rotctld limit\n");
  {
    Sim s(50.0);
    struct { uint32_t v; int pwm; } t[] = {{0, 96}, {2000, 96}, {2400, 104}, {3200, 144}, {3600, 176}, {4000, 214}, {4400, 253}, {9900, 253}};
    for (auto &x : t) CHECK(s.rot.pwm_for_speed(x.v) == x.pwm, "VL%u -> PWM %d (want %d)", x.v, s.rot.pwm_for_speed(x.v), x.pwm);
    s.send("CW0,150\n"); CHECK(s.rot.pwm_for_speed(4000) == 150, "MaxSpeed caps the curve: %d", s.rot.pwm_for_speed(4000));
    s.send("CW1,110\n"); CHECK(s.rot.pwm_for_speed(0) == 110, "the floor lifts the slowest speed: %d", s.rot.pwm_for_speed(0));
  }
  for (uint32_t v : {2000u, 3000u, 4000u}) {               // sim motor = the measured curve (motor.h)
    Sim s(20.0);
    s.send("VR" + std::to_string(v) + "\n"); s.run(6000);
    double p0 = s.theta; s.run(10000);
    double sp = (s.theta - p0) / 10.0;
    printf("  VR%u: %.2f deg/s at PWM %d\n", v, sp, s.pwm);
    CHECK(std::fabs(sp - v / 1000.0) < 0.1 * v / 1000.0, "VR%u runs at %.2f deg/s", v, sp);
    s.allow_hard_cut = true; s.send("SA\n"); settle(s);
    CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
  }
  {
    Sim s(-30.0);
    s.us += 3600ull * 1000000;                           // 1 h uptime: 7-digit timestamps, as on the real controller
    uint16_t next = 0; int lost = 0; size_t n = 0;
    g_lg_longest = 0;
    s.send("AZ390.0\n");
    for (int i = 0; i < 200 && (s.rot.moving() || i < 3); i++) { s.run(1000); n += read_log(s, next, &lost).size(); }
    s.send("AZ-30.0\n");
    for (int i = 0; i < 200 && (s.rot.moving() || i < 3); i++) { s.run(1000); n += read_log(s, next, &lost).size(); }
    printf("  2 x 420 deg: %zu events read, seq %u, lost %d, longest LG reply %zu chars\n", n, s.rot.log_seq, lost, g_lg_longest);
    CHECK(s.rot.log_seq > 1000 && lost == 0 && n == s.rot.log_seq, "all events read (%zu of %u, lost %d)", n, s.rot.log_seq, lost);
    CHECK(g_lg_longest <= LG_REPLY_MAX && g_lg_longest < 127, "LG replies stay within rotctld's 127 chars: %zu", g_lg_longest);
  }
}

static void test_v12() {
  printf("1.2: edge positions (contact closed 0.79 deg of each degree): precision at rest, calibration, register g, duty\n");
  {
    Sim s(150.0);
    std::mt19937 r(3);
    double fw_err = 0, tgt_err = 0, sum_t = 0; int n = 0;
    for (int i = 0; i < 80; i++) {
      double t = std::uniform_int_distribution<int>(1000, 2000)(r) / 10.0;
      char b[32]; snprintf(b, sizeof b, "AZ%.1f\n", t); s.send(b); settle(s);
      fw_err = std::max(fw_err, std::fabs(s.fw_pos() - s.theta));
      tgt_err = std::max(tgt_err, std::fabs(s.theta - t)); sum_t += std::fabs(s.theta - t); n++;
    }
    printf("  80 random gotos (both directions): max |fw - true| %.2f, |true - target| max %.2f mean %.2f\n", fw_err, tgt_err, sum_t / n);
    CHECK(fw_err <= 0.15, "position at rest within 0.15 deg of the truth: %.2f", fw_err);
    CHECK(tgt_err <= 0.45, "landing within 0.45 deg (worst case: a deliberate stay) of the target: %.2f", tgt_err);
    int16_t d = s.rot.duty_permille();
    printf("  PD duty: %s\n", s.send("PD\n").c_str());
    CHECK(d >= 770 && d <= 810, "measured closed share %d permille (sim 790)", d);
  }
  for (double start : {200.3, 200.9}) {                      // closed / open contact at the calibration
    Sim s(start);
    int lvl = s.obs_level;
    s.send("CW7,201.0\n");
    s.send("AZ205\n"); settle(s);
    double off = s.fw_pos() - s.theta, exp_off = 201.0 - start;
    printf("  CW7 at true %.1f (contact %s): fw-true after a move %.2f (calibration offset %.2f)\n", start, lvl ? "open" : "closed", off, exp_off);
    CHECK(std::fabs(off - exp_off) <= 0.25, "calibration keeps its offset (%.2f vs %.2f)", off, exp_off);
    CHECK(s.rot.errors == 0, "errors %d", s.rot.errors);
  }
  {
    Sim s(100.0);
    CHECK(s.send("CR g\n") == "CRg,0.8\r\n", "register g default: %s", s.send("CR g\n").c_str());
    s.send("CWg,0.5\n"); CHECK(s.send("CRg\n") == "CRg,0.5\r\n", "CWg,0.5 stored");
    s.send("CWg,1.5\n"); CHECK(s.send("CRg\n") == "CRg,0.5\r\n", "CWg,1.5 refused");
  }
}

static void test_v15() {
  printf("1.5: AR / AZ while moving and coasting follow the true position (interpolated), edge value for comparison\n");
  Sim s(100.0);
  double max_live = 0, max_edge = 0, sum_live = 0, sum_edge = 0, back = 0; int n = 0;
  for (double tgt : {160.0, 105.0, 107.0, 140.3, 139.1}) {
    char b[32]; snprintf(b, sizeof b, "AZ%.1f\n", tgt); s.send(b);
    double last = -1e9; int dirn = tgt > s.theta ? 1 : -1;
    for (int i = 0; i < 1200; i++) {
      s.run(100);
      std::string r = s.send("AR\n");
      double live = atof(r.c_str() + 2), edge = s.rot.pos() / 10.0;
      bool speed_known = s.rot.run_edges >= 2 || !s.rot.driving();
      if ((s.rot.moving() || s.omega != 0) && speed_known) {
        max_live = std::max(max_live, std::fabs(live - s.theta)); max_edge = std::max(max_edge, std::fabs(edge - s.theta));
        sum_live += std::fabs(live - s.theta); sum_edge += std::fabs(edge - s.theta); n++;
      }
      if ((s.rot.moving() || s.omega != 0) && last > -1e8) back = std::max(back, (last - live) * dirn);
      last = live;
      if (!s.rot.moving() && s.omega == 0 && i > 30) break;
    }
    s.run(2500);
  }
  printf("  %d samples while moving (speed known): |AR - true| mean %.2f max %.2f (edge only: mean %.2f max %.2f), largest step back %.2f\n",
         n, sum_live / n, max_live, sum_edge / n, max_edge, back);
  CHECK(sum_live / n <= 0.5 * sum_edge / n && max_live <= 0.4, "live position follows the truth (mean %.2f max %.2f)", sum_live / n, max_live);
  CHECK(back <= 0.15, "live position never jumps back against the travel by more than 0.15 (%.2f)", back);
  CHECK(s.violations == 0, "violations %d (%s)", s.violations, s.last_violation.c_str());
}

int main() {
  test_v15();
  test_v12();
  test_v11();
  test_protocol();
  test_log();
  test_goto_basic();
  test_overlap();
  test_reversal();
  test_tracking();
  test_stop_and_manual();
  test_short_moves();
  test_jam();
  test_noise();
  test_settings_limits();
  test_full_range();
  test_endurance();
  test_review();
  printf("\n%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
