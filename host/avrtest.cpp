// AVR-level test: the compiled firmware (build/sq6emm_rotor.ino.elf) runs in simavr (ATmega328P, 16 MHz).
// A motor model is wired to the real pins: reed pulses into PD2, PWM measured on PB3 (OC2A), relays on PB0/PB1,
// brake on PB2, buttons on PC3..PC5, encoder voltages on ADC6/ADC2, HD44780 LCD on PD4..PD7/PB4/PB5, UART for
// Easycomm III. Checks what the host simulator cannot: register setup, interrupts, timing, stack, EEPROM, LCD.
#include "motor.h"
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_adc.h>
#include <simavr/avr_eeprom.h>
extern "C" {
#include <simavr/parts/hd44780.h>
}
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <functional>

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } \
                           else { printf("ok   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Bench {
  avr_t *avr = nullptr;
  elf_firmware_t fw;
  hd44780_t lcd;
  // motor
  double theta = 0, omega = 0, duty = 0;
  int relay_cw = 0, relay_ccw = 0, brake = 0;
  // PWM measurement on PB3
  int pwm_level = 0; uint64_t pwm_last = 0, pwm_high = 0, step_start = 0;
  uint64_t pwm_rises = 0, last_pwm_high_cycle = 0;
  // serial
  std::string rx; uint64_t rx_last_cycle = 0;
  // rule checks
  int violations = 0; std::string last_violation;
  uint16_t sp_min = 0xFFFF;
  int resets = 0; uint16_t last_sp = 0;
  uint8_t last_pd2 = 2;

  static double ms_of(uint64_t c) { return c / 16000.0; }
  double now_ms() const { return ms_of(avr->cycle); }

  void violate(const std::string &m) { violations++; last_violation = m; printf("  VIOLATION %.1f ms: %s\n", now_ms(), m.c_str()); }

  struct PinRef { Bench *b; int pin; } pinref[4];
  static void on_pin(avr_irq_t *, uint32_t v, void *p) {
    Bench &b = *((PinRef *)p)->b;
    int pin = ((PinRef *)p)->pin;
    if (pin == 3) {                                   // PB3 = PWM
      if (b.pwm_level) b.pwm_high += b.avr->cycle - b.pwm_last;
      if (v && !b.pwm_level) b.pwm_rises++;
      b.pwm_level = v; b.pwm_last = b.avr->cycle;
      if (v) b.last_pwm_high_cycle = b.avr->cycle;
      return;
    }
    if (pin == 0 || pin == 1) {                       // relays
      int &r = pin == 0 ? b.relay_cw : b.relay_ccw;
      if ((int)v != r) {
        bool current = b.pwm_on() || (b.avr->cycle - b.last_pwm_high_cycle) < 16000;   // PWM active now or in the last 1 ms
        if (current) b.violate(std::string("relay ") + (pin ? "CCW" : "CW") + " switched with PWM active");
      }
      r = v;
      if (b.relay_cw && b.relay_ccw) b.violate("both relays on");
      return;
    }
    if (pin == 2) b.brake = v;
  }
  static void on_uart(avr_irq_t *, uint32_t v, void *p) {
    Bench &b = *(Bench *)p; b.rx += (char)v; b.rx_last_cycle = b.avr->cycle;
  }

  // simavr does not drive OC2A in PWM modes: duty and frequency come from the Timer2 registers the firmware writes
  bool pwm_on() const { return (avr->data[0xB0] & 0x80) && avr->data[0xB3]; }          // COM2A1 connected, OCR2A > 0
  double reg_duty() const {
    if (avr->data[0xB0] & 0x80) return avr->data[0xB3] / 255.0;
    return (avr->data[0x25] >> 3) & 1;                                                   // PB3 as a plain output
  }
  double reg_hz() const {
    static const int presc[8] = {0, 1, 8, 32, 64, 128, 256, 1024};
    int cs = avr->data[0xB1] & 7; if (!cs) return 0;
    bool fast = (avr->data[0xB0] & 3) == 3;
    return 16e6 / presc[cs] / (fast ? 256 : 510);
  }
  uint8_t contact() const { double f = theta - floor(theta); return f < 0.79 ? 0 : 1; }   // contact closed 0.79 deg of each degree (measured)

  void boot(const std::vector<uint8_t> &eeprom, double start_deg) {
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware("build/sq6emm_rotor.ino.elf", &fw)) { printf("cannot read the ELF\n"); exit(2); }
    strcpy(fw.mmcu, "atmega328p"); fw.frequency = 16000000;
    avr = avr_make_mcu_by_name(fw.mmcu);
    avr_init(avr);
    avr_load_firmware(avr, &fw);
    avr->vcc = avr->avcc = 5000; avr->aref = 2500;
    uint32_t f = 0; avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &f); f &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &f);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), on_uart, this);
    for (int pin : {0, 1, 2, 3}) {
      pinref[pin] = {this, pin};
      avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), pin), on_pin, &pinref[pin]);
    }
    hd44780_init(avr, &lcd, 16, 2);
    for (int i = 0; i < 4; i++) avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('D'), 4 + i), lcd.irq + IRQ_HD44780_D4 + i);
    avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 4), lcd.irq + IRQ_HD44780_RS);
    avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 5), lcd.irq + IRQ_HD44780_E);
    if (!eeprom.empty()) {
      avr_eeprom_desc_t d; d.ee = (uint8_t *)eeprom.data(); d.offset = 0; d.size = eeprom.size();
      avr_ioctl(avr, AVR_IOCTL_EEPROM_SET, &d);
    }
    for (int pin : {3, 4, 5}) button(pin, 1);
    adc(6, 2500); adc(2, 2500);                      // encoder contacts open: pull-ups = AREF -> 1023
    theta = start_deg;
    set_pd2();
  }
  void button(int pc, int level) { drive('C', pc, level); }
  void adc(int ch, uint32_t mv) { avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0 + ch), mv); }
  // external drive: without it simavr re-applies the pull-up whenever the firmware writes PORTx (the LCD on PORTD)
  void drive(char port, int pin, int level) {
    uint8_t &mask = ext_mask[port - 'B'], &val = ext_val[port - 'B'];
    mask |= 1 << pin; if (level) val |= 1 << pin; else val &= ~(1 << pin);
    avr_ioport_external_t e; e.name = port; e.mask = mask; e.value = val;
    avr_ioctl(avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(port), &e);
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(port), pin), level);
  }
  uint8_t ext_mask[3] = {0, 0, 0}, ext_val[3] = {0, 0, 0};
  void reapply_inputs() {
    for (char port : {'C', 'D'}) for (int pin = 0; pin < 8; pin++)
      if (ext_mask[port - 'B'] & (1 << pin)) {     // toggle: simavr skips raising an IRQ to its cached value
        int v = (ext_val[port - 'B'] >> pin) & 1;
        drive(port, pin, !v); drive(port, pin, v);
      }
  }
  void set_pd2() {
    uint8_t c = contact();
    if (c != last_pd2) { drive('D', 2, c); last_pd2 = c; }
  }

  // motor model, every 50 us of simulated time
  void physics() {
    uint64_t now = avr->cycle, span = now - step_start;
    if (!span) return;
    step_start = now;
    double frac = reg_duty();
    pwm_level = pwm_on();
    if (pwm_level) last_pwm_high_cycle = now;
    double dt = span / 16e6;
    duty += (frac - duty) * std::min(1.0, dt / 0.002);      // motor inductance smooths the PWM
    int rel = relay_cw ? 1 : (relay_ccw ? -1 : 0);
    if (rel && duty > 0.01) {
      if (std::fabs(omega) < 0.02 && duty < 0.24) omega = 0;
      else { double tgt = rel * motor_speed((uint8_t)(duty * 255 + 0.5)); omega += (tgt - omega) * dt / 0.10; }
    } else {
      { double nw = omega - (omega / 0.5 + (omega > 0 ? 6.0 : -6.0)) * dt; omega = (nw > 0) != (omega > 0) || std::fabs(nw) < 0.02 ? 0 : nw; }   // coasting as host/sim.h
    }
    theta += omega * dt;
    set_pd2();
  }

  void run_ms(double ms) {
    uint64_t end = avr->cycle + (uint64_t)(ms * 16000), next_phys = avr->cycle + 800;
    while (avr->cycle < end) {
      int st = avr_run(avr);
      if (st == cpu_Done || st == cpu_Crashed) { printf("CPU stopped (%d) at %.1f ms\n", st, now_ms()); exit(3); }
      uint16_t sp = avr->data[R_SPL] | (avr->data[R_SPH] << 8);
      if (sp < sp_min && avr->cycle > 16000) sp_min = sp;
      if (sp == 0x8FF && last_sp < 0x8F0) resets++;   // the C startup code sets SP = RAMEND: a (re)boot
      last_sp = sp;
      if (avr->cycle >= next_phys) { physics(); next_phys += 800; }
    }
  }
  template <class F> bool run_until(F cond, double max_ms) {
    for (double t = 0; t < max_ms; t += 10) { run_ms(10); if (cond()) return true; }
    return false;
  }

  // send a line at 9600 baud, return the reply line (or "" after timeout); *lat = ms from the last sent char to the reply
  std::string cmd(const std::string &s, double *lat = nullptr, double timeout_ms = 300) {
    rx.clear();
    for (char c : s + "\r") { avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), (uint8_t)c); run_ms(1.05); }
    uint64_t sent = avr->cycle;
    run_until([&] { return rx.find('\n') != std::string::npos; }, timeout_ms);
    if (lat) *lat = rx.find('\n') != std::string::npos ? ms_of(rx_last_cycle - sent) : -1;
    std::string r = rx; while (!r.empty() && (r.back() == '\n' || r.back() == '\r')) r.pop_back();
    return r;
  }
  double fw_az() { std::string r = cmd("AZ"); return r.size() > 2 ? atof(r.c_str() + 2) : -999; }
  double fw_raw() { std::string r = cmd("AR"); return r.size() > 2 ? atof(r.c_str() + 2) : -999; }
  bool idle() { return cmd("GS") == "GS5" || cmd("GS") == "GS1"; }
  std::string lcd_line(int row) { std::string s((const char *)lcd.vram + (row ? 0x40 : 0), 16); for (auto &c : s) if (c == (char)0xDF) c = '*'; return s; }
  // newest valid position slot of the EEPROM ring (256.., 128 x 5 bytes): returns the count, *flags
  int ring_newest(int *flags) {
    int best = -1; uint8_t bseq = 0; int bc = 0;
    for (int i = 0; i < 128; i++) {
      uint8_t b[5]; for (int k = 0; k < 5; k++) b[k] = ee(256 + i * 5 + k);
      if (b[0] == 0xFF && b[1] == 0xFF && b[2] == 0xFF && b[3] == 0xFF) continue;
      if (crc8(b, 4) != b[4]) continue;
      if (best < 0 || (int8_t)(b[2] - bseq) > 0) { best = i; bseq = b[2]; bc = (int16_t)(b[0] | (b[1] << 8)); *flags = b[3]; }
    }
    return best < 0 ? -9999 : bc;
  }
  static uint8_t crc8(const uint8_t *p, int n) {
    uint8_t c = 0x5A;
    while (n--) { c ^= *p++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (c << 1) ^ 0x07 : c << 1; }
    return c;
  }
  uint8_t ee(int addr) { avr_eeprom_desc_t d; d.offset = 0; d.size = 0; d.ee = nullptr; avr_ioctl(avr, AVR_IOCTL_EEPROM_GET, &d); return d.ee[addr]; }
  double pwm_hz(double window_ms) { run_ms(window_ms); return pwm_on() ? reg_hz() : 0; }
};

static std::vector<uint8_t> k3ng_eeprom(float az) {
  std::vector<uint8_t> e(1024, 0xFF);
  e[0] = 111;                                         // K3NG magic
  memcpy(&e[9], &az, 4);                             // config_t.last_azimuth (offset verified with avr-gcc)
  return e;
}

int main() {
  printf("== boot after the K3NG firmware (EEPROM magic 111, last_azimuth 91.0)\n");
  Bench b; b.boot(k3ng_eeprom(91.0f), 91.0);
  b.run_ms(1500);
  CHECK(b.lcd_line(0).find("Az") == 0, "LCD line 1 after the splash: '%s'", b.lcd_line(0).c_str());
  double lat;
  std::string r = b.cmd("AZ", &lat);
  CHECK(r == "AZ91.0", "position taken over from K3NG: %s (reply %.1f ms after the command)", r.c_str(), lat);
  CHECK(lat >= 0 && lat < 50, "reply latency %.1f ms (Hamlib timeout 200 ms)", lat);
  CHECK(b.cmd("VE") == "VESQ6EMM-rotor-1.3", "VE: %s", b.cmd("VE").c_str());
  CHECK(b.cmd("GE") == "GE0", "no errors: %s", b.cmd("GE").c_str());
  CHECK(b.cmd("CR2") == "CR2,490", "PWM register: %s", b.cmd("CR2").c_str());
  CHECK(b.cmd("IP") == "IP8" || b.cmd("IP") == "IP0", "IP inputs (no button pressed): %s", b.cmd("IP").c_str());
  CHECK(b.cmd("AN") == "AN1023,1023", "AN encoder ADC with AREF 2.5 V: %s", b.cmd("AN").c_str());
  uint8_t wdt = b.avr->data[0x60];
  CHECK(wdt & 0x08, "watchdog enabled (WDTCSR 0x%02x)", wdt);
  CHECK(b.relay_cw == 0 && b.relay_ccw == 0 && b.pwm_level == 0, "outputs off at rest");

  printf("== move 91 -> 120 at 490 Hz\n");
  b.cmd("AZ120.0");
  b.run_ms(4000);
  double hz = b.pwm_hz(1000);
  CHECK(b.relay_cw == 1 && b.brake == 1, "CW relay and brake release on during the move");
  CHECK(std::fabs(hz - 490) < 5, "PWM frequency on PB3: %.0f Hz", hz);
  CHECK(b.run_until([&] { return b.idle(); }, 30000), "move finished at %.1f s", b.now_ms() / 1000);
  b.run_ms(1500);
  printf("  true %.2f fw %.1f, LCD '%s' / '%s'\n", b.theta, b.fw_az(), b.lcd_line(0).c_str(), b.lcd_line(1).c_str());
  CHECK(std::fabs(b.theta - 120.0) <= 0.4 && std::fabs(b.fw_az() - b.theta) <= 0.2, "arrived: fw %.1f true %.2f", b.fw_az(), b.theta);
  char azs[12]; snprintf(azs, sizeof azs, "%.1f", b.fw_az());
  CHECK(b.lcd_line(0).find(azs) != std::string::npos && b.lcd_line(1).find("On target") == 0, "LCD shows the position %s and 'On target'", azs);
  int fl = -1; int pos_t = b.ring_newest(&fl); float mirror; uint8_t m[4] = {b.ee(9), b.ee(10), b.ee(11), b.ee(12)}; memcpy(&mirror, m, 4);
  CHECK(pos_t == (int)lround(b.fw_az() * 10) && fl == 2 && std::fabs(mirror - pos_t / 10.0f) < 0.01f,
        "EEPROM: newest ring slot %d tenths (flags %d = tenths), K3NG mirror %.1f", pos_t, fl, mirror);
  b.run_ms(2000);
  CHECK(b.brake == 0, "brake engaged again after the hold time");

  printf("== 980 Hz, then 3.9 kHz refused\n");
  b.cmd("CW2,980");
  b.cmd("AZ130.0"); b.run_ms(2000);
  hz = b.pwm_hz(300);
  CHECK(std::fabs(hz - 980) < 10, "PWM at 980 Hz setting: %.0f Hz", hz);
  b.run_until([&] { return b.idle(); }, 20000);
  b.cmd("CW2,3921");
  CHECK(b.cmd("CR2") == "CR2,980", "3.9 kHz refused without register e: %s", b.cmd("CR2").c_str());
  b.cmd("CW2,490");

  printf("== across north: 124 -> 350 goes CCW to raw -10\n");
  b.cmd("AZ350");
  CHECK(b.run_until([&] { return b.idle(); }, 60000), "move finished");
  printf("  GS %s GE %s PD %s LCD '%s'\n", b.cmd("GS").c_str(), b.cmd("GE").c_str(), b.cmd("PD").c_str(), b.lcd_line(1).c_str());
  b.run_ms(1500);
  CHECK(std::fabs(b.fw_raw() + 10.0) <= 0.5, "AR %.1f (true %.2f)", b.fw_raw(), b.theta);
  CHECK(b.lcd_line(0).find("<ovl") != std::string::npos, "LCD overlap marker: '%s'", b.lcd_line(0).c_str());

  printf("== LG reply (long, non-blocking output) during a move\n");
  b.cmd("AZ200"); b.run_ms(3000);
  double lg_lat; std::string lg = b.cmd("LG0", &lg_lat);
  int recs = 0; for (char c : lg) recs += c == ';';
  CHECK(lg.rfind("LG", 0) == 0 && recs >= 3 && lg.size() + 2 <= 118 && lg.back() == '+',
        "LG0 during a move: %d records + '+', %zu chars + CR LF (<= 118 = LG_REPLY_MAX), complete %.0f ms after the command", recs, lg.size(), lg_lat);
  b.run_until([&] { return b.idle(); }, 60000); b.run_ms(1500);
  CHECK(std::fabs(b.fw_az() - 200.0) <= 0.5 && std::fabs(b.theta - 200.0) < 1.0, "move with an LG poll ends on target: fw %.1f true %.2f", b.fw_az(), b.theta);

  printf("== reset (DTR / watchdog / brown-out) during a move\n");
  b.cmd("AZ230"); b.run_ms(4000);
  int fl2 = -1; int during = b.ring_newest(&fl2);
  CHECK(fl2 == 3 && std::abs(during - 2000) <= 3, "MOVING slot written at the start: %d tenths, flags %d (moving + tenths)", during, fl2);
  double true_at_reset = b.theta;
  avr_reset(b.avr);
  b.reapply_inputs();                                // simavr clears the external pin levels on reset
  b.run_ms(1200);
  printf("  reset at true %.2f, after reboot AZ %s GE %s LCD '%s'\n", true_at_reset, b.cmd("AZ").c_str(), b.cmd("GE").c_str(), b.lcd_line(1).c_str());
  CHECK(b.cmd("GE") == "GE64", "reboot after a reset during a move -> GE64 (position uncertain): %s", b.cmd("GE").c_str());
  CHECK(b.lcd_line(1).find("Pos uncertain") == 0, "LCD: '%s'", b.lcd_line(1).c_str());
  CHECK(!b.relay_cw && !b.relay_ccw && !b.pwm_on(), "nothing moves by itself after the reset");
  b.cmd("RESET");
  CHECK(b.cmd("GE") == "GE64", "RESET does not clear the position warning");
  char cal[32]; snprintf(cal, sizeof cal, "CW7,%.1f", std::round(b.theta * 2) / 2); b.cmd(cal);
  CHECK(b.cmd("GE") == "GE0", "CW7 clears it");
  b.run_ms(1500);

  printf("== STOP during a move\n");
  b.cmd("AZ300");
  b.run_ms(6000);
  CHECK(b.relay_cw == 1, "moving CW");
  avr_raise_irq(avr_io_getirq(b.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), 'S'); b.run_ms(1.05);
  avr_raise_irq(avr_io_getirq(b.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), 'A'); b.run_ms(1.05);
  avr_raise_irq(avr_io_getirq(b.avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT), '\r');
  uint64_t t_stop = b.avr->cycle;
  b.run_until([&] { return !b.pwm_level && b.avr->cycle - b.last_pwm_high_cycle > 16000 * 3; }, 100);
  double cut_ms = Bench::ms_of(b.last_pwm_high_cycle > t_stop ? b.last_pwm_high_cycle - t_stop : 0);
  b.run_until([&] { return !b.relay_cw; }, 200);
  double rel_ms = Bench::ms_of(b.avr->cycle - t_stop);
  CHECK(cut_ms < 3, "PWM off %.2f ms after SA", cut_ms);
  CHECK(rel_ms >= 29 && rel_ms < 60, "relay released %.1f ms after SA (30 ms settle)", rel_ms);
  b.run_ms(1500);

  printf("== CW button (A5) held 3 s, released\n");
  double p0 = b.theta;
  b.button(5, 0); b.run_ms(3000);
  CHECK(b.relay_cw == 1 && b.pwm_level + b.duty > 0, "button CW drives the CW relay");
  b.button(5, 1);
  b.run_until([&] { return !b.relay_cw; }, 3000);
  b.run_ms(1500);
  CHECK(b.theta > p0 + 2 && b.idle(), "moved %.1f deg and stopped", b.theta - p0);

  printf("== totals\n");
  CHECK(b.violations == 0, "hardware rules: %d violations %s", b.violations, b.last_violation.c_str());
  CHECK(b.resets == 2, "boots: %d (power-up + the deliberate reset; a watchdog or crash reset would add more)", b.resets);
  int ram_end = 0x8FF, free_min = b.sp_min - 0x100 - 1506 + 1;   // SRAM 0x100..0x8FF, .data+.bss = 1506 bytes
  printf("  lowest SP 0x%03X -> at least %d bytes never touched between the variables and the stack\n", b.sp_min, free_min);
  CHECK(free_min > 200, "stack margin %d bytes", free_min);
  (void)ram_end;

  printf("== boot with an empty EEPROM\n");
  Bench e; e.boot({}, 0.0);
  e.run_ms(1600);
  CHECK(e.cmd("GE") == "GE16", "no stored position -> GE16: %s", e.cmd("GE").c_str());
  CHECK(e.lcd_line(1).find("No position") == 0, "LCD: '%s'", e.lcd_line(1).c_str());
  e.cmd("CW7,200.5");
  CHECK(e.cmd("AZ") == "AZ200.5" && e.cmd("GE") == "GE0", "CW7 calibrates and clears the error");

  printf("== brown-out during a position save: newest slot corrupt -> the previous slot\n");
  {
    std::vector<uint8_t> img = k3ng_eeprom(50.0f);
    auto slot = [&](int i, int cnt, uint8_t seq, uint8_t flags, bool corrupt) {
      uint8_t s5[5] = {(uint8_t)cnt, (uint8_t)(cnt >> 8), seq, flags, 0}; s5[4] = Bench::crc8(s5, 4) ^ (corrupt ? 0x55 : 0);
      memcpy(&img[256 + i * 5], s5, 5);
    };
    slot(0, 200, 7, 1, false);                        // MOVING marker at 100.0
    slot(1, 260, 8, 0, false);                        // saved after the move: 130.0
    slot(2, 300, 9, 1, true);                         // next move started, write torn
    Bench c; c.boot(img, 130.0); c.run_ms(1000);
    CHECK(c.cmd("AZ") == "AZ130.0" && c.cmd("GE") == "GE0", "slot 1 used: %s %s", c.cmd("AZ").c_str(), c.cmd("GE").c_str());
    slot(1, 260, 8, 0, true);                         // and if the end-of-move save is the torn one
    Bench d; d.boot(img, 100.0); d.run_ms(1000);
    CHECK(d.cmd("AZ") == "AZ100.0" && d.cmd("GE") == "GE64", "torn end-of-move save -> MOVING slot -> uncertain: %s %s", d.cmd("AZ").c_str(), d.cmd("GE").c_str());
  }

  printf("== settings image with a valid CRC but impossible values -> defaults\n");
  {
    std::vector<uint8_t> img = k3ng_eeprom(10.0f);
    // Settings (AVR, packed): pwm_max u8, pwm_floor u8, pwm_freq u16, az_min i16, az_max i16, ramp_up u16, ramp_down u16,
    // slow_zone u16, report_raw, overshoot, jam, endpoints, fast_pwm (u8 each), park i16
    uint8_t st[22] = {253, 64, 0xEA, 0x01, 0xD4, 0xFE, 0x3C, 0x0F, 0, 0, 0xE8, 0x03, 30, 0, 0, 0, 1, 1, 0, 0, 0};
    img[128] = 'S'; img[129] = 'R'; img[130] = 1;
    memcpy(&img[131], st, 21); img[131 + 21] = Bench::crc8(st, 21);
    Bench e2; e2.boot(img, 10.0); e2.run_ms(1000);
    CHECK(e2.cmd("CR5") == "CR5,2000", "ramp_up 0 rejected, defaults used: %s", e2.cmd("CR5").c_str());
  }

  printf("== 1.1 EEPROM (settings version 1, position slot in 0.5 deg counts) taken over\n");
  {
    std::vector<uint8_t> img = k3ng_eeprom(10.0f);
    uint8_t st[21] = {253, 64, 0xEA, 0x01, 0xD4, 0xFE, 0x3C, 0x0F, 0xB8, 0x0B, 0xE8, 0x03, 30, 0, 0, 0, 1, 1, 0, 0, 0};   // ramp_up 3000
    img[128] = 'S'; img[129] = 'R'; img[130] = 1;
    memcpy(&img[131], st, 21); img[131 + 21] = Bench::crc8(st, 21);
    uint8_t sl[5] = {685 & 0xFF, 685 >> 8, 7, 0, 0}; sl[4] = Bench::crc8(sl, 4);   // 685 counts = 342.5 deg
    memcpy(&img[256], sl, 5);
    Bench o; o.boot(img, 342.5); o.run_ms(1000);
    CHECK(o.cmd("CR5") == "CR5,3000" && o.cmd("CRg") == "CRg,0.8" && o.cmd("CRh") == "CRh,200" && o.cmd("CRi") == "CRi,65" && o.ee(130) == 3,
          "settings kept (%s), new registers default (%s %s), image now version %d", o.cmd("CR5").c_str(), o.cmd("CRg").c_str(), o.cmd("CRh").c_str(), o.ee(130));
    CHECK(o.cmd("AR") == "AR342.5" && o.cmd("GE") == "GE0", "old slot 685 counts -> %s %s", o.cmd("AR").c_str(), o.cmd("GE").c_str());
    o.cmd("AZ346.0");
    o.run_until([&] { return o.idle(); }, 30000); o.run_ms(1500);
    CHECK(std::fabs(o.theta - 346.0) <= 0.4 && std::fabs(o.fw_az() - o.theta) <= 0.2, "then a move lands: fw %.1f true %.2f", o.fw_az(), o.theta);
  }

  printf("== buttons held at power-up (short / water) never start a move\n");
  {
    Bench h; h.boot(k3ng_eeprom(150.0f), 150.0);
    h.button(5, 0); h.button(4, 0);                    // CW and CCW held from the start
    h.run_ms(3000);
    CHECK(!h.relay_cw && !h.relay_ccw && h.cmd("GS") == "GS1", "held at power-up: no move (GS %s)", h.cmd("GS").c_str());
    h.button(4, 1); h.button(5, 1); h.run_ms(200);      // released once ...
    h.button(5, 0); h.run_ms(500);                      // ... then a real press works
    CHECK(h.relay_cw == 1, "press after release moves CW");
    h.button(5, 1); h.run_ms(3000);
  }

  printf("\n%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
