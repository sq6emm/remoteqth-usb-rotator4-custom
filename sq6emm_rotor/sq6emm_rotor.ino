// SQ6EMM rotor controller firmware - hardware layer (ATmega328P, RemoteQTH Rotator Interface 4.0.3)
// Easycomm III on the USB serial port (Hamlib rotctld -m 204), SPID RAU pulse sensing, soft start / soft stop.
// The motion logic lives in core.cpp (also built and tested on the PC, see host/).
#include <stddef.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <avr/wdt.h>
#include "config.h"
#include "core.h"
#include "proto.h"
#include "enc.h"

Rotor rot;
Proto proto(rot);
Enc enc;
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ---------------------------------------------------------------- EEPROM
// K3NG data (bytes 0..66) is left alone, except last_azimuth (float at byte 9, verified with avr-gcc), mirrored so the
// previous firmware still finds the position if it is ever flashed back. Own data: settings at 128 (magic, version,
// CRC, validated on load) and a ring of position slots at 256..895: every save writes the next slot with a sequence
// number, so a brown-out during a write leaves the previous slot valid and the wear is spread over 128 slots.
// A slot with the MOVING flag is written when the motor starts; the slot after the move clears it. Finding MOVING in
// the newest slot at power-up means a reset during a move: the position may be off (GE 64).
#define EE_K3NG_MAGIC_ADDR  0
#define EE_K3NG_MAGIC       111
#define EE_K3NG_LAST_AZ     9
#define EE_SET_ADDR         128
#define EE_SET_MAGIC0       'S'
#define EE_SET_MAGIC1       'R'
#define EE_SET_VERSION      2      // 2 (1.2): + edge_frac at the end; a version 1 image is taken over
#define EE_RING_ADDR        256
#define EE_RING_SLOTS       128
#define EE_SLOT             5      // pos lo, pos hi, seq, flags, crc
#define SLOT_MOVING         1
#define SLOT_TENTHS         2      // 1.2: the value is the position in tenths (before: 0.5 deg counts)
#define MIRROR_EVERY_MS     600000UL   // the K3NG mirror is refreshed at most every 10 min (and after CW7)

static uint8_t crc8(const uint8_t *p, uint8_t n) {
  uint8_t c = 0x5A;
  while (n--) { c ^= *p++; for (uint8_t i = 0; i < 8; i++) c = (c & 0x80) ? (c << 1) ^ 0x07 : c << 1; }
  return c;
}

static void save_settings() {
  const uint8_t *p = (const uint8_t *)&rot.s;
  EEPROM.update(EE_SET_ADDR, EE_SET_MAGIC0); EEPROM.update(EE_SET_ADDR + 1, EE_SET_MAGIC1);
  EEPROM.update(EE_SET_ADDR + 2, EE_SET_VERSION);
  for (uint8_t i = 0; i < sizeof(Settings); i++) EEPROM.update(EE_SET_ADDR + 3 + i, p[i]);
  EEPROM.update(EE_SET_ADDR + 3 + sizeof(Settings), crc8(p, sizeof(Settings)));
}

static bool load_settings() {
  uint8_t ver = EEPROM.read(EE_SET_ADDR + 2);
  if (EEPROM.read(EE_SET_ADDR) != EE_SET_MAGIC0 || EEPROM.read(EE_SET_ADDR + 1) != EE_SET_MAGIC1 ||
      (ver != EE_SET_VERSION && ver != 1)) return false;
  uint8_t n = ver == 1 ? offsetof(Settings, edge_frac) : sizeof(Settings);   // version 1: everything before edge_frac
  Settings t = rot.s;                            // defaults (set before): fields a version 1 image lacks keep them
  uint8_t *p = (uint8_t *)&t;
  for (uint8_t i = 0; i < n; i++) p[i] = EEPROM.read(EE_SET_ADDR + 3 + i);
  if (crc8(p, n) != EEPROM.read(EE_SET_ADDR + 3 + n)) return false;
  if (!settings_ok(t)) return false;              // a CRC8 can pass by chance: every field must be in range too
  rot.s = t;
  if (ver != EE_SET_VERSION) save_settings();
  return true;
}

static uint8_t ring_next = 0, ring_seq = 0;       // where the next slot goes, its sequence number
static bool ring_moving = false;                  // the newest slot has the MOVING flag
static uint32_t t_mirror = 0; static bool mirror_done = false;

static bool slot_read(uint8_t i, int16_t *c, uint8_t *seq, uint8_t *flags) {
  uint8_t b[EE_SLOT];
  for (uint8_t k = 0; k < EE_SLOT; k++) b[k] = EEPROM.read(EE_RING_ADDR + i * EE_SLOT + k);
  if (b[0] == 0xFF && b[1] == 0xFF && b[2] == 0xFF && b[3] == 0xFF) return false;   // blank
  if (crc8(b, 4) != b[4]) return false;
  *c = (int16_t)(b[0] | (b[1] << 8)); *seq = b[2]; *flags = b[3];
  return true;
}

static void slot_write(int16_t c, uint8_t flags) {
  flags |= SLOT_TENTHS;
  uint8_t b[EE_SLOT] = {(uint8_t)c, (uint8_t)(c >> 8), ring_seq, flags, 0};
  b[4] = crc8(b, 4);
  for (uint8_t k = 0; k < EE_SLOT; k++) EEPROM.update(EE_RING_ADDR + ring_next * EE_SLOT + k, b[k]);
  ring_next = (ring_next + 1) % EE_RING_SLOTS; ring_seq++;
  ring_moving = flags & SLOT_MOVING;
}

static void save_position(uint32_t now) {
  int16_t c = rot.pos();
  slot_write(c, 0);
  rot.saved(c);
  if (EEPROM.read(EE_K3NG_MAGIC_ADDR) == EE_K3NG_MAGIC && (!mirror_done || rot.calibrated_now || now - t_mirror >= MIRROR_EVERY_MS)) {
    float f = c * 0.1f; EEPROM.put(EE_K3NG_LAST_AZ, f);
    t_mirror = now; mirror_done = true;
  }
  rot.calibrated_now = false;
}

// 0 = no position, 1 = position (newest slot), 2 = position but a reset happened during a move, 3 = taken over from K3NG
static uint8_t load_position() {
  bool any = false; uint8_t best = 0, bseq = 0, bflags = 0; int16_t bc = 0;
  for (uint8_t i = 0; i < EE_RING_SLOTS; i++) {
    int16_t c; uint8_t seq, flags;
    if (!slot_read(i, &c, &seq, &flags)) continue;
    if (!any || (int8_t)(seq - bseq) > 0) { any = true; best = i; bseq = seq; bflags = flags; bc = c; }
  }
  if (any) {
    rot.set_raw(bflags & SLOT_TENTHS ? bc : bc * 5); ring_next = (best + 1) % EE_RING_SLOTS; ring_seq = bseq + 1; ring_moving = bflags & SLOT_MOVING;
    return ring_moving ? 2 : 1;
  }
  if (EEPROM.read(EE_K3NG_MAGIC_ADDR) == EE_K3NG_MAGIC) {          // first start after the K3NG firmware
    float f; EEPROM.get(EE_K3NG_LAST_AZ, f);
    if (f == f && f > -45.0f && f < 405.0f) { rot.set_raw((int16_t)(f >= 0 ? f * 10 + 0.5f : f * 10 - 0.5f)); return 3; }
  }
  return 0;
}

// ---------------------------------------------------------------- outputs
void hw_relay(int8_t d) {
  digitalWrite(PIN_RELAY_CW, LOW); digitalWrite(PIN_RELAY_CCW, LOW);   // never both
  if (d > 0) digitalWrite(PIN_RELAY_CW, HIGH);
  else if (d < 0) digitalWrite(PIN_RELAY_CCW, HIGH);
}

void hw_brake(bool release) { digitalWrite(PIN_BRAKE, release ? HIGH : LOW); }

void hw_pwm(uint8_t d) {
  if (!d) { TCCR2A &= ~_BV(COM2A1); PORTB &= ~_BV(PB3); return; }    // pin low, no 1-count spikes
  OCR2A = d;
  TCCR2A |= _BV(COM2A1);
}

// Timer2 on OC2A (D11): only these frequencies exist (prescaler steps); phase-correct preferred
struct PwmStep { uint16_t hz; uint8_t fast; uint8_t cs; };
static const PwmStep pwm_steps[] = {
  {30, 0, 7}, {61, 1, 7}, {122, 0, 6}, {245, 0, 5}, {490, 0, 4}, {980, 0, 3},
  {1953, 1, 3}, {3921, 0, 2}, {7812, 1, 2}, {31372, 0, 1}, {62500, 1, 1},
};

uint16_t hw_pwm_freq(uint16_t hz) {
  uint8_t k = 0;
  for (uint8_t i = 0; i < sizeof(pwm_steps) / sizeof(pwm_steps[0]); i++) if (pwm_steps[i].hz <= hz) k = i;
  uint8_t keep = TCCR2A & _BV(COM2A1);
  TCCR2A = keep | (pwm_steps[k].fast ? (_BV(WGM21) | _BV(WGM20)) : _BV(WGM20));
  TCCR2B = pwm_steps[k].cs;
  return pwm_steps[k].hz;
}

// ---------------------------------------------------------------- interrupts
// pulse line sampling: Timer1 CTC at PULSE_SAMPLE_HZ (Timer0 = millis, Timer2 = motor PWM; D9/D10 are plain GPIO)
ISR(TIMER1_COMPA_vect) {
  rot.sample((PIND & _BV(PD2)) ? 1 : 0, millis());
}

static void pulse_timer_start() {
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);                 // CTC, clk/8 = 2 MHz
  OCR1A = (F_CPU / 8 / PULSE_SAMPLE_HZ) - 1;
  TIMSK1 = _BV(OCIE1A);
}

// preset encoder: A6 / A2 alternately, ~4.8 kHz each, external AREF (board divider) - never use analogRead()
static volatile uint8_t adc_ch = 0;
ISR(ADC_vect) {
  uint16_t v = ADC;
  uint8_t ch = adc_ch;
  adc_ch = ch ^ 1;
  ADMUX = adc_ch ? ENC_ADC_B : ENC_ADC_A;          // REFS = 00: AREF pin
  ADCSRA |= _BV(ADSC);
  enc.sample(ch, v, millis());
}

static void adc_start() {
  ADMUX = ENC_ADC_A;
  ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);   // 125 kHz ADC clock
  ADCSRA |= _BV(ADSC);
}

// ---------------------------------------------------------------- diagnostics for IP / AN
static char diag[24];
static const char *diag_inputs() {
  uint8_t b = 0;
  if (digitalRead(PIN_BTN_CW) == LOW) b |= 1;
  if (digitalRead(PIN_BTN_CCW) == LOW) b |= 2;
  if (digitalRead(PIN_BTN_START) == LOW) b |= 4;
  if (PIND & _BV(PD2)) b |= 8;
  fmt_u32(diag, b);
  return diag;
}
static const char *diag_analog() {
  uint16_t a, b;
  ATOMIC_BEGIN a = enc.adc[0]; b = enc.adc[1]; ATOMIC_END
  char *p = fmt_u32(diag, a); *p++ = ','; fmt_u32(p, b);
  return diag;
}

// ---------------------------------------------------------------- buttons, preset, display
// A button held (or shorted) at power-up never starts a move: it must be seen released once first.
struct Button {
  uint8_t pin, state, raw, armed; uint32_t t;
  explicit Button(uint8_t p) : pin(p), state(1), raw(1), armed(0), t(0) {}
  int8_t update(uint32_t now) {                    // +1 pressed, -1 released, 0 nothing
    uint8_t r = digitalRead(pin) == LOW;
    if (r != raw) { raw = r; t = now; }
    if (r == state || now - t < 30) return 0;
    state = r;
    if (!r) { if (!armed) { armed = 1; return 0; } return -1; }
    return armed ? 1 : 0;
  }
};
static Button b_cw(PIN_BTN_CW), b_ccw(PIN_BTN_CCW), b_start(PIN_BTN_START);
static bool manual_by_button = false;
static int16_t preset = 0; static bool preset_on = false; static uint32_t t_preset = 0;

static void buttons(uint32_t now) {
  int8_t e;
  if ((e = b_cw.update(now)) > 0) { rot.set_speed(0); rot.manual(1); manual_by_button = true; }
  else if (e < 0 && manual_by_button) { rot.soft_stop(); manual_by_button = false; }
  if ((e = b_ccw.update(now)) > 0) { rot.set_speed(0); rot.manual(-1); manual_by_button = true; }
  else if (e < 0 && manual_by_button) { rot.soft_stop(); manual_by_button = false; }
  if (b_start.update(now) > 0) {
    if (rot.moving()) rot.soft_stop();
    else if (preset_on) { rot.set_speed(0); rot.goto_raw(preset); preset_on = false; }
  }
  int16_t st;
  ATOMIC_BEGIN st = enc.steps; enc.steps = 0; ATOMIC_END
  if (st) {
    if (!preset_on) preset = (rot.pos() / 10) * 10;
    int32_t v = (int32_t)preset + (int32_t)st * 10;
    if (v < rot.s.az_min) v = rot.s.az_min;
    if (v > rot.s.az_max) v = rot.s.az_max;
    preset = v; preset_on = true; t_preset = now;
  }
  if (preset_on && now - t_preset > PRESET_SHOW_MS) preset_on = false;
}

static char lcd_l[2][17];
static void lcd_line(uint8_t row, const char *s) {
  char b[17]; uint8_t i = 0;
  for (; s[i] && i < 16; i++) b[i] = s[i];
  for (; i < 16; i++) b[i] = ' ';
  b[16] = 0;
  if (!strcmp(b, lcd_l[row])) return;
  strcpy(lcd_l[row], b);
  lcd.setCursor(0, row); lcd.print(b);
}

static void display() {
  char b[24], *p;
  int16_t raw = rot.pos();
  p = b; *p++ = 'A'; *p++ = 'z'; *p++ = ' ';
  int16_t a = rot.az();
  if (a < 1000) *p++ = ' ';
  if (a < 100) *p++ = ' ';
  p = fmt_tenths(p, a); *p++ = '\xDF';
  if (raw < 0) { strcpy(p, " <ovl"); } else if (raw >= 3600) { strcpy(p, " ovl>"); } else *p = 0;
  lcd_line(0, b);

  uint8_t e = rot.errors;
  if (preset_on) { p = b; strcpy(p, "Preset "); p = fmt_tenths(p + 7, preset); *p++ = '\xDF'; *p = 0; }
  else if (e & ER_JAM) strcpy(b, "ERR jam/stall");
  else if (e & ER_TIMEOUT) strcpy(b, "ERR timeout");
  else if (e & ER_SENSOR) strcpy(b, "ERR pulse line");
  else if (e & ER_NOPOS) strcpy(b, "No position!");
  else if ((e & ER_UNCERT) && !rot.moving()) strcpy(b, "Pos uncertain");
  else if (rot.moving()) {
    p = b; *p++ = rot.dir() > 0 ? '>' : '<'; *p++ = rot.dir() > 0 ? '>' : '<'; *p++ = ' ';
    if (rot.manual_mode()) { strcpy(p, "manual"); p += 6; } else p = fmt_tenths(p, rot.target());
    *p++ = ' ';
    p = fmt_u32(p, (uint32_t)rot.pwm_now * 100 / 255); *p++ = '%'; *p = 0;
  } else strcpy(b, (rot.status() & ST_POINTING) ? "On target" : "Idle");
  lcd_line(1, b);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  MCUSR = 0; wdt_disable();
  digitalWrite(PIN_RELAY_CW, LOW); digitalWrite(PIN_RELAY_CCW, LOW); digitalWrite(PIN_BRAKE, LOW);
  pinMode(PIN_RELAY_CW, OUTPUT); pinMode(PIN_RELAY_CCW, OUTPUT); pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_PWM, LOW); pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_PULSE, INPUT_PULLUP);
  pinMode(PIN_BTN_START, INPUT_PULLUP); pinMode(PIN_BTN_CCW, INPUT_PULLUP); pinMode(PIN_BTN_CW, INPUT_PULLUP);

  rot.defaults();
  if (!load_settings()) { rot.defaults(); save_settings(); }
  delayMicroseconds(200);                         // pull-up settled: the contact level picks the stretch of the position
  rot.last_level = (PIND & _BV(PD2)) ? 1 : 0;
  uint8_t lp = load_position();
  if (lp == 0) { rot.set_raw(0); rot.errors |= ER_NOPOS; }
  else if (lp == 2) rot.errors |= ER_UNCERT;

  Serial.begin(SERIAL_BAUD);
  proto.diag_inputs = diag_inputs;
  proto.diag_analog = diag_analog;

  lcd.begin(16, 2);
  lcd.setCursor(0, 0); lcd.print("SQ6EMM rotor");
  lcd.setCursor(0, 1); lcd.print(FW_VERSION + 7);   // "rotor-1.0"
  delay(600);
  lcd.clear();

  rot.last_level = (PIND & _BV(PD2)) ? 1 : 0;
  rot.begin(millis());                            // counter limits are set before the sampling interrupt runs
  pulse_timer_start();
  adc_start();
  wdt_enable(WDTO_2S);                            // a hung loop resets the MCU: relays and PWM drop to off
}

void loop() {
  wdt_reset();
  uint32_t now = millis();
  // non-blocking replies: an LG reply (~180 chars) would block ~120 ms in Serial.print with the 64-byte TX buffer.
  // While a reply is going out, input waits in the RX buffer (Hamlib waits for the reply anyway).
  static const char *tx = nullptr;
  if (tx) {
    int n = Serial.availableForWrite();
    while (n-- > 0 && *tx) Serial.write(*tx++);
    if (!*tx) tx = nullptr;
  }
  while (!tx && Serial.available()) tx = proto.feed((char)Serial.read(), now);
  buttons(now);
  rot.tick(now);
  if (rot.driving() && !ring_moving && !(rot.errors & ER_NOPOS)) slot_write(rot.pos(), SLOT_MOVING);
  if (rot.save_due(now, ring_moving)) save_position(now);
  if (rot.settings_dirty) { save_settings(); rot.settings_dirty = false; }
  static uint32_t t_lcd = 0;
  if (now - t_lcd >= 250) { t_lcd = now; display(); }
}
