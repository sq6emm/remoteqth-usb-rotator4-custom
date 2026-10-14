// SQ6EMM rotor controller firmware - hardware layer (ATmega328P, RemoteQTH Rotator Interface 4.0.3)
// Easycomm III on the USB serial port (Hamlib rotctld -m 204), SPID RAU pulse sensing, soft start / soft stop.
// The motion logic lives in core.cpp (also built and tested on the PC, see host/).
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
// K3NG data (bytes 0..~80) is left alone, except last_azimuth which is mirrored so that the previous firmware
// still knows the position if it is ever flashed back. Own data: settings at 128, position at 192.
#define EE_K3NG_MAGIC_ADDR  0
#define EE_K3NG_MAGIC       111
#define EE_K3NG_LAST_AZ     9      // struct config_t: magic(1) + 4 x int(2) -> float last_azimuth
#define EE_SET_ADDR         128
#define EE_POS_ADDR         192
#define EE_SET_MAGIC0       'S'
#define EE_SET_MAGIC1       'R'
#define EE_SET_VERSION      1

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
  if (EEPROM.read(EE_SET_ADDR) != EE_SET_MAGIC0 || EEPROM.read(EE_SET_ADDR + 1) != EE_SET_MAGIC1 ||
      EEPROM.read(EE_SET_ADDR + 2) != EE_SET_VERSION) return false;
  Settings t; uint8_t *p = (uint8_t *)&t;
  for (uint8_t i = 0; i < sizeof(Settings); i++) p[i] = EEPROM.read(EE_SET_ADDR + 3 + i);
  if (crc8(p, sizeof(Settings)) != EEPROM.read(EE_SET_ADDR + 3 + sizeof(Settings))) return false;
  rot.s = t;
  return true;
}

static void save_position() {
  int16_t c = rot.cnt_copy();
  uint8_t b[2] = {(uint8_t)c, (uint8_t)(c >> 8)};
  EEPROM.update(EE_POS_ADDR, b[0]); EEPROM.update(EE_POS_ADDR + 1, b[1]); EEPROM.update(EE_POS_ADDR + 2, crc8(b, 2));
  if (EEPROM.read(EE_K3NG_MAGIC_ADDR) == EE_K3NG_MAGIC) { float f = c * 0.5f; EEPROM.put(EE_K3NG_LAST_AZ, f); }
  rot.saved();
}

static bool load_position() {
  uint8_t b[2] = {EEPROM.read(EE_POS_ADDR), EEPROM.read(EE_POS_ADDR + 1)};
  if (crc8(b, 2) == EEPROM.read(EE_POS_ADDR + 2)) { rot.cnt = (int16_t)(b[0] | (b[1] << 8)); return true; }
  if (EEPROM.read(EE_K3NG_MAGIC_ADDR) == EE_K3NG_MAGIC) {          // first start after the K3NG firmware
    float f; EEPROM.get(EE_K3NG_LAST_AZ, f);
    if (f == f && f > -45.0f && f < 405.0f) { rot.cnt = (int16_t)(f >= 0 ? f * 2 + 0.5f : f * 2 - 0.5f); return true; }
  }
  return false;
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
struct Button {
  uint8_t pin, state, raw; uint32_t t;
  explicit Button(uint8_t p) : pin(p), state(0), raw(0), t(0) {}
  int8_t update(uint32_t now) {                    // +1 pressed, -1 released, 0 nothing
    uint8_t r = digitalRead(pin) == LOW;
    if (r != raw) { raw = r; t = now; }
    if (r != state && now - t >= 30) { state = r; return r ? 1 : -1; }
    return 0;
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
  else if (rot.moving()) {
    p = b; *p++ = rot.dir() > 0 ? '>' : '<'; *p++ = rot.dir() > 0 ? '>' : '<'; *p++ = ' ';
    p = fmt_tenths(p, rot.target()); *p++ = ' ';
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
  if (!load_settings()) save_settings();
  if (!load_position()) { rot.cnt = 0; rot.errors |= ER_NOPOS; }

  Serial.begin(SERIAL_BAUD);
  proto.diag_inputs = diag_inputs;
  proto.diag_analog = diag_analog;

  lcd.begin(16, 2);
  lcd.setCursor(0, 0); lcd.print("SQ6EMM rotor");
  lcd.setCursor(0, 1); lcd.print(FW_VERSION + 7);   // "rotor-1.0"
  delay(1200);
  lcd.clear();

  rot.last_level = (PIND & _BV(PD2)) ? 1 : 0;
  pulse_timer_start();
  adc_start();
  rot.begin(millis());
  wdt_enable(WDTO_2S);                            // a hung loop resets the MCU: relays and PWM drop to off
}

void loop() {
  wdt_reset();
  uint32_t now = millis();
  while (Serial.available()) {
    const char *r = proto.feed((char)Serial.read(), now);
    if (r) Serial.print(r);
  }
  buttons(now);
  rot.tick(now);
  if (rot.save_due(now)) save_position();
  if (rot.settings_dirty) { save_settings(); rot.settings_dirty = false; }
  static uint32_t t_lcd = 0;
  if (now - t_lcd >= 250) { t_lcd = now; display(); }
}
