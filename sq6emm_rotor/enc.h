// Preset encoder decoder (pure logic, fed from the ADC interrupt with alternating A6 / A2 samples).
// Per-channel hysteresis + debounce, quadrature decoding, a skipped state counts as two steps in the last
// direction; one click = ENC_TRANSITIONS transitions; fast spinning counts ENC_FAST_STEP degrees per click.
#pragma once
#include <stdint.h>
#include "config.h"

struct Enc {
  volatile uint8_t level[2] = {1, 1}, raw[2] = {1, 1}, stable[2] = {0, 0};
  volatile uint16_t adc[2] = {0, 0};
  volatile int16_t steps = 0;                // degrees not yet taken by the main loop
  volatile uint16_t transitions = 0, invalid = 0, clicks = 0;
  uint8_t last_ab = 3; int8_t last_d = 0, acc = 0;
  uint32_t click_t[5] = {0, 0, 0, 0, 0};

  // ch 0 = A, 1 = B; decodes after each B sample (both channels fresh)
  void sample(uint8_t ch, uint16_t v, uint32_t now_ms) {
    static const int8_t qdec[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};   // [(old << 2) | new]
    adc[ch] = v;
    uint8_t r = raw[ch];
    if (v > ENC_HIGH) r = 1; else if (v < ENC_LOW) r = 0;
    raw[ch] = r;
    if (r != level[ch]) { if (++stable[ch] >= ENC_STABLE) { level[ch] = r; stable[ch] = 0; } }
    else stable[ch] = 0;
    if (ch != 1) return;
    uint8_t ab = (level[1] << 1) | level[0];
    if (ab == last_ab) return;
    int8_t d = qdec[(last_ab << 2) | ab];
    if (ENC_REVERSE) d = -d;
    last_ab = ab;
    if (d == 0) { invalid++; if (!last_d) return; d = 2 * last_d; }
    else last_d = d;
    transitions++;
    acc += d;
    if (ab != 3) return;                       // detent position: both contacts open
    int8_t click = acc >= ENC_TRANSITIONS / 2 ? 1 : (acc <= -(ENC_TRANSITIONS / 2) ? -1 : 0);
    acc = 0;
    if (!click) return;
    for (uint8_t i = 0; i < 4; i++) click_t[i] = click_t[i + 1];
    click_t[4] = now_ms;
    clicks++;
    steps += click * ((now_ms - click_t[0]) < ENC_FAST_MS ? ENC_FAST_STEP : 1);
  }
};
