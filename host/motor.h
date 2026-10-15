// Motor speed model shared by the simulators: steady speed (deg/s) per PWM at 490 Hz, measured on the real rotator
// 2026-10-15 (measurements/2026-10-15). Points below PWM 96 are estimates (PWM 80: a single measurement).
#pragma once
#include <stdint.h>
inline double motor_speed(uint8_t pwm) {
  static const double P[] = {56, 64, 80, 96, 112, 176, 253, 255}, V[] = {0, 1.4, 1.9, 2.0, 2.8, 3.6, 4.4, 4.45};
  if (pwm <= P[0]) return 0;
  for (int i = 1; i < 8; i++)
    if (pwm <= P[i]) return V[i - 1] + (V[i] - V[i - 1]) * (pwm - P[i - 1]) / (P[i] - P[i - 1]);
  return V[7];
}
