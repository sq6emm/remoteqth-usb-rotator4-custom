// Platform glue: the core compiles for the AVR and for the host simulator.
#pragma once
#include <stdint.h>

#if defined(__AVR__)
  #include <avr/interrupt.h>
  #define ATOMIC_BEGIN { uint8_t _sreg = SREG; cli();
  #define ATOMIC_END   SREG = _sreg; }
#else
  #define ATOMIC_BEGIN {
  #define ATOMIC_END   }
#endif

// Implemented by the hardware layer (sq6emm_rotor.ino or host/sim.cpp)
void hw_relay(int8_t dir);          // +1 CW, -1 CCW, 0 both off (never both on)
void hw_pwm(uint8_t duty);          // 0..255 on the motor MOSFET
void hw_brake(bool release);        // brake relay (RL3) + START LED
uint16_t hw_pwm_freq(uint16_t hz);  // set Timer2 to the nearest possible frequency <= hz, returns the real one
