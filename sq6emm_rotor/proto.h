// Easycomm III command interpreter (Hamlib rotator model EasycommIII).
#pragma once
#include <stdint.h>
#include "core.h"

#define PROTO_LINE 64
#define PROTO_REPLY 200

class Proto {
public:
  explicit Proto(Rotor &r) : rot(r) {}
  // feed one received character; returns a reply line (with "\r\n") when a line produced one, else nullptr
  const char *feed(char c, uint32_t now_ms);
  // hooks for things the core does not own (set by the hardware layer)
  const char *(*diag_inputs)() = nullptr;       // IP reply body
  const char *(*diag_analog)() = nullptr;       // AN reply body
private:
  Rotor &rot;
  char line[PROTO_LINE]; uint8_t len = 0; bool overflow = false;
  char reply[PROTO_REPLY]; uint8_t rlen = 0;
  void run_line(uint32_t now);
  void cmd(const char *tok, const char *next, bool *used_next, uint32_t now);
  void put(const char *s);
  void put_tenths(int16_t v);
  void put_u32(uint32_t v);
};
