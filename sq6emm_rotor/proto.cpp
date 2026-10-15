#include "proto.h"
#include <string.h>

// Easycomm: 2-letter commands with the value attached ("AZ123.4"), separated by space / CR / LF; queries are
// answered with the command echoed and the value ("AZ123.4"), several answers of one line joined by spaces.
// Hamlib quirks handled here: "SA SE " on one line, a ';' after some commands ("GS\n;"), "CR 0" with a space.
// Set commands get no reply (Hamlib does not read one). Unknown commands are ignored, as the standard allows.

const char *Proto::feed(char c, uint32_t now_ms) {
  if (c == '\r' || c == '\n') {
    rlen = 0; reply[0] = 0;
    if (len && !overflow) { line[len] = 0; run_line(now_ms); }
    len = 0; overflow = false;
    if (!rlen) return nullptr;
    put("\r\n");
    return reply;
  }
  if (c == ';' || (uint8_t)c < 0x20) return nullptr;
  if (len < PROTO_LINE - 1) line[len++] = c; else overflow = true;
  return nullptr;
}

void Proto::put(const char *s) {
  while (*s && rlen < PROTO_REPLY - 1) reply[rlen++] = *s++;
  reply[rlen] = 0;
}
void Proto::put_tenths(int16_t v) { char b[10]; fmt_tenths(b, v); put(b); }
void Proto::put_u32(uint32_t v) { char b[12]; fmt_u32(b, v); put(b); }

void Proto::run_line(uint32_t now) {
  char *tok[12]; uint8_t n = 0;
  char *p = line;
  while (*p && n < 12) {
    while (*p == ' ' || *p == '\t') *p++ = 0;
    if (!*p) break;
    tok[n++] = p;
    while (*p && *p != ' ' && *p != '\t') p++;
  }
  for (uint8_t i = 0; i < n; i++) {
    bool used = false;
    uint8_t before = rlen;
    if (rlen) put(" ");
    cmd(tok[i], i + 1 < n ? tok[i + 1] : nullptr, &used, now);
    if (rlen == before + (before ? 1 : 0)) { rlen = before; reply[rlen] = 0; }   // no answer: drop the separator
    if (used) i++;
  }
}

static bool is(const char *t, char a, char b) { return t[0] == a && t[1] == b; }
static void fmt_i16(char *o, int16_t v) { if (v < 0) { *o++ = '-'; v = -v; } fmt_u32(o, (uint16_t)v); }

void Proto::cmd(const char *t, const char *next, bool *used_next, uint32_t now) {
  if (!strcmp(t, "RESET")) { rot.stop_now(); rot.clear_errors(); return; }
  if (!strcmp(t, "PARK")) { rot.set_speed(0); rot.goto_raw(rot.s.park); return; }
  if (!t[0] || !t[1]) return;
  const char *arg = t + 2;
  bool ok;

  if (is(t, 'A', 'Z')) {
    if (!*arg) { put("AZ"); put_tenths(rot.s.report_raw ? rot.pos() : rot.az()); return; }
    int16_t v = parse_tenths(arg, &ok);
    if (!ok) return;
    rot.set_speed(0);                             // a goto runs at MaxSpeed (VL/VR speeds are for manual moves)
    if (v >= 0 && v <= 3600) rot.goto_az(v);     // a heading: the firmware picks the turn
    else rot.goto_raw(v);                         // outside 0..360: an explicit position in the overlap
    return;
  }
  if (is(t, 'E', 'L')) { if (!*arg) put("EL0.0"); return; }   // azimuth-only rotator
  if (is(t, 'S', 'A')) { rot.stop_now(); return; }            // explicit STOP: immediate
  if (is(t, 'S', 'E')) return;
  if (is(t, 'M', 'L')) { rot.manual(-1); return; }
  if (is(t, 'M', 'R')) { rot.manual(1); return; }
  if (is(t, 'V', 'L') || is(t, 'V', 'R')) {     // velocity in mdeg/s (Hamlib: (speed 1..100 - 1) * 100), e.g. VL3000 = 3 deg/s
    uint32_t v = 0; const char *q = arg;
    if (!*q) return;
    while (*q >= '0' && *q <= '9' && v < 100000UL) v = v * 10 + (*q++ - '0');
    if (*q) return;
    rot.set_speed(rot.pwm_for_speed(v));          // real speed from the measured curve; 0 = slowest, stop is SA
    rot.manual(t[1] == 'R' ? 1 : -1);
    return;
  }
  if (is(t, 'G', 'S')) { put("GS"); put_u32(rot.status()); return; }
  if (is(t, 'G', 'E')) { put("GE"); put_u32(rot.errors); return; }
  if (is(t, 'C', 'R')) {
    char r = *arg;
    if (!r && next) { r = next[0]; *used_next = true; }
    if (!r) return;
    char b[12];
    put("CR"); char rs[2] = {r, 0}; put(rs); put(",");
    put(rot.reg_read(r, b) ? b : "?");
    return;
  }
  if (is(t, 'C', 'W')) {                        // CW<reg>,<value>  (also "CW <reg>,<value>")
    const char *body = arg;
    if (!*body && next) { body = next; *used_next = true; }
    if (!body[0] || body[1] != ',') return;
    rot.reg_write(body[0], body + 2);
    return;
  }
  if (is(t, 'V', 'E')) { put("VE" FW_VERSION); return; }
  if (is(t, 'I', 'P')) { put("IP"); put(diag_inputs ? diag_inputs() : "0"); return; }
  if (is(t, 'A', 'N')) { put("AN"); put(diag_analog ? diag_analog() : "0"); return; }
  // SQ6EMM extensions
  if (is(t, 'A', 'R')) { put("AR"); put_tenths(rot.pos()); return; }          // raw position (which turn)
  if (is(t, 'L', 'G')) {                       // event log: LG = "LG<next seq>@<ms>:", LG<seq> = up to 6 records from seq, "+" = more waiting
    if (!*arg) { uint16_t h; ATOMIC_BEGIN h = rot.log_seq; ATOMIC_END put("LG"); put_u32(h); put("@"); put_u32(now); put(":"); return; }
    uint32_t from = 0; const char *q = arg;
    while (*q >= '0' && *q <= '9') from = from * 10 + (*q++ - '0');
    if (*q) return;
    LogRec r[6]; uint16_t first;
    uint8_t n = rot.log_read((uint16_t)from, r, 6, &first);
    put("LG"); put_u32(first); put("@"); put_u32(now); put(":");
    uint8_t i = 0;
    for (; i < n; i++) {                        // whole records only, the reply stays within LG_REPLY_MAX
      char b[32], *e = fmt_u32(b, r[i].t);
      *e++ = ','; fmt_i16(e, r[i].c); e += strlen(e);
      *e++ = ','; e = fmt_u32(e, r[i].p);
      *e++ = ','; e = fmt_u32(e, r[i].k);
      *e++ = ';'; *e = 0;
      if (rlen + (e - b) + 1 + 2 > LG_REPLY_MAX) break;   // + "+" + "\r\n"
      put(b);
    }
    if (i < n || n == 6) put("+");             // more records may be waiting: read again right away
    return;
  }
  if (is(t, 'P', 'D')) {                                                       // pulse / motion diagnostics
    uint32_t a, r, c;
    ATOMIC_BEGIN a = rot.n_acc; r = rot.n_rej; c = rot.n_coast; ATOMIC_END
    put("PDa"); put_u32(a); put(",r"); put_u32(r); put(",c"); put_u32(c);
    put(",h"); put_u32(rot.n_holds); put(",s"); put_u32(rot.n_stalls); put(",t"); put_u32(rot.n_timeouts);
    put(",p"); put_u32(rot.pwm_now); put(",f"); put_u32(rot.pwm_hz);
    int16_t d = rot.duty_permille();                // measured contact closed share of a degree, per mille
    put(",e"); if (d < 0) put("-"); else put_u32((uint32_t)d);
    return;
  }
}
