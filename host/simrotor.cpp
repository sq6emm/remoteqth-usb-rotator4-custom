// The firmware core + simulated rotator behind a pseudo-terminal, in real time: point Hamlib at it
//   ./simrotor [start_deg]   -> prints the pty path, e.g. /dev/pts/3
//   rotctld -m 204 -r /dev/pts/3 -s 9600
#define SIM_HW_IMPL
#include "sim.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <cstring>

Sim *g_sim = nullptr;

int main(int argc, char **argv) {
  double start = argc > 1 ? atof(argv[1]) : 91.0;
  int m = posix_openpt(O_RDWR | O_NOCTTY);
  if (m < 0 || grantpt(m) || unlockpt(m)) { perror("pty"); return 1; }
  const char *name = ptsname(m);
  int keep = open(name, O_RDWR | O_NOCTTY);        // keep the slave open so the master never sees EOF
  struct termios t; tcgetattr(keep, &t); cfmakeraw(&t); tcsetattr(keep, TCSANOW, &t);
  fcntl(m, F_SETFL, O_NONBLOCK);
  printf("%s\n", name); fflush(stdout);

  Sim s(start);
  struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
  uint32_t last_log = 0;
  for (;;) {
    char buf[256];
    ssize_t n = read(m, buf, sizeof buf);
    for (ssize_t i = 0; i < n; i++) {
      const char *r = s.feed(buf[i]);
      if (r) { ssize_t w = write(m, r, strlen(r)); (void)w; }
    }
    s.run(1);
    if (s.ms() - last_log >= 1000) {
      last_log = s.ms();
      fprintf(stderr, "t=%5us true %7.2f fw %6.1f pwm %3d relay %+d st %d err %d viol %d\n", s.ms() / 1000, s.theta, s.fw_pos(),
              s.pwm, s.relay, s.rot.status(), s.rot.errors, s.violations);
    }
    next.tv_nsec += 1000000;
    if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
  }
}
