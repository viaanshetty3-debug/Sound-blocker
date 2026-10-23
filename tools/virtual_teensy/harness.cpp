// Virtual Teensy: runs the unmodified window_anc_fxlms.ino on a computer.
//
// What is real: every line of the firmware (setup, loop, ISR maths, serial command parser,
// calibration/active/bypass state machine) and its serial protocol.
//
// What is NOT real:
//  * Timing. The firmware times its ISR with the ARM cycle counter. A computer's speed says
//    nothing about a 600 MHz Cortex-M7, so the cycle counter here reports an ASSUMED
//    execution time (--isr-us, default 250 us = the most the ISR may take without losing a
//    sample). Every reply to a line command is preceded by "@VIRTUAL ..." so tools can tell.
//  * The acoustics. The microphones and the DAC are connected to a toy plant (two tones plus
//    noise, pure-delay primary, secondary and feedback paths) so that the loop has something
//    to adapt to. Its dB figures say nothing about a real window.
//
// Ticks run at 4,000 per second of wall-clock time so that commands like
// `timing_tool.py measure --seconds 5` behave as they would on the board.
//
// Usage:  harness [--isr-us X] [--pty]
//   default: serial over stdin/stdout.  --pty: create a pseudo-terminal and print its path
//   on stderr ("PORT /dev/ttys012"), so any serial program can connect to it.
#include <Arduino.h>
#include <SPI.h>
#include <IntervalTimer.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <random>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <deque>
#include <string>

void setup();
void loop();

const uint32_t F_CPU_ACTUAL = 600000000UL;
uint32_t ARM_DEMCR = 0;
uint32_t ARM_DWT_CTRL = 0;
HostSerial Serial;
HostSPI SPI;

static const double FS = 4000.0;
static double g_isr_us = 250.0;
static int g_in_fd = 0, g_out_fd = 1;
static std::deque<char> g_rx;
static std::string g_line;                // incoming bytes since the last newline (for @VIRTUAL)

static void (*g_isr)() = nullptr;
static uint64_t g_tick = 0;               // ISR ticks run so far
static uint64_t g_ms_offset = 0;          // virtual milliseconds spent in delay() before the timer
static uint64_t g_cycles = 0;             // value the cycle counter reads
static int g_reads_in_isr = -1;           // -1: not inside the ISR

// ---------------------------------------------------------------------------- cycle counter
uint32_t vt_cyccnt() {
  if (g_reads_in_isr >= 0) {
    // ancISR() reads the counter twice: before and after ancStep(). The second read
    // returns the first plus the ASSUMED execution time.
    uint64_t v = g_cycles + (g_reads_in_isr > 0 ? (uint64_t)llround(g_isr_us * 600.0) : 0);
    g_reads_in_isr++;
    return (uint32_t)v;
  }
  return (uint32_t)g_cycles;
}

// ---------------------------------------------------------------------------- toy plant
static std::mt19937_64 g_rng(2026);
static std::normal_distribution<double> g_gauss(0.0, 1.0);
static const int HIST = 16;
static double g_noise_hist[HIST];         // outdoor noise, [0] = this tick
static double g_dac_hist[HIST];           // DAC output, [j] = written j ticks ago (j >= 1 valid)
static int g_adc_ref = 2048, g_adc_err = 2048;
static double g_dac_now = 0.0;

static int to_adc(double v) {
  // 12-bit codes plus 1.2 LSB rms noise, as in simulator/simulate_realistic_hil.py
  double code = std::round(2048.0 + v * 2048.0 + 1.2 * g_gauss(g_rng));
  return (int)std::fmin(4095.0, std::fmax(0.0, code));
}

static void plant_step() {
  for (int j = HIST - 1; j > 0; j--) {
    g_noise_hist[j] = g_noise_hist[j - 1];
    g_dac_hist[j] = g_dac_hist[j - 1];
  }
  double t = (double)g_tick / FS;
  g_noise_hist[0] = 0.25 * std::sin(2 * M_PI * 120.0 * t) + 0.15 * std::sin(2 * M_PI * 75.0 * t + 1.0)
                    + 0.02 * g_gauss(g_rng);
  g_dac_hist[0] = 0.0;                    // filled when the ISR writes the DAC this tick
  double ref = g_noise_hist[0] + 0.05 * g_dac_hist[4];
  double err = 0.5 * g_noise_hist[6] + 0.6 * g_dac_hist[2];
  g_adc_ref = to_adc(ref);
  g_adc_err = to_adc(err);
}

void vt_dac_command(uint16_t cmd) {
  g_dac_now = ((double)(cmd & 0x0FFF) - 2048.0) / 2048.0;
}

// ---------------------------------------------------------------------------- Arduino API
void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return HIGH; }  // button not pressed
void digitalWriteFast(uint8_t, uint8_t) {}
int analogRead(uint8_t pin) { return pin == A0 ? g_adc_ref : g_adc_err; }
void analogReadResolution(unsigned int) {}
void analogReadAveraging(unsigned int) {}
uint32_t millis() { return (uint32_t)(g_ms_offset + g_tick / 4); }
void delay(uint32_t ms) { g_ms_offset += ms; }
void noInterrupts() {}
void interrupts() {}
void vt_timer_begin(void (*fn)(), uint32_t) { g_isr = fn; }

int HostSerial::available() { return (int)g_rx.size(); }
int HostSerial::read() {
  if (g_rx.empty()) return -1;
  char c = g_rx.front();
  g_rx.pop_front();
  return (unsigned char)c;
}
void HostSerial::write_str(const char* s) {
  size_t n = strlen(s);
  while (n > 0) {
    ssize_t w = ::write(g_out_fd, s, n);
    if (w < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN) { usleep(1000); continue; }
      _exit(0);                           // host went away
    }
    s += w;
    n -= (size_t)w;
  }
}
void HostSerial::print(long long v) { char b[32]; snprintf(b, sizeof b, "%lld", v); write_str(b); }
void HostSerial::print(unsigned long long v) { char b[32]; snprintf(b, sizeof b, "%llu", v); write_str(b); }
void HostSerial::print(double v, int digits) { char b[64]; snprintf(b, sizeof b, "%.*f", digits, v); write_str(b); }

// ---------------------------------------------------------------------------- main loop
static void run_tick() {
  plant_step();
  g_cycles = g_tick * (F_CPU_ACTUAL / 4000UL);
  g_dac_now = g_dac_hist[1];              // DAC holds its last value if the ISR does not write
  g_reads_in_isr = 0;
  if (g_isr) g_isr();
  g_reads_in_isr = -1;
  g_dac_hist[0] = g_dac_now;
  g_tick++;
}

static void receive(const char* buf, ssize_t n) {
  for (ssize_t i = 0; i < n; i++) {
    char c = buf[i];
    if (c == '\n' || c == '\r') {
      if (!g_line.empty()) {
        char note[160];
        snprintf(note, sizeof note,
                 "@VIRTUAL isr_us_assumed=%.3f plant=toy timing=assumed_not_measured\r\n", g_isr_us);
        Serial.print(note);
      }
      g_line.clear();
    } else {
      g_line.push_back(c);
    }
    g_rx.push_back(c);
  }
}

static double now_s() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char** argv) {
  bool use_pty = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--isr-us") && i + 1 < argc) g_isr_us = atof(argv[++i]);
    else if (!strcmp(argv[i], "--pty")) use_pty = true;
    else { fprintf(stderr, "usage: %s [--isr-us X] [--pty]\n", argv[0]); return 2; }
  }
  if (!(g_isr_us >= 0.0 && g_isr_us < 7000.0)) { fprintf(stderr, "--isr-us out of range\n"); return 2; }

  if (use_pty) {
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0 || grantpt(m) || unlockpt(m)) { perror("pty"); return 1; }
    termios tio;
    int s = open(ptsname(m), O_RDWR | O_NOCTTY);  // keep the slave open so the master never sees EOF
    if (s >= 0 && tcgetattr(s, &tio) == 0) { cfmakeraw(&tio); tcsetattr(s, TCSANOW, &tio); }
    fprintf(stderr, "PORT %s\n", ptsname(m));
    g_in_fd = g_out_fd = m;
  }
  fcntl(g_in_fd, F_SETFL, fcntl(g_in_fd, F_GETFL) | O_NONBLOCK);
  setvbuf(stderr, nullptr, _IONBF, 0);

  setup();
  const double t0 = now_s();
  char buf[256];
  for (;;) {
    pollfd p = {g_in_fd, POLLIN, 0};
    if (poll(&p, 1, 1) > 0) {
      ssize_t n = ::read(g_in_fd, buf, sizeof buf);
      if (n > 0) receive(buf, n);
      else if (n == 0 && !use_pty) return 0;      // stdin closed: host is done
    }
    uint64_t target = (uint64_t)((now_s() - t0) * FS);
    while (g_tick < target) {
      run_tick();
      if ((g_tick & 7) == 0) loop();
    }
    loop();
  }
}
