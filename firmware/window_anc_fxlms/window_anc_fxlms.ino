/*
 * ============================================================================
 * OPEN-WINDOW ACTIVE NOISE CONTROL (ANC) FIRMWARE — TEENSY 4.0 / 4.1
 * Target: Low-Frequency Traffic Bass & Rumble (40 Hz - 250 Hz)
 * Algorithm: Two-Stage Normalized Leaky FxLMS with Auto-Calibration
 * ============================================================================
 *
 * TIMING (read this before trusting the loop to cancel anything):
 * - Sample Rate (Fs): 4,000 Hz (period 250 us). Output is a zero-order hold, which adds
 *   about half a sample period (0.125 ms) on average.
 * - The ISR work (2x analogRead, 128-tap FxLMS, SPI write) is timed by the firmware itself:
 *   send "?timing" over serial, or run `python3 tools/timing_tool.py measure --port <port>`.
 *   The worst case must stay well under 250 us.
 * - The dominant delay is the ANALOG anti-alias / reconstruction filters, not the code.
 *   See HARDWARE_AND_BAFFLE_GUIDE.md: the original passive RC filters total about 1.45 ms
 *   with the hold, against about 1.0 ms of acoustic head start, so the controller would have
 *   to predict the noise ~0.4 ms ahead, which limits broadband reduction badly. The active
 *   Butterworth filters in the guide total about 0.58 ms.
 * - `python3 tools/timing_tool.py budget` prints the full delay budget.
 *
 * PINOUT (Teensy 4.0):
 * - Pin A0 (14): Reference Mic Preamp Input (Outdoor Mic, 0-3.3V, 1.65V bias)
 * - Pin A1 (15): Error Mic Preamp Input     (Indoor Mic,  0-3.3V, 1.65V bias)
 * - Pin 10 (CS): MCP4921 SPI DAC Chip Select
 * - Pin 11 (MOSI): MCP4921 SPI SDI
 * - Pin 13 (SCK):  MCP4921 SPI SCK
 * - Pin 2:       Pushbutton to GND (short press: toggle ANC, hold >1 s: recalibrate)
 * - Pin 13 is the SPI clock (SCK) and also the on-board LED, so the LED just flickers
 *   with DAC writes. It is not a status indicator; use the Serial monitor for status.
 */

#include <Arduino.h>
#include <SPI.h>
#include <IntervalTimer.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// 1. DSP & ALGORITHM CONFIGURATION
// ============================================================================
#define SAMPLE_RATE_HZ      4000.0f
#define SAMPLE_PERIOD_US    250       // 1 / 4000 Hz = 250 microseconds

#define W_TAPS              128       // Control filter length (32 ms window @ 4 kHz)
#define S_TAPS              64        // Secondary path model length (16 ms window)

// Learning rates & stability parameters
#define MU_S_IDENT          0.08f     // Step size for 3-second secondary path calibration
#define MU_FXLMS            0.015f    // Step size for real-time ANC adaptation
#define LEAKAGE_FACTOR      0.99995f  // Leaky LMS weight decay (prevents weight drift/overload)
#define EPSILON_NORM        1e-4f     // Regularization floor for Normalized LMS

// Calibration settings
#define CALIB_DURATION_SEC  3.0f
#define CALIB_SAMPLES       ((uint32_t)(SAMPLE_RATE_HZ * CALIB_DURATION_SEC))
#define CALIB_NOISE_AMP     0.18f     // White noise amplitude during startup calibration

// Hardware Pins
#define PIN_REF_MIC         A0        // Outdoor Reference Microphone
#define PIN_ERR_MIC         A1        // Indoor Error Microphone
#define PIN_DAC_CS          10        // MCP4921 DAC Chip Select
#define PIN_BUTTON          2         // Mode / Recalibrate Button

// ADC / DAC Constants (12-bit: 0..4095, midpoint = 2048)
#define ADC_MIDPOINT        2048.0f
#define ADC_SCALE           (1.0f / 2048.0f)
#define DAC_MIDPOINT        2048

// ============================================================================
// 2. DSP STATE BUFFERS & FILTER WEIGHTS
// ============================================================================
enum SystemState {
  STATE_CALIBRATING = 0,
  STATE_ANC_ACTIVE  = 1,
  STATE_ANC_BYPASS  = 2
};

volatile SystemState sysState = STATE_CALIBRATING;
volatile uint32_t calibCounter = 0;

// Adaptive filter weights
float W[W_TAPS];                      // Primary ANC control filter W(z)
float S_hat[S_TAPS];                  // Estimated secondary path model S_hat(z)
float F_hat[S_TAPS];                  // Optional acoustic feedback neutralization model

// Circular delay lines (power-of-two sizes for fast bitwise wrap-around)
float x_buf[W_TAPS];                  // Reference mic history x(n - k)
float xf_buf[W_TAPS];                 // Filtered-x history x'(n - k)
float s_buf[S_TAPS];                  // Secondary path input history for x(n) -> x'(n)
float y_buf[S_TAPS];                  // Speaker output history y(n - k) for calibration

uint16_t idx_w = 0;                   // Circular index for W_TAPS buffers
uint16_t idx_s = 0;                   // Circular index for S_TAPS buffers

// DC-blocking High-Pass Filters (fc ~ 25 Hz) to strip 1.65V analog bias
// y[n] = alpha * (y[n-1] + x[n] - x[n-1])
const float HPF_ALPHA = 0.962f;
float ref_prev_in = 0.0f, ref_prev_out = 0.0f;
float err_prev_in = 0.0f, err_prev_out = 0.0f;

// Real-time telemetry accumulators (updated in ISR, printed in main loop)
volatile float rms_ref_accum = 0.0f;
volatile float rms_err_accum = 0.0f;
volatile uint32_t telemetry_count = 0;
volatile bool divergence_flag = false;

IntervalTimer ancTimer;

// Runtime-adjustable parameters (defaults from the #defines above; change over serial with
// "set mu <v>" / "set leak <v>", see tools/timing_tool.py). 32-bit float writes are atomic
// on the Cortex-M7, so the ISR can read them without locking.
volatile float g_mu   = MU_FXLMS;
volatile float g_leak = LEAKAGE_FACTOR;

// ISR timing instrumentation (cycle counter, 1 tick = 1 / F_CPU_ACTUAL s)
struct IsrStats {
  uint32_t n;
  uint64_t exec_sum;     // total cycles spent inside ancStep()
  uint32_t exec_max;     // worst-case cycles inside ancStep()
  uint32_t int_min;      // shortest / longest cycles between consecutive ISR entries
  uint32_t int_max;
  uint32_t overruns;     // ISR runs that took longer than one sample period
};
volatile IsrStats isrStats[3];            // indexed by SystemState at ISR entry
volatile uint32_t lastIsrCycles = 0;
volatile bool haveLastIsr = false;
const uint32_t PERIOD_CYCLES = (uint32_t)(F_CPU_ACTUAL / 4000UL);

void resetIsrStats() {
  for (int i = 0; i < 3; i++) {
    isrStats[i].n = 0;
    isrStats[i].exec_sum = 0;
    isrStats[i].exec_max = 0;
    isrStats[i].int_min = 0xFFFFFFFFUL;
    isrStats[i].int_max = 0;
    isrStats[i].overruns = 0;
  }
}

// Fast deterministic PRNG (Linear Congruential Generator) for calibration noise
uint32_t prng_state = 0x12345678;
inline float fastWhiteNoise() {
  prng_state = prng_state * 1664525UL + 1013904223UL;
  int32_t signed_val = (int32_t)(prng_state >> 16) - 32768;
  return (float)signed_val * (1.0f / 32768.0f);
}

// ============================================================================
// 3. LOW-LATENCY SPI DAC WRITE (MCP4921 12-Bit DAC)
// ============================================================================
inline void writeDAC(float normalized_sample) {
  // Hard clip to [-0.95, +0.95] to keep the DAC off its rails
  if (normalized_sample > 0.95f)  normalized_sample = 0.95f;
  if (normalized_sample < -0.95f) normalized_sample = -0.95f;

  int32_t dac_val = DAC_MIDPOINT + (int32_t)(normalized_sample * 2047.0f);
  if (dac_val < 0)    dac_val = 0;
  if (dac_val > 4095) dac_val = 4095;

  // MCP4921 command word:
  // Bit 15: 0 (Write to DAC_A)
  // Bit 14: 1 (Buffered Vref)
  // Bit 13: 1 (Gain = 1x)
  // Bit 12: 1 (Active mode, not shutdown)
  // Bits 11..0: 12-bit data
  uint16_t cmd = 0x7000 | (uint16_t)(dac_val & 0x0FFF);

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWriteFast(PIN_DAC_CS, LOW);
  SPI.transfer16(cmd);
  digitalWriteFast(PIN_DAC_CS, HIGH);
  SPI.endTransaction();
}

// ============================================================================
// 4. RESET & INITIALIZATION HELPERS
// ============================================================================
void resetFilters(bool clearSecondaryPath) {
  noInterrupts();
  for (int i = 0; i < W_TAPS; i++) {
    W[i] = 0.0f;
    x_buf[i] = 0.0f;
    xf_buf[i] = 0.0f;
  }
  for (int i = 0; i < S_TAPS; i++) {
    s_buf[i] = 0.0f;
    y_buf[i] = 0.0f;
    if (clearSecondaryPath) {
      S_hat[i] = 0.0f;
      F_hat[i] = 0.0f;
    }
  }
  idx_w = 0;
  idx_s = 0;
  interrupts();
}

void startCalibration() {
  resetFilters(true);
  calibCounter = 0;
  sysState = STATE_CALIBRATING;
  Serial.println(F("\n[ANC] Starting 3-second Secondary Path Calibration (S_hat)..."));
}

// ============================================================================
// 5. HARD REAL-TIME 4,000 Hz INTERRUPT SERVICE ROUTINE (ISR)
// ============================================================================
// One control step (read mics, FxLMS, write DAC). Wrapped by ancISR() below, which times it.
static inline void ancStep() {
  // 1. Sample both microphones immediately (12-bit SAR ADC: 0..4095)
  float raw_ref = ((float)analogRead(PIN_REF_MIC) - ADC_MIDPOINT) * ADC_SCALE;
  float raw_err = ((float)analogRead(PIN_ERR_MIC) - ADC_MIDPOINT) * ADC_SCALE;

  // 2. Apply 25 Hz DC-blocking High-Pass Filter
  float x_n = HPF_ALPHA * (ref_prev_out + raw_ref - ref_prev_in);
  ref_prev_in = raw_ref;
  ref_prev_out = x_n;

  float e_n = HPF_ALPHA * (err_prev_out + raw_err - err_prev_in);
  err_prev_in = raw_err;
  err_prev_out = e_n;

  float y_out = 0.0f;

  // --------------------------------------------------------------------------
  // STAGE 1: OFFLINE SECONDARY PATH IDENTIFICATION (S_hat & F_hat)
  // Injects white noise v(n) to learn Speaker -> Error Mic impulse response
  // and Speaker -> Reference Mic acoustic feedback response.
  // --------------------------------------------------------------------------
  if (sysState == STATE_CALIBRATING) {
    float v_n = fastWhiteNoise() * CALIB_NOISE_AMP;
    y_out = v_n;

    // Write speaker output immediately to minimize latency
    writeDAC(y_out);

    // Store speaker output v(n) in circular buffer y_buf
    idx_s = (idx_s == 0) ? (S_TAPS - 1) : (idx_s - 1);
    y_buf[idx_s] = v_n;

    // Predict error mic reading: e_hat = sum(S_hat[k] * v(n-k))
    // Predict feedback at ref mic: f_hat = sum(F_hat[k] * v(n-k))
    float e_hat = 0.0f;
    float f_hat = 0.0f;
    float power_v = EPSILON_NORM;

    uint16_t p = idx_s;
    for (int k = 0; k < S_TAPS; k++) {
      float v_k = y_buf[p];
      e_hat += S_hat[k] * v_k;
      f_hat += F_hat[k] * v_k;
      power_v += v_k * v_k;
      p = (p + 1) & (S_TAPS - 1);
    }

    // Update S_hat and F_hat via Normalized LMS
    float err_s = e_n - e_hat;
    float err_f = x_n - f_hat;
    float step = MU_S_IDENT / power_v;

    p = idx_s;
    for (int k = 0; k < S_TAPS; k++) {
      float v_k = y_buf[p];
      S_hat[k] += step * err_s * v_k;
      F_hat[k] += step * err_f * v_k;
      p = (p + 1) & (S_TAPS - 1);
    }

    calibCounter++;
    if (calibCounter >= CALIB_SAMPLES) {
      // Clear state history before entering active ANC mode
      for (int i = 0; i < S_TAPS; i++) {
        s_buf[i] = 0.0f;
        y_buf[i] = 0.0f;
      }
      sysState = STATE_ANC_ACTIVE;
    }
    return;
  }

  // --------------------------------------------------------------------------
  // STAGE 2: REAL-TIME ACTIVE NOISE CANCELLATION (Normalized Leaky FxLMS)
  // --------------------------------------------------------------------------
  if (sysState == STATE_ANC_ACTIVE) {
    // 2a. Neutralize acoustic feedback from speaker to outdoor reference mic.
    // y_buf[0] is the output from ONE tick ago here (this tick's output is pushed later),
    // but F_hat[k] was learned against the output k ticks ago (calibration pushes the current
    // output before updating). So y_buf[j] pairs with F_hat[j + 1]. The old pairing
    // (F_hat[j] with y_buf[j]) was one sample off; tests/test_feedback_alignment.py shows it
    // leaves the feedback un-cancelled while this pairing removes it.
    uint16_t ps = idx_s;
    float fb_est = 0.0f;
    for (int k = 0; k < S_TAPS - 1; k++) {
      fb_est += F_hat[k + 1] * y_buf[ps];
      ps = (ps + 1) & (S_TAPS - 1);
    }
    float x_clean = x_n - fb_est;

    // 2b. Push clean reference sample into W_TAPS buffer
    idx_w = (idx_w == 0) ? (W_TAPS - 1) : (idx_w - 1);
    x_buf[idx_w] = x_clean;

    // 2c. Compute anti-noise speaker output: y(n) = sum(W[k] * x(n-k))
    float y_n = 0.0f;
    uint16_t pw = idx_w;
    for (int k = 0; k < W_TAPS; k++) {
      y_n += W[k] * x_buf[pw];
      pw = (pw + 1) & (W_TAPS - 1);
    }

    // 2d. Output anti-noise sample to DAC immediately!
    // Everything after this point happens AFTER the speaker is already driven,
    // so the LMS update math adds ZERO latency to the audio path.
    writeDAC(-y_n);

    // 2e. Record speaker output (-y_n) for next sample's feedback neutralization
    idx_s = (idx_s == 0) ? (S_TAPS - 1) : (idx_s - 1);
    y_buf[idx_s] = -y_n;
    s_buf[idx_s] = x_clean;

    // 2f. Compute Filtered-Reference signal: x'(n) = sum(S_hat[m] * x(n-m))
    float xf_n = 0.0f;
    ps = idx_s;
    for (int m = 0; m < S_TAPS; m++) {
      xf_n += S_hat[m] * s_buf[ps];
      ps = (ps + 1) & (S_TAPS - 1);
    }
    xf_buf[idx_w] = xf_n;

    // 2g. Compute signal power of filtered-x buffer for Normalized step size
    float xf_power = EPSILON_NORM;
    pw = idx_w;
    for (int k = 0; k < W_TAPS; k++) {
      float val = xf_buf[pw];
      xf_power += val * val;
      pw = (pw + 1) & (W_TAPS - 1);
    }

    // 2h. Normalized Leaky FxLMS Weight Update:
    // Since speaker outputs -y(n), residual error is e(n) = d(n) - [S * y](n).
    // Gradient descent update: W[k] = LEAK * W[k] + (mu / power) * e(n) * xf(n-k)
    float norm_step = (g_mu * e_n) / xf_power;
    pw = idx_w;
    for (int k = 0; k < W_TAPS; k++) {
      W[k] = (g_leak * W[k]) + norm_step * xf_buf[pw];
      pw = (pw + 1) & (W_TAPS - 1);
    }

    // 2i. Safety watchdog: y_n beyond the DAC range (clipped at +-0.95) means the filter is
    // diverging or hit a loud impulse (e.g. door slam): reset W(z), keep S_hat(z).
    if (fabsf(y_n) > 1.5f) {
      for (int k = 0; k < W_TAPS; k++) {
        W[k] = 0.0f;
      }
      divergence_flag = true;
    }
  } else {
    // STATE_ANC_BYPASS: Mute speaker output so user can compare ANC ON vs OFF
    writeDAC(0.0f);
  }

  // Accumulate RMS metrics for Serial monitor
  rms_ref_accum += x_n * x_n;
  rms_err_accum += e_n * e_n;
  telemetry_count++;
}

// Timer ISR: times ancStep() with the CPU cycle counter and records the interval between
// consecutive ISR entries (jitter). The measurement itself costs a few cycles.
void FASTRUN ancISR() {
  uint32_t t0 = ARM_DWT_CYCCNT;
  SystemState st = sysState;
  ancStep();
  uint32_t t1 = ARM_DWT_CYCCNT;

  volatile IsrStats &S = isrStats[st];
  uint32_t exec = t1 - t0;
  S.n++;
  S.exec_sum += exec;
  if (exec > S.exec_max) S.exec_max = exec;
  if (exec > PERIOD_CYCLES) S.overruns++;
  if (haveLastIsr) {
    uint32_t iv = t0 - lastIsrCycles;
    if (iv < S.int_min) S.int_min = iv;
    if (iv > S.int_max) S.int_max = iv;
  }
  lastIsrCycles = t0;
  haveLastIsr = true;
}

// ============================================================================
// 6. ARDUINO SETUP & BACKGROUND TELEMETRY LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_DAC_CS, OUTPUT);
  digitalWriteFast(PIN_DAC_CS, HIGH);

  // Configure Teensy 4.0 SAR ADC for fast 12-bit conversions (~3-4 us)
  analogReadResolution(12);
  analogReadAveraging(4);

  SPI.begin();
  writeDAC(0.0f);

  delay(500);
  Serial.println(F("=========================================================="));
  Serial.println(F(" Open-Window Active Noise Control (FxLMS) — Teensy 4.0"));
  Serial.println(F(" Keys: 't' toggle ANC | 'c' recalibrate. Lines: ?timing ?params set anc|mu|leak"));
  Serial.println(F("=========================================================="));

  startCalibration();

  // Start hardware timer interrupt every 250 microseconds (4,000 Hz)
  // Make sure the cycle counter used for ISR timing is running
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  resetIsrStats();

  ancTimer.begin(ancISR, SAMPLE_PERIOD_US);
}

// Pushbutton on PIN_BUTTON: short press toggles ANC/bypass, holding >1 s recalibrates.
// Polled from loop() with a simple debounce; the ISR is not involved.
void handleButton() {
  static bool wasDown = false;
  static uint32_t downSince = 0;
  static bool longHandled = false;
  bool down = (digitalRead(PIN_BUTTON) == LOW);
  uint32_t now = millis();

  if (down && !wasDown) {
    downSince = now;
    longHandled = false;
  } else if (down && wasDown && !longHandled && (now - downSince) > 1000) {
    longHandled = true;
    startCalibration();
  } else if (!down && wasDown && !longHandled && (now - downSince) > 30) {
    if (sysState == STATE_ANC_ACTIVE) {
      sysState = STATE_ANC_BYPASS;
      Serial.println(F("[ANC] Mode -> BYPASS (Speaker Muted)"));
    } else if (sysState == STATE_ANC_BYPASS) {
      sysState = STATE_ANC_ACTIVE;
      Serial.println(F("[ANC] Mode -> ACTIVE"));
    }
  }
  wasDown = down;
}

// ----------------------------------------------------------------------------
// Serial interface.
//  * Single keys (old behaviour): 't' toggle ANC/bypass, 'c' recalibrate.
//  * Line commands for tools/timing_tool.py. Replies start with '@' so they cannot be
//    confused with the human-readable telemetry:
//      ?timing          ISR timing report since the last report (then resets the counters)
//      ?params          current settings
//      set anc on|off   ANC active / bypass (ignored while calibrating)
//      set mu <v>       FxLMS step size, 0.0005 .. 0.1
//      set leak <v>     leakage factor, 0.999 .. 1.0
//      calibrate        start the 3 s secondary-path calibration
// ----------------------------------------------------------------------------
static const char* stateName(int st) {
  return st == STATE_CALIBRATING ? "CALIBRATING" : (st == STATE_ANC_ACTIVE ? "ACTIVE" : "BYPASS");
}

void printTiming() {
  IsrStats snap[3];
  noInterrupts();
  for (int i = 0; i < 3; i++) {
    snap[i].n = isrStats[i].n;
    snap[i].exec_sum = isrStats[i].exec_sum;
    snap[i].exec_max = isrStats[i].exec_max;
    snap[i].int_min = isrStats[i].int_min;
    snap[i].int_max = isrStats[i].int_max;
    snap[i].overruns = isrStats[i].overruns;
  }
  resetIsrStats();
  haveLastIsr = false;                   // the gap while reporting is not ISR jitter
  interrupts();

  const float us_per_cycle = 1.0e6f / (float)F_CPU_ACTUAL;
  for (int i = 0; i < 3; i++) {
    if (snap[i].n == 0) continue;
    Serial.print(F("@TIMING state=")); Serial.print(stateName(i));
    Serial.print(F(" n=")); Serial.print(snap[i].n);
    Serial.print(F(" exec_us_avg=")); Serial.print((float)snap[i].exec_sum / (float)snap[i].n * us_per_cycle, 3);
    Serial.print(F(" exec_us_max=")); Serial.print(snap[i].exec_max * us_per_cycle, 3);
    if (snap[i].int_max > 0) {
      Serial.print(F(" interval_us_min=")); Serial.print(snap[i].int_min * us_per_cycle, 3);
      Serial.print(F(" interval_us_max=")); Serial.print(snap[i].int_max * us_per_cycle, 3);
    }
    Serial.print(F(" overruns=")); Serial.print(snap[i].overruns);
    Serial.print(F(" period_us=")); Serial.println(SAMPLE_PERIOD_US);
  }
  Serial.println(F("@END"));
}

void printParams() {
  Serial.print(F("@PARAMS state=")); Serial.print(stateName(sysState));
  Serial.print(F(" fs_hz=")); Serial.print((int)SAMPLE_RATE_HZ);
  Serial.print(F(" w_taps=")); Serial.print(W_TAPS);
  Serial.print(F(" s_taps=")); Serial.print(S_TAPS);
  Serial.print(F(" mu=")); Serial.print((float)g_mu, 6);
  Serial.print(F(" leak=")); Serial.print((float)g_leak, 6);
  Serial.print(F(" cpu_hz=")); Serial.println((uint32_t)F_CPU_ACTUAL);
}

void handleLine(char* line) {
  if (strcmp(line, "?timing") == 0) {
    printTiming();
  } else if (strcmp(line, "?params") == 0) {
    printParams();
  } else if (strcmp(line, "calibrate") == 0) {
    startCalibration();
    Serial.println(F("@OK calibrating"));
  } else if (strcmp(line, "set anc on") == 0 || strcmp(line, "set anc off") == 0) {
    if (sysState == STATE_CALIBRATING) {
      Serial.println(F("@ERR busy calibrating"));
    } else {
      sysState = (line[8] == 'n') ? STATE_ANC_ACTIVE : STATE_ANC_BYPASS;
      Serial.println(F("@OK anc"));
    }
  } else if (strncmp(line, "set mu ", 7) == 0) {
    float v = atof(line + 7);
    if (v >= 0.0005f && v <= 0.1f) { g_mu = v; Serial.println(F("@OK mu")); }
    else Serial.println(F("@ERR mu out of range 0.0005..0.1"));
  } else if (strncmp(line, "set leak ", 9) == 0) {
    float v = atof(line + 9);
    if (v >= 0.999f && v <= 1.0f) { g_leak = v; Serial.println(F("@OK leak")); }
    else Serial.println(F("@ERR leak out of range 0.999..1.0"));
  } else {
    Serial.println(F("@ERR unknown command"));
  }
}

void handleSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (len == 0 && (c == 't' || c == 'T' || c == 'c' || c == 'C')) {
      // Single-key shortcuts (work with or without a trailing newline in the serial monitor)
      if (c == 't' || c == 'T') {
        if (sysState == STATE_ANC_ACTIVE) {
          sysState = STATE_ANC_BYPASS;
          Serial.println(F("[ANC] Mode -> BYPASS (Speaker Muted)"));
        } else if (sysState == STATE_ANC_BYPASS) {
          sysState = STATE_ANC_ACTIVE;
          Serial.println(F("[ANC] Mode -> ACTIVE"));
        }
      } else {
        startCalibration();
      }
    } else if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = 0;
        handleLine(buf);
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;                           // overlong line: drop it
    }
  }
}

void loop() {
  handleButton();

  handleSerial();

  // Print acoustic telemetry every 0.5 seconds (2,000 samples @ 4 kHz)
  if (telemetry_count >= 2000) {
    noInterrupts();
    float sum_ref = rms_ref_accum;
    float sum_err = rms_err_accum;
    uint32_t count = telemetry_count;
    bool div_reset = divergence_flag;
    rms_ref_accum = 0.0f;
    rms_err_accum = 0.0f;
    telemetry_count = 0;
    divergence_flag = false;
    interrupts();

    if (div_reset) {
      Serial.println(F("[WARN] Impulse overload detected -> Adaptive filter W(z) auto-reset."));
    }

    float rms_ref = sqrtf(sum_ref / (float)count);
    float rms_err = sqrtf(sum_err / (float)count);
    // NOTE: this compares two DIFFERENT microphones (indoor error vs outdoor reference), whose
    // gains and positions differ, so it is only a rough level indicator. To judge the effect of
    // ANC, toggle it ('t' or the button) and compare the Indoor Error RMS between the two states.
    float atten_db = 20.0f * log10f((rms_err + 1e-6f) / (rms_ref + 1e-6f));

    if (sysState == STATE_CALIBRATING) {
      Serial.println(F("[CALIBRATING] Learning speaker & window acoustic path..."));
    } else {
      Serial.print(F("[STATUS: "));
      Serial.print(sysState == STATE_ANC_ACTIVE ? F("ANC ON ] ") : F("BYPASS ] "));
      Serial.print(F("Outdoor RMS: "));
      Serial.print(rms_ref, 4);
      Serial.print(F(" | Indoor Error RMS: "));
      Serial.print(rms_err, 4);
      Serial.print(F(" | Error/Ref level: "));
      Serial.print(atten_db, 1);
      Serial.println(F(" dB"));
    }
  }
}
