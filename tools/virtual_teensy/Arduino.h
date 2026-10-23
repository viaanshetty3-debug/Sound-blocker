// Host stand-in for the parts of the Teensy core that window_anc_fxlms.ino uses.
// Only the virtual Teensy (tools/virtual_teensy.py) builds against this; it is not a
// Teensy emulator. Pins, ADC and DAC are connected to the toy plant in harness.cpp.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define FASTRUN
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define A1 15

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))

extern const uint32_t F_CPU_ACTUAL;

// Cycle counter. The harness decides what it reads (see vt_cyccnt in harness.cpp).
uint32_t vt_cyccnt();
#define ARM_DWT_CYCCNT (vt_cyccnt())
extern uint32_t ARM_DEMCR;
extern uint32_t ARM_DWT_CTRL;
#define ARM_DEMCR_TRCENA (1u << 24)
#define ARM_DWT_CTRL_CYCCNTENA 1u

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWriteFast(uint8_t pin, uint8_t val);
int analogRead(uint8_t pin);
void analogReadResolution(unsigned int bits);
void analogReadAveraging(unsigned int n);
uint32_t millis();
void delay(uint32_t ms);
void noInterrupts();
void interrupts();

class HostSerial {
 public:
  void begin(uint32_t) {}
  int available();
  int read();
  void print(const __FlashStringHelper* s) { write_str(reinterpret_cast<const char*>(s)); }
  void print(const char* s) { write_str(s); }
  void print(char c) { char b[2] = {c, 0}; write_str(b); }
  void print(int v) { print((long long)v); }
  void print(unsigned int v) { print((unsigned long long)v); }
  void print(long v) { print((long long)v); }
  void print(unsigned long v) { print((unsigned long long)v); }
  void print(long long v);
  void print(unsigned long long v);
  void print(double v, int digits = 2);
  template <typename T> void println(T v) { print(v); write_str("\r\n"); }
  void println(double v, int digits) { print(v, digits); write_str("\r\n"); }
  void println() { write_str("\r\n"); }
 private:
  void write_str(const char* s);
};
extern HostSerial Serial;
