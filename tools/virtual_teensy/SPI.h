// Host stand-in for the Teensy SPI library: transfer16() hands the MCP4921 command word
// to the virtual DAC in harness.cpp.
#pragma once
#include <stdint.h>

#define MSBFIRST 1
#define SPI_MODE0 0

struct SPISettings {
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

void vt_dac_command(uint16_t cmd);

class HostSPI {
 public:
  void begin() {}
  void beginTransaction(const SPISettings&) {}
  void endTransaction() {}
  uint16_t transfer16(uint16_t cmd) { vt_dac_command(cmd); return 0; }
};
extern HostSPI SPI;
