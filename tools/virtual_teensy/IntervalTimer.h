// Host stand-in for IntervalTimer: the harness calls the stored function every virtual tick.
#pragma once
#include <stdint.h>

void vt_timer_begin(void (*fn)(), uint32_t period_us);

class IntervalTimer {
 public:
  bool begin(void (*fn)(), uint32_t period_us) { vt_timer_begin(fn, period_us); return true; }
};
