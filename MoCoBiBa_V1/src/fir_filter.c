#include "fir_filter.h"

static int16_t filter_taps[SAMPLEFILTER_TAP_NUM] = {
  -33,
  238,
  703,
  1484,
  2512,
  3617,
  4559,
  5101,
  5101,
  4559,
  3617,
  2512,
  1484,
  703,
  238,
  -33
};

void SampleFilter_init(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, int16_t input) {
  f->history[(f->last_index++) & 15] = input;
}

int16_t SampleFilter_get(SampleFilter* f) {
  int32_t acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    acc += (int32_t)f->history[(index--) & 15] * filter_taps[i];
  };
  return acc >> 16;
}