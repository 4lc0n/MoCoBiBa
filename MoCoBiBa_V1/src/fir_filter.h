#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <stdlib.h>
#include <stdint.h>

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 100 Hz

fixed point precision: 16 bits

* 0 Hz - 4 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 15 Hz - 50 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SAMPLEFILTER_TAP_NUM 16

typedef struct {
  int16_t history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, int16_t input);
int16_t SampleFilter_get(SampleFilter* f);

#endif