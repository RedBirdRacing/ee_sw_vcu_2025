// A library containing simple DSP functions, for ADC filtering, buffer comparisons and more
#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stdint.h>

template <typename T>
T FIR_filter(T *buffer, float *kernel, int buf_size, float kernel_sum);

template <typename T>
T average(T val1, T val2);

template <typename T>
T AVG_filter(T *buffer, uint8_t buf_size);

#endif