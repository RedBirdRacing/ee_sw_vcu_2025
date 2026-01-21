/**
 * @brief Applies a Finite Impulse Response (FIR) filter on the signal buffer.
 *
 * @tparam T Type of the buffer elements
 * @param buffer Pointer to the input signal buffer
 * @param kernel Pointer to the FIR filter kernel
 * @param buf_size Size of the buffer and kernel
 * @param kernel_sum Sum of all values in the kernel for normalization
 * @return constexpr T Filtered output value
 */

#ifndef SIGNAL_PROCESSING_HPP
#define SIGNAL_PROCESSING_HPP

#include <stdint.h>

template <typename T>
constexpr T FIR_filter(T *buffer, float *kernel, int buf_size, float kernel_sum);

template <typename T>
constexpr T average(T val1, T val2);

template <typename T>
constexpr T AVG_filter(T *buffer, uint8_t buf_size);

#include "Signal_Processing.tpp" // implementation

#endif // SIGNAL_PROCESSING_HPP