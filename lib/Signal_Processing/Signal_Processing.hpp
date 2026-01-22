/**
 * @file Signal_Processing.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of signal processing functions
 * @version 1.0
 * @date 2026-01-15
 * @see Signal_Processing.tpp
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