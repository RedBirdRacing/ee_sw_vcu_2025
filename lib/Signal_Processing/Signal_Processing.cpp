#include "Signal_Processing.h"

// Apply a FIR filter on the signal buffer
// The buffer size must be the same as the kernel
// Filtered output will be stored in the output_buf
template <typename T>
constexpr T FIR_filter(T *buffer, float *kernel, int buf_size, float kernel_sum)
{
    float sum = 0;

    for (int i = 0; i < buf_size; ++i)
    {
        sum += static_cast<float>(buffer[i]) * kernel[i];
    }

    // Kernel sum is the sum of all values in the kernel. This normalize the output value
    return static_cast<T>(sum / kernel_sum);
}

template <typename T>
constexpr T average(T val1, T val2)
{
    return (val1 + val2) / 2;
}

/**
 * @brief Applies an average filter on the given buffer.
 * This function calculates the average of all elements in the buffer.
 * 
 * @param buffer Pointer to the buffer containing elements to average.
 * @param buf_size Size of the buffer.
 * @return The average value of the elements in the buffer.
 */
template <typename T>
constexpr T AVG_filter(T *buffer, uint8_t buf_size)
{
    T sum = 0;

    for (int i = 0; i < buf_size; ++i)
        sum += buffer[i];
    return sum / buf_size;
}