#include "Signal_Processing.h"

/**
 * @brief Applies a Finite Impulse Response (FIR) filter on the given buffer.
 * This function convolves the input buffer with the provided kernel and normalizes the result.
 * 
 * @param buffer Pointer to the input buffer containing signal samples.
 * @param kernel Pointer to the FIR filter kernel.
 * @param buf_size Size of the input buffer and kernel.
 * @param kernel_sum Sum of all values in the kernel for normalization.
 * @return The filtered output value.
 */
template <typename T>
constexpr T FIR_filter(T *buffer, float *kernel, int buf_size, float kernel_sum)
{
    if (buffer == nullptr || kernel == nullptr || buf_size <= 0 || kernel_sum == 0.0f)
    {
        return static_cast<T>(0);
    }

    float sum = 0;

    for (int i = 0; i < buf_size; ++i)
    {
        sum += static_cast<float>(buffer[i]) * kernel[i];
    }

    // Kernel sum is the sum of all values in the kernel. This normalize the output value
    return static_cast<T>(sum / kernel_sum);
}

/**
 * @brief Computes the average of two values.
 * 
 * @param val1 The first value.
 * @param val2 The second value.
 * @return The average of val1 and val2.
 */
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
    if (buffer == nullptr || buf_size == 0)
    {
        return static_cast<T>(0);
    }

    T sum = 0;

    for (int i = 0; i < buf_size; ++i)
        sum += buffer[i];
    return sum / buf_size;
}