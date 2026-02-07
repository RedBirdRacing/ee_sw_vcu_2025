/**
 * @file SignalProcessing.tpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of Signal Processing functions
 * @version 2.1
 * @date 2026-01-28
 * @see Signal_Processing.hpp
 */

#include "SignalProcessing.hpp"

// === AverageFilter ===

/**
 * @brief Constructor for AverageFilter.
 * Initializes the circular buffer to zero.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam SIZE Number of samples to average over.
 */
template <typename TypeInput, typename TypeMid, uint16_t SIZE>
AverageFilter<TypeInput, TypeMid, SIZE>::AverageFilter() = default;

/**
 * @brief Adds a new sample to the AverageFilter.
 * Stores the sample in the circular buffer and updates the index.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam SIZE Number of samples to average over.
 * @param sample New input sample to add.
 */
template <typename TypeInput, typename TypeMid, uint16_t SIZE>
void AverageFilter<TypeInput, TypeMid, SIZE>::addSample(TypeInput sample)
{
    buffer[index] = sample;
    index = (index + 1) % SIZE;
}

/**
 * @brief Retrieves the filtered value from the AverageFilter.
 * Computes the average of the samples in the circular buffer.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam SIZE Number of samples to average over.
 * @return Filtered average value.
 */
template <typename TypeInput, typename TypeMid, uint16_t SIZE>
TypeInput AverageFilter<TypeInput, TypeMid, SIZE>::getFiltered() const
{
    TypeMid sum = 0;
    for (uint16_t i = 0; i < SIZE; ++i)
    {
        sum += buffer[i];
    }
    return static_cast<TypeInput>(sum / static_cast<TypeMid>(SIZE));
}

// == ExponentialFilter ===

/**
 * @brief Constructor for ExponentialFilter.
 * Initializes the last output value to zero.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam OLD_RATIO Weighting ratio for the old value.
 * @tparam NEW_RATIO Weighting ratio for the new sample.
 */
template <typename TypeInput, typename TypeMid, uint8_t OLD_RATIO, uint8_t NEW_RATIO>
ExponentialFilter<TypeInput, TypeMid, OLD_RATIO, NEW_RATIO>::ExponentialFilter() = default;

/**
 * @brief Adds a new sample to the ExponentialFilter.
 * Updates the filtered value using the exponential moving average formula.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam OLD_RATIO Weighting ratio for the old value.
 * @tparam NEW_RATIO Weighting ratio for the new sample.
 * @param sample New input sample to add.
 */
template <typename TypeInput, typename TypeMid, uint8_t OLD_RATIO, uint8_t NEW_RATIO>
void ExponentialFilter<TypeInput, TypeMid, OLD_RATIO, NEW_RATIO>::addSample(TypeInput sample)
{
    last_out = static_cast<TypeMid>(last_out * OLD_RATIO + sample * NEW_RATIO + (OLD_RATIO + NEW_RATIO) / 2) / (OLD_RATIO + NEW_RATIO);
}

/**
 * @brief Retrieves the filtered value from the ExponentialFilter.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam OLD_RATIO Weighting ratio for the old value.
 * @tparam NEW_RATIO Weighting ratio for the new sample.
 * @return Filtered value.
 */
template <typename TypeInput, typename TypeMid, uint8_t OLD_RATIO, uint8_t NEW_RATIO>
TypeInput ExponentialFilter<TypeInput, TypeMid, OLD_RATIO, NEW_RATIO>::getFiltered() const
{
    return last_out;
}