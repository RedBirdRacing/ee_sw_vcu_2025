/**
 * @file SignalProcessing.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of signal processing functions
 * @version 2.1
 * @date 2026-01-28
 * @see SignalProcessing.tpp
 * @dir SignalProcessing @brief The SignalProcessing library contains signal processing functions, including the AverageFilter and ExponentialFilter class templates for filtering ADC readings from the pedals.
 */
#ifndef SIGNAL_PROCESSING_HPP
#define SIGNAL_PROCESSING_HPP

#include <stdint.h>

/**
 * @brief Abstract Base Class for signal filters.
 * Defines the interface for adding samples and retrieving filtered values.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 */
template <typename TypeInput, typename TypeMid>
class Filter
{
public:
    virtual void addSample(TypeInput sample) = 0;
    virtual TypeInput getFiltered() const = 0;
};

/**
 * @brief Filter with simple moving average algorithm.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam SIZE Number of samples to average over.
 */
template <typename TypeInput, typename TypeMid, uint16_t SIZE>
class AverageFilter : public Filter<TypeInput, TypeMid>
{
public:
    AverageFilter();
    void addSample(TypeInput sample) override;
    TypeInput getFiltered() const override;

private:
    TypeInput buffer[SIZE]; /**< Circular buffer for storing samples */
    uint16_t index = 0;     /**< Current index in the circular buffer */
};

/**
 * @brief Filter with exponential moving average algorithm.
 * @details Old samples "decay" naturally. The formula used is:
 * f(t) = (f(t-1) * OLD_RATIO + sample * NEW_RATIO + (OLD_RATIO + NEW_RATIO) / 2) / (OLD_RATIO + NEW_RATIO)
 * Due to round down, the results won't ever reach maximum, especially if OLD_RATIO >> NEW_RATIO, so the use of curve is important.
 * @tparam TypeInput Type of the input samples.
 * @tparam TypeMid Type used for intermediate calculations.
 * @tparam OLD_RATIO Weighting ratio for the old value.
 * @tparam NEW_RATIO Weighting ratio for the new sample.
 */
template <typename TypeInput, typename TypeMid, uint8_t OLD_RATIO = 31, uint8_t NEW_RATIO = 1>
class ExponentialFilter : public Filter<TypeInput, TypeMid>
{
public:
    ExponentialFilter();
    void addSample(TypeInput sample) override;
    TypeInput getFiltered() const override;

private:
    TypeInput last_out = 0; /**< Last output value for exponential filter, input for next calculation */
};

#include "SignalProcessing.tpp" // implementation

#endif // SIGNAL_PROCESSING_HPP