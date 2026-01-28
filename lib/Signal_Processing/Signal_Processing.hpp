/**
 * @file Signal_Processing.hpp
 * @author Planeson, Red Bird Racing
 * @brief Declaration of signal processing functions
 * @version 2.0
 * @date 2026-01-28
 * @see Signal_Processing.tpp
 */
#ifndef SIGNAL_PROCESSING_HPP
#define SIGNAL_PROCESSING_HPP

#include <stdint.h>

template <typename TypeInput, typename TypeMid, uint16_t Size>
class Filter
{
public:
    virtual void addSample(TypeInput sample) = 0;
    virtual TypeInput getFiltered() const = 0;
};

template <typename TypeInput, typename TypeMid, uint16_t Size>
class AverageFilter : public Filter<TypeInput, TypeMid, Size>
{
public:
    AverageFilter();
    void addSample(TypeInput sample) override;
    TypeInput getFiltered() const override;

private:
    TypeInput buffer[Size];
    uint16_t index = 0;
};

#include "Signal_Processing.tpp" // implementation

#endif // SIGNAL_PROCESSING_HPP