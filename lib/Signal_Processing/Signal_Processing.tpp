/**
 * @file Signal_Processing.tpp
 * @author Planeson, Red Bird Racing
 * @brief Definition of Signal Processing functions
 * @version 2.0
 * @date 2026-01-28
 * @see Signal_Processing.hpp
 */

#include "Signal_Processing.hpp"

// === AverageFilter ===
template <typename TypeInput, typename TypeMid, uint16_t Size>
AverageFilter<TypeInput, TypeMid, Size>::AverageFilter() = default;

template <typename TypeInput, typename TypeMid, uint16_t Size>
void AverageFilter<TypeInput, TypeMid, Size>::addSample(TypeInput sample)
{
    buffer[index] = sample;
    index = (index + 1) % Size;
}

template <typename TypeInput, typename TypeMid, uint16_t Size>
TypeInput AverageFilter<TypeInput, TypeMid, Size>::getFiltered() const
{
    TypeMid sum = 0;
    for (uint16_t i = 0; i < Size; ++i)
    {
        sum += buffer[i];
    }
    return static_cast<TypeInput>(sum / static_cast<TypeMid>(Size));
}

