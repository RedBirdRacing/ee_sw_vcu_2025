#include <stdint.h>

template <typename T, uint8_t N>
struct miniArray
{
    T data[N];

    // element access
    constexpr T& operator[](size_t i) { return data[i]; }
    constexpr const T& operator[](size_t i) const { return data[i]; }
};