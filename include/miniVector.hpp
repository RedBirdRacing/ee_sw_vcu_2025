#ifndef MINIVECTOR_HPP
#define MINIVECTOR_HPP
#include <stdint.h>

template <typename T, uint8_t N>
struct miniVector
{
    T data[N];
    uint8_t length;

    /**
     * @brief Push element to the end of the miniVector
     * 
     * @param value Element to be pushed, by value
     * @return bool Success status
     */
    bool push(T value)
    {
        if (length >= N)
            return false; // full
        data[length++] = value;
        return true;
    }
    
    /**
     * @brief Remove element from miniVector, then shift left remaining
     * 
     * @param value Element to be removed, by value
     * @return uint8_t index of removed element, or -1 if not found
     */
    uint8_t remove(T value)
    {
        for (int i = 0; i < N; ++i)
        {
            if (data[i] == value)
            {
                // shift elements left
                --length;
                for (int j = i; j < length; ++j)
                {
                    data[j] = data[j + 1];
                }
                return true;
            }
        }
        return false;
    }
};

#endif // MINIVECTOR_HPP