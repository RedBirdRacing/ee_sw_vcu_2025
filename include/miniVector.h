#include <stdint.h>

template <typename T, uint8_t N>
struct miniVector
{
    T data[N];
    uint8_t length;

    // element access, add, remove
    /**
     * @brief Access element at index i
     * 
     * @param i Index of the element
     * @return T& Reference to the element
     */
    T& operator[](size_t i) { return data[i]; }

    /**
     * @brief Access element at index i as const
     * 
     * @param i Index of the element
     * @return const T& Reference to the element
     */
    const T& operator[](size_t i) const { return data[i]; }

    /**
     * @brief Get length of the miniVector
     * 
     * @return uint8_t 
     */
    uint8_t length(){ return length;}

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
     * @return bool Success status
     */
    bool remove(T value)
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