#ifndef QUEUE_HPP
#define QUEUE_HPP

#include <stdint.h>
// using uint8_t for size
// highest capacity is 255
template <typename T, uint8_t size>
class RingBuffer
{
public:
    // Constructor
    // Initializes the buffer and head pointer
    constexpr RingBuffer() : buffer{0}, head(0), count(0) {}

    /**
     * @brief Pushes a new value into the ring buffer.
     * This will overwrite the oldest value if the buffer is full.
     *
     * @param val The value to be added to the buffer.
     */
    void push(T val)
    {
        buffer[head] = val;
        head = (head + 1) % size;
        if (count < size)
            ++count;
    }

    /**
     * @brief Returns the elements in the buffer in linear order.
     *
     * @param out Pointer to an array where the linear buffer will be stored. This array shall be created by the caller and must have at least 'size' elements.
     * @note The order of elements in the output array will be from oldest to newest.
     */
    void getLinearBuffer(T *out)
    {
        if (out == nullptr)
            return;

        for (int i = 0; i < count; ++i)
        {
            out[i] = buffer[(head + i) % size];
        }
    }

    T buffer[size];
    uint8_t head;
    uint8_t count;
};

#endif // QUEUE_HPP