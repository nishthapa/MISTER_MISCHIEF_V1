#pragma once
#include <stdint.h>

template <uint8_t SIZE>
class MedianFilter {
private:
    float buffer[SIZE];
    uint8_t index = 0;
    uint8_t count = 0;

public:
    MedianFilter() {
        for(int i = 0; i < SIZE; i++) buffer[i] = 0.0f;
    }

    float addSample(float sample) {
        // 1. Add new sample to the rolling buffer
        buffer[index] = sample;
        index = (index + 1) % SIZE;
        if (count < SIZE) count++;

        // 2. Create a local copy to sort (so we don't destroy the time-history)
        float sorted[SIZE];
        for (uint8_t i = 0; i < count; i++) {
            sorted[i] = buffer[i];
        }

        // 3. Insertion Sort (The fastest algorithm for tiny arrays like 5)
        for (uint8_t i = 1; i < count; i++) {
            float key = sorted[i];
            int8_t j = i - 1;
            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }

        // 4. Return the middle value!
        return sorted[count / 2];
    }
};