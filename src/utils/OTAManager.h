#pragma once

#include <Arduino.h>

class OTAManager {
public:
    static void begin();
    static void handle();
};