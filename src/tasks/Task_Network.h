#pragma once

#include <Arduino.h>
#include "core/CommandProcessor.h"
#include "comms/telemetry/TelemetryStreamer.h"
#include "core/BehaviourEngine.h"

// 1. Define the Context Struct
struct NetworkContext {
    CommandProcessor* cli;
    Comms::TelemetryStreamer* router;
    BehaviourEngine* brain;
};

void NetworkTask(void *pvParameters);