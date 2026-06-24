#pragma once

#include "core/interfaces/IBehaviourDecider.h"
#include "utils/RemoteLogger.h"

extern RemoteLogger logger;

class AIDecider : public IBehaviourDecider {
public:
    AIDecider() {}

    void evaluate(const GlobalDataBank& dataBus, SystemMode currentMode, SystemMode& outRequestedMode, RobotMood& outMood) override {
        
        // --- SKELETON OUTPUT ---
        // Tomorrow, the TensorFlow Lite inference will replace these two lines!
        outRequestedMode = SystemMode::MODE_NORMAL_DRIVING;
        outMood = Moods::HAPPY; 

        // Just a friendly debug ping so you know the AI brain is successfully running
        static unsigned long lastLogTime = 0;
        if (millis() - lastLogTime > 5000) {
            logger.println("[AI-BRAIN] Active, but Neural Network is missing. Defaulting to Normal Driving.");
            lastLogTime = millis();
        }
    }
};