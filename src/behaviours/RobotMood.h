#pragma once

#include <stdint.h>

// 1. NEW: The 8-bit identifier for Telemetry
enum class MoodState : uint8_t {
    HAPPY = 0,
    SAD = 1,
    ANGRY = 2,
    SCARED = 3,
    CURIOUS = 4,
    SLEEPY = 5,
    GROGGY = 6
};

// A blueprint of how a mood affects physical movement
struct RobotMood {
    MoodState id;               // <--- ADD THIS: The 1-byte ID tag
    float speedMultiplier;      // 1.0 is normal, 1.5 is fast/angry, 0.5 is sleepy
    float pidAggression;        // Modifies the P-term of your PID controller
    bool allowJerkyMovements;   // A flag to change driving style
    // You can add LED colors, sound frequencies, etc. later!
};

// --- Pre-defined Personalities ---
namespace Moods {

    constexpr unsigned long GROGGY_DURATION_MS = 10000; // 10 seconds

    // 3. The Pre-defined Profiles
    // Update profiles to include their ID tag
    constexpr RobotMood GROGGY = {
        .id = MoodState::GROGGY,
        .speedMultiplier = 0.3f,    // Very slow
        .pidAggression = 0.4f,      // Sluggish balancing
        .allowJerkyMovements = false
    };
    
    constexpr RobotMood HAPPY = {
        .id = MoodState::HAPPY,
        .speedMultiplier = 1.0f,
        .pidAggression = 1.0f,
        .allowJerkyMovements = false
    };

    constexpr RobotMood ANGRY = {
        .id = MoodState::ANGRY,
        .speedMultiplier = 1.8f,    // Drives much faster
        .pidAggression = 1.5f,      // Fights balancing much harder (looks twitchy)
        .allowJerkyMovements = true // Allows sudden stops
    };

    constexpr RobotMood SLEEPY = {
        .id = MoodState::SLEEPY,
        .speedMultiplier = 0.5f,    // Sluggish
        .pidAggression = 0.6f,      // Wobbly balancing
        .allowJerkyMovements = false
    };
}