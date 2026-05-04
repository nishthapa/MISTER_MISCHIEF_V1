#pragma once

// A blueprint of how a mood affects physical movement
struct RobotMood {
    float speedMultiplier;      // 1.0 is normal, 1.5 is fast/angry, 0.5 is sleepy
    float pidAggression;        // Modifies the P-term of your PID controller
    bool allowJerkyMovements;   // A flag to change driving style
    // You can add LED colors, sound frequencies, etc. later!
};

// --- Pre-defined Personalities ---
namespace Moods {
    constexpr RobotMood HAPPY = {
        .speedMultiplier = 1.0f,
        .pidAggression = 1.0f,
        .allowJerkyMovements = false
    };

    constexpr RobotMood ANGRY = {
        .speedMultiplier = 1.8f,    // Drives much faster
        .pidAggression = 1.5f,      // Fights balancing much harder (looks twitchy)
        .allowJerkyMovements = true // Allows sudden stops
    };

    constexpr RobotMood SLEEPY = {
        .speedMultiplier = 0.5f,    // Sluggish
        .pidAggression = 0.6f,      // Wobbly balancing
        .allowJerkyMovements = false
    };
}