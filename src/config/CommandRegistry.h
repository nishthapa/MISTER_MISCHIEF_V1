#pragma once
#include <Arduino.h>
#include "utils/RemoteLogger.h"

namespace CLI {
    // The super-efficient integer codes for our commands
    enum class CommandCode {
        CALIB_GYRO,
        CALIB_ACCEL,
        CALIB_MAG,
        CALIB_SONAR,
        UNKNOWN
    };

    // The Parser: Converts the string from Putty into the integer code
    inline CommandCode parseCommand(String cmd) {
        cmd.trim();
        cmd.toLowerCase();

        if (cmd == "calib_gyro")  return CommandCode::CALIB_GYRO;
        if (cmd == "calib_accel") return CommandCode::CALIB_ACCEL;
        if (cmd == "calib_mag")   return CommandCode::CALIB_MAG;
        if (cmd == "calib_sonar") return CommandCode::CALIB_SONAR;
        
        return CommandCode::UNKNOWN;
    }

    // The Help Menu
    inline void printHelpMenu(class RemoteLogger& logger) {
        logger.println("\n===========================================");
        logger.println("INVALID COMMAND. Available Commands:");
        logger.println("  calib_gyro   : Calibrates the Gyroscope (Keep robot perfectly still)");
        logger.println("  calib_accel  : Calibrates the Accelerometer (Ensure robot is perfectly level)");
        logger.println("  calib_mag    : Calibrates the Compass (Rotate robot in all 3D axes)");
        logger.println("  calib_sonar  : Calibrates environmental offset for HC-SR04");
        logger.println("===========================================\n");
    }
}