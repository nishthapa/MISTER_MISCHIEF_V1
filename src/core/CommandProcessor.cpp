#include "core/CommandProcessor.h"
#include "config/ConfigurationManager.h"
#include "config/FactoryDefaults.h"
#include "config/PinConfig.h"
#include "config/I2CRegistry.h"
#include "config/ChannelRegistry.h"
#include "config/SystemConfig.h"
#include "utils/RemoteLogger.h"
#include "utils/RadioManager.h"
#include "hal/interfaces/I_IMU.h" // for calibration commands
#include "core/PIDController.h"
#include "core/BehaviourEngine.h"
#include "behaviours/Mode_AutoTune.h"

extern RemoteLogger logger;
extern I_IMU* imu; // Reaches into main.cpp to grab the global IMU!

extern BehaviourEngine brain;
extern Mode_AutoTune autotuneMode;

// --- All the PID controllers for live-tuning access in the CLI ---
extern PIDController pointTurnPID;
extern PIDController arcTurnPID;
extern PIDController distancePID;

CommandProcessor::CommandProcessor() {
    // ==========================================
    // THE DYNAMIC COMMAND REGISTRATION
    // ==========================================
    // We bind the string command to the specific memory address of the handler function.
    registry.registerCommand("set",   std::bind(&CommandProcessor::handleSet,   this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("get",   std::bind(&CommandProcessor::handleGet,   this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("reset", std::bind(&CommandProcessor::handleReset, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("calib", std::bind(&CommandProcessor::handleCalib, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("autotune", std::bind(&CommandProcessor::handleAutotune, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("connect",    std::bind(&CommandProcessor::handleConnect,    this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("disconnect", std::bind(&CommandProcessor::handleDisconnect, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("connect",    std::bind(&CommandProcessor::handleConnect,    this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("disconnect", std::bind(&CommandProcessor::handleDisconnect, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("reboot",     std::bind(&CommandProcessor::handleReboot,     this, std::placeholders::_1, std::placeholders::_2));
}

// ==========================================
// TERMINAL EMULATOR ENGINE
// ==========================================

// The Autocomplete Dictionary
const char* autoDict[] = {
    "set", "get", "reset", "calib", "default", "ALL",
    "connect", "disconnect", "reboot", "wifi", "bluetooth",
    "SERIAL_BAUD_RATE", "CRUISING_SPEED", "OBSTACLE_TRIGGER_CM", "MAINTAIN_DISTANCE_CM", "MOTOR_MIN_PWM",
    "OBS_SWEEP_ANGLE", "OBS_SWEEP_SPEED", "OBS_SWEEP_PAUSE", "OBS_CLEAR_THRESH", "OBS_HYSTERESIS",
    "OBSTACLE_ESCAPE_BASE_SPEED", "OBSTACLE_BACKUP_DURATION_MS", "OBSTACLE_SWEEP_ANGLE_DEG", "OBSTACLE_SWEEP_TIMEOUT_MS",
    "OBSTACLE_ALIGN_TIMEOUT_MS", "OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG", "OBSTACLE_ESCAPE_DURATION_MS",
    "DIZZY_SPIN_PWM", "DIZZY_SPIN_TIME", "DIZZY_COOLDOWN", "SLEEP_TIMEOUT_MS", "SLEEP_WAKE_G",
    "COMPASS_LOCK_ENTRY_SETTLE_MS", "COMPASS_LOCK_EXIT_SETTLE_MS",
    "IMU_GYRO_DEADBAND", "SONAR_MAX_DIST",
    "MADGWICK_FILTER_BETA",
    "AUTOTUNE_START_DELAY_MS", "AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS"
    "PID_POINT_P", "PID_POINT_I", "PID_POINT_D", "PID_POINT_LIM", "PID_POINT_ILIM", "PID_POINT_DEAD",
    "PID_ARC_P", "PID_ARC_I", "PID_ARC_D", "PID_ARC_LIM", "PID_ARC_ILIM", "PID_ARC_DEAD",
    "PID_DIST_P", "PID_DIST_I", "PID_DIST_D", "PID_DIST_LIM", "PID_DIST_ILIM", "PID_DIST_DEAD",
    "PID_OBSTACLE_P", "PID_OBSTACLE_I", "PID_OBSTACLE_D", "PID_OBSTACLE_LIM", "PID_OBSTACLE_ILIM", "PID_OBSTACLE_DEAD",
    "TILT_HANDLING_THRESHOLD", "GFORCE_LIFT_UP_THRESHOLD", "GFORCE_LIFT_DOWN_THRESHOLD", "LIFT_ENERGY_SPIKE_THRESHOLD",
    "UPRIGHT_ANGLE_TOLERANCE", "PERFECTLY_STILL_ENERGY", "STEADY_HOLD_ENERGY_MAX",
    "DIZZY_ENERGY_DEADBAND", "DIZZY_CHARGE_RATE", "DIZZY_DECAY_RATE", "DIZZY_TRIGGER_THRESHOLD",
    "ENERGY_EMA_ALPHA", "ENERGY_EMA_BETA",
    "DISTANCE_HOLD_FRUSTRATION_LIMIT", "FRUSTRATION_COOLDOWN_RATE", "FRUSTRATION_HEATUP_RATE",
    "BRAIN_ACTIVE", "SERIAL_DEBUG_MASTER", "SERIAL_DEBUG_IMU", "SERIAL_DEBUG_SONAR", "SERIAL_DEBUG_MOTOR_DRIVER",
    "DEBUG_ACTIVE", "DEBUG_USB", "DEBUG_WIFI", "DEBUG_BLUETOOTH", "ACTIVE_DEBUG_MODE",
    "WIFI_SSID", "WIFI_PASSWORD", "WIFI_ACTIVE", "BT_NAME", "BT_ACTIVE",
    "PIN_MOTOR_LEFT_FWD", "PIN_MOTOR_LEFT_REV", "PIN_MOTOR_RIGHT_FWD", "PIN_MOTOR_RIGHT_REV",
    "PIN_SONAR_TRIG", "PIN_SONAR_ECHO", "PIN_I2C_SCL", "PIN_I2C_SDA", "PIN_IMU_INT",
    "I2C_ADDR_MPU6050", "CH_MOTOR_LEFT_FWD", "CH_MOTOR_RIGHT_FWD", "TELNET_PORT", "MAIN_LOOP_TICK_RATE_MS"
};

const int dictSize = sizeof(autoDict) / sizeof(autoDict[0]);

const char* sysVariables[] = {
    "SERIAL_BAUD_RATE", "CRUISING_SPEED", "OBSTACLE_TRIGGER_CM", "MAINTAIN_DISTANCE_CM", "MOTOR_MIN_PWM",
    "OBS_SWEEP_ANGLE", "OBS_SWEEP_SPEED", "OBS_SWEEP_PAUSE", "OBS_CLEAR_THRESH", "OBS_HYSTERESIS",
    "OBSTACLE_ESCAPE_BASE_SPEED", "OBSTACLE_BACKUP_DURATION_MS", "OBSTACLE_SWEEP_ANGLE_DEG", "OBSTACLE_SWEEP_TIMEOUT_MS",
    "OBSTACLE_ALIGN_TIMEOUT_MS", "OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG", "OBSTACLE_ESCAPE_DURATION_MS",
    "DIZZY_SPIN_PWM", "DIZZY_SPIN_TIME", "DIZZY_COOLDOWN", "SLEEP_TIMEOUT_MS", "SLEEP_WAKE_G",
    "COMPASS_LOCK_ENTRY_SETTLE_MS", "COMPASS_LOCK_EXIT_SETTLE_MS",
    "IMU_GYRO_DEADBAND", "SONAR_MAX_DIST",
    "MADGWICK_FILTER_BETA",
    "AUTOTUNE_START_DELAY_MS", "AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS",
    "PID_POINT_P", "PID_POINT_I", "PID_POINT_D", "PID_POINT_LIM", "PID_POINT_ILIM", "PID_POINT_DEAD",
    "PID_ARC_P", "PID_ARC_I", "PID_ARC_D", "PID_ARC_LIM", "PID_ARC_ILIM", "PID_ARC_DEAD",
    "PID_DIST_P", "PID_DIST_I", "PID_DIST_D", "PID_DIST_LIM", "PID_DIST_ILIM", "PID_DIST_DEAD",
    "PID_OBSTACLE_P", "PID_OBSTACLE_I", "PID_OBSTACLE_D", "PID_OBSTACLE_LIM", "PID_OBSTACLE_ILIM", "PID_OBSTACLE_DEAD",
    "TILT_HANDLING_THRESHOLD", "GFORCE_LIFT_UP_THRESHOLD", "GFORCE_LIFT_DOWN_THRESHOLD", "LIFT_ENERGY_SPIKE_THRESHOLD",
    "UPRIGHT_ANGLE_TOLERANCE", "PERFECTLY_STILL_ENERGY", "STEADY_HOLD_ENERGY_MAX",
    "DIZZY_ENERGY_DEADBAND", "DIZZY_CHARGE_RATE", "DIZZY_DECAY_RATE", "DIZZY_TRIGGER_THRESHOLD",
    "ENERGY_EMA_ALPHA", "ENERGY_EMA_BETA",
    "DISTANCE_HOLD_FRUSTRATION_LIMIT", "FRUSTRATION_COOLDOWN_RATE", "FRUSTRATION_HEATUP_RATE",
    "BRAIN_ACTIVE", "SERIAL_DEBUG_MASTER", "SERIAL_DEBUG_IMU", "SERIAL_DEBUG_SONAR", "SERIAL_DEBUG_MOTOR_DRIVER",
    "DEBUG_ACTIVE", "DEBUG_USB", "DEBUG_WIFI", "DEBUG_BLUETOOTH", "ACTIVE_DEBUG_MODE",
    "WIFI_SSID", "WIFI_PASSWORD", "WIFI_ACTIVE", "BT_NAME", "BT_ACTIVE",
    // Hardware specific static variables that we want to be able to read but not write through the CLI:
    "PIN_MOTOR_LEFT_FWD", "PIN_MOTOR_LEFT_REV", "PIN_MOTOR_RIGHT_FWD", "PIN_MOTOR_RIGHT_REV",
    "PIN_SONAR_TRIG", "PIN_SONAR_ECHO", "PIN_I2C_SCL", "PIN_I2C_SDA", "PIN_IMU_INT",
    "I2C_ADDR_MPU6050", "CH_MOTOR_LEFT_FWD", "CH_MOTOR_RIGHT_FWD", "TELNET_PORT", "MAIN_LOOP_TICK_RATE_MS"
};

const int sysVarCount = sizeof(sysVariables) / sizeof(sysVariables[0]);

void CommandProcessor::redrawCLI() {
    // 1. Carriage Return to the start of the line
    Serial.print("\r");
    
    // 2. Draw the pristine prompt and the current buffer
    Serial.print("mischief> ");
    Serial.print(cliBuffer);
    
    // 3. THE SMART ERASER
    // Calculate exactly how many ghost characters are left over from the last draw
    int charsToErase = lastBufferLength - cliBuffer.length();
    if (charsToErase < 0) charsToErase = 0;
    
    // Add 1 extra space just as a safety buffer
    int spacesToPrint = charsToErase + 1;
    
    // Print exactly enough spaces to wipe the ghosts away cleanly
    for (int i = 0; i < spacesToPrint; i++) {
        Serial.print(" ");
    }
    
    // 4. Walk the cursor back over the spaces we just printed, PLUS the actual cursor offset
    int spacesToMoveBack = spacesToPrint + (cliBuffer.length() - cursorPos);
    for (int i = 0; i < spacesToMoveBack; i++) {
        Serial.print("\b");
    }
    
    // 5. Save the new length so the eraser knows what to do next time!
    lastBufferLength = cliBuffer.length();
}

void CommandProcessor::processChar(char c) {
    // Reset tab cycle if any other key is pressed
    if (c != '\t') isTabbing = false;

    // --- ANSI ESCAPE SEQUENCE DECODER (Arrows) ---
    if (c == 27) { 
        delay(5); 
        if (Serial.available() >= 2) {
            char bracket = Serial.read();
            char arrow = Serial.read();
            if (bracket == '[') {
                if (arrow == 'A') { // UP ARROW
                    if (historyCount > 0) {
                        if (historyIndex == -1) historyIndex = historyCount - 1; 
                        else if (historyIndex > 0) historyIndex--;               
                        cliBuffer = cmdHistory[historyIndex];
                        cursorPos = cliBuffer.length(); 
                        redrawCLI();
                    }
                } 
                else if (arrow == 'B') { // DOWN ARROW
                    if (historyIndex != -1) {
                        if (historyIndex < historyCount - 1) {
                            historyIndex++;
                            cliBuffer = cmdHistory[historyIndex];
                        } else {
                            historyIndex = -1; 
                            cliBuffer = "";
                        }
                        cursorPos = cliBuffer.length();
                        redrawCLI();
                    }
                }
                else if (arrow == 'C') { // RIGHT ARROW (Optimized ASCII)
                    if (cursorPos < cliBuffer.length()) {
                        Serial.print(cliBuffer[cursorPos]); // Reprint the char to push cursor right!
                        cursorPos++;
                    }
                }
                else if (arrow == 'D') { // LEFT ARROW (Optimized ASCII)
                    if (cursorPos > 0) {
                        cursorPos--;
                        Serial.print("\b"); // Standard backspace physically moves cursor left!
                    }
                }
            }
        }
        return; 
    }

    if (c == '\r') return; 

    // --- TAB AUTOCOMPLETE ---
    if (c == '\t') {
        if (!isTabbing) {
            wordStartIndex = cliBuffer.lastIndexOf(' ', cursorPos - 1) + 1;
            if (wordStartIndex < 0) wordStartIndex = 0;
            tabPrefix = cliBuffer.substring(wordStartIndex, cursorPos);
            tabPrefix.toLowerCase(); 
            tabDictIndex = 0;
        } else {
            tabDictIndex++; 
        }

        for (int i = 0; i < dictSize; i++) {
            int checkIndex = (tabDictIndex + i) % dictSize; 
            String dictWord = String(autoDict[checkIndex]);
            String lowerDict = dictWord;
            lowerDict.toLowerCase();

            if (lowerDict.startsWith(tabPrefix)) {
                tabDictIndex = checkIndex;
                cliBuffer = cliBuffer.substring(0, wordStartIndex) + dictWord + cliBuffer.substring(cursorPos);
                cursorPos = wordStartIndex + dictWord.length();
                redrawCLI();
                break;
            }
        }
        isTabbing = true;
        return;
    }

    // --- BACKSPACE DECODER ---
    if (c == '\b' || c == 127) { 
        if (cursorPos > 0) {
            if (cursorPos == cliBuffer.length()) {
                // Optimized: Backspacing at the very end
                cliBuffer.remove(cliBuffer.length() - 1);
                cursorPos--;
                Serial.print("\b \b"); 
                lastBufferLength = cliBuffer.length(); // <-- ADD THIS
            } else {
                // Splicing in the middle
                cliBuffer = cliBuffer.substring(0, cursorPos - 1) + cliBuffer.substring(cursorPos);
                cursorPos--;
                redrawCLI(); 
            }
        }
        return;
    }

    // --- ENTER KEY ---
    if (c == '\n') {
        Serial.println(); 
        if (cliBuffer.length() > 0) { 
            
            // SAVE TO HISTORY
            if (historyCount == 0 || cmdHistory[historyCount - 1] != cliBuffer) {
                if (historyCount < HISTORY_MAX) {
                    cmdHistory[historyCount++] = cliBuffer;
                } else {
                    for (int i = 0; i < HISTORY_MAX - 1; i++) cmdHistory[i] = cmdHistory[i + 1];
                    cmdHistory[HISTORY_MAX - 1] = cliBuffer;
                }
            }
            historyIndex = -1; 

            processInput(cliBuffer); 
            
            cliBuffer = ""; 
            cursorPos = 0; 
            lastBufferLength = 0; // <-- ADD THIS TO RESET THE ERASER!

        } else {
            // EMERGENCY BRAKE
            if (Config.SERIAL_DEBUG_MASTER) {
                Config.SERIAL_DEBUG_MASTER = false;
                ConfigSys.save();
                logger.println("[SYSTEM] Telemetry Paused! Type 'set SERIAL_DEBUG_MASTER on' to resume.");
            }
        }
        Serial.print("mischief> "); 
        return;
    } 

    // --- STANDARD TYPING ---
    if (c >= 32 && c <= 126) { 
        if (cursorPos == cliBuffer.length()) {
            // Optimized: Typing at the very end of the string
            cliBuffer += c;
            cursorPos++;
            Serial.print(c);
        } else {
            // Splicing in the middle (Requires full redraw)
            cliBuffer = cliBuffer.substring(0, cursorPos) + c + cliBuffer.substring(cursorPos);
            cursorPos++;
            redrawCLI();
        }
    }
}

void CommandProcessor::processInput(String input) {
    input.trim();
    if (input.length() == 0) return;

    // --- STATE MACHINE: Are we waiting for Y/N? ---
    if (waitingForResetConfirm) { handleResetConfirm(input); return; }
    if (waitingForAutotuneConfirm) { handleAutotuneConfirm(input); return; } // Waiting for autotune y/n confirmation

    // 1. EXTRACT COMMAND
    int firstSpace = input.indexOf(' ');
    String cmd = (firstSpace == -1) ? input : input.substring(0, firstSpace);
    cmd.toLowerCase();

    String remainder = (firstSpace == -1) ? "" : input.substring(firstSpace + 1);
    remainder.trim();

    // 2. EXTRACT VARIABLE NAME
    int secondSpace = remainder.indexOf(' ');
    String varName = (secondSpace == -1) ? remainder : remainder.substring(0, secondSpace);
    varName.toUpperCase();

    // 3. EXTRACT VALUE (The Quote-Preserving Tokenizer!)
    String valStr = "";
    remainder = (secondSpace == -1) ? "" : remainder.substring(secondSpace + 1);
    remainder.trim();

    if (remainder.startsWith("\"")) {
        // It's a quoted string! Find the closing quote.
        int closingQuote = remainder.indexOf('"', 1);
        if (closingQuote != -1) {
            // PRESERVE the quotes so the handler can validate them!
            valStr = remainder.substring(0, closingQuote + 1); 
        } else {
            // User forgot the closing quote
            valStr = remainder;
        }
    } else {
        // Standard single-word value. Read exactly until the next space.
        int thirdSpace = remainder.indexOf(' ');
        if (thirdSpace != -1) {
            valStr = remainder.substring(0, thirdSpace);
        } else {
            valStr = remainder; 
        }
    }

    // 4. ASK THE REGISTRY TO EXECUTE IT
    if (!registry.executeCommand(cmd, varName, valStr)) {
        logger.printf("Unknown command: %s\n", cmd.c_str());
        logger.printf("Available commands: %s\n", registry.getAvailableCommands().c_str());
    }
}

/*void CommandProcessor::handleSet(String varName, String valStr) {
    if (varName == "") {
        logger.println("Usage: set <VARIABLE> <VALUE>");
        return;
    }

    // --- THE EDGE CASE FIX ---
    if (valStr == "") {
        logger.printf("Error: Please provide a value to set for %s\n", varName.c_str());
        return;
    }

    // ==========================================
    // CONTEXT-AWARE STRING VALIDATION
    // ==========================================
    bool isStringVariable = (varName == "WIFI_SSID" || varName == "WIFI_PASSWORD" || varName == "BT_NAME");
    
    if (isStringVariable) {
        // Check if it is perfectly wrapped in double quotes
        if (valStr.startsWith("\"") && valStr.endsWith("\"") && valStr.length() >= 2) {
            // Valid! Strip the quotes off so we don't save literal quotes to the hard drive
            valStr = valStr.substring(1, valStr.length() - 1);
        } else {
            // Invalid! Block the save and educate the user.
            logger.println("\n[ERROR] Text variables must be enclosed in double quotes!");
            logger.printf("Example: set %s \"My Value\"\n", varName.c_str());
            return; // Abort the command entirely
        }
    }

    if (varName == "CRUISING_SPEED") { Config.CRUISING_SPEED = valStr.toFloat(); }
    else if (varName == "OBSTACLE_TRIGGER_CM") { Config.OBSTACLE_TRIGGER_CM = valStr.toFloat(); }
    else if (varName == "MAINTAIN_DIST_CM") { Config.MAINTAIN_DIST_CM = valStr.toFloat(); }

    // --- NETWORK VARS ---
    else if (varName == "WIFI_SSID") { Config.WIFI_SSID = valStr; }
    else if (varName == "WIFI_PASSWORD") { Config.WIFI_PASSWORD = valStr; }
    else if (varName == "BT_NAME") { Config.BT_NAME = valStr; }
    else if (varName == "WIFI_ACTIVE") { 
        valStr.toLowerCase();
        Config.WIFI_ACTIVE = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    else if (varName == "BT_ACTIVE") { 
        valStr.toLowerCase();
        Config.BT_ACTIVE = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
 
    // --- SYSTEM VARS ---
    else if (varName == "BRAIN_ACTIVE") { 
        valStr.toLowerCase();
        Config.BRAIN_ACTIVE = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    
    // --- DEBUG VARS ---
    else if (varName == "SERIAL_DEBUG_MASTER") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_MASTER = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    
    /* FOR LATER: Granular debug controls for each subsystem!
    else if (varName == "SERIAL_DEBUG_IMU") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_IMU = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    else if (varName == "SERIAL_DEBUG_SONAR") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_SONAR = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    else if (varName == "SERIAL_DEBUG_MOTOR_DRIVER") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_MOTOR_DRIVER = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }//
    
    else {
        logger.printf("Unknown variable: %s\n", varName.c_str());
        return;
    }

    ConfigSys.save();
    logger.printf("Successfully set %s to %s\n", varName.c_str(), valStr.c_str());
}*/


// ==========================================
// THE SPECIFIC HANDLERS
// ==========================================
void CommandProcessor::handleSet(String varName, String valStr) {
    if (varName == "") {
        logger.println("Usage: set <VARIABLE> <VALUE>");
        return;
    }

    // --- THE EDGE CASE FIX ---
    if (valStr == "") {
        logger.printf("Error: Please provide a value to set for %s\n", varName.c_str());
        return;
    }

    // ==========================================
    // CONTEXT-AWARE STRING VALIDATION
    // ==========================================
    bool isStringVariable = (varName == "WIFI_SSID" || varName == "WIFI_PASSWORD" || varName == "BT_NAME");
    
    if (isStringVariable) {
        // Check if it is perfectly wrapped in double quotes
        if (valStr.startsWith("\"") && valStr.endsWith("\"") && valStr.length() >= 2) {
            // Valid! Strip the quotes off so we don't save literal quotes to the hard drive
            valStr = valStr.substring(1, valStr.length() - 1);
        } else {
            // Invalid! Block the save and educate the user.
            logger.println("\n[ERROR] Text variables must be enclosed in double quotes!");
            logger.printf("Example: set %s \"My Value\"\n", varName.c_str());
            return; // Abort the command entirely
        }
    }

    // Catch attempts to modify read-only variables
    if (varName.startsWith("PIN_") || varName.startsWith("CH_") || varName.startsWith("I2C_")) {
        logger.printf("[ERROR] %s is a physical hardware binding and cannot be changed at runtime.\n", varName.c_str());
        return;
    }

    // ==========================================
    // VARIABLE ASSIGNMENT & TYPE CONVERSION
    // ==========================================
    if (varName == "SERIAL_BAUD_RATE") { Config.SERIAL_BAUD_RATE = valStr.toInt(); }

    // --- MOVEMENT TUNING VARS ---
    else if (varName == "CRUISING_SPEED") { Config.CRUISING_SPEED = valStr.toFloat(); }
    else if (varName == "OBSTACLE_TRIGGER_CM") { Config.OBSTACLE_TRIGGER_CM = valStr.toFloat(); }
    else if (varName == "MAINTAIN_DISTANCE_CM") { Config.MAINTAIN_DISTANCE_CM = valStr.toFloat(); }
    else if (varName == "MOTOR_MIN_PWM") { Config.MOTOR_MIN_PWM = valStr.toInt(); }

    // --- OBSTACLE AVOIDANCE TUNING VARS PART 1 ---
    else if (varName == "OBS_SWEEP_ANGLE") { Config.OBS_SWEEP_ANGLE = valStr.toFloat(); }
    else if (varName == "OBS_SWEEP_SPEED") { Config.OBS_SWEEP_SPEED = valStr.toFloat(); }
    else if (varName == "OBS_SWEEP_PAUSE") { Config.OBS_SWEEP_PAUSE = valStr.toInt(); }
    else if (varName == "OBS_CLEAR_THRESH") { Config.OBS_CLEAR_THRESH = valStr.toFloat(); }
    else if (varName == "OBS_HYSTERESIS") { Config.OBS_HYSTERESIS = valStr.toFloat(); }

    // --- OBSTACLE AVOIDANCE TUNING VARS PART 2 ---
    else if (varName == "OBSTACLE_ESCAPE_BASE_SPEED") { Config.OBSTACLE_ESCAPE_BASE_SPEED = valStr.toFloat(); }
    else if (varName == "OBSTACLE_BACKUP_DURATION_MS") { Config.OBSTACLE_BACKUP_DURATION_MS = valStr.toInt(); }
    else if (varName == "OBSTACLE_SWEEP_ANGLE_DEG") { Config.OBSTACLE_SWEEP_ANGLE_DEG = valStr.toFloat(); }
    else if (varName == "OBSTACLE_SWEEP_TIMEOUT_MS") { Config.OBSTACLE_SWEEP_TIMEOUT_MS = valStr.toInt(); }
    else if (varName == "OBSTACLE_ALIGN_TIMEOUT_MS") { Config.OBSTACLE_ALIGN_TIMEOUT_MS = valStr.toInt(); }
    else if (varName == "OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG") { Config.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG = valStr.toFloat(); }
    else if (varName == "OBSTACLE_ESCAPE_DURATION_MS") { Config.OBSTACLE_ESCAPE_DURATION_MS = valStr.toInt(); }

    // --- MODE TUNING VARS ---
    else if (varName == "DIZZY_SPIN_PWM") { Config.DIZZY_SPIN_PWM = valStr.toInt(); }
    else if (varName == "DIZZY_SPIN_TIME") { Config.DIZZY_SPIN_TIME = valStr.toInt(); }
    else if (varName == "DIZZY_COOLDOWN") { Config.DIZZY_COOLDOWN = valStr.toInt(); }
    else if (varName == "SLEEP_TIMEOUT_MS") { Config.SLEEP_TIMEOUT_MS = valStr.toInt(); }
    else if (varName == "SLEEP_WAKE_G") { Config.SLEEP_WAKE_G = valStr.toFloat(); }

    // --- COMPASS LOCK ENTRY & EXIT SETTLING VARS ---
    else if (varName == "COMPASS_LOCK_ENTRY_SETTLE_MS") { Config.COMPASS_LOCK_ENTRY_SETTLE_MS = valStr.toInt(); }
    else if (varName == "COMPASS_LOCK_EXIT_SETTLE_MS") { Config.COMPASS_LOCK_EXIT_SETTLE_MS = valStr.toInt(); }

    // --- SENSOR TUNING VARS ---
    else if (varName == "IMU_GYRO_DEADBAND") { Config.IMU_GYRO_DEADBAND = valStr.toFloat(); }
    else if (varName == "SONAR_MAX_DIST") { Config.SONAR_MAX_DIST = valStr.toFloat(); }

    // APPLY THE NEW MADGWICK FILTER AS WELL AS SAVE IT IN THE NVS
    else if (varName == "MADGWICK_FILTER_BETA") {
        Config.MADGWICK_FILTER_BETA = valStr.toFloat();
        imu->setFilterBeta(Config.MADGWICK_FILTER_BETA);
    }

    // AUTOTUNE START DELAY
    else if (varName == "AUTOTUNE_START_DELAY_MS") { Config.AUTOTUNE_START_DELAY_MS = valStr.toInt(); }
    else if (varName == "AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS") { Config.AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS = valStr.toInt(); }

    // --- PID TUNING VARS ---

    // --- PID: Unified Point Turn ---
    else if (varName == "PID_POINT_P") { Config.PID_POINT_P = valStr.toFloat(); }
    else if (varName == "PID_POINT_I") { Config.PID_POINT_I = valStr.toFloat(); }
    else if (varName == "PID_POINT_D") { Config.PID_POINT_D = valStr.toFloat(); }
    else if (varName == "PID_POINT_LIM") { Config.PID_POINT_LIM = valStr.toFloat(); }
    else if (varName == "PID_POINT_ILIM") { Config.PID_POINT_ILIM = valStr.toFloat(); }
    else if (varName == "PID_POINT_DEAD") { Config.PID_POINT_DEAD = valStr.toFloat(); }

    // --- PID: Unified Arc Turn ---
    else if (varName == "PID_ARC_P") { Config.PID_ARC_P = valStr.toFloat(); }
    else if (varName == "PID_ARC_I") { Config.PID_ARC_I = valStr.toFloat(); }
    else if (varName == "PID_ARC_D") { Config.PID_ARC_D = valStr.toFloat(); }
    else if (varName == "PID_ARC_LIM") { Config.PID_ARC_LIM = valStr.toFloat(); }
    else if (varName == "PID_ARC_ILIM") { Config.PID_ARC_ILIM = valStr.toFloat(); }
    else if (varName == "PID_ARC_DEAD") { Config.PID_ARC_DEAD = valStr.toFloat(); }

    /*
    else if (varName == "PID_HEADING_P") { Config.PID_HEADING_P = valStr.toFloat(); }
    else if (varName == "PID_HEADING_I") { Config.PID_HEADING_I = valStr.toFloat(); }
    else if (varName == "PID_HEADING_D") { Config.PID_HEADING_D = valStr.toFloat(); }
    else if (varName == "PID_HEADING_LIM") { Config.PID_HEADING_LIM = valStr.toFloat(); }
    else if (varName == "PID_HEADING_ILIM") { Config.PID_HEADING_ILIM = valStr.toFloat(); }
    else if (varName == "PID_HEADING_DEAD") { Config.PID_HEADING_DEAD = valStr.toFloat(); }

    else if (varName == "PID_COMPASS_P") { Config.PID_COMPASS_P = valStr.toFloat(); }
    else if (varName == "PID_COMPASS_I") { Config.PID_COMPASS_I = valStr.toFloat(); }
    else if (varName == "PID_COMPASS_D") { Config.PID_COMPASS_D = valStr.toFloat(); }
    else if (varName == "PID_COMPASS_LIM") { Config.PID_COMPASS_LIM = valStr.toFloat(); }
    else if (varName == "PID_COMPASS_ILIM") { Config.PID_COMPASS_ILIM = valStr.toFloat(); }
    else if (varName == "PID_COMPASS_DEAD") { Config.PID_COMPASS_DEAD = valStr.toFloat(); }
    */

    else if (varName == "PID_DIST_P") { Config.PID_DIST_P = valStr.toFloat(); }
    else if (varName == "PID_DIST_I") { Config.PID_DIST_I = valStr.toFloat(); }
    else if (varName == "PID_DIST_D") { Config.PID_DIST_D = valStr.toFloat(); }
    else if (varName == "PID_DIST_LIM") { Config.PID_DIST_LIM = valStr.toFloat(); }
    else if (varName == "PID_DIST_ILIM") { Config.PID_DIST_ILIM = valStr.toFloat(); }
    else if (varName == "PID_DIST_DEAD") { Config.PID_DIST_DEAD = valStr.toFloat(); }

    /*
    else if (varName == "PID_OBSTACLE_P") { Config.PID_OBSTACLE_P = valStr.toFloat(); }
    else if (varName == "PID_OBSTACLE_I") { Config.PID_OBSTACLE_I = valStr.toFloat(); }
    else if (varName == "PID_OBSTACLE_D") { Config.PID_OBSTACLE_D = valStr.toFloat(); }
    else if (varName == "PID_OBSTACLE_LIM") { Config.PID_OBSTACLE_LIM = valStr.toFloat(); }
    else if (varName == "PID_OBSTACLE_ILIM") { Config.PID_OBSTACLE_ILIM = valStr.toFloat(); }
    else if (varName == "PID_OBSTACLE_DEAD") { Config.PID_OBSTACLE_DEAD = valStr.toFloat(); }
    */

    else if (varName == "TILT_HANDLING_THRESHOLD") { Config.TILT_HANDLING_THRESHOLD = valStr.toFloat(); }
    else if (varName == "GFORCE_LIFT_UP_THRESHOLD") { Config.GFORCE_LIFT_UP_THRESHOLD = valStr.toFloat(); }
    else if (varName == "GFORCE_LIFT_DOWN_THRESHOLD") { Config.GFORCE_LIFT_DOWN_THRESHOLD = valStr.toFloat(); }
    else if (varName == "LIFT_ENERGY_SPIKE_THRESHOLD") { Config.LIFT_ENERGY_SPIKE_THRESHOLD = valStr.toFloat(); }
    else if (varName == "UPRIGHT_ANGLE_TOLERANCE") { Config.UPRIGHT_ANGLE_TOLERANCE = valStr.toFloat(); }
    else if (varName == "PERFECTLY_STILL_ENERGY") { Config.PERFECTLY_STILL_ENERGY = valStr.toFloat(); }
    else if (varName == "STEADY_HOLD_ENERGY_MAX") { Config.STEADY_HOLD_ENERGY_MAX = valStr.toFloat(); }
    else if (varName == "DIZZY_ENERGY_DEADBAND") { Config.DIZZY_ENERGY_DEADBAND = valStr.toFloat(); }
    else if (varName == "DIZZY_CHARGE_RATE") { Config.DIZZY_CHARGE_RATE = valStr.toFloat(); }
    else if (varName == "DIZZY_DECAY_RATE") { Config.DIZZY_DECAY_RATE = valStr.toFloat(); }
    else if (varName == "DIZZY_TRIGGER_THRESHOLD") { Config.DIZZY_TRIGGER_THRESHOLD = valStr.toFloat(); }
    else if (varName == "ENERGY_EMA_ALPHA") { Config.ENERGY_EMA_ALPHA = valStr.toFloat(); }
    else if (varName == "ENERGY_EMA_BETA") { Config.ENERGY_EMA_BETA = valStr.toFloat(); }
    else if (varName == "DISTANCE_HOLD_FRUSTRATION_LIMIT") { Config.DISTANCE_HOLD_FRUSTRATION_LIMIT = valStr.toInt(); }
    else if (varName == "FRUSTRATION_COOLDOWN_RATE") { Config.FRUSTRATION_COOLDOWN_RATE = valStr.toFloat(); }
    else if (varName == "FRUSTRATION_HEATUP_RATE") { Config.FRUSTRATION_HEATUP_RATE = valStr.toFloat(); }

    // --- NETWORK VARS ---
    else if (varName == "WIFI_SSID") { Config.WIFI_SSID = valStr; }
    else if (varName == "WIFI_PASSWORD") { Config.WIFI_PASSWORD = valStr; }
    else if (varName == "BT_NAME") { Config.BT_NAME = valStr; }
    else if (varName == "WIFI_ACTIVE") { Config.WIFI_ACTIVE = (valStr == "on" || valStr == "1" || valStr == "true"); }
    else if (varName == "BT_ACTIVE") { Config.BT_ACTIVE = (valStr == "on" || valStr == "1" || valStr == "true"); }

    // --- SYSTEM VARS ---
    else if (varName == "BRAIN_ACTIVE") { Config.BRAIN_ACTIVE = (valStr == "on" || valStr == "1" || valStr == "true"); }
    else if (varName == "SERIAL_DEBUG_MASTER") { Config.SERIAL_DEBUG_MASTER = (valStr == "on" || valStr == "1" || valStr == "true"); }

    // ==========================================
    // BITMASK TELEMETRY SWITCHING
    // ==========================================
    else if (varName == "DEBUG_ACTIVE") { 
        valStr.toLowerCase();
        bool debugOn = (valStr == "on" || valStr == "1" || valStr == "true");
        if (!debugOn) {
            // Master Kill Switch: Wipe all bits to 0
            Config.ACTIVE_DEBUG_MODE = 0; 
        } else {
            // Master On: Restore default outputs
            Config.ACTIVE_DEBUG_MODE = FactoryDefaults::ACTIVE_DEBUG_MODE; 
        }
    }
    else if (varName == "DEBUG_USB") { 
        valStr.toLowerCase();
        if (valStr == "on" || valStr == "1" || valStr == "true") {
            Config.ACTIVE_DEBUG_MODE |= FactoryDefaults::DEBUG_USB; // OR operator (Turns bit ON)
        } else {
            Config.ACTIVE_DEBUG_MODE &= ~FactoryDefaults::DEBUG_USB; // AND NOT operator (Turns bit OFF)
        }
    }
    else if (varName == "DEBUG_WIFI") { 
        valStr.toLowerCase();
        if (valStr == "on" || valStr == "1" || valStr == "true") {
            Config.ACTIVE_DEBUG_MODE |= FactoryDefaults::DEBUG_WIFI; 
        } else {
            Config.ACTIVE_DEBUG_MODE &= ~FactoryDefaults::DEBUG_WIFI; 
        }
    }
    else if (varName == "DEBUG_BLUETOOTH") { 
        valStr.toLowerCase();
        if (valStr == "on" || valStr == "1" || valStr == "true") {
            Config.ACTIVE_DEBUG_MODE |= FactoryDefaults::DEBUG_BLUETOOTH; 
        } else {
            Config.ACTIVE_DEBUG_MODE &= ~FactoryDefaults::DEBUG_BLUETOOTH; 
        }
    }
    //////////////////////////////////////

    else if (varName == "DEBUG_USB") { 
        valStr.toLowerCase();
        Config.DEBUG_USB = (valStr == "on" || valStr == "1" || valStr == "true"); 
    }
    else if (varName == "DEBUG_WIFI") { 
        valStr.toLowerCase();
        Config.DEBUG_WIFI = (valStr == "on" || valStr == "1" || valStr == "true"); 
    }
    else if (varName == "DEBUG_BLUETOOTH") { 
        valStr.toLowerCase();
        Config.DEBUG_BLUETOOTH = (valStr == "on" || valStr == "1" || valStr == "true"); 
    }

    else if (varName == "ACTIVE_DEBUG_MODE") { Config.ACTIVE_DEBUG_MODE = (valStr == "on" || valStr == "1" || valStr == "true"); }
    
    /* FOR LATER: Granular debug controls for each subsystem!
    else if (varName == "SERIAL_DEBUG_IMU") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_IMU = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    else if (varName == "SERIAL_DEBUG_SONAR") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_SONAR = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }
    else if (varName == "SERIAL_DEBUG_MOTOR_DRIVER") { 
        valStr.toLowerCase();
        Config.SERIAL_DEBUG_MOTOR_DRIVER = (valStr == "on" || valStr == "true" || valStr == "1"); 
    }*/
    
    else {
        logger.printf("Unknown variable: %s\n", varName.c_str());
        return;
    }

    ConfigSys.save();
    logger.printf("Successfully set %s to %s\n", varName.c_str(), valStr.c_str());

    // ==========================================
    // THE ELEGANT IMMEDIATE PID UPDATER CHECK
    // ==========================================
    // If the user just modified a PID value, push the whole updated 
    // profile into the active memory of the running PID object immediately!
    // SUPER CLEVER WORKAROUND OF APPLYING ALL PIDS OF A MODE IN ONE GO BY CHECKING THE PREFIX OF THE VARIABLE NAME!
    // OTHERWISE, WE'D NEED TO WRITE A TON OF REDUNDANT CODE TO CHECK FOR EACH INDIVIDUAL PID VARIABLE AND THEN RE-APPLY THE TUNINGS,
    // WHICH WOULD BE ANNOYING TO MAINTAIN AND EASY TO SCREW UP BY FORGETTING ONE!
    
    if (varName.startsWith("PID_POINT")) {
        pointTurnPID.setTunings(Config.PID_POINT_P, Config.PID_POINT_I, Config.PID_POINT_D, Config.PID_POINT_ILIM, Config.PID_POINT_LIM);
        logger.printf("UPDATED UNIFIED POINT TURN PID: P=%.2f | I=%.2f | D=%.2f\n", Config.PID_POINT_P, Config.PID_POINT_I, Config.PID_POINT_D);
    }
    else if (varName.startsWith("PID_ARC")) {
        arcTurnPID.setTunings(Config.PID_ARC_P, Config.PID_ARC_I, Config.PID_ARC_D, Config.PID_ARC_ILIM, Config.PID_ARC_LIM);
        logger.printf("UPDATED UNIFIED ARC TURN PID: P=%.2f | I=%.2f | D=%.2f\n", Config.PID_ARC_P, Config.PID_ARC_I, Config.PID_ARC_D);
    }

    /*
    if (varName.startsWith("PID_HEADING")) {
        headingPID.setTunings(Config.PID_HEADING_P, Config.PID_HEADING_I, Config.PID_HEADING_D, Config.PID_HEADING_ILIM, Config.PID_HEADING_LIM);
        logger.printf("UPDATED HEADING PID. NEW TUNINGS: P = %.2f | I = %.2f | D=%.2f\n", Config.PID_HEADING_P, Config.PID_HEADING_I, Config.PID_HEADING_D);
    }
    else if (varName.startsWith("PID_COMPASS")) {
        compassPID.setTunings(Config.PID_COMPASS_P, Config.PID_COMPASS_I, Config.PID_COMPASS_D, Config.PID_COMPASS_ILIM, Config.PID_COMPASS_LIM);
        logger.printf("UPDATED COMPASS PID. NEW TUNINGS: P = %.2f | I = %.2f | D=%.2f\n", Config.PID_COMPASS_P, Config.PID_COMPASS_I, Config.PID_COMPASS_D);
    }*/
    else if (varName.startsWith("PID_DIST")) {
        distancePID.setTunings(Config.PID_DIST_P, Config.PID_DIST_I, Config.PID_DIST_D, Config.PID_DIST_ILIM, Config.PID_DIST_LIM);
        logger.printf("UPDATED DISTANCE PID. NEW TUNINGS: P = %.2f | I = %.2f | D=%.2f\n", Config.PID_DIST_P, Config.PID_DIST_I, Config.PID_DIST_D);
    }

    /*
    else if (varName.startsWith("PID_OBSTACLE")) {
        obstacleAvoidancePID.setTunings(Config.PID_OBSTACLE_P, Config.PID_OBSTACLE_I, Config.PID_OBSTACLE_D, Config.PID_OBSTACLE_ILIM, Config.PID_OBSTACLE_LIM);
        logger.printf("UPDATED OBSTACLE PID. NEW TUNINGS: P = %.2f | I = %.2f | D=%.2f\n", Config.PID_OBSTACLE_P, Config.PID_OBSTACLE_I, Config.PID_OBSTACLE_D);
    }*/

    // SMART DEBUG MODE (MEDIUM) SWITCHER
    else if (varName.startsWith("DEBUG_") || varName == "ACTIVE_DEBUG_MODE") {
    logger.setMode(Config.ACTIVE_DEBUG_MODE);
}
}

void CommandProcessor::handleGet(String varName, String valStr) {
    // 1. Did the user type "get default <variable>"?
    bool wantDefaultOnly = false;
    if (varName == "DEFAULT") {
        wantDefaultOnly = true;
        varName = valStr; // Shift the target variable over!
        varName.toUpperCase();
    }

    // ==========================================
    // THE PROFESSIONAL "GET ALL" CHECK
    // ==========================================
    if (varName == "ALL") {
        logger.println("\n=== ALL SYSTEM VARIABLES ===");
        
        // Dynamically loop through the central variable list!
        for (int i = 0; i < sysVarCount; i++) {
            if (wantDefaultOnly) {
                handleGet("DEFAULT", sysVariables[i]);
            } else {
                handleGet(sysVariables[i], "");
            }
        }
        
        logger.println("============================\n");
        return;
    }

    // 2. The Elegant Formatter
    if (varName == "SERIAL_BAUD_RATE") { 
        if (wantDefaultOnly) logger.printf("[SERIAL_BAUD_RATE] Default: %d\n", FactoryDefaults::SERIAL_BAUD_RATE);
        else logger.printf("[SERIAL_BAUD_RATE] Current: %d | Default: %d\n", Config.SERIAL_BAUD_RATE, FactoryDefaults::SERIAL_BAUD_RATE);
    }
    
    else if (varName == "CRUISING_SPEED") { 
        if (wantDefaultOnly) logger.printf("[CRUISING_SPEED] Default: %.1f\n", FactoryDefaults::CRUISING_SPEED);
        else logger.printf("[CRUISING_SPEED] Current: %.1f | Default: %.1f\n", Config.CRUISING_SPEED, FactoryDefaults::CRUISING_SPEED);
    }
    else if (varName == "OBSTACLE_TRIGGER_CM") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_TRIGGER_CM] Default: %.1f\n", FactoryDefaults::OBSTACLE_TRIGGER_CM);
        else logger.printf("[OBSTACLE_TRIGGER_CM] Current: %.1f | Default: %.1f\n", Config.OBSTACLE_TRIGGER_CM, FactoryDefaults::OBSTACLE_TRIGGER_CM);
    }
    else if (varName == "MAINTAIN_DISTANCE_CM") { 
        if (wantDefaultOnly) logger.printf("[MAINTAIN_DISTANCE_CM] Default: %.1f\n", FactoryDefaults::MAINTAIN_DISTANCE_CM);
        else logger.printf("[MAINTAIN_DISTANCE_CM] Current: %.1f | Default: %.1f\n", Config.MAINTAIN_DISTANCE_CM, FactoryDefaults::MAINTAIN_DISTANCE_CM);
    }
    else if (varName == "MOTOR_MIN_PWM") { 
        if (wantDefaultOnly) logger.printf("[MOTOR_MIN_PWM] Default: %d\n", FactoryDefaults::MOTOR_MIN_PWM);
        else logger.printf("[MOTOR_MIN_PWM] Current: %d | Default: %d\n", Config.MOTOR_MIN_PWM, FactoryDefaults::MOTOR_MIN_PWM);
    }

    // --- OBSTACLE AVOIDANCE TUNING VARS PART 1---
    else if (varName == "OBS_SWEEP_ANGLE") { 
        if (wantDefaultOnly) logger.printf("[OBS_SWEEP_ANGLE] Default: %.1f\n", FactoryDefaults::OBS_SWEEP_ANGLE);
        else logger.printf("[OBS_SWEEP_ANGLE] Current: %.1f | Default: %.1f\n", Config.OBS_SWEEP_ANGLE, FactoryDefaults::OBS_SWEEP_ANGLE);
    }
    else if (varName == "OBS_SWEEP_SPEED") { 
        if (wantDefaultOnly) logger.printf("[OBS_SWEEP_SPEED] Default: %.1f\n", FactoryDefaults::OBS_SWEEP_SPEED);
        else logger.printf("[OBS_SWEEP_SPEED] Current: %.1f | Default: %.1f\n", Config.OBS_SWEEP_SPEED, FactoryDefaults::OBS_SWEEP_SPEED);
    }
    else if (varName == "OBS_SWEEP_PAUSE") { 
        if (wantDefaultOnly) logger.printf("[OBS_SWEEP_PAUSE] Default: %d\n", FactoryDefaults::OBS_SWEEP_PAUSE);
        else logger.printf("[OBS_SWEEP_PAUSE] Current: %d | Default: %d\n", Config.OBS_SWEEP_PAUSE, FactoryDefaults::OBS_SWEEP_PAUSE);
    }
    else if (varName == "OBS_CLEAR_THRESH") { 
        if (wantDefaultOnly) logger.printf("[OBS_CLEAR_THRESH] Default: %.1f\n", FactoryDefaults::OBS_CLEAR_THRESH);
        else logger.printf("[OBS_CLEAR_THRESH] Current: %.1f | Default: %.1f\n", Config.OBS_CLEAR_THRESH, FactoryDefaults::OBS_CLEAR_THRESH);
    }
    else if (varName == "OBS_HYSTERESIS") { 
        if (wantDefaultOnly) logger.printf("[OBS_HYSTERESIS] Default: %.1f\n", FactoryDefaults::OBS_HYSTERESIS);
        else logger.printf("[OBS_HYSTERESIS] Current: %.1f | Default: %.1f\n", Config.OBS_HYSTERESIS, FactoryDefaults::OBS_HYSTERESIS);
    }

    // --- OBSTACLE AVOIDANCE TUNING VARS PART 2 ---
    else if (varName == "OBSTACLE_ESCAPE_BASE_SPEED") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_ESCAPE_BASE_SPEED] Default: %.1f\n", FactoryDefaults::OBSTACLE_ESCAPE_BASE_SPEED);
        else logger.printf("[OBSTACLE_ESCAPE_BASE_SPEED] Current: %.1f | Default: %.1f\n", Config.OBSTACLE_ESCAPE_BASE_SPEED, FactoryDefaults::OBSTACLE_ESCAPE_BASE_SPEED);
    }
    else if (varName == "OBSTACLE_BACKUP_DURATION_MS") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_BACKUP_DURATION_MS] Default: %lu\n", FactoryDefaults::OBSTACLE_BACKUP_DURATION_MS);
        else logger.printf("[OBSTACLE_BACKUP_DURATION_MS] Current: %lu | Default: %lu\n", Config.OBSTACLE_BACKUP_DURATION_MS, FactoryDefaults::OBSTACLE_BACKUP_DURATION_MS);
    }
    else if (varName == "OBSTACLE_SWEEP_ANGLE_DEG") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_SWEEP_ANGLE_DEG] Default: %.1f\n", FactoryDefaults::OBSTACLE_SWEEP_ANGLE_DEG);
        else logger.printf("[OBSTACLE_SWEEP_ANGLE_DEG] Current: %.1f | Default: %.1f\n", Config.OBSTACLE_SWEEP_ANGLE_DEG, FactoryDefaults::OBSTACLE_SWEEP_ANGLE_DEG);
    }
    else if (varName == "OBSTACLE_SWEEP_TIMEOUT_MS") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_SWEEP_TIMEOUT_MS] Default: %lu\n", FactoryDefaults::OBSTACLE_SWEEP_TIMEOUT_MS);
        else logger.printf("[OBSTACLE_SWEEP_TIMEOUT_MS] Current: %lu | Default: %lu\n", Config.OBSTACLE_SWEEP_TIMEOUT_MS, FactoryDefaults::OBSTACLE_SWEEP_TIMEOUT_MS);
    }
    else if (varName == "OBSTACLE_ALIGN_TIMEOUT_MS") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_ALIGN_TIMEOUT_MS] Default: %lu\n", FactoryDefaults::OBSTACLE_ALIGN_TIMEOUT_MS);
        else logger.printf("[OBSTACLE_ALIGN_TIMEOUT_MS] Current: %lu | Default: %lu\n", Config.OBSTACLE_ALIGN_TIMEOUT_MS, FactoryDefaults::OBSTACLE_ALIGN_TIMEOUT_MS);
    }
    else if (varName == "OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG] Default: %.1f\n", FactoryDefaults::OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG);
        else logger.printf("[OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG] Current: %.1f | Default: %.1f\n", Config.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG, FactoryDefaults::OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG);
    }
    else if (varName == "OBSTACLE_ESCAPE_DURATION_MS") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_ESCAPE_DURATION_MS] Default: %lu\n", FactoryDefaults::OBSTACLE_ESCAPE_DURATION_MS);
        else logger.printf("[OBSTACLE_ESCAPE_DURATION_MS] Current: %lu | Default: %lu\n", Config.OBSTACLE_ESCAPE_DURATION_MS, FactoryDefaults::OBSTACLE_ESCAPE_DURATION_MS);
    }

    // --- MODE TUNING VARS ---
    else if (varName == "DIZZY_SPIN_PWM") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_SPIN_PWM] Default: %d\n", FactoryDefaults::DIZZY_SPIN_PWM);
        else logger.printf("[DIZZY_SPIN_PWM] Current: %d | Default: %d\n", Config.DIZZY_SPIN_PWM, FactoryDefaults::DIZZY_SPIN_PWM);
    }
    else if (varName == "DIZZY_SPIN_TIME") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_SPIN_TIME] Default: %d\n", FactoryDefaults::DIZZY_SPIN_TIME);
        else logger.printf("[DIZZY_SPIN_TIME] Current: %d | Default: %d\n", Config.DIZZY_SPIN_TIME, FactoryDefaults::DIZZY_SPIN_TIME);
    }
    else if (varName == "DIZZY_COOLDOWN") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_COOLDOWN] Default: %d\n", FactoryDefaults::DIZZY_COOLDOWN);
        else logger.printf("[DIZZY_COOLDOWN] Current: %d | Default: %d\n", Config.DIZZY_COOLDOWN, FactoryDefaults::DIZZY_COOLDOWN);
    }
    else if (varName == "SLEEP_TIMEOUT_MS") { 
        if (wantDefaultOnly) logger.printf("[SLEEP_TIMEOUT_MS] Default: %d\n", FactoryDefaults::SLEEP_TIMEOUT_MS);
        else logger.printf("[SLEEP_TIMEOUT_MS] Current: %d | Default: %d\n", Config.SLEEP_TIMEOUT_MS, FactoryDefaults::SLEEP_TIMEOUT_MS);
    }
    else if (varName == "SLEEP_WAKE_G") { 
        if (wantDefaultOnly) logger.printf("[SLEEP_WAKE_G] Default: %.1f\n", FactoryDefaults::SLEEP_WAKE_G);
        else logger.printf("[SLEEP_WAKE_G] Current: %.1f | Default: %.1f\n", Config.SLEEP_WAKE_G, FactoryDefaults::SLEEP_WAKE_G);
    }

    // --- COMPASS LOCK ENTRY & EXIT SETTLING VARS ---
    else if (varName == "COMPASS_LOCK_ENTRY_SETTLE_MS") { 
        if (wantDefaultOnly) logger.printf("[COMPASS_LOCK_ENTRY_SETTLE_MS] Default: %d\n", FactoryDefaults::COMPASS_LOCK_ENTRY_SETTLE_MS);
        else logger.printf("[COMPASS_LOCK_ENTRY_SETTLE_MS] Current: %d | Default: %d\n", Config.COMPASS_LOCK_ENTRY_SETTLE_MS, FactoryDefaults::COMPASS_LOCK_ENTRY_SETTLE_MS);
    }
    else if (varName == "COMPASS_LOCK_EXIT_SETTLE_MS") { 
        if (wantDefaultOnly) logger.printf("[COMPASS_LOCK_EXIT_SETTLE_MS] Default: %d\n", FactoryDefaults::COMPASS_LOCK_EXIT_SETTLE_MS);
        else logger.printf("[COMPASS_LOCK_EXIT_SETTLE_MS] Current: %d | Default: %d\n", Config.COMPASS_LOCK_EXIT_SETTLE_MS, FactoryDefaults::COMPASS_LOCK_EXIT_SETTLE_MS);
    }

    // --- SENSOR TUNING VARS ---
    else if (varName == "IMU_GYRO_DEADBAND") { 
        if (wantDefaultOnly) logger.printf("[IMU_GYRO_DEADBAND] Default: %.1f\n", FactoryDefaults::IMU_GYRO_DEADBAND);
        else logger.printf("[IMU_GYRO_DEADBAND] Current: %.1f | Default: %.1f\n", Config.IMU_GYRO_DEADBAND, FactoryDefaults::IMU_GYRO_DEADBAND);
    }
    else if (varName == "SONAR_MAX_DIST") { 
        if (wantDefaultOnly) logger.printf("[SONAR_MAX_DIST] Default: %.1f\n", FactoryDefaults::SONAR_MAX_DIST);
        else logger.printf("[SONAR_MAX_DIST] Current: %.1f | Default: %.1f\n", Config.SONAR_MAX_DIST, FactoryDefaults::SONAR_MAX_DIST);
    }

    else if (varName == "MADGWICK_FILTER_BETA") { 
        if (wantDefaultOnly) logger.printf("[MADGWICK_FILTER_BETA] Default: %.4f\n", FactoryDefaults::MADGWICK_FILTER_BETA);
        else logger.printf("[MADGWICK_FILTER_BETA] Current: %.4f | Default: %.4f\n", Config.MADGWICK_FILTER_BETA, FactoryDefaults::MADGWICK_FILTER_BETA);
    }

    else if (varName == "AUTOTUNE_START_DELAY_MS") { 
        if (wantDefaultOnly) logger.printf("[AUTOTUNE_START_DELAY_MS] Default: %d\n", FactoryDefaults::AUTOTUNE_START_DELAY_MS);
        else logger.printf("[AUTOTUNE_START_DELAY_MS] Current: %d | Default: %d\n", Config.AUTOTUNE_START_DELAY_MS, FactoryDefaults::AUTOTUNE_START_DELAY_MS);
    }
    else if (varName == "AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS") { 
        if (wantDefaultOnly) logger.printf("[AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS] Default: %d\n", FactoryDefaults::AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS);
        else logger.printf("[AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS] Current: %d | Default: %d\n", Config.AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS, FactoryDefaults::AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS);
    }

    else if (varName.startsWith("PID_POINT")){
        logger.printf("\t\t\t --- [UNIFIED PID: POINT TURN (STATIONARY)] ---\n");
        logger.printf("CURRENT:\tP=%.2f\t|\tI=%.2f\t|\tD=%.2f\t|\tLIM=%.2f\t|\tDEAD=%.2f\n", Config.PID_POINT_P, Config.PID_POINT_I, Config.PID_POINT_D, Config.PID_POINT_LIM, Config.PID_POINT_DEAD);
    }
    else if (varName.startsWith("PID_ARC")){
        logger.printf("\t\t\t --- [UNIFIED PID: ARC TURN (ROLLING)] ---\n");
        logger.printf("CURRENT:\tP=%.2f\t|\tI=%.2f\t|\tD=%.2f\t|\tLIM=%.2f\t|\tDEAD=%.2f\n", Config.PID_ARC_P, Config.PID_ARC_I, Config.PID_ARC_D, Config.PID_ARC_LIM, Config.PID_ARC_DEAD);
    }

    /*
    //experimental elegant "get all PID tunings at once" check by prefix!
    else if (varName.startsWith("PID_HEADING")){
        if (wantDefaultOnly) {
            logger.printf("\t\t\t\t\t --- [PID_HEADING] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_HEADING_P | PID_HEADING_I | PID_HEADING_D | PID_HEADING_LIM | PID_HEADING_ILIM | PID_HEADING_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_HEADING_P, FactoryDefaults::PID_HEADING_I, FactoryDefaults::PID_HEADING_D, FactoryDefaults::PID_HEADING_LIM, FactoryDefaults::PID_HEADING_ILIM, FactoryDefaults::PID_HEADING_DEAD);
        }
        else {
            logger.printf("\t\t\t\t\t --- [PID_HEADING] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_HEADING_P | PID_HEADING_I | PID_HEADING_D | PID_HEADING_LIM | PID_HEADING_ILIM | PID_HEADING_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_HEADING_P, FactoryDefaults::PID_HEADING_I, FactoryDefaults::PID_HEADING_D, FactoryDefaults::PID_HEADING_LIM, FactoryDefaults::PID_HEADING_ILIM, FactoryDefaults::PID_HEADING_DEAD);
            logger.printf("CURRENT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", Config.PID_HEADING_P, Config.PID_HEADING_I, Config.PID_HEADING_D, Config.PID_HEADING_LIM, Config.PID_HEADING_ILIM, Config.PID_HEADING_DEAD);
        }
    }
    else if (varName.startsWith("PID_COMPASS")){
        if (wantDefaultOnly) {
            logger.printf("\t\t\t\t\t --- [PID_COMPASS] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_COMPASS_P | PID_COMPASS_I | PID_COMPASS_D | PID_COMPASS_LIM | PID_COMPASS_ILIM | PID_COMPASS_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_COMPASS_P, FactoryDefaults::PID_COMPASS_I, FactoryDefaults::PID_COMPASS_D, FactoryDefaults::PID_COMPASS_LIM, FactoryDefaults::PID_COMPASS_ILIM, FactoryDefaults::PID_COMPASS_DEAD);
        }
        else {
            logger.printf("\t\t\t\t\t --- [PID_COMPASS] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_COMPASS_P | PID_COMPASS_I | PID_COMPASS_D | PID_COMPASS_LIM | PID_COMPASS_ILIM | PID_COMPASS_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_COMPASS_P, FactoryDefaults::PID_COMPASS_I, FactoryDefaults::PID_COMPASS_D, FactoryDefaults::PID_COMPASS_LIM, FactoryDefaults::PID_COMPASS_ILIM, FactoryDefaults::PID_COMPASS_DEAD);
            logger.printf("CURRENT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", Config.PID_COMPASS_P, Config.PID_COMPASS_I, Config.PID_COMPASS_D, Config.PID_COMPASS_LIM, Config.PID_COMPASS_ILIM, Config.PID_COMPASS_DEAD);
        }
    }*/
    else if (varName.startsWith("PID_DIST")){
        if (wantDefaultOnly) {
            logger.printf("\t\t\t\t\t --- [PID_DIST] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_DIST_P | PID_DIST_I | PID_DIST_D | PID_DIST_LIM | PID_DIST_ILIM | PID_DIST_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_DIST_P, FactoryDefaults::PID_DIST_I, FactoryDefaults::PID_DIST_D, FactoryDefaults::PID_DIST_LIM, FactoryDefaults::PID_DIST_ILIM, FactoryDefaults::PID_DIST_DEAD);
        }
        else {
            logger.printf("\t\t\t\t\t --- [PID_DIST] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_DIST_P | PID_DIST_I | PID_DIST_D | PID_DIST_LIM | PID_DIST_ILIM | PID_DIST_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_DIST_P, FactoryDefaults::PID_DIST_I, FactoryDefaults::PID_DIST_D, FactoryDefaults::PID_DIST_LIM, FactoryDefaults::PID_DIST_ILIM, FactoryDefaults::PID_DIST_DEAD);
            logger.printf("CURRENT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", Config.PID_DIST_P, Config.PID_DIST_I, Config.PID_DIST_D, Config.PID_DIST_LIM, Config.PID_DIST_ILIM, Config.PID_DIST_DEAD);
        }
    }

    /*
    else if (varName.startsWith("PID_OBSTACLE")){
        if (wantDefaultOnly) {
            logger.printf("\t\t\t\t\t --- [PID_OBSTACLE] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_OBSTACLE_P | PID_OBSTACLE_I | PID_OBSTACLE_D | PID_OBSTACLE_LIM | PID_OBSTACLE_ILIM | PID_OBSTACLE_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_OBSTACLE_P, FactoryDefaults::PID_OBSTACLE_I, FactoryDefaults::PID_OBSTACLE_D, FactoryDefaults::PID_OBSTACLE_LIM, FactoryDefaults::PID_OBSTACLE_ILIM, FactoryDefaults::PID_OBSTACLE_DEAD);
        }
        else {
            logger.printf("\t\t\t\t\t --- [PID_OBSTACLE] ---\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("\t  PID_OBSTACLE_P | PID_OBSTACLE_I | PID_OBSTACLE_D | PID_OBSTACLE_LIM | PID_OBSTACLE_ILIM | PID_OBSTACLE_DEAD\n");
            logger.printf("------------------------------------------------------------------------------------------------------------------------\n");
            logger.printf("DEFAULT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", FactoryDefaults::PID_OBSTACLE_P, FactoryDefaults::PID_OBSTACLE_I, FactoryDefaults::PID_OBSTACLE_D, FactoryDefaults::PID_OBSTACLE_LIM, FactoryDefaults::PID_OBSTACLE_ILIM, FactoryDefaults::PID_OBSTACLE_DEAD);
            logger.printf("CURRENT:\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\t|\t%.2f\n", Config.PID_OBSTACLE_P, Config.PID_OBSTACLE_I, Config.PID_OBSTACLE_D, Config.PID_OBSTACLE_LIM, Config.PID_OBSTACLE_ILIM, Config.PID_OBSTACLE_DEAD);
        }
    }*/
    

    
    // --- PID TUNING VARS ---
    /*
    else if (varName == "PID_HEADING_P") { 
        if (wantDefaultOnly) logger.printf("[PID_HEADING_P] Default: %.4f\n", FactoryDefaults::PID_HEADING_P);
        else logger.printf("[PID_HEADING_P] Current: %.4f | Default: %.4f\n", Config.PID_HEADING_P, FactoryDefaults::PID_HEADING_P);
    }
    else if (varName == "PID_HEADING_I") { 
        if (wantDefaultOnly) logger.printf("[PID_HEADING_I] Default: %.1f\n", FactoryDefaults::PID_HEADING_I);
        else logger.printf("[PID_HEADING_I] Current: %.1f | Default: %.1f\n", Config.PID_HEADING_I, FactoryDefaults::PID_HEADING_I);
    }
    else if (varName == "PID_HEADING_D") { 
        if (wantDefaultOnly) logger.printf("[PID_HEADING_D] Default: %.1f\n", FactoryDefaults::PID_HEADING_D);
        else logger.printf("[PID_HEADING_D] Current: %.1f | Default: %.1f\n", Config.PID_HEADING_D, FactoryDefaults::PID_HEADING_D);
    }
    else if (varName == "PID_HEADING_LIM") { 
        if (wantDefaultOnly) logger.printf("[PID_HEADING_LIM] Default: %.1f\n", FactoryDefaults::PID_HEADING_LIM);
        else logger.printf("[PID_HEADING_LIM] Current: %.1f | Default: %.1f\n", Config.PID_HEADING_LIM, FactoryDefaults::PID_HEADING_LIM);
    }
    else if (varName == "PID_HEADING_ILIM") { 
        if (wantDefaultOnly) logger.printf("[PID_HEADING_ILIM] Default: %.1f\n", FactoryDefaults::PID_HEADING_ILIM);
        else logger.printf("[PID_HEADING_ILIM] Current: %.1f | Default: %.1f\n", Config.PID_HEADING_ILIM, FactoryDefaults::PID_HEADING_ILIM);
    }
    else if (varName == "PID_HEADING_DEAD") { 
        if (wantDefaultOnly) logger.printf("[PID_HEADING_DEAD] Default: %.1f\n", FactoryDefaults::PID_HEADING_DEAD);
        else logger.printf("[PID_HEADING_DEAD] Current: %.1f | Default: %.1f\n", Config.PID_HEADING_DEAD, FactoryDefaults::PID_HEADING_DEAD);
    }

    else if (varName == "PID_COMPASS_P") { 
        if (wantDefaultOnly) logger.printf("[PID_COMPASS_P] Default: %.1f\n", FactoryDefaults::PID_COMPASS_P);
        else logger.printf("[PID_COMPASS_P] Current: %.1f | Default: %.1f\n", Config.PID_COMPASS_P, FactoryDefaults::PID_COMPASS_P);
    }
    else if (varName == "PID_COMPASS_I") { 
        if (wantDefaultOnly) logger.printf("[PID_COMPASS_I] Default: %.1f\n", FactoryDefaults::PID_COMPASS_I);
        else logger.printf("[PID_COMPASS_I] Current: %.1f | Default: %.1f\n", Config.PID_COMPASS_I, FactoryDefaults::PID_COMPASS_I);
    }
    else if (varName == "PID_COMPASS_D") { 
        if (wantDefaultOnly) logger.printf("[PID_COMPASS_D] Default: %.1f\n", FactoryDefaults::PID_COMPASS_D);
        else logger.printf("[PID_COMPASS_D] Current: %.1f | Default: %.1f\n", Config.PID_COMPASS_D, FactoryDefaults::PID_COMPASS_D);
    }
    else if (varName == "PID_COMPASS_LIM") { 
        if (wantDefaultOnly) logger.printf("[PID_COMPASS_LIM] Default: %.1f\n", FactoryDefaults::PID_COMPASS_LIM);
        else logger.printf("[PID_COMPASS_LIM] Current: %.1f | Default: %.1f\n", Config.PID_COMPASS_LIM, FactoryDefaults::PID_COMPASS_LIM);
    }
    else if (varName == "PID_COMPASS_ILIM") { 
        if (wantDefaultOnly) logger.printf("[PID_COMPASS_ILIM] Default: %.1f\n", FactoryDefaults::PID_COMPASS_ILIM);
        else logger.printf("[PID_COMPASS_ILIM] Current: %.1f | Default: %.1f\n", Config.PID_COMPASS_ILIM, FactoryDefaults::PID_COMPASS_ILIM);
    }
    else if (varName == "PID_COMPASS_DEAD") { 
        if (wantDefaultOnly) logger.printf("[PID_COMPASS_DEAD] Default: %.1f\n", FactoryDefaults::PID_COMPASS_DEAD);
        else logger.printf("[PID_COMPASS_DEAD] Current: %.1f | Default: %.1f\n", Config.PID_COMPASS_DEAD, FactoryDefaults::PID_COMPASS_DEAD);
    }

    else if (varName == "PID_DIST_P") { 
        if (wantDefaultOnly) logger.printf("[PID_DIST_P] Default: %.1f\n", FactoryDefaults::PID_DIST_P);
        else logger.printf("[PID_DIST_P] Current: %.1f | Default: %.1f\n", Config.PID_DIST_P, FactoryDefaults::PID_DIST_P);
    }
    else if (varName == "PID_DIST_I") { 
        if (wantDefaultOnly) logger.printf("[PID_DIST_I] Default: %.1f\n", FactoryDefaults::PID_DIST_I);
        else logger.printf("[PID_DIST_I] Current: %.1f | Default: %.1f\n", Config.PID_DIST_I, FactoryDefaults::PID_DIST_I);
    }
    else if (varName == "PID_DIST_D") { 
        if (wantDefaultOnly) logger.printf("[PID_DIST_D] Default: %.1f\n", FactoryDefaults::PID_DIST_D);
        else logger.printf("[PID_DIST_D] Current: %.1f | Default: %.1f\n", Config.PID_DIST_D, FactoryDefaults::PID_DIST_D);
    }
    else if (varName == "PID_DIST_LIM") { 
        if (wantDefaultOnly) logger.printf("[PID_DIST_LIM] Default: %.1f\n", FactoryDefaults::PID_DIST_LIM);
        else logger.printf("[PID_DIST_LIM] Current: %.1f | Default: %.1f\n", Config.PID_DIST_LIM, FactoryDefaults::PID_DIST_LIM);
    }
    else if (varName == "PID_DIST_ILIM") { 
        if (wantDefaultOnly) logger.printf("[PID_DIST_ILIM] Default: %.1f\n", FactoryDefaults::PID_DIST_ILIM);
        else logger.printf("[PID_DIST_ILIM] Current: %.1f | Default: %.1f\n", Config.PID_DIST_ILIM, FactoryDefaults::PID_DIST_ILIM);
    }
    else if (varName == "PID_DIST_DEAD") { 
        if (wantDefaultOnly) logger.printf("[PID_DIST_DEAD] Default: %.1f\n", FactoryDefaults::PID_DIST_DEAD);
        else logger.printf("[PID_DIST_DEAD] Current: %.1f | Default: %.1f\n", Config.PID_DIST_DEAD, FactoryDefaults::PID_DIST_DEAD);
    }

    else if (varName == "PID_OBSTACLE_P") { 
        if (wantDefaultOnly) logger.printf("[PID_OBSTACLE_P] Default: %.1f\n", FactoryDefaults::PID_OBSTACLE_P);
        else logger.printf("[PID_OBSTACLE_P] Current: %.1f | Default: %.1f\n", Config.PID_OBSTACLE_P, FactoryDefaults::PID_OBSTACLE_P);
    }
    else if (varName == "PID_OBSTACLE_I") { 
        if (wantDefaultOnly) logger.printf("[PID_OBSTACLE_I] Default: %.1f\n", FactoryDefaults::PID_OBSTACLE_I);
        else logger.printf("[PID_OBSTACLE_I] Current: %.1f | Default: %.1f\n", Config.PID_OBSTACLE_I, FactoryDefaults::PID_OBSTACLE_I);
    }
    else if (varName == "PID_OBSTACLE_D") { 
        if (wantDefaultOnly) logger.printf("[PID_OBSTACLE_D] Default: %.1f\n", FactoryDefaults::PID_OBSTACLE_D);
        else logger.printf("[PID_OBSTACLE_D] Current: %.1f | Default: %.1f\n", Config.PID_OBSTACLE_D, FactoryDefaults::PID_OBSTACLE_D);
    }
    else if (varName == "PID_OBSTACLE_LIM") { 
        if (wantDefaultOnly) logger.printf("[PID_OBSTACLE_LIM] Default: %.1f\n", FactoryDefaults::PID_OBSTACLE_LIM);
        else logger.printf("[PID_OBSTACLE_LIM] Current: %.1f | Default: %.1f\n", Config.PID_OBSTACLE_LIM, FactoryDefaults::PID_OBSTACLE_LIM);
    }
    else if (varName == "PID_OBSTACLE_ILIM") { 
        if (wantDefaultOnly) logger.printf("[PID_OBSTACLE_ILIM] Default: %.1f\n", FactoryDefaults::PID_OBSTACLE_ILIM);
        else logger.printf("[PID_OBSTACLE_ILIM] Current: %.1f | Default: %.1f\n", Config.PID_OBSTACLE_ILIM, FactoryDefaults::PID_OBSTACLE_ILIM);
    }
    else if (varName == "PID_OBSTACLE_DEAD") { 
        if (wantDefaultOnly) logger.printf("[PID_OBSTACLE_DEAD] Default: %.1f\n", FactoryDefaults::PID_OBSTACLE_DEAD);
        else logger.printf("[PID_OBSTACLE_DEAD] Current: %.1f | Default: %.1f\n", Config.PID_OBSTACLE_DEAD, FactoryDefaults::PID_OBSTACLE_DEAD);
    }*/

    // --- PHYSICS & HANDLING TUNING VARS ---
    else if (varName == "TILT_HANDLING_THRESHOLD") { 
        if (wantDefaultOnly) logger.printf("[TILT_HANDLING_THRESHOLD] Default: %.1f\n", FactoryDefaults::TILT_HANDLING_THRESHOLD);
        else logger.printf("[TILT_HANDLING_THRESHOLD] Current: %.1f | Default: %.1f\n", Config.TILT_HANDLING_THRESHOLD, FactoryDefaults::TILT_HANDLING_THRESHOLD);
    }
    else if (varName == "GFORCE_LIFT_UP_THRESHOLD") { 
        if (wantDefaultOnly) logger.printf("[GFORCE_LIFT_UP_THRESHOLD] Default: %.1f\n", FactoryDefaults::GFORCE_LIFT_UP_THRESHOLD);
        else logger.printf("[GFORCE_LIFT_UP_THRESHOLD] Current: %.1f | Default: %.1f\n", Config.GFORCE_LIFT_UP_THRESHOLD, FactoryDefaults::GFORCE_LIFT_UP_THRESHOLD);
    }
    else if (varName == "GFORCE_LIFT_DOWN_THRESHOLD") { 
        if (wantDefaultOnly) logger.printf("[GFORCE_LIFT_DOWN_THRESHOLD] Default: %.1f\n", FactoryDefaults::GFORCE_LIFT_DOWN_THRESHOLD);
        else logger.printf("[GFORCE_LIFT_DOWN_THRESHOLD] Current: %.1f | Default: %.1f\n", Config.GFORCE_LIFT_DOWN_THRESHOLD, FactoryDefaults::GFORCE_LIFT_DOWN_THRESHOLD);
    }
    else if (varName == "LIFT_ENERGY_SPIKE_THRESHOLD") { 
        if (wantDefaultOnly) logger.printf("[LIFT_ENERGY_SPIKE_THRESHOLD] Default: %.1f\n", FactoryDefaults::LIFT_ENERGY_SPIKE_THRESHOLD);
        else logger.printf("[LIFT_ENERGY_SPIKE_THRESHOLD] Current: %.1f | Default: %.1f\n", Config.LIFT_ENERGY_SPIKE_THRESHOLD, FactoryDefaults::LIFT_ENERGY_SPIKE_THRESHOLD);
    }
    else if (varName == "UPRIGHT_ANGLE_TOLERANCE") { 
        if (wantDefaultOnly) logger.printf("[UPRIGHT_ANGLE_TOLERANCE] Default: %.1f\n", FactoryDefaults::UPRIGHT_ANGLE_TOLERANCE);
        else logger.printf("[UPRIGHT_ANGLE_TOLERANCE] Current: %.1f | Default: %.1f\n", Config.UPRIGHT_ANGLE_TOLERANCE, FactoryDefaults::UPRIGHT_ANGLE_TOLERANCE);
    }
    else if (varName == "PERFECTLY_STILL_ENERGY") { 
        if (wantDefaultOnly) logger.printf("[PERFECTLY_STILL_ENERGY] Default: %.1f\n", FactoryDefaults::PERFECTLY_STILL_ENERGY);
        else logger.printf("[PERFECTLY_STILL_ENERGY] Current: %.1f | Default: %.1f\n", Config.PERFECTLY_STILL_ENERGY, FactoryDefaults::PERFECTLY_STILL_ENERGY);
    }
    else if (varName == "STEADY_HOLD_ENERGY_MAX") { 
        if (wantDefaultOnly) logger.printf("[STEADY_HOLD_ENERGY_MAX] Default: %.1f\n", FactoryDefaults::STEADY_HOLD_ENERGY_MAX);
        else logger.printf("[STEADY_HOLD_ENERGY_MAX] Current: %.1f | Default: %.1f\n", Config.STEADY_HOLD_ENERGY_MAX, FactoryDefaults::STEADY_HOLD_ENERGY_MAX);
    }
    else if (varName == "DIZZY_ENERGY_DEADBAND") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_ENERGY_DEADBAND] Default: %.1f\n", FactoryDefaults::DIZZY_ENERGY_DEADBAND);
        else logger.printf("[DIZZY_ENERGY_DEADBAND] Current: %.1f | Default: %.1f\n", Config.DIZZY_ENERGY_DEADBAND, FactoryDefaults::DIZZY_ENERGY_DEADBAND);
    }
    else if (varName == "DIZZY_CHARGE_RATE") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_CHARGE_RATE] Default: %.1f\n", FactoryDefaults::DIZZY_CHARGE_RATE);
        else logger.printf("[DIZZY_CHARGE_RATE] Current: %.1f | Default: %.1f\n", Config.DIZZY_CHARGE_RATE, FactoryDefaults::DIZZY_CHARGE_RATE);
    }
    else if (varName == "DIZZY_DECAY_RATE") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_DECAY_RATE] Default: %.1f\n", FactoryDefaults::DIZZY_DECAY_RATE);
        else logger.printf("[DIZZY_DECAY_RATE] Current: %.1f | Default: %.1f\n", Config.DIZZY_DECAY_RATE, FactoryDefaults::DIZZY_DECAY_RATE);
    }
    else if (varName == "DIZZY_TRIGGER_THRESHOLD") { 
        if (wantDefaultOnly) logger.printf("[DIZZY_TRIGGER_THRESHOLD] Default: %.1f\n", FactoryDefaults::DIZZY_TRIGGER_THRESHOLD);
        else logger.printf("[DIZZY_TRIGGER_THRESHOLD] Current: %.1f | Default: %.1f\n", Config.DIZZY_TRIGGER_THRESHOLD, FactoryDefaults::DIZZY_TRIGGER_THRESHOLD);
    }
    else if (varName == "ENERGY_EMA_ALPHA") { 
        if (wantDefaultOnly) logger.printf("[ENERGY_EMA_ALPHA] Default: %.2f\n", FactoryDefaults::ENERGY_EMA_ALPHA);
        else logger.printf("[ENERGY_EMA_ALPHA] Current: %.2f | Default: %.2f\n", Config.ENERGY_EMA_ALPHA, FactoryDefaults::ENERGY_EMA_ALPHA);
    }
    else if (varName == "ENERGY_EMA_BETA") { 
        if (wantDefaultOnly) logger.printf("[ENERGY_EMA_BETA] Default: %.2f\n", FactoryDefaults::ENERGY_EMA_BETA);
        else logger.printf("[ENERGY_EMA_BETA] Current: %.2f | Default: %.2f\n", Config.ENERGY_EMA_BETA, FactoryDefaults::ENERGY_EMA_BETA);
    }
    else if (varName == "DISTANCE_HOLD_FRUSTRATION_LIMIT") { 
        if (wantDefaultOnly) logger.printf("[DISTANCE_HOLD_FRUSTRATION_LIMIT] Default: %d\n", FactoryDefaults::DISTANCE_HOLD_FRUSTRATION_LIMIT);
        else logger.printf("[DISTANCE_HOLD_FRUSTRATION_LIMIT] Current: %d | Default: %d\n", Config.DISTANCE_HOLD_FRUSTRATION_LIMIT, FactoryDefaults::DISTANCE_HOLD_FRUSTRATION_LIMIT);
    }
    else if (varName == "FRUSTRATION_COOLDOWN_RATE") { 
        if (wantDefaultOnly) logger.printf("[FRUSTRATION_COOLDOWN_RATE] Default: %.2f\n", FactoryDefaults::FRUSTRATION_COOLDOWN_RATE);
        else logger.printf("[FRUSTRATION_COOLDOWN_RATE] Current: %.2f | Default: %.2f\n", Config.FRUSTRATION_COOLDOWN_RATE, FactoryDefaults::FRUSTRATION_COOLDOWN_RATE);
    }
    else if (varName == "FRUSTRATION_HEATUP_RATE") { 
        if (wantDefaultOnly) logger.printf("[FRUSTRATION_HEATUP_RATE] Default: %.2f\n", FactoryDefaults::FRUSTRATION_HEATUP_RATE);
        else logger.printf("[FRUSTRATION_HEATUP_RATE] Current: %.2f | Default: %.2f\n", Config.FRUSTRATION_HEATUP_RATE, FactoryDefaults::FRUSTRATION_HEATUP_RATE);
    }

    // --- NETWORK VARS ---
    else if (varName == "WIFI_SSID") { 
        if (wantDefaultOnly) logger.printf("[WIFI_SSID] Default: \"%s\"\n", FactoryDefaults::WIFI_SSID);
        else logger.printf("[WIFI_SSID] Current: \"%s\" | Default: \"%s\"\n", Config.WIFI_SSID.c_str(), FactoryDefaults::WIFI_SSID);
    }
    else if (varName == "WIFI_PASSWORD") { 
        String masked = (Config.WIFI_PASSWORD == "") ? "<empty>" : "********";
        String defMasked = (String(FactoryDefaults::WIFI_PASSWORD) == "") ? "<empty>" : "********";
        if (wantDefaultOnly) logger.printf("[WIFI_PASSWORD] Default: %s\n", defMasked.c_str());
        else logger.printf("[WIFI_PASSWORD] Current: %s | Default: %s\n", masked.c_str(), defMasked.c_str());
    }
    else if (varName == "BT_NAME") { 
        if (wantDefaultOnly) logger.printf("[BT_NAME] Default: \"%s\"\n", FactoryDefaults::BT_NAME);
        else logger.printf("[BT_NAME] Current: \"%s\" | Default: \"%s\"\n", Config.BT_NAME.c_str(), FactoryDefaults::BT_NAME);
    }
    else if (varName == "WIFI_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[WIFI_ACTIVE] Default: %s\n", FactoryDefaults::WIFI_ACTIVE ? "ON" : "OFF");
        else logger.printf("[WIFI_ACTIVE] Current: %s | Default: %s\n", Config.WIFI_ACTIVE ? "ON" : "OFF", FactoryDefaults::WIFI_ACTIVE ? "ON" : "OFF");
    }
    else if (varName == "BT_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[BT_ACTIVE] Default: %s\n", FactoryDefaults::BT_ACTIVE ? "ON" : "OFF");
        else logger.printf("[BT_ACTIVE] Current: %s | Default: %s\n", Config.BT_ACTIVE ? "ON" : "OFF", FactoryDefaults::BT_ACTIVE ? "ON" : "OFF");
    }

    // --- SYSTEM VARS ---
    else if (varName == "BRAIN_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[BRAIN_ACTIVE] Default: %s\n", FactoryDefaults::BRAIN_ACTIVE ? "ON" : "OFF");
        else logger.printf("[BRAIN_ACTIVE] Current: %s | Default: %s\n", Config.BRAIN_ACTIVE ? "ON" : "OFF", FactoryDefaults::BRAIN_ACTIVE ? "ON" : "OFF");
    }
    else if (varName == "SERIAL_DEBUG_MASTER") { 
        if (wantDefaultOnly) logger.printf("[SERIAL_DEBUG_MASTER] Default: %s\n", FactoryDefaults::SERIAL_DEBUG_MASTER ? "ON" : "OFF");
        else logger.printf("[SERIAL_DEBUG_MASTER] Current: %s | Default: %s\n", Config.SERIAL_DEBUG_MASTER ? "ON" : "OFF", FactoryDefaults::SERIAL_DEBUG_MASTER ? "ON" : "OFF");
    }

    else if (varName == "DEBUG_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[DEBUG_ACTIVE] Default: %s\n", FactoryDefaults::DEBUG_ACTIVE ? "ON" : "OFF");
        else logger.printf("[DEBUG_ACTIVE] Current: %s | Default: %s\n", Config.DEBUG_ACTIVE ? "ON" : "OFF", FactoryDefaults::DEBUG_ACTIVE ? "ON" : "OFF");
    }
    else if (varName == "DEBUG_USB") { 
        if (wantDefaultOnly) logger.printf("[DEBUG_USB] Default: %s\n", FactoryDefaults::DEBUG_USB ? "ON" : "OFF");
        else logger.printf("[DEBUG_USB] Current: %s | Default: %s\n", Config.DEBUG_USB ? "ON" : "OFF", FactoryDefaults::DEBUG_USB ? "ON" : "OFF");
    }
    else if (varName == "DEBUG_WIFI") { 
        if (wantDefaultOnly) logger.printf("[DEBUG_WIFI] Default: %s\n", FactoryDefaults::DEBUG_WIFI ? "ON" : "OFF");
        else logger.printf("[DEBUG_WIFI] Current: %s | Default: %s\n", Config.DEBUG_WIFI ? "ON" : "OFF", FactoryDefaults::DEBUG_WIFI ? "ON" : "OFF");
    }
    else if (varName == "DEBUG_BLUETOOTH") { 
        if (wantDefaultOnly) logger.printf("[DEBUG_BLUETOOTH] Default: %s\n", FactoryDefaults::DEBUG_BLUETOOTH ? "ON" : "OFF");
        else logger.printf("[DEBUG_BLUETOOTH] Current: %s | Default: %s\n", Config.DEBUG_BLUETOOTH ? "ON" : "OFF", FactoryDefaults::DEBUG_BLUETOOTH ? "ON" : "OFF");
    }
    else if (varName == "ACTIVE_DEBUG_MODE") { 
        if (wantDefaultOnly) logger.printf("[ACTIVE_DEBUG_MODE] Default: %s\n", FactoryDefaults::ACTIVE_DEBUG_MODE? "ON" : "OFF");
        else logger.printf("[ACTIVE_DEBUG_MODE] Current: %s | Default: %s\n", Config.ACTIVE_DEBUG_MODE ? "ON" : "OFF", FactoryDefaults::ACTIVE_DEBUG_MODE ? "ON" : "OFF");
    }

    else if (varName == "") {
        logger.println("Usage: get <VARIABLE> or get default <VARIABLE>");
    }

    // ==========================================
    // READ-ONLY STATIC VARIABLES (Hardware & Pins)
    // ==========================================
    // --- PIN CONFIGS ---
    else if (varName == "PIN_MOTOR_LEFT_FWD") { logger.printf("[PIN_MOTOR_LEFT_FWD] Static: %d\n", HardwarePins::PIN_MOTOR_LEFT_FWD); }
    else if (varName == "PIN_MOTOR_LEFT_REV") { logger.printf("[PIN_MOTOR_LEFT_REV] Static: %d\n", HardwarePins::PIN_MOTOR_LEFT_REV); }
    else if (varName == "PIN_MOTOR_RIGHT_FWD") { logger.printf("[PIN_MOTOR_RIGHT_FWD] Static: %d\n", HardwarePins::PIN_MOTOR_RIGHT_FWD); }
    else if (varName == "PIN_MOTOR_RIGHT_REV") { logger.printf("[PIN_MOTOR_RIGHT_REV] Static: %d\n", HardwarePins::PIN_MOTOR_RIGHT_REV); }
    else if (varName == "PIN_SONAR_TRIG") { logger.printf("[PIN_SONAR_TRIG] Static: %d\n", HardwarePins::PIN_SONAR_TRIG); }
    else if (varName == "PIN_SONAR_ECHO") { logger.printf("[PIN_SONAR_ECHO] Static: %d\n", HardwarePins::PIN_SONAR_ECHO); }
    else if (varName == "PIN_I2C_SCL") { logger.printf("[PIN_I2C_SCL] Static: %d\n", HardwarePins::PIN_I2C_SCL); }
    else if (varName == "PIN_I2C_SDA") { logger.printf("[PIN_I2C_SDA] Static: %d\n", HardwarePins::PIN_I2C_SDA); }
    else if (varName == "PIN_IMU_INT") { logger.printf("[PIN_IMU_INT] Static: %d\n", HardwarePins::PIN_IMU_INT); }

    // --- I2C & PWM CHANNELS ---
    else if (varName == "I2C_ADDR_MPU6050") { logger.printf("[I2C_ADDR_MPU6050] Static: 0x%02X\n", I2CRegistry::I2C_ADDR_MPU6050); }
    else if (varName == "CH_MOTOR_LEFT_FWD") { logger.printf("[CH_MOTOR_LEFT_FWD] Static: %d\n", ChannelRegistry::CH_MOTOR_LEFT_FWD); }
    else if (varName == "CH_MOTOR_LEFT_REV") { logger.printf("[CH_MOTOR_LEFT_REV] Static: %d\n", ChannelRegistry::CH_MOTOR_LEFT_REV); }
    else if (varName == "CH_MOTOR_RIGHT_FWD") { logger.printf("[CH_MOTOR_RIGHT_FWD] Static: %d\n", ChannelRegistry::CH_MOTOR_RIGHT_FWD); }
    else if (varName == "CH_MOTOR_RIGHT_REV") { logger.printf("[CH_MOTOR_RIGHT_REV] Static: %d\n", ChannelRegistry::CH_MOTOR_RIGHT_REV); }

    // --- SYSTEM & OS ---
    else if (varName == "TELNET_PORT") { logger.printf("[TELNET_PORT] Static: %d\n", SystemConfig::TELNET_PORT); }
    else if (varName == "MAIN_LOOP_TICK_RATE_MS") { logger.printf("[MAIN_LOOP_TICK_RATE_MS] Static: %d ms\n", SystemConfig::MAIN_LOOP_TICK_RATE_MS); }
    else { 
        logger.printf("Unknown variable: %s\n", varName.c_str()); 
    }
}

void CommandProcessor::handleReset(String varName, String dummyVal) {
    if (varName == "ALL") {
        logger.println("\n[WARNING] This operation will reset ALL configuration variables to Factory Defaults.");
        logger.print("How do you want to proceed? (y/n): ");
        waitingForResetConfirm = true; 
    } 
    else if (varName != "") {
        if (ConfigSys.resetVariable(varName)) {
            logger.printf("Reset %s to default.\n", varName.c_str());
        } else {
            logger.printf("Unknown variable: %s\n", varName.c_str());
        }
    } else {
        logger.println("Usage: reset ALL or reset <VARIABLE>");
    }
}

void CommandProcessor::handleCalib(String varName, String dummyVal) {
    if (varName == "GYRO") {
        imu->calibrateGyro();
    } 
    else if (varName == "ACCEL") {
        imu->calibrateAccel();
    } 
    else if (varName == "MAG") {
        imu->calibrateMag();
    } 
    else {
        logger.println("Usage: calib GYRO | calib ACCEL | calib MAG");
    }
}


void CommandProcessor::handleAutotune(String varName, String dummyVal) {
    logger.println("\n[WARNING] Keep Robot on a perfectly level floor. Autotuning might take a while.");
    logger.print("Continue? (y/n): ");
    waitingForAutotuneConfirm = true;
}

void CommandProcessor::handleAutotuneConfirm(String input) {
    waitingForAutotuneConfirm = false;
    input.trim();
    input.toLowerCase();
    
    if (input == "y" || input == "yes") {
        logger.println("Accepted. Disabling Autonomous Brain...");
        
        // Use the centralized BRAIN_ACTIVE variable flag to disable the brain during autotuning
        // 1. Temporarily disable the brain in RAM (Do not save to NVS!)
        // We don't want to save this to permanent flash memory,
        // just in case the robot loses power during the tune and wakes up brain-dead!
        Config.BRAIN_ACTIVE = false; 
        
        // 2. Force the state transition
        brain.changeMode(&autotuneMode); 
    } else {
        logger.println("Autotune aborted.");
    }
}

void CommandProcessor::handleResetConfirm(String input) {
    input.toLowerCase();
    if (input == "y" || input == "yes") {
        logger.println("\nExecuting Factory Reset...");
        ConfigSys.factoryReset();
        printDefaults();
        
        logger.println("\nFactory Reset Complete. Rebooting in 3 seconds...");
        delay(3000);
        ESP.restart();
    } else {
        logger.println("\nFactory Reset ABORTED.");
    }
    waitingForResetConfirm = false; 
}

void CommandProcessor::printDefaults() {
    logger.println("\n--- FACTORY DEFAULTS APPLIED ---");
    logger.printf("CRUISING_SPEED = %.1f\n", FactoryDefaults::CRUISING_SPEED);
    logger.printf("OBSTACLE_TRIGGER_CM = %.1f\n", FactoryDefaults::OBSTACLE_TRIGGER_CM);
    logger.printf("MAINTAIN_DIST_CM = %.1f\n", FactoryDefaults::MAINTAIN_DISTANCE_CM);
    logger.printf("BRAIN_ACTIVE = %s\n", FactoryDefaults::BRAIN_ACTIVE ? "ON" : "OFF");
    logger.printf("SERIAL_DEBUG_MASTER = %s\n", FactoryDefaults::SERIAL_DEBUG_MASTER ? "ON" : "OFF");
    logger.println("--------------------------------");
}

void CommandProcessor::handleConnect(String target, String dummyVal) {
    target.toLowerCase();

    if (target == "wifi") {
        if (Config.WIFI_SSID == "") {
            logger.println("\n[ERROR] Wi-Fi SSID is not configured!");
            logger.println("To fix this, type the following commands:");
            logger.println("  1. set WIFI_SSID MyNetworkName");
            logger.println("  2. set WIFI_PASSWORD MyPassword123");
            logger.println("  3. connect wifi");
            return;
        }
        
        logger.printf("\n[NETWORK] Attempting to connect to Wi-Fi SSID: \"%s\"...\n", Config.WIFI_SSID.c_str());
        Config.WIFI_ACTIVE = true;
        ConfigSys.save(); // Save intent to permanent memory
        
        RadioManager::connectWiFi(Config.WIFI_SSID, Config.WIFI_PASSWORD);
        
        if (WiFi.status() == WL_CONNECTED) {
            logger.printf("[NETWORK] Wi-Fi Subsystem activated. IP: %s\n", WiFi.localIP().toString().c_str());
        } else {
            logger.println("[NETWORK] Connection failed. Check SSID and Password.");
        }
    } 
    else if (target == "bluetooth" || target == "bt") {
        logger.printf("\n[NETWORK] Starting Bluetooth Service as: \"%s\"...\n", Config.BT_NAME.c_str());
        Config.BT_ACTIVE = true;
        ConfigSys.save();
        
        RadioManager::connectBluetooth(Config.BT_NAME);
        logger.println("[NETWORK] Bluetooth Subsystem activated.");
    }
    else {
        logger.println("Usage: connect wifi OR connect bluetooth");
    }
}

void CommandProcessor::handleDisconnect(String target, String dummyVal) {
    target.toLowerCase();

    if (target == "wifi") {
        logger.println("\n[NETWORK] Disconnecting from Wi-Fi and disabling radio...");
        Config.WIFI_ACTIVE = false;
        ConfigSys.save();
        
        RadioManager::disconnectWiFi();
        logger.println("[NETWORK] Wi-Fi offline.");
    } 
    else if (target == "bluetooth" || target == "bt") {
        logger.println("\n[NETWORK] Shutting down Bluetooth radio...");
        Config.BT_ACTIVE = false;
        ConfigSys.save();
        
        RadioManager::disconnectBluetooth();
        logger.println("[NETWORK] Bluetooth offline.");
    }
    else {
        logger.println("Usage: disconnect wifi OR disconnect bluetooth");
    }
}

void CommandProcessor::handleReboot(String varName, String dummyVal) {
    logger.println("\n[SYSTEM] Rebooting in 3 seconds...");
    delay(3000);
    ESP.restart(); // Physically resets the ESP32!
}