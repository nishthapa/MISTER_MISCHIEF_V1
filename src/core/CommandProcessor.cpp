#include "core/CommandProcessor.h"
#include "config/ConfigurationManager.h"
#include "config/FactoryDefaults.h"
#include "utils/RemoteLogger.h"

extern RemoteLogger logger;

CommandProcessor::CommandProcessor() {
    // ==========================================
    // THE DYNAMIC COMMAND REGISTRATION
    // ==========================================
    // We bind the string command to the specific memory address of the handler function.
    registry.registerCommand("set",   std::bind(&CommandProcessor::handleSet,   this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("get",   std::bind(&CommandProcessor::handleGet,   this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("reset", std::bind(&CommandProcessor::handleReset, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("calib", std::bind(&CommandProcessor::handleCalib, this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("connect",    std::bind(&CommandProcessor::handleConnect,    this, std::placeholders::_1, std::placeholders::_2));
    registry.registerCommand("disconnect", std::bind(&CommandProcessor::handleDisconnect, this, std::placeholders::_1, std::placeholders::_2));
}

// ==========================================
// TERMINAL EMULATOR ENGINE
// ==========================================

// The Autocomplete Dictionary
const char* autoDict[] = {
    "set", "get", "reset", "calib", "default", "ALL",
    "CRUISING_SPEED", "OBSTACLE_TRIGGER_CM", "MAINTAIN_DIST_CM",
    "BRAIN_ACTIVE", "SERIAL_DEBUG_MASTER", "SERIAL_DEBUG_IMU",
    "SERIAL_DEBUG_SONAR", "SERIAL_DEBUG_MOTOR_DRIVER",
    "WIFI_SSID", "WIFI_PASSWORD", "WIFI_ACTIVE", "BT_NAME", "BT_ACTIVE"
};
const int dictSize = sizeof(autoDict) / sizeof(autoDict[0]);

void CommandProcessor::redrawCLI() {
    // 1. THE NUCLEAR ERASE
    // Return to index 0, blast 80 blank spaces to obliterate any native terminal 
    // Tab or Backspace ghosting, and then return to index 0 again.
    Serial.print("\r                                                                                \r");
    
    // 2. Draw the pristine prompt and buffer
    Serial.print("mischief> ");
    Serial.print(cliBuffer);
    
    // 3. Walk the cursor back to exactly where you were editing
    int spacesToMoveBack = cliBuffer.length() - cursorPos;
    for (int i = 0; i < spacesToMoveBack; i++) {
        Serial.print("\b");
    }
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
                else if (arrow == 'C') { // RIGHT ARROW
                    if (cursorPos < cliBuffer.length()) {
                        cursorPos++;
                        redrawCLI();
                    }
                }
                else if (arrow == 'D') { // LEFT ARROW
                    if (cursorPos > 0) {
                        cursorPos--;
                        redrawCLI();
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
            cliBuffer = cliBuffer.substring(0, cursorPos - 1) + cliBuffer.substring(cursorPos);
            cursorPos--;
            redrawCLI();
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

            processInput(cliBuffer); // Internal call!
            
            cliBuffer = ""; 
            cursorPos = 0; 
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
        cliBuffer = cliBuffer.substring(0, cursorPos) + c + cliBuffer.substring(cursorPos);
        cursorPos++;
        redrawCLI();
    }
}

void CommandProcessor::processInput(String input) {
    input.trim();
    if (input.length() == 0) return;

    // --- STATE MACHINE: Are we waiting for Y/N? ---
    if (waitingForResetConfirm) {
        handleResetConfirm(input);
        return;
    }

    // 1. SPLIT THE TOKENS
    int firstSpace = input.indexOf(' ');
    String cmd = (firstSpace == -1) ? input : input.substring(0, firstSpace);
    cmd.toLowerCase();

    String remainder = (firstSpace == -1) ? "" : input.substring(firstSpace + 1);
    remainder.trim();
    int secondSpace = remainder.indexOf(' ');
    
    String varName = (secondSpace == -1) ? remainder : remainder.substring(0, secondSpace);
    varName.toUpperCase();

    String valStr = (secondSpace == -1) ? "" : remainder.substring(secondSpace + 1);
    valStr.trim();

    // 2. ASK THE REGISTRY TO EXECUTE IT
    // No more if/else blocks! The map instantly routes to the correct function.
    if (!registry.executeCommand(cmd, varName, valStr)) {
        logger.printf("Unknown command: %s\n", cmd.c_str());
        logger.printf("Available commands: %s\n", registry.getAvailableCommands().c_str());
    }
}

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
    }*/
    
    else {
        logger.printf("Unknown variable: %s\n", varName.c_str());
        return;
    }

    ConfigSys.save();
    logger.printf("Successfully set %s to %s\n", varName.c_str(), valStr.c_str());
}

void CommandProcessor::handleGet(String varName, String valStr) {
    // 1. Did the user type "get default <variable>"?
    bool wantDefaultOnly = false;
    if (varName == "DEFAULT") {
        wantDefaultOnly = true;
        varName = valStr; // Shift the target variable over!
        varName.toUpperCase();
    }

    // 2. The Elegant Formatter
    if (varName == "CRUISING_SPEED") { 
        if (wantDefaultOnly) logger.printf("[CRUISING_SPEED] Default: %.1f\n", FactoryDefaults::CRUISING_SPEED);
        else logger.printf("[CRUISING_SPEED] Current: %.1f | Default: %.1f\n", Config.CRUISING_SPEED, FactoryDefaults::CRUISING_SPEED);
    }
    else if (varName == "OBSTACLE_TRIGGER_CM") { 
        if (wantDefaultOnly) logger.printf("[OBSTACLE_TRIGGER_CM] Default: %.1f\n", FactoryDefaults::OBSTACLE_TRIGGER_CM);
        else logger.printf("[OBSTACLE_TRIGGER_CM] Current: %.1f | Default: %.1f\n", Config.OBSTACLE_TRIGGER_CM, FactoryDefaults::OBSTACLE_TRIGGER_CM);
    }
    else if (varName == "MAINTAIN_DIST_CM") { 
        if (wantDefaultOnly) logger.printf("[MAINTAIN_DIST_CM] Default: %.1f\n", FactoryDefaults::MAINTAIN_DIST_CM);
        else logger.printf("[MAINTAIN_DIST_CM] Current: %.1f | Default: %.1f\n", Config.MAINTAIN_DIST_CM, FactoryDefaults::MAINTAIN_DIST_CM);
    }
    else if (varName == "BRAIN_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[BRAIN_ACTIVE] Default: %s\n", FactoryDefaults::BRAIN_ACTIVE ? "ON" : "OFF");
        else logger.printf("[BRAIN_ACTIVE] Current: %s | Default: %s\n", Config.BRAIN_ACTIVE ? "ON" : "OFF", FactoryDefaults::BRAIN_ACTIVE ? "ON" : "OFF");
    }
    else if (varName == "SERIAL_DEBUG_MASTER") { 
        if (wantDefaultOnly) logger.printf("[SERIAL_DEBUG_MASTER] Default: %s\n", FactoryDefaults::SERIAL_DEBUG_MASTER ? "ON" : "OFF");
        else logger.printf("[SERIAL_DEBUG_MASTER] Current: %s | Default: %s\n", Config.SERIAL_DEBUG_MASTER ? "ON" : "OFF", FactoryDefaults::SERIAL_DEBUG_MASTER ? "ON" : "OFF");
    }

    /* FOR LATER: Granular debug controls for each subsystem!
    else if (varName == "SERIAL_DEBUG_IMU") { 
        if (wantDefaultOnly) logger.printf("[SERIAL_DEBUG_IMU] Default: %s\n", FactoryDefaults::SERIAL_DEBUG_IMU ? "ON" : "OFF");
        else logger.printf("[SERIAL_DEBUG_IMU] Current: %s | Default: %s\n", Config.SERIAL_DEBUG_IMU ? "ON" : "OFF", FactoryDefaults::SERIAL_DEBUG_IMU ? "ON" : "OFF");
    }
    else if (varName == "SERIAL_DEBUG_SONAR") { 
        if (wantDefaultOnly) logger.printf("[SERIAL_DEBUG_SONAR] Default: %s\n", FactoryDefaults::SERIAL_DEBUG_SONAR ? "ON" : "OFF");
        else logger.printf("[SERIAL_DEBUG_SONAR] Current: %s | Default: %s\n", Config.SERIAL_DEBUG_SONAR ? "ON" : "OFF", FactoryDefaults::SERIAL_DEBUG_SONAR ? "ON" : "OFF");
    }
    else if (varName == "SERIAL_DEBUG_MOTOR_DRIVER") { 
        if (wantDefaultOnly) logger.printf("[SERIAL_DEBUG_MOTOR_DRIVER] Default: %s\n", FactoryDefaults::SERIAL_DEBUG_MOTOR_DRIVER ? "ON" : "OFF");
        else logger.printf("[SERIAL_DEBUG_MOTOR_DRIVER] Current: %s | Default: %s\n", Config.SERIAL_DEBUG_MOTOR_DRIVER ? "ON" : "OFF", FactoryDefaults::SERIAL_DEBUG_MOTOR_DRIVER ? "ON" : "OFF");
    }
    */

    // NETWORK COMMAND CHECKS
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
    else if (varName == "WIFI_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[WIFI_ACTIVE] Default: %s\n", FactoryDefaults::WIFI_ACTIVE ? "ON" : "OFF");
        else logger.printf("[WIFI_ACTIVE] Current: %s | Default: %s\n", Config.WIFI_ACTIVE ? "ON" : "OFF", FactoryDefaults::WIFI_ACTIVE ? "ON" : "OFF");
    }
    else if (varName == "BT_NAME") { 
        if (wantDefaultOnly) logger.printf("[BT_NAME] Default: \"%s\"\n", FactoryDefaults::BT_NAME);
        else logger.printf("[BT_NAME] Current: \"%s\" | Default: \"%s\"\n", Config.BT_NAME.c_str(), FactoryDefaults::BT_NAME);
    }
    else if (varName == "BT_ACTIVE") { 
        if (wantDefaultOnly) logger.printf("[BT_ACTIVE] Default: %s\n", FactoryDefaults::BT_ACTIVE ? "ON" : "OFF");
        else logger.printf("[BT_ACTIVE] Current: %s | Default: %s\n", Config.BT_ACTIVE ? "ON" : "OFF", FactoryDefaults::BT_ACTIVE ? "ON" : "OFF");
    }


    else if (varName == "") {
        logger.println("Usage: get <VARIABLE> or get default <VARIABLE>");
    }
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
    // You can wire your IMU calibration logic back in here later!
    logger.println("Calibration command received.");
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
    logger.printf("MAINTAIN_DIST_CM = %.1f\n", FactoryDefaults::MAINTAIN_DIST_CM);
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
        ConfigSys.save();
        
        // TODO: Call your actual WiFiConfig::init() function here later!
        logger.println("[NETWORK] Wi-Fi Subsystem activated.");
    } 
    else if (target == "bluetooth" || target == "bt") {
        logger.printf("\n[NETWORK] Starting Bluetooth Service as: \"%s\"...\n", Config.BT_NAME.c_str());
        Config.BT_ACTIVE = true;
        ConfigSys.save();
        
        // TODO: Call your Bluetooth Init logic here later!
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
        // TODO: Call actual WiFi disconnect logic here
    } 
    else if (target == "bluetooth" || target == "bt") {
        logger.println("\n[NETWORK] Shutting down Bluetooth radio...");
        Config.BT_ACTIVE = false;
        ConfigSys.save();
        // TODO: Call actual BT disconnect logic here
    }
    else {
        logger.println("Usage: disconnect wifi OR disconnect bluetooth");
    }
}