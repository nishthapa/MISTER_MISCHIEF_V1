#pragma once

#include <Arduino.h>
#include "config/CommandRegistry.h"

class CommandProcessor {
private:
    CommandRegistry registry;
    bool waitingForResetConfirm = false;

    // --- TERMINAL EMULATOR STATE ---
    static const int HISTORY_MAX = 5;
    String cmdHistory[HISTORY_MAX];
    int historyCount = 0;
    int historyIndex = -1;

    String cliBuffer = "";
    int cursorPos = 0;
    int lastBufferLength = 0; // To optimize redraws with tab autocomplete

    bool isTabbing = false;
    String tabPrefix = "";
    int tabDictIndex = 0;
    int wordStartIndex = 0;

    bool waitingForAutotuneConfirm = false;

    // --- FOR THE MOTOR TEST WIZARD ---
    // Handled by Mode_Diagnostics now
    // int motorWizardState = 0; 
    // int leftMotorTestAns = 0;
    void handleMotorWizardInput(String input);

    // Autotuning state handlers
    void handleAutotune(String varName, String dummyVal);
    void handleAutotuneConfirm(String input);

    // Terminal Helpers
    void redrawCLI();
    void processInput(String input); // Moved to private! Only the terminal calls this now.

    // --- COMMAND HANDLERS ---
    void handleSet(String varName, String valStr);
    void handleGet(String varName, String dummyVal); 
    void handleReset(String varName, String dummyVal);
    void handleCalib(String varName, String dummyVal);
    
    void handleTest(String varName, String dummyVal);

    // THE NEW NETWORK COMMANDS
    void handleConnect(String target, String dummyVal);
    void handleDisconnect(String target, String dummyVal);
    
    void handleResetConfirm(String input);
    void printDefaults();

    void handleReboot(String varName, String dummyVal); // NEW!

public:
    CommandProcessor();
    
    // THE NEW BRIDGE: main.cpp feeds raw bytes here!
    void processChar(char c); 
};