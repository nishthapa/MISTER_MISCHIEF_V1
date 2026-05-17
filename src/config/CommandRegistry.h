#pragma once
#include <Arduino.h>
#include <map>
#include <functional>

// Define the blueprint for a command callback (Delegate)
// It accepts the Variable Name and the Value String
using CommandHandler = std::function<void(String, String)>;

class CommandRegistry {
private:
    std::map<String, CommandHandler> commands;

public:
    // Add a new command to the dictionary
    void registerCommand(String cmdName, CommandHandler handler) {
        cmdName.toLowerCase(); // Enforce lowercase rule for the keys
        commands[cmdName] = handler;
    }

    // Look up the string in the map and fire the attached function
    bool executeCommand(String cmdName, String varName, String valStr) {
        cmdName.toLowerCase();
        
        // If the command exists in the dictionary, execute it!
        if (commands.find(cmdName) != commands.end()) {
            commands[cmdName](varName, valStr);
            return true;
        }
        return false; // Command not found
    }
    
    // Helper to dynamically build the "Available Commands" help text
    String getAvailableCommands() {
        String list = "";
        for (auto const& pair : commands) {
            list += pair.first + ", ";
        }
        if (list.length() > 0) list = list.substring(0, list.length() - 2);
        return list;
    }
};