// CommandHandler.cpp (Corrected Version for Stage 1)

#include "CommandHandler.h"
#include "RobotController.h"   // 引用完整的定義
#include "TelemetrySystem.h" // 引用完整的定義
#include <vector>

// 從 homing_main.cpp 移過來的外部變數宣告，使其在此檔案中可見
// 這樣 'freq' 指令才能正確工作
extern unsigned long g_print_interval_millis;


CommandHandler::CommandHandler() : _robot(nullptr), _telemetry(nullptr) {
    // 建構式，初始化指標為空
}

void CommandHandler::begin(RobotController* robot, TelemetrySystem* telemetry) {
    _robot = robot;
    _telemetry = telemetry;
}

void CommandHandler::executeCommand(String command) {
    if (!_robot || !_telemetry) {
        Serial.println("[FATAL] CommandHandler not initialized!");
        return;
    }
    
    // 1. 將指令字串分割成參數列表 (Tokenizing)
    std::vector<String> args;
    String current_arg = "";
    for (int i = 0; i < command.length(); i++) {
        if (command.charAt(i) == ' ') {
            if (current_arg.length() > 0) {
                args.push_back(current_arg);
                current_arg = "";
            }
        } else {
            current_arg += command.charAt(i);
        }
    }
    if (current_arg.length() > 0) {
        args.push_back(current_arg);
    }

    if (args.empty()) {
        return; // 空指令，直接返回
    }

    // 2. 根據第一個參數 (Action) 分派任務 (Dispatching)
    String action = args[0];
    action.toLowerCase();

    if (action == "set") {
        handleSetCommand(args);
    } else if (action == "get") {
        handleGetCommand(args);
    } else if (action == "reset") {
        handleResetCommand(args);
    } 
    // --- 在此階段，我們只實現 set, get, reset ---
    // --- 其他指令暫時回覆 "未實現" ---
    else if (action == "move" || action == "stand" || action == "status" ||
             action == "mode" || action == "freq" || action == "focus" ||
             action == "cal" || action == "stop" || action == "reboot" ||
             action == "raw" || action == "test")
    {
        Serial.printf("  [INFO] Command '%s' will be implemented in a later stage.\n", action.c_str());
    }
    else {
        Serial.printf("  [ERROR] Unknown command: '%s'\n", command.c_str());
    }
}

void CommandHandler::handleSetCommand(const std::vector<String>& args) {
    // 語法: set <target> <param> <value>
    if (args.size() != 4) {
        Serial.println("  [ERROR] Invalid format. Use: set <target> <param> <value>");
        return;
    }

    String target_str = args[1];
    String param_name = args[2];
    float value = args[3].toFloat();
    param_name.toLowerCase();

    char target_type = target_str.charAt(0);
    
    if (target_str == "all") {
        _robot->setParamOverride(RobotController::ParamScope::GLOBAL, -1, param_name.c_str(), value);
        Serial.printf("  [OK] Global param '%s' set to %.4f\n", param_name.c_str(), value);
    } else if (target_type == 'g') {
        String group_name_short = target_str.substring(1);
        std::vector<int> motor_ids = _robot->getMotorIdsForGroup(group_name_short.c_str());
        if (motor_ids.empty()) {
            Serial.printf("  [ERROR] Unknown group: '%s'\n", target_str.c_str());
            return;
        }
        for (int id : motor_ids) {
            _robot->setParamOverride(RobotController::ParamScope::MOTOR, id, param_name.c_str(), value);
        }
        Serial.printf("  [OK] Group '%s' param '%s' set to %.4f for %d motors\n", target_str.c_str(), param_name.c_str(), value, motor_ids.size());
    } else if (target_type == 'm') {
        int motor_id = target_str.substring(1).toInt();
        _robot->setParamOverride(RobotController::ParamScope::MOTOR, motor_id, param_name.c_str(), value);
        Serial.printf("  [OK] Motor %d param '%s' set to %.4f\n", motor_id, param_name.c_str(), value);
    } else {
        Serial.printf("  [ERROR] Unknown target: '%s'\n", target_str.c_str());
    }
}

void CommandHandler::handleGetCommand(const std::vector<String>& args) {
    // 語法: get <target> [source]
    if (args.size() < 2 || args.size() > 3) {
        Serial.println("  [ERROR] Invalid format. Use: get <target> [source]");
        return;
    }

    String target_str = args[1];
    bool get_source = (args.size() == 3 && args[2] == "source");
    
    if (target_str == "all") {
        // 'get all source' 沒有意義，因為全域的來源就是 "Global" 或 "Default"
        if (get_source) {
            Serial.println("  [INFO] Use 'get m<id> source' to see sources.");
            return;
        }
        Serial.println("--- Global Cascade Parameters (Effective Values) ---");
        // 我們通過獲取 m0 的參數來 "模擬" 全域參數的顯示，因為此時 m0 尚未被覆蓋
        CascadeParams params = _robot->getEffectiveParams(0); // 假設 m0 繼承全域
        char buf[120];
        snprintf(buf, sizeof(buf), "c: %.2f, kp: %.1f, ki: %.1f, max_vel: %.2f, max_err: %.2f",
                 params.c, params.vel_kp, params.vel_ki, 
                 params.max_target_velocity_rad_s, params.integral_max_error_rad);
        Serial.println(buf);
    } else if (target_str.charAt(0) == 'g') {
        String group_name_short = target_str.substring(1);
        std::vector<int> motor_ids = _robot->getMotorIdsForGroup(group_name_short.c_str());
        if (motor_ids.empty()) { /* ... error handling ... */ return; }

        Serial.printf("--- Parameters for Group '%s' ---\n", target_str.c_str());
        if (get_source) {
             Serial.println("ID | c(src) | kp(src) | ki(src) | max_vel(src) | max_err(src)");
        } else {
             Serial.println("ID |    c    |  vel_kp |  vel_ki | max_vel | max_err");
        }
        Serial.println("---+---------+---------+---------+---------+-----------");
        for (int id : motor_ids) {
            if(get_source) {
                ParamSourceInfo s = _robot->getParamSourceInfo(id);
                char buf[120];
                snprintf(buf, sizeof(buf), "%2d | %-7.7s | %-7.7s | %-7.7s | %-12.12s | %-11.11s",
                     id, s.c_source.c_str(), s.vel_kp_source.c_str(), s.vel_ki_source.c_str(),
                     s.max_vel_source.c_str(), s.max_err_source.c_str());
                Serial.println(buf);
            } else {
                printParams(id);
            }
        }
    } else if (target_str.charAt(0) == 'm') {
        int motor_id = target_str.substring(1).toInt();
        if (get_source) {
            Serial.printf("--- Parameter Sources for Motor %d ---\n", motor_id);
            ParamSourceInfo s = _robot->getParamSourceInfo(motor_id);
            Serial.printf("  c:       %s\n", s.c_source.c_str());
            Serial.printf("  vel_kp:  %s\n", s.vel_kp_source.c_str());
            // ...以此類推...
        } else {
            Serial.printf("--- Parameters for Motor %d ---\n", motor_id);
            printParams(motor_id);
        }
    } else {
        Serial.printf("  [ERROR] Unknown target: '%s'\n", target_str.c_str());
    }
}

void CommandHandler::handleResetCommand(const std::vector<String>& args) {
    // 語法: reset <target> [param]
    if (args.size() < 2 || args.size() > 3) {
        Serial.println("  [ERROR] Invalid format. Use: reset <target> [param]");
        return;
    }

    String target_str = args[1];
    String param_name = (args.size() == 3) ? args[2] : "";
    param_name.toLowerCase();

    if (target_str == "global") {
        _robot->resetParamOverride(RobotController::ParamScope::GLOBAL, -1, "");
        Serial.println("  [OK] Global params reset to system defaults.");
    } else if (target_str == "all") {
        // 終極重置
        _robot->resetParamOverride(RobotController::ParamScope::GLOBAL, -1, "");
        for (int i=0; i < NUM_ROBOT_MOTORS; ++i) {
            _robot->resetParamOverride(RobotController::ParamScope::MOTOR, i, "");
        }
        Serial.println("  [OK] All global and motor-specific params reset.");
    } else if (target_str.charAt(0) == 'g') {
        String group_name_short = target_str.substring(1);
        std::vector<int> motor_ids = _robot->getMotorIdsForGroup(group_name_short.c_str());
        if (motor_ids.empty()) { /* ... error handling ... */ return; }
        for (int id : motor_ids) {
            _robot->resetParamOverride(RobotController::ParamScope::MOTOR, id, param_name.c_str());
        }
        Serial.printf("  [OK] Group '%s' params reset.\n", target_str.c_str());
    } else if (target_str.charAt(0) == 'm') {
        int motor_id = target_str.substring(1).toInt();
        _robot->resetParamOverride(RobotController::ParamScope::MOTOR, motor_id, param_name.c_str());
        Serial.printf("  [OK] Motor %d params reset.\n", motor_id);
    } else {
        Serial.printf("  [ERROR] Unknown target: '%s'\n", target_str.c_str());
    }
}

void CommandHandler::printParams(int motor_id) {
    CascadeParams params = _robot->getEffectiveParams(motor_id);
    char buf[120];
    snprintf(buf, sizeof(buf), "%2d | %7.2f | %7.1f | %7.1f | %7.2f | %7.2f",
             motor_id, params.c, params.vel_kp, params.vel_ki, 
             params.max_target_velocity_rad_s, params.integral_max_error_rad);
    Serial.println(buf);
}