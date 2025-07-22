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
    } else if (action == "move" || action == "stand") {
        handleMoveCommand(args);
    } else if (action == "mode" || action == "freq" || action == "focus") {
        // 現在正確地傳入 action 字串
        handleConfigCommand(action, args); 
    } else if (action == "cal" || action == "stop" || action == "reboot" || action == "status") {
        // 現在正確地傳入 action 字串
        handleSystemCommand(action, args);
    } else if (action == "raw" || action == "test") {
        // 現在正確地傳入 action 字串
        handleTestCommand(action, args);
    } else {
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

void CommandHandler::handleMoveCommand(const std::vector<String>& args) {
    // 函式入口點的初步檢查
    if (args.empty()) {
        // 正常情況下不會發生，但作為防禦性程式設計
        return; 
    }

    // 獲取指令動作 (例如 "stand", "move")
    String action = args[0];
    action.toLowerCase(); // 統一轉為小寫，增加指令容錯性

    // --- 處理特殊指令: "stand" ---
    // "stand" 是一個無參數的特殊 move 指令
    if (action == "stand") {
        if (args.size() != 1) {
            Serial.println("  [ERROR] 'stand' command takes no arguments.");
            return;
        }
        Serial.println("--> Command: [stand]");
        // 使用預先定義好的校準姿態來讓機器人站立
        _robot->setRobotPoseCascade(manual_calibration_pose_rad);
        return; // 處理完畢，直接返回
    }

    // --- 處理一般指令: "move" ---
    // "move" 指令至少需要3個參數: move <target> <value(s)>
    if (action != "move" || args.size() < 3) {
        Serial.println("  [ERROR] Invalid format. Use: move <target> <value(s)>");
        Serial.println("  Examples: 'move m0 1.57', 'move gh 0.1', 'stand'");
        return;
    }

    // 獲取移動目標 (例如 "m0", "all", "gh", "gl0")
    String target_str = args[1];
    target_str.toLowerCase(); // 同樣轉為小寫

    // --- 1. 處理目標: "all" ---
    if (target_str == "all") {
        // "move all" 有兩種形式:
        // a) move all <angle> (3個參數) -> 所有馬達移動到同一個角度
        if (args.size() == 3) {
            float angle = args[2].toFloat();
            std::array<float, NUM_ROBOT_MOTORS> all_angles;
            all_angles.fill(angle);
            _robot->setAllJointsCascade(all_angles);
            Serial.printf("  [OK] Moving all motors to %.4f rad\n", angle);
        
        // b) move all <a0> <a1> ... <a11> (14個參數) -> 每個馬達移動到各自指定角度
        } else if (args.size() == 14) { // 1 (move) + 1 (all) + 12 (angles) = 14
            std::array<float, NUM_ROBOT_MOTORS> all_angles;
            bool conversion_ok = true;
            for(int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
                // toFloat() 在轉換失敗時返回 0.0，這裡暫時接受此行為
                all_angles[i] = args[i + 2].toFloat();
            }
            _robot->setAllJointsCascade(all_angles);
            Serial.println("  [OK] Setting all 12 motor positions from list.");
        } else {
            Serial.println("  [ERROR] 'move all' requires either 1 angle argument or 12 angle arguments.");
        }
        return;
    }

    // --- 2. 處理目標: 單一馬達 "m<id>" ---
    if (target_str.startsWith("m")) {
        if (args.size() != 3) {
            Serial.println("  [ERROR] Motor move format: move m<id> <angle>");
            return;
        }
        int motor_id = target_str.substring(1).toInt();
        String angle_str = args[2];
        float angle_rad;

        // 檢查是否為相對移動指令 (以 '+' 或 '-' 開頭)
        if (angle_str.startsWith("+") || angle_str.startsWith("-")) {
            // 獲取當前目標位置作為相對移動的基準
            float current_target = _robot->getTargetPosition_rad(motor_id);
            float delta = angle_str.toFloat();
            angle_rad = current_target + delta;
            Serial.printf("  [OK] Moving motor %d relatively by %.4f rad (%.4f -> %.4f)\n", motor_id, delta, current_target, angle_rad);
        } else {
            // 絕對移動
            angle_rad = angle_str.toFloat();
            Serial.printf("  [OK] Moving motor %d to absolute position %.4f rad\n", motor_id, angle_rad);
        }
        _robot->setTargetPositionCascade(motor_id, angle_rad);
        return;
    }
    
    // --- 3. 處理目標: 群組 "g<name>" ---
    if (target_str.startsWith("g")) {
        String group_name_short = target_str.substring(1);

        // 3a. 處理功能組 (gh, gu, gl) - 需要1個角度參數
        if (group_name_short == "h" || group_name_short == "u" || group_name_short == "l") {
            if (args.size() != 3) {
                Serial.printf("  [ERROR] Functional group '%s' requires 1 angle. Use: move g%s <angle>\n", group_name_short.c_str(), group_name_short.c_str());
                return;
            }
            RobotController::JointGroup func_group;
            if (group_name_short == "h") func_group = RobotController::JointGroup::HIP;
            else if (group_name_short == "u") func_group = RobotController::JointGroup::UPPER;
            else func_group = RobotController::JointGroup::LOWER;
            
            float angle = args[2].toFloat();
            _robot->setJointGroupPositionCascade(func_group, angle);
            Serial.printf("  [OK] Moving group '%s' to %.4f rad\n", target_str.c_str(), angle);
        
        // 3b. 處理腿部對 (gf, gr) - 需要3個角度參數
        } else if (group_name_short == "f" || group_name_short == "r") {
            if (args.size() != 5) { // 1(move)+1(gf/gr)+3(angles) = 5
                Serial.printf("  [ERROR] Leg pair '%s' requires 3 angles. Use: move g%s <h> <u> <l>\n", group_name_short.c_str(), group_name_short.c_str());
                return;
            }
            RobotController::JointGroup pair_group = (group_name_short == "f") ? RobotController::JointGroup::LEG_FRONT : RobotController::JointGroup::LEG_REAR;
            float h = args[2].toFloat();
            float u = args[3].toFloat();
            float l = args[4].toFloat();
            _robot->setLegPairCascade(pair_group, h, u, l);
            Serial.printf("  [OK] Moving leg pair '%s' symmetrically to (H:%.2f, U:%.2f, L:%.2f)\n", group_name_short.c_str(), h, u, l);

        // 3c. 處理單獨的腿組 (gl0, gl1, etc.) - 需要3個角度參數
        } else if (group_name_short.startsWith("l") && group_name_short.length() > 1 && isDigit(group_name_short.charAt(1))) {
            if (args.size() != 5) { // 1(move)+1(gl0)+3(angles) = 5
                Serial.println("  [ERROR] Leg group move requires 3 angles. Use: move gl<id> <h> <u> <l>");
                return;
            }
            int leg_id = group_name_short.substring(1).toInt();
            float h = args[2].toFloat();
            float u = args[3].toFloat();
            float l = args[4].toFloat();
            _robot->setLegJointsCascade(leg_id, h, u, l);
            Serial.printf("  [OK] Moving leg %d to (H:%.2f, U:%.2f, L:%.2f)\n", leg_id, h, u, l);
        
        // 3d. 無效的群組名稱
        } else {
            Serial.printf("  [ERROR] Unknown group specifier: '%s'. Valid groups: g(h|u|l|f|r) or gl(0-3).\n", target_str.c_str());
        }
        return;
    }

    // --- 最終的錯誤處理 ---
    // 如果程式執行到這裡，表示 target_str 不是任何已知的目標類型
    Serial.printf("  [ERROR] Unknown target for move command: '%s'\n", target_str.c_str());
}


void CommandHandler::handleConfigCommand(const String& action, const std::vector<String>& args) {
    if (action == "mode") {
        if (args.size() != 2) { Serial.println("  [ERROR] Use: mode <h/c/d>"); return; }
        String mode_char = args[1];
        if (mode_char == "h") _telemetry->setPrintMode(TelemetrySystem::PrintMode::HUMAN_STATUS);
        else if (mode_char == "c") _telemetry->setPrintMode(TelemetrySystem::PrintMode::CSV_LOG);
        else if (mode_char == "d") _telemetry->setPrintMode(TelemetrySystem::PrintMode::DASHBOARD);
        else { Serial.println("  [ERROR] Unknown mode. Use h, c, or d."); return; }
        Serial.printf("  [OK] Telemetry mode set to %s.\n", args[1].c_str());

    } else if (action == "freq") {
        if (args.size() != 2) { Serial.println("  [ERROR] Use: freq <hz>"); return; }
        int hz = args[1].toInt();
        if (hz > 0 && hz <= 1000) {
            g_print_interval_millis = 1000 / hz;
            Serial.printf("  [OK] Monitor frequency set to %d Hz.\n", hz);
        } else {
            Serial.println("  [ERROR] Invalid frequency.");
        }

    } else if (action == "focus") {
        if (args.size() != 2) { Serial.println("  [ERROR] Use: focus m<id> | off"); return; }
        String target_str = args[1];
        if (target_str == "off") {
            _telemetry->setFocusMotor(-1);
            Serial.println("  [OK] Telemetry focus turned off.");
        } else if (target_str.charAt(0) == 'm') {
            int motor_id = target_str.substring(1).toInt();
            _telemetry->setFocusMotor(motor_id);
            Serial.printf("  [OK] Telemetry focused on motor %d.\n", motor_id);
        } else {
            Serial.println("  [ERROR] Invalid focus target.");
        }
    }
}

void CommandHandler::handleSystemCommand(const String& action, const std::vector<String>& args) {
    if (action == "status") {
        _telemetry->printAsHumanStatus();
    } else if (action == "cal") {
        _robot->performManualCalibration();
    } else if (action == "stop") {
        _robot->setIdle();
    } else if (action == "reboot") {
        Serial.println("--> Rebooting...");
        delay(100);
        #ifdef __arm__
        SCB_AIRCR = 0x05FA0004;
        #endif
    }
}

void CommandHandler::handleTestCommand(const String& action, const std::vector<String>& args) {
    if (action == "raw") {
        // 語法: raw m<id> <mA>
        if (args.size() != 3 || args[1].charAt(0) != 'm') {
            Serial.println("  [ERROR] Use: raw m<id> <mA>");
            return;
        }
        int motor_id = args[1].substring(1).toInt();
        int16_t current = args[2].toInt();
        _robot->setSingleMotorCurrent(motor_id, current);

    } else if (action == "test") {
        // 語法: test wiggle m<id>
        if (args.size() != 3 || args[1] != "wiggle" || args[2].charAt(0) != 'm') {
            Serial.println("  [ERROR] Use: test wiggle m<id>");
            return;
        }
        int motor_id = args[2].substring(1).toInt();
        _robot->startWiggleTest(motor_id);
    }
}