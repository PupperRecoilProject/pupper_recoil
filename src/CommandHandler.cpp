// CommandHandler.cpp (Corrected Version for Stage 1)

#include "CommandHandler.h"
#include "RobotController.h"   // 引用完整的定義
#include "TelemetrySystem.h" // 引用完整的定義
#include <vector>
#include <array>



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



// =================================================================
//   主指令執行函式 (Top-Level Command Executor)
// =================================================================
void CommandHandler::executeCommand(String command) {
    // 防呆：確保核心模組已初始化
    if (!_robot || !_telemetry) {
        Serial.println("[FATAL] CommandHandler not initialized!");
        return;
    }

    // 將指令傳給遙測系統，並在終端回顯
    _telemetry->setLastCommand(command);
    Serial.printf("[CMD] Executing: %s\n", command.c_str());
    
    // 步驟 1: 將指令字串按空格分割成參數列表 (Tokenizing)
    std::vector<String> args;
    String current_arg;
    for (size_t i = 0; i < command.length(); i++) {
        char c = command.charAt(i);
        if (c == ' ') {
            if (current_arg.length() > 0) {
                args.push_back(current_arg);
                current_arg = "";
            }
        } else {
            current_arg += c;
        }
    }
    if (current_arg.length() > 0) {
        args.push_back(current_arg);
    }

    // 如果沒有參數（例如，只按了Enter），則直接返回
    if (args.empty()) return;

    // 步驟 2: 第一層分派 - 根據指令動詞分派到對應的處理函式
    String action = args[0];
    action.toLowerCase();

    if (action == "move" || action == "stand") {
        handleMoveCommand(args);
    } else if (action == "set") {
        handleSetCommand(args);
    } else if (action == "get" || action == "status") {
        handleGetCommand(args);
    } else if (action == "reset") {
        handleResetCommand(args);
    } else if (action == "monitor" || action == "focus" || action == "cal"  || 
               action == "stop"    || action == "reboot"  || action == "raw" || 
               action == "test")
    {
        handleSystemCommand(args);
    } else {
        Serial.printf("  [ERROR] Unknown command action: '%s'\n", action.c_str());
    }
}



// =================================================================
//   第二層處理函式 (Second-Level Handlers)
// =================================================================

void CommandHandler::handleMoveCommand(const std::vector<String>& args) {
    String action = args[0];
    action.toLowerCase();

    // 處理特例 "stand"
    if (action == "stand") {
        if (args.size() != 1) { Serial.println("  [ERROR] Use: stand"); return; }
        Serial.println("  [CMD] Robot standing up...");
        _robot->setRobotPoseCascade(manual_calibration_pose_rad);
        return;
    }

    // --- 所有 'move' 指令至少需要3個參數: move <target> <value...> ---
    if (args.size() < 2) {
        Serial.println("  [ERROR] Invalid move command. Use: move <target> <value...>");
        return;
    }
    String target_str = args[1];
    target_str.toLowerCase();

    // --- 修改：實現精煉後的相對/絕對運動邏輯 ---
    if (target_str.startsWith("m")) {
        // 情況 1: 絕對運動 (參數數量為 3)
        // 語法: move m<id> <value>
        if (args.size() == 3) {
            String id_str = target_str.substring(1);
            // 可以在此處加入對 id_str 和 args[2] 的健壯性驗證
            int motor_id = id_str.toInt();
            float value = args[2].toFloat();
            _robot->setTargetPositionCascade(motor_id, value);
        } 
        // 情況 2: 相對運動 (參數數量為 4，且第3個參數是 "+=")
        // 語法: move m<id> += <value>
        else if (args.size() == 4 && args[2] == "+=") {
            String id_str = target_str.substring(1);
            // 可以在此處加入對 id_str 和 args[3] 的健壯性驗證
            int motor_id = id_str.toInt();
            float value = args[3].toFloat();
            _robot->moveMotorRelative_rad(motor_id, value);
        } 
        // 其他所有情況都是錯誤的語法
        else {
            Serial.println("  [ERROR] Invalid syntax. Use 'move m<id> <angle>' for absolute or 'move m<id> += <angle>' for relative.");
        }

    } else if (target_str.startsWith("g")) { // move g<name> ...
        String group_name = target_str.substring(1);
        if (group_name == "h" || group_name == "u" || group_name == "l") { // move g<h/u/l> <angle>
            if (args.size() != 3) { Serial.println("  [ERROR] Use: move g<h/u/l> <angle>"); return; }
            float angle = args[2].toFloat();
            RobotController::JointGroup group;
            if(group_name == "h") group = RobotController::JointGroup::HIP;
            else if (group_name == "u") group = RobotController::JointGroup::UPPER;
            else group = RobotController::JointGroup::LOWER;
            _robot->setJointGroupPositionCascade(group, angle);
        } else if (group_name.startsWith("l")) { // move gl<id> <h> <u> <l>
            if (args.size() != 5) { Serial.println("  [ERROR] Use: move gl<id> <h> <u> <l>"); return; }
            int leg_id = group_name.substring(1).toInt();
            float h = args[2].toFloat(); float u = args[3].toFloat(); float l = args[4].toFloat();
            _robot->setLegJointsCascade(leg_id, h, u, l);
        } else if (group_name == "f" || group_name == "r") { // move g<f/r> <h> <u> <l>
            if (args.size() != 5) { Serial.println("  [ERROR] Use: move g<f/r> <h> <u> <l>"); return; }
            RobotController::JointGroup group = (group_name == "f") ? RobotController::JointGroup::LEG_FRONT : RobotController::JointGroup::LEG_REAR;
            float h = args[2].toFloat(); float u = args[3].toFloat(); float l = args[4].toFloat();
            _robot->setLegPairCascade(group, h, u, l);
        } else {
            Serial.printf("  [ERROR] Unknown or invalid group for 'move': '%s'\n", target_str.c_str());
            Serial.println("  Valid groups for 'move': g<h/u/l>, gl<0-3>, g<f/r>");
        }

    } else if (target_str == "all") { // move all <12 angles>
        if (args.size() != 1 + 1 + 12) { Serial.println("  [ERROR] Use: move all <angle0> <angle1> ... <angle11>"); return; }
        std::array<float, NUM_ROBOT_MOTORS> pose_rad;
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            pose_rad[i] = args[i + 2].toFloat();
        }
        _robot->setRobotPoseCascade(pose_rad);

    } else {
        Serial.printf("  [ERROR] Unknown move target: '%s'\n", target_str.c_str());
    }
}

void CommandHandler::handleSetCommand(const std::vector<String>& args) {
    if (args.size() != 4) {
        Serial.println("  [ERROR] Invalid format. Use: set <target> <param> <value>");
        return;
    }

    String target_str = args[1];
    String param_name = args[2];
    float value = args[3].toFloat();
    target_str.toLowerCase();
    param_name.toLowerCase();
    
    if (target_str == "all") {
        _robot->setParamOverride(RobotController::ParamScope::GLOBAL, -1, param_name.c_str(), value);
        Serial.printf("  [OK] Global param '%s' set to %.4f\n", param_name.c_str(), value);
    } else if (target_str.startsWith("g")) {
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
    } else if (target_str.startsWith("m")) {
        // --- 強化：同樣增加對 motor_id 的驗證 ---
        String id_str = target_str.substring(1);
        for(char c : id_str) { if(!isDigit(c)) { Serial.printf("  [ERROR] Invalid motor ID: '%s'\n", id_str.c_str()); return; } }
        int motor_id = id_str.toInt();
        if (motor_id < 0 || motor_id >= NUM_ROBOT_MOTORS) { Serial.printf("  [ERROR] Motor ID %d out of range (0-%d).\n", motor_id, NUM_ROBOT_MOTORS - 1); return; }

        // --- 強化：對設定值 value 進行驗證 ---
        if (value == 0.0f && args[3] != "0" && args[3] != "0.0") {
            Serial.printf("  [ERROR] Invalid parameter value: '%s'\n", args[3].c_str());
            return;
        }

        _robot->setParamOverride(RobotController::ParamScope::MOTOR, motor_id, param_name.c_str(), value);
        Serial.printf("  [OK] Motor %d param '%s' set to %.4f\n", motor_id, param_name.c_str(), value);
    } else {
        Serial.printf("  [ERROR] Unknown target: '%s'\n", target_str.c_str());
    }
}


void CommandHandler::handleGetCommand(const std::vector<String>& args) {
    String action = args[0];
    action.toLowerCase();

    if (action == "status") {
        if (args.size() != 1) { Serial.println("  [ERROR] Use: status"); return; }
        // 強制切換到人類可讀模式並打印一次
        _telemetry->setPrintMode(TelemetrySystem::PrintMode::HUMAN_STATUS);
        _telemetry->updateAndPrint();
        return;
    }

    // --- 'get' 指令 ---
    if (args.size() < 2 || args.size() > 3) {
        Serial.println("  [ERROR] Invalid format. Use: get <target> [source]");
        return;
    }

    String target_str = args[1];
    target_str.toLowerCase();
    bool get_source = (args.size() == 3 && String(args[2]).toLowerCase() == "source");
    
    if (target_str == "all") {
        if (get_source) { Serial.println("  [INFO] 'get all source' is not a valid query."); return; }
        Serial.println("--- Global Cascade Parameters (Effective Values) ---");
        CascadeParams params = _robot->getEffectiveParams(0); // Use motor 0 to represent global
        printParams(-1, params); // Use -1 to indicate it's a global print
    } else if (target_str.startsWith("g")) {
        String group_name_short = target_str.substring(1);
        std::vector<int> motor_ids = _robot->getMotorIdsForGroup(group_name_short.c_str());
        if (motor_ids.empty()) { Serial.printf("  [ERROR] Unknown group: '%s'\n", target_str.c_str()); return; }

        Serial.printf("--- Parameters for Group '%s' ---\n", target_str.c_str());
        if (get_source) {
             Serial.println("ID | c(src)    | kp(src)   | ki(src)   | max_vel(src) | max_err(src)");
        } else {
             Serial.println("ID |    c    |  vel_kp |  vel_ki | max_vel | max_err");
        }
        Serial.println("---+-----------+-----------+-----------+--------------+-------------");
        for (int id : motor_ids) {
            if(get_source) printParamSources(id); else printParams(id, _robot->getEffectiveParams(id));
        }
    } else if (target_str.startsWith("m")) {
        int motor_id = target_str.substring(1).toInt();
        if (get_source) {
            Serial.printf("--- Parameter Sources for Motor %d ---\n", motor_id);
            printParamSources(motor_id, true);
        } else {
            Serial.printf("--- Effective Parameters for Motor %d ---\n", motor_id);
            printParams(motor_id, _robot->getEffectiveParams(motor_id));
        }
    } else {
        Serial.printf("  [ERROR] Unknown target: '%s'\n", target_str.c_str());
    }
}


void CommandHandler::handleResetCommand(const std::vector<String>& args) {
    if (args.size() < 2 || args.size() > 3) {
        Serial.println("  [ERROR] Invalid format. Use: reset <target> [param]");
        return;
    }

    String target_str = args[1];
    String param_name = (args.size() == 3) ? args[2] : "";
    target_str.toLowerCase();
    param_name.toLowerCase();

    if (target_str == "global") {
        _robot->resetParamOverride(RobotController::ParamScope::GLOBAL, -1, param_name.c_str());
        Serial.printf("  [OK] Global param '%s' reset to system default.\n", param_name.c_str());
    } else if (target_str == "all") {
        // 終極重置
        _robot->resetParamOverride(RobotController::ParamScope::GLOBAL, -1, "");
        for (int i=0; i < NUM_ROBOT_MOTORS; ++i) {
            _robot->resetParamOverride(RobotController::ParamScope::MOTOR, i, "");
        }
        Serial.println("  [OK] All global and motor-specific params reset.");
    } else if (target_str.startsWith("g")) {
        String group_name_short = target_str.substring(1);
        std::vector<int> motor_ids = _robot->getMotorIdsForGroup(group_name_short.c_str());
        if (motor_ids.empty()) { /* ... */ return; }
        for (int id : motor_ids) {
            _robot->resetParamOverride(RobotController::ParamScope::MOTOR, id, param_name.c_str());
        }
        Serial.printf("  [OK] Group '%s' param '%s' reset.\n", target_str.c_str(), param_name.c_str());
    } else if (target_str.startsWith("m")) {
        int motor_id = target_str.substring(1).toInt();
        _robot->resetParamOverride(RobotController::ParamScope::MOTOR, motor_id, param_name.c_str());
        Serial.printf("  [OK] Motor %d param '%s' reset.\n", motor_id, param_name.c_str());
    } else {
        Serial.printf("  [ERROR] Unknown target: '%s'\n", target_str.c_str());
    }
}


void CommandHandler::handleSystemCommand(const std::vector<String>& args) {
    String action = args[0];
    action.toLowerCase();

    if(action == "monitor") {
        if (args.size() < 2) { Serial.println("  [ERROR] Use: monitor <subcommand> [value]"); return; }
        String sub_cmd = args[1];
        sub_cmd.toLowerCase();

        // 格式切換 (吸收舊 mode 指令)
        if (sub_cmd == "h" || sub_cmd == "human") _telemetry->setPrintMode(TelemetrySystem::PrintMode::HUMAN_STATUS);
        else if (sub_cmd == "c" || sub_cmd == "csv") _telemetry->setPrintMode(TelemetrySystem::PrintMode::CSV_LOG);
        else if (sub_cmd == "d" || sub_cmd == "dashboard") _telemetry->setPrintMode(TelemetrySystem::PrintMode::DASHBOARD);
        // 暫停/恢復
        else if (sub_cmd == "pause") _telemetry->pause();
        else if (sub_cmd == "resume") _telemetry->resume();
        // 頻率設定 (吸收 freq 指令)
        else if (sub_cmd == "freq") {
            if (args.size() != 3) { Serial.println("  [ERROR] Use: monitor freq <hz>"); return; }
            int hz = args[2].toInt();
            if (hz > 0) {
                g_print_interval_millis = 1000 / hz;
                Serial.printf("  [OK] Monitor frequency set to %d Hz.\n", hz);
            } else { Serial.println("  [ERROR] Frequency must be > 0."); }
        } else {
            Serial.printf("  [ERROR] Unknown 'monitor' subcommand: %s\n", sub_cmd.c_str());
        }

    } else if (action == "focus") {
        if (args.size() != 2) { Serial.println("  [ERROR] Use: focus <m_id|off>"); return; }
        String target = args[1];
        target.toLowerCase();
        if (target == "off") _telemetry->setFocusMotor(-1);
        else if (target.startsWith("m")) _telemetry->setFocusMotor(target.substring(1).toInt());
        else Serial.println("  [ERROR] Invalid focus target.");

    } else if (action == "cal") {
        if (args.size() != 1) { Serial.println("  [ERROR] Use: cal"); return; }
        _robot->performManualCalibration();

    } else if (action == "stop") {
        if (args.size() != 1) { Serial.println("  [ERROR] Use: stop"); return; }
        _robot->setIdle();
    
    } else if (action == "reboot") {
        if (args.size() != 1) { Serial.println("  [ERROR] Use: reboot"); return; }
        Serial.println("Rebooting system...");
        delay(100);
        void (*resetFunc)(void) = 0;
        resetFunc();
    
    } else if (action == "raw") {
        if(args.size() != 3 || !String(args[1]).toLowerCase().startsWith("m")) { Serial.println("  [ERROR] Use: raw m<id> <mA>"); return; }
        int motor_id = String(args[1]).substring(1).toInt();
        _robot->setSingleMotorCurrent(motor_id, args[2].toInt());
    
    } else if (action == "test") {
        if(args.size() != 3 || args[1] != "wiggle" || !String(args[2]).toLowerCase().startsWith("m")) { Serial.println("  [ERROR] Use: test wiggle m<id>"); return; }
        int motor_id = String(args[2]).substring(1).toInt();
        _robot->startWiggleTest(motor_id);
    }
}



// =================================================================
//   輔助打印函式 (Helper Print Functions)
// =================================================================
void CommandHandler::printParams(int motor_id, const CascadeParams& params) {
    char buf[120];
    if (motor_id == -1) { // -1 is a special value for printing global params
        snprintf(buf, sizeof(buf), "Global | %7.2f | %7.1f | %7.1f | %7.2f | %7.2f",
             params.c, params.vel_kp, params.vel_ki, 
             params.max_target_velocity_rad_s, params.integral_max_error_rad);
    } else {
        snprintf(buf, sizeof(buf), "%2d     | %7.2f | %7.1f | %7.1f | %7.2f | %7.2f",
             motor_id, params.c, params.vel_kp, params.vel_ki, 
             params.max_target_velocity_rad_s, params.integral_max_error_rad);
    }
    Serial.println(buf);
}

void CommandHandler::printParamSources(int motor_id, bool single_line) {
    ParamSourceInfo s = _robot->getParamSourceInfo(motor_id);
    char buf[120];
    if(single_line) {
        snprintf(buf, sizeof(buf), "%2d     | %-9s | %-9s | %-9s | %-12s | %-11s",
                motor_id, s.c_source.c_str(), s.vel_kp_source.c_str(), s.vel_ki_source.c_str(),
                s.max_vel_source.c_str(), s.max_err_source.c_str());
        Serial.println(buf);
    } else {
        Serial.printf("  c:       %s\n", s.c_source.c_str());
        Serial.printf("  vel_kp:  %s\n", s.vel_kp_source.c_str());
        Serial.printf("  vel_ki:  %s\n", s.vel_ki_source.c_str());
        Serial.printf("  max_vel: %s\n", s.max_vel_source.c_str());
        Serial.printf("  max_err: %s\n", s.max_err_source.c_str());
    }
}
