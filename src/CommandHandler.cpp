// CommandHandler.cpp (Corrected Version for Stage 1)

#include "CommandHandler.h"
#include "RobotController.h"   // 引用完整的定義
#include "TelemetrySystem.h" // 引用完整的定義

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

    // --- High-Level Control (預設使用 Cascade Controller) ---

    if (command == "stand") {
        Serial.println("--> Command: [stand]");
        Serial.println("  Activating CASCADED control for stable standing pose.");
        _robot->setRobotPoseCascade(manual_calibration_pose_rad);

    } else if (command.startsWith("pos ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            float angle_rad = command.substring(space2 + 1).toFloat();
            _robot->setTargetPositionCascade(motorID, angle_rad);
        } else {
            Serial.println("  [ERROR] Invalid format. Use: pos <id> <radians>");
        }

    } else if (command.startsWith("group ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            String group_str = command.substring(space1 + 1, space2);
            float angle_rad = command.substring(space2 + 1).toFloat();
            group_str.toLowerCase();

            RobotController::JointGroup group;
            bool valid_group = true;
            if (group_str == "hip")       group = RobotController::JointGroup::HIP;
            else if (group_str == "upper")  group = RobotController::JointGroup::UPPER;
            else if (group_str == "lower")  group = RobotController::JointGroup::LOWER;
            else valid_group = false;

            if (valid_group) {
                _robot->setJointGroupPositionCascade(group, angle_rad);
            } else {
                Serial.println("  [ERROR] Invalid group. Use: hip, upper, or lower");
            }
        } else {
            Serial.println("  [ERROR] Invalid format. Use: group <group_name> <radians>");
        }
    
    // --- Cascade Controller Tuning ---
    
    } else if (command.startsWith("set_param ")) {
        // <<< MODIFIED: 明確告知使用者此功能暫時停用 >>>
        Serial.printf("--> Command: [%s]\n", command.c_str());
        Serial.println("  [INFO] 'set_param' is being refactored. This command is temporarily disabled.");
        Serial.println("  It will be re-enabled in Stage 2 with enhanced features.");

    } else if (command.startsWith("get_params")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        String arg = command.substring(10).trim();
        if (arg == "all") {
            Serial.println("--- Cascade Parameters for All Motors (Effective Values) ---");
            Serial.println("ID |    c    |  vel_kp |  vel_ki | max_vel | max_err");
            Serial.println("---+---------+---------+---------+---------+---------");
            for (int i=0; i < NUM_ROBOT_MOTORS; i++) {
                // <<< CORRECTED: 改為呼叫新的函式 >>>
                CascadeParams params = _robot->getEffectiveParams(i);
                char buf[120];
                snprintf(buf, sizeof(buf), "%2d | %7.2f | %7.1f | %7.1f | %7.2f | %7.2f",
                         i, params.c, params.vel_kp, params.vel_ki, 
                         params.max_target_velocity_rad_s, params.integral_max_error_rad);
                Serial.println(buf);
            }
        } else {
            int motorID = arg.toInt();
            if (motorID >= 0 && motorID < NUM_ROBOT_MOTORS) {
                // <<< CORRECTED: 改為呼叫新的函式，並修復變數名錯誤 (i -> motorID) >>>
                CascadeParams params = _robot->getEffectiveParams(motorID);
                Serial.printf("--- Cascade Parameters for Motor %d (Effective Values) ---\n", motorID);
                Serial.printf("  c:       %.4f\n", params.c);
                Serial.printf("  vel_kp:  %.4f\n", params.vel_kp);
                Serial.printf("  vel_ki:  %.4f\n", params.vel_ki);
                Serial.printf("  max_vel: %.4f\n", params.max_target_velocity_rad_s);
                Serial.printf("  max_err: %.4f\n", params.integral_max_error_rad);
            } else {
                Serial.println("  [ERROR] Invalid format. Use: get_params <id> or get_params all");
            }
        }

    // --- System & Calibration ---

    } else if (command == "cal") {
        Serial.println("--> Command: [cal]");
        _robot->performManualCalibration();

    } else if (command == "stop") {
        Serial.println("--> Command: [stop]");
        _robot->setIdle();

    } else if (command == "reboot") {
        Serial.println("--> Command: [reboot]");
        delay(100);
        #ifdef __arm__
        SCB_AIRCR = 0x05FA0004;
        #endif

    // --- Testing & Debugging ---
    
    } else if (command.startsWith("test_pid ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);
        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            float angle_rad = command.substring(space2 + 1).toFloat();
            _robot->setTargetPositionPID(motorID, angle_rad);
        } else {
            Serial.println("  [ERROR] Invalid format. Use: test_pid <id> <radians>");
        }

    } else if (command.startsWith("test_wiggle ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int motorID = command.substring(12).toInt();
        _robot->startWiggleTest(motorID);

    } else if (command.startsWith("raw ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);
        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            int current = command.substring(space2 + 1).toInt();
            _robot->setSingleMotorCurrent(motorID, current);
        } else {
            Serial.println("  [ERROR] Invalid format. Use: raw <id> <current_mA>");
        }

    } else if (command.startsWith("print ")) {
        String arg = command.substring(6);
        if (arg == "on") {
            Serial.println("--> Command: [print on]. Enabling extra data printing.");
            // g_enable_extra_prints is now a private member of CommandHandler
            // This logic will be handled by TelemetrySystem in a future refactor,
            // but for now, we keep it here. We need to modify TelemetrySystem
            // to be aware of this flag. For simplicity in Stage 1, we comment this out
            // as it requires more changes to TelemetrySystem.
            Serial.println("  [INFO] 'print' command will be fully functional later.");

        } else if (arg == "off") {
            Serial.println("--> Command: [print off]. Disabling extra data printing.");
            // g_enable_extra_prints = false;
        } else {
            Serial.println("  [ERROR] Invalid format. Use: print <on|off>");
        }

    } else if (command.startsWith("monitor ")) {
        String mode_str = command.substring(8);
        mode_str.trim();
        if (mode_str == "human") {
            _telemetry->setPrintMode(TelemetrySystem::PrintMode::HUMAN_STATUS);
        } else if (mode_str == "csv") {
            _telemetry->setPrintMode(TelemetrySystem::PrintMode::CSV_LOG);
        } else if (mode_str == "dashboard") {
            _telemetry->setPrintMode(TelemetrySystem::PrintMode::DASHBOARD);
        } else {
            Serial.println("  [ERROR] Unknown monitor mode. Use: human, csv, or dashboard");
        } 

    } else if (command.startsWith("freq ")) {
        int hz = command.substring(5).toInt();
        if (hz > 0 && hz <= 100) {
            // This now correctly modifies the global variable in main
            g_print_interval_millis = 1000 / hz;
            Serial.printf("--> [OK] Monitor frequency set to %d Hz (interval: %ld ms).\n", hz, g_print_interval_millis);
        } else {
            Serial.println("  [ERROR] Invalid frequency. Please use a value between 1 and 100.");
        }
    
    } else if (command.startsWith("focus ")) {
        String arg = command.substring(6);
        arg.trim();
        if (arg == "off") {
            _telemetry->setFocusMotor(-1);
        } else {
            // <<< CORRECTED: Added the missing variable declaration >>>
            bool is_valid_number = false;
            if (arg.length() > 0) {
                if (isDigit(arg.charAt(0)) || (arg.charAt(0) == '-' && arg.length() > 1 && isDigit(arg.charAt(1)))) {
                    is_valid_number = true;
                    // A simple check to see if it's all digits after the potential '-'
                    for (int k=1; k < arg.length(); k++) {
                        if (!isDigit(arg.charAt(k))) {
                            is_valid_number = false;
                            break;
                        }
                    }
                }
            }

            if (is_valid_number) {
                int motorID = arg.toInt();
                _telemetry->setFocusMotor(motorID);
            } else {
                 Serial.println("  [ERROR] Invalid argument. Use a motor ID number or 'off'.");
            }
        }
    
    } else if (command.startsWith("leg ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int s1 = command.indexOf(' ');
        int s2 = command.indexOf(' ', s1 + 1);
        int s3 = command.indexOf(' ', s2 + 1);
        int s4 = command.indexOf(' ', s3 + 1);

        if (s1!=-1 && s2!=-1 && s3!=-1 && s4!=-1) {
            int leg_id = command.substring(s1 + 1, s2).toInt();
            float hip_rad = command.substring(s2 + 1, s3).toFloat();
            float upper_rad = command.substring(s3 + 1, s4).toFloat();
            float lower_rad = command.substring(s4 + 1).toFloat();
            _robot->setLegJointsCascade(leg_id, hip_rad, upper_rad, lower_rad);
        } else {
            Serial.println("  [ERROR] Invalid format. Use: leg <id> <hip_rad> <upper_rad> <lower_rad>");
        }

    } else if (command.startsWith("leg_pair ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int s1 = command.indexOf(' ');
        int s2 = command.indexOf(' ', s1 + 1);
        int s3 = command.indexOf(' ', s2 + 1);
        int s4 = command.indexOf(' ', s3 + 1);
        
        if (s1!=-1 && s2!=-1 && s3!=-1 && s4!=-1) {
            String group_str = command.substring(s1 + 1, s2);
            float hip_rad = command.substring(s2 + 1, s3).toFloat();
            float upper_rad = command.substring(s3 + 1, s4).toFloat();
            float lower_rad = command.substring(s4 + 1).toFloat();
            group_str.toLowerCase();

            RobotController::JointGroup group;
            bool valid_group = true;
            if (group_str == "front")      group = RobotController::JointGroup::LEG_FRONT;
            else if (group_str == "rear")  group = RobotController::JointGroup::LEG_REAR;
            else valid_group = false;

            if (valid_group) {
                _robot->setLegPairCascade(group, hip_rad, upper_rad, lower_rad);
            } else {
                Serial.println("  [ERROR] Invalid group. Use: front or rear");
            }
        } else {
            Serial.println("  [ERROR] Invalid format. Use: leg_pair <group> <hip_rad> <upper_rad> <lower_rad>");
        }
    }
    
    // --- Fallback for Unknown Commands ---
    else {
        Serial.printf("  [ERROR] Unknown command: '%s'\n", command.c_str());
    }
}