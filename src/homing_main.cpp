// --- START OF FILE homing_main.cpp ---

// src/imu_motor_test_main.cpp

#include <Arduino.h>
#include <MotorController.h>
#include "RobotController.h"
#include <LSM6DSO_SPI.h>
#include "AHRS.h"
#include "TelemetrySystem.h"

// --- 建立全域物件 ---
LSM6DSO         myIMU;
MotorController myMotorControl;
RobotController myRobot(&myMotorControl);
SimpleAHRS      myAHRS;
TelemetrySystem myTelemetry(&myRobot, &myAHRS, &myMotorControl); 

// =================================================================
// --- 固定頻率控制設定 ---
// =================================================================
const int CONTROL_FREQUENCY_HZ = 1000;
const long CONTROL_INTERVAL_MICROS = 1000000 / CONTROL_FREQUENCY_HZ;
unsigned long last_control_time_micros = 0;

const int PRINT_FREQUENCY_HZ = 2;
long g_print_interval_millis = 1000 / 2;
long last_print_time_millis = 0;

// 用於控制是否打印額外數據的旗標
bool g_enable_extra_prints = false;

// =================================================================
// --- 新的、可靠的序列埠指令讀取邏輯 ---
// =================================================================
const int SERIAL_BUFFER_SIZE = 128;   // 指令緩衝區最大長度
char serial_buffer[SERIAL_BUFFER_SIZE]; // 用來儲存傳入字元的緩衝區
int serial_buffer_pos = 0;            // 目前寫入到緩衝區的位置

void checkAndProcessSerial(); // 檢查並處理序列埠數據的新函式
void handleSerialCommand(String command);
//void printRobotStatus();

// =================================================================
//   SETUP - 程式啟動時只會執行一次
// =================================================================
void setup() {
    // ... (setup 函式的內容完全不變，從 Serial.begin() 到結束)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);

    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- Pupper Robot Control System (v2.0) ---");
    Serial.println("========================================");
    Serial.println("Syntax: <required>, [optional], on|off (options)");
    Serial.println("");

    Serial.println("--- 1. Core Control ---");
    Serial.println("  stand                 - Robot enters stable standing pose.");
    Serial.println("  pos <id> <rad>          - Move a single motor to a position.");
    Serial.println("  group <type> <rad>      - Move a joint group (hip, upper, lower).");
    Serial.println("  stop                  - (SAFETY) Stop all motors and enter IDLE mode.");
    Serial.println("");
    
    Serial.println("--- 2. System & Calibration ---");
    Serial.println("  cal                   - Perform manual calibration (must be in IDLE).");
    Serial.println("  reboot                - Reboot the microcontroller.");
    Serial.println("");

    Serial.println("--- 3. Parameter Tuning (NEW) ---");
    Serial.println("  set_param <scope> <name> <val> - Set a controller parameter.");
    Serial.println("    scope: global | motor <id> | group <g_name>");
    Serial.println("    name : pos_kp, vel_kp, vel_ki, max_vel, max_int");
    Serial.println("    e.g. : set_param motor 3 vel_kp 650.0");
    Serial.println("  reset_param <scope>            - Reset parameters.");
    Serial.println("    scope: global | motor <id> | group <g_name> | all");
    Serial.println("    e.g. : reset_param motor 3");
    Serial.println("  get_param [scope]              - Print current parameters.");
    Serial.println("    scope: global | motor <id> | group <g_name> | all");
    Serial.println("    e.g. : get_param all");
    Serial.println("");
    
    Serial.println("--- 4. Monitoring & Debugging ---");
    Serial.println("  monitor <mode>        - Set output format (human, csv, dashboard).");
    Serial.println("  focus <id|off>        - Focus on a motor's data.");
    Serial.println("  freq <hz>             - Set monitor print frequency (1-100).");
    Serial.println("  print <on|off>        - Toggle extra data printing in 'human' mode.");
    Serial.println("");

    Serial.println("--- 5. Low-Level Testing ---");
    Serial.println("  raw <id> <mA>         - Manually set raw current for one motor.");
    Serial.println("  test_wiggle <id>      - Start wiggle test for a motor.");
    Serial.println("  test_pid <id> <rad>   - Test the old SINGLE-LOOP PID controller.");
    Serial.println("========================================\n");

    
    digitalWrite(LED_BUILTIN, LOW);
}


// =================================================================
//   LOOP - 採用新的指令處理邏輯
// =================================================================
void loop() {
    // -------------------------------------------------
    // 1. 高頻任務 (每個迴圈都執行)
    // -------------------------------------------------
    myMotorControl.pollAll(); 
    
    // *** 修改: 呼叫新的指令處理函式，取代舊的邏輯 ***
    checkAndProcessSerial();

    // -------------------------------------------------
    // 2. 固定頻率的控制任務 (例如 1000Hz)
    // -------------------------------------------------
    unsigned long current_micros = micros();
    if (current_micros - last_control_time_micros >= CONTROL_INTERVAL_MICROS) {
        last_control_time_micros = current_micros;
        
        myIMU.readSensor(); //從 myAHRS 物件中獲取姿態角
        myAHRS.update(myIMU.gyroDPS[0], myIMU.gyroDPS[1], myIMU.gyroDPS[2], myIMU.accG[0], myIMU.accG[1], myIMU.accG[2]);
        
        myRobot.update();
    }

    // -------------------------------------------------
    // 3. 低頻率的數據打印任務 (例如 1Hz)
    // -------------------------------------------------
    unsigned long current_millis = millis();
    if (current_millis - last_print_time_millis >= g_print_interval_millis) {
        last_print_time_millis = current_millis;

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        myTelemetry.updateAndPrint();
    }
}


// =================================================================
//   輔助函式 (Helper Functions)
// =================================================================

/**
 * @brief 檢查序列埠是否有數據，並在接收到換行符時處理指令。
 *        這個函式取代了舊的 loop() 中的 Serial.readStringUntil() 邏輯。
 */
void checkAndProcessSerial() {
    while (Serial.available() > 0) {
        char incoming_char = Serial.read();

        // 如果收到換行符 '\n' (Enter 鍵)
        if (incoming_char == '\n') {
            // 在緩衝區末端加上字串結束符
            serial_buffer[serial_buffer_pos] = '\0';
            
            // 將緩衝區內容轉換為 String 物件來處理
            String command = String(serial_buffer);
            command.trim(); // 去掉前後的空白

            // 只有在指令非空時才處理
            if (command.length() > 0) {
                handleSerialCommand(command);
            }

            // 重置緩衝區，準備接收下一條指令
            serial_buffer_pos = 0;
            serial_buffer[0] = '\0';

        } 
        // 如果收到退格鍵 (Backspace)
        else if (incoming_char == '\b' || incoming_char == 127) {
            if (serial_buffer_pos > 0) {
                serial_buffer_pos--; // 緩衝區位置後退一格
            }
        }
        // 如果是普通字元，且緩衝區未滿
        else if (serial_buffer_pos < SERIAL_BUFFER_SIZE - 1) {
            // 將字元存入緩衝區，並移動位置
            serial_buffer[serial_buffer_pos] = incoming_char;
            serial_buffer_pos++;
        }
    }
}


/**
 * @brief 處理從序列埠接收到的完整指令字串。
 *        此函式根據指令關鍵字，呼叫 RobotController 中對應的功能。
 * @param command 從序列埠讀取並整理好的字串指令。
 */
void handleSerialCommand(String command) {
    
    // --- High-Level Control (預設使用 Cascade Controller) ---

    if (command == "stand") {
        Serial.println("--> Command: [stand]");
        Serial.println("  Activating CASCADED control for stable standing pose.");
        // 保留原有功能：呼叫 setRobotPoseCascade 並傳入完整的校準姿態
        myRobot.setRobotPoseCascade(manual_calibration_pose_rad);

    } else if (command.startsWith("pos ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            float angle_rad = command.substring(space2 + 1).toFloat();
            // 【核心改變】呼叫為單關節設計的 Cascade 控制函式，以獲得更平滑的運動
            myRobot.setTargetPositionCascade(motorID, angle_rad);
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
                // 【核心改變】呼叫為關節組設計的 Cascade 控制函式
                myRobot.setJointGroupPositionCascade(group, angle_rad);
            } else {
                Serial.println("  [ERROR] Invalid group. Use: hip, upper, or lower");
            }
        } else {
            Serial.println("  [ERROR] Invalid format. Use: group <group_name> <radians>");
        }

    // --- System & Calibration ---

    } else if (command == "cal") {
        Serial.println("--> Command: [cal]");
        // 【名稱變更】原 `calibrate` 指令
        myRobot.performManualCalibration();

    } else if (command == "stop") {
        Serial.println("--> Command: [stop]");
        // (保留不變) 核心安全指令
        myRobot.setIdle();

    } else if (command == "reboot") {
        Serial.println("--> Command: [reboot]");
        // (保留不變)
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
            // 【新增功能】呼叫舊版的單環 PID 控制器，用於性能比較和調試
            myRobot.setTargetPositionPID(motorID, angle_rad);
        } else {
            Serial.println("  [ERROR] Invalid format. Use: test_pid <id> <radians>");
        }

    } else if (command.startsWith("test_wiggle ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        // 【名稱變更】原 `wiggle` 指令
        int motorID = command.substring(12).toInt();
        myRobot.startWiggleTest(motorID);

    } else if (command.startsWith("raw ")) {
        Serial.printf("--> Command: [%s]\n", command.c_str());
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);
        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            int current = command.substring(space2 + 1).toInt();
            // 【名稱變更】原 `motor` 指令
            myRobot.setSingleMotorCurrent(motorID, current);
        } else {
            Serial.println("  [ERROR] Invalid format. Use: raw <id> <current_mA>");
        }

    } else if (command.startsWith("print ")) {
        // 【功能整合】將 `printon` 和 `printoff` 合併
        String arg = command.substring(6);
        if (arg == "on") {
            Serial.println("--> Command: [print on]. Enabling extra data printing.");
            g_enable_extra_prints = true;
        } else if (arg == "off") {
            Serial.println("--> Command: [print off]. Disabling extra data printing.");
            g_enable_extra_prints = false;
        } else {
            Serial.println("  [ERROR] Invalid format. Use: print <on|off>");
        }

    } else if (command.startsWith("monitor ")) {
        // 截取 "monitor " 後面的模式字串
        String mode_str = command.substring(8);
        mode_str.trim(); // 去掉前後可能存在的空格

        // 根據模式字串，呼叫 TelemetrySystem 中對應的設定函式
        if (mode_str == "human") {
            myTelemetry.setPrintMode(TelemetrySystem::PrintMode::HUMAN_STATUS);
        } else if (mode_str == "csv") {
            myTelemetry.setPrintMode(TelemetrySystem::PrintMode::CSV_LOG);
        } else if (mode_str == "dashboard") {
            myTelemetry.setPrintMode(TelemetrySystem::PrintMode::DASHBOARD);
        } else {
            // 如果模式無效，給出錯誤提示
            Serial.println("  [ERROR] Unknown monitor mode. Use: human, csv, or dashboard");
        } 

    } else if (command.startsWith("freq ")) {
        int hz = command.substring(5).toInt();
        if (hz > 0 && hz <= 100) { // 設定一個合理的頻率上下限
            g_print_interval_millis = 1000 / hz;
            Serial.printf("--> [OK] Monitor frequency set to %d Hz (interval: %ld ms).\n", hz, g_print_interval_millis);
        } else {
            Serial.println("  [ERROR] Invalid frequency. Please use a value between 1 and 100.");
        }
    
    } else if (command.startsWith("focus ")) {
        // 截取 "focus " 後面的參數
        String arg = command.substring(6);
        arg.trim(); // 去掉前後空格

        // 如果參數是 'off'，則取消焦點
        if (arg == "off") {
            myTelemetry.setFocusMotor(-1); // -1 在 TelemetrySystem 中被定義為"取消焦點"
        } else {
            // 檢查輸入是否為有效的數字（包括負號開頭的情況）
            // 這可以防止 'focus abc' 這樣的指令被 toInt() 錯誤地轉換為 0
            bool is_valid_number = false;
            if (arg.length() > 0) {
                if (isDigit(arg.charAt(0)) || (arg.charAt(0) == '-' && arg.length() > 1 && isDigit(arg.charAt(1)))) {
                    is_valid_number = true;
                }
            }

            if (is_valid_number) {
                int motorID = arg.toInt();
                myTelemetry.setFocusMotor(motorID); // 將ID傳給遙測系統
            } else {
                 Serial.println("  [ERROR] Invalid argument. Use a motor ID number or 'off'.");
            }
        }
    
    } else if (command.startsWith("set_param ")) {
        // 指令格式: set_param <scope_type> <scope_name> <param_name> <value>
        // 或者     set_param <scope_type> <param_name> <value>  (for global)
        command.remove(0, 10); // 移除 "set_param "
        
        int space1 = command.indexOf(' ');
        if (space1 == -1) { Serial.println("  [ERROR] Invalid format for set_param."); return; }
        String scope_type = command.substring(0, space1);

        if (scope_type == "global") {
            command.remove(0, space1 + 1);
            int space2 = command.indexOf(' ');
            if (space2 == -1) { Serial.println("  [ERROR] Invalid format for set_param global."); return; }
            String param_name = command.substring(0, space2);
            float value = command.substring(space2 + 1).toFloat();
            myRobot.setParameter(scope_type.c_str(), "", param_name.c_str(), value);
        } else if (scope_type == "motor" || scope_type == "group") {
            command.remove(0, space1 + 1);
            int space2 = command.indexOf(' ');
            if (space2 == -1) { Serial.println("  [ERROR] Invalid format for set_param motor/group."); return; }
            String scope_name = command.substring(0, space2);
            command.remove(0, space2 + 1);
            int space3 = command.indexOf(' ');
            if (space3 == -1) { Serial.println("  [ERROR] Invalid format for set_param motor/group."); return; }
            String param_name = command.substring(0, space3);
            float value = command.substring(space3 + 1).toFloat();
            myRobot.setParameter(scope_type.c_str(), scope_name.c_str(), param_name.c_str(), value);
        } else {
            Serial.println("  [ERROR] Invalid scope type. Use: global, motor, or group.");
        }

    } else if (command.startsWith("reset_param ")) {
        // 指令格式: reset_param <scope_type> [scope_name]
        command.remove(0, 12); // 移除 "reset_param "
        
        int space1 = command.indexOf(' ');
        String scope_type, scope_name;
        if (space1 == -1) { // 只有一個詞，如 "global" 或 "all"
            scope_type = command;
            scope_name = "";
            if (scope_type == "all") { // 特殊處理 all，它其實是 group 的一種
                 myRobot.resetParameter("group", "all");
            } else {
                 myRobot.resetParameter(scope_type.c_str(), scope_name.c_str());
            }
        } else { // 有兩個詞，如 "motor 3"
            scope_type = command.substring(0, space1);
            scope_name = command.substring(space1 + 1);
            myRobot.resetParameter(scope_type.c_str(), scope_name.c_str());
        }

    } else if (command.startsWith("get_param")) {
        // 指令格式: get_param [scope_type] [scope_name]
        command.remove(0, 9); // 移除 "get_param"
        command.trim();
        
        if (command.length() == 0) { // 如果後面沒東西，就是 get_param global
            myRobot.printParameters("global", "");
        } else {
            int space1 = command.indexOf(' ');
            String scope_type, scope_name;
            if (space1 == -1) { // 只有一個詞，如 "all"
                scope_type = "group"; // all 被當作一個 group
                scope_name = command;
                myRobot.printParameters(scope_type.c_str(), scope_name.c_str());
            } else { // 兩個詞，如 "motor 3"
                scope_type = command.substring(0, space1);
                scope_name = command.substring(space1 + 1);
                myRobot.printParameters(scope_type.c_str(), scope_name.c_str());
            }
        }
    }

    // --- Fallback for Unknown Commands ---
    else {
        Serial.printf("  [ERROR] Unknown command: '%s'\n", command.c_str());
    }

}

/**
 * @brief 打印所有相關的狀態和數據到序列埠 (此函式內容不變)
 */
void printRobotStatus() {
    char buf[120];

    Serial.println("---------------- ROBOT STATUS ----------------");

    Serial.print("Robot Mode: ");
    Serial.println(myRobot.getModeString());

    // 校準狀態的顯示
    Serial.print(" | Calibrated: ");
    Serial.println(myRobot.isCalibrated() ? "YES" : "NO");
    // --- IMU & AHRS Status ---
    Serial.println("Source          |      X / Roll      |      Y / Pitch     |      Z / Yaw");
    Serial.println("----------------+--------------------+--------------------+--------------------");
    snprintf(buf, sizeof(buf), "%-15s | X:     %+10.3f | Y:     %+10.3f | Z:     %+10.3f", 
             "IMU Gyro (dps)", myIMU.gyroDPS[0], myIMU.gyroDPS[1], myIMU.gyroDPS[2]);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "%-15s | X:     %+10.3f | Y:     %+10.3f | Z:     %+10.3f", 
             "IMU Accel (g)", myIMU.accG[0], myIMU.accG[1], myIMU.accG[2]);
    Serial.println(buf);
    Serial.println("----------------+--------------------+--------------------+--------------------");
    snprintf(buf, sizeof(buf), "%-15s | Roll:  %+10.2f | Pitch: %+10.2f | Yaw:   %+10.2f", 
             "AHRS Attitude", myAHRS.roll, myAHRS.pitch, myAHRS.yaw);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "%-15s | X:     %+10.3f | Y:     %+10.3f | Z:     %+10.3f", 
             "Linear Acc (g)", myAHRS.linearAccel[0], myAHRS.linearAccel[1], myAHRS.linearAccel[2]);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "%-15s | X:     %+10.3f | Y:     %+10.3f | Z:     %+10.3f", 
             "Est. Vel (m/s)", myAHRS.velocity[0], myAHRS.velocity[1], myAHRS.velocity[2]);
    Serial.println(buf);
    Serial.println("---"); 

    Serial.println("Motor Data (ID | Calib. Pos (rad) | Velocity (radps) | Offset (rad))"); 
    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        float pos_rad = myRobot.getMotorPosition_rad(i);
        float vel_rad = myRobot.getMotorVelocity_rad(i);
        float offset_rad = myMotorControl.getOffset_rad(i);

        snprintf(buf, sizeof(buf), "Motor %2d | Pos: %+9.4f | Vel: %+9.4f | Offset: %+9.4f", i, pos_rad, vel_rad, offset_rad);
        Serial.println(buf);
    }

    if (g_enable_extra_prints) {
        Serial.println("---");
        // 修改標題以容納更多數據
        Serial.println("Currents & Torque (ID | Target (mA) | Actual (mA) | Actual (A) | Torque (Nm))");
        for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
            // 獲取所有相關數據
            int16_t target_current_mA = myRobot.getTargetCurrent_mA(i);
            // *** 修正：呼叫新的、正確的函式名 ***
            int16_t actual_current_mA = myMotorControl.getRawCurrent_mA(i);
            // *** 新增：獲取安培和扭矩 ***
            float   actual_current_A  = myMotorControl.getCurrent_A(i);
            float   torque_Nm         = myMotorControl.getTorque_Nm(i);

            // 使用 snprintf 格式化輸出
            snprintf(buf, sizeof(buf), "Motor %2d | Tgt: %-8d | Act: %-8d | Act: %+7.3f | Torque: %+8.4f", 
                     i, 
                     target_current_mA, 
                     actual_current_mA, 
                     actual_current_A, 
                     torque_Nm);
            Serial.println(buf);
        }
    }


    Serial.println("================================================\n");
}