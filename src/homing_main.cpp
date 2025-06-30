// --- START OF FILE homing_main.cpp ---

// src/imu_motor_test_main.cpp

#include <Arduino.h>
#include <MotorController.h>  // 引入我們自己封裝的馬達控制器
#include "RobotController.h"  // 引入新的高階機器人控制器 (注意: .h 檔在 include/ 資料夾)
#include <LSM6DSO_SPI.h>      // 引入 IMU 程式庫

// --- 建立全域物件 ---
LSM6DSO         myIMU;              // IMU 感測器物件
MotorController myMotorControl;       // 馬達控制器物件 (底層)
RobotController myRobot(&myMotorControl); // 機器人控制器物件 (高層)，並將馬達控制器傳入

// =================================================================
// --- 固定頻率控制設定 ---
// =================================================================
// --- 控制迴圈頻率 ---
const int CONTROL_FREQUENCY_HZ = 1000; // 控制迴圈頻率 (Hz)。1000Hz (1ms) 是一個常見且高性能的選擇。
const long CONTROL_INTERVAL_MICROS = 1000000 / CONTROL_FREQUENCY_HZ; // 計算對應的執行間隔 (微秒)
unsigned long last_control_time_micros = 0; // 上次控制迴圈執行的時間戳
// --- 數據打印頻率設定 ---
const int PRINT_FREQUENCY_HZ = 1; // 預設 1Hz，避免影響性能
const long PRINT_INTERVAL_MILLIS = 1000 / PRINT_FREQUENCY_HZ;
long last_print_time_millis = 0; // 使用獨立的時間戳

// --- 函式原型宣告 ---
void handleSerialCommand(String command);
void printRobotStatus();

// =================================================================
//   SETUP - 程式啟動時只會執行一次
// =================================================================
void setup() {
    // 初始化板載 LED 作為狀態指示燈
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // 啟動時先點亮
    delay(500);

    // 初始化序列埠通訊，用於打印數據和接收指令
    Serial.begin(115200);
    while (!Serial && millis() < 4000); // 等待序列埠連接，最多 4 秒

    Serial.println("\n--- Pupper Robot Control System ---");

    // --- 初始化 IMU ---
    if (myIMU.begin()) {
        Serial.println("[SUCCESS] IMU Initialized.");
    } else {
        Serial.println("[FAILURE] IMU Initialization Failed! Halting.");
        while(1); // 初始化失敗，停止運行
    }
    
    // --- 初始化馬達控制器 ---
    myMotorControl.begin(); // 雖然是空的，但保留好習慣
    Serial.println("[SUCCESS] Motor Controller Initialized.");
    
    // --- 初始化機器人控制器 ---
    myRobot.begin(); // 初始化機器人控制器的參數 (如歸零方向等)
    Serial.println("[SUCCESS] Robot Controller Initialized.");
    Serial.println("----------------------------------------");
    
    // --- 打印指令說明 ---
    Serial.println("Commands:");
    Serial.println("  --- High-level ---");
    Serial.println("  home          - Start automatic homing sequence.");
    Serial.println("  pos <id> <rad> - Set target position for one motor (e.g., 'pos 0 1.57').");
    Serial.println("  wiggle <id>   - Start wiggle test for a motor.");
    Serial.println("  --- Manual & Stop ---");
    Serial.println("  motor <id> <mA> - Manually set current for one motor (e.g., 'motor 0 500').");
    Serial.println("  stop          - Stop all motors and enter IDLE mode.");
    Serial.println("  reboot        - Reboot the microcontroller.");
    Serial.println("========================================\n");
    
    digitalWrite(LED_BUILTIN, LOW); // 初始化完成，熄滅 LED
}

// =================================================================
//   LOOP - 採用新的固定頻率邏輯
// =================================================================
void loop() {
    // -------------------------------------------------
    // 1. 高頻任務 (每個迴圈都執行)
    // -------------------------------------------------
    // 持續輪詢 CAN bus，盡快接收馬達回傳數據
    myMotorControl.pollAll(); 
    
    // 持續檢查用戶指令，確保即時響應
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        handleSerialCommand(command);
    }

    // -------------------------------------------------
    // 2. 固定頻率的控制任務 (例如 1000Hz)
    // -------------------------------------------------
    // 獲取當前時間 (微秒)
    unsigned long current_micros = micros();
    // 檢查距離上次執行是否已超過設定的間隔
    if (current_micros - last_control_time_micros >= CONTROL_INTERVAL_MICROS) {
        // 更新上次執行時間戳
        last_control_time_micros = current_micros;

        // --- 在這個固定頻率的區塊內執行所有時間敏感的操作 ---
        
        // a. 讀取感測器
        myIMU.readSensor();

        // b. 更新機器人主控制器 (計算並發送馬達指令)
        // 這確保了我們的控制指令是以穩定的 1000Hz 頻率發送的
        myRobot.update();
    }

    // -------------------------------------------------
    // 3. 低頻率的數據打印任務 (例如 1Hz)
    // -------------------------------------------------
    // 獲取當前時間 (毫秒)
    unsigned long current_millis = millis();
    // 檢查距離上次打印是否已超過設定的間隔
    if (current_millis - last_print_time_millis >= PRINT_INTERVAL_MILLIS) {
        // 更新上次打印時間戳
        last_print_time_millis = current_millis;

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // 閃爍 LED
        printRobotStatus(); // 呼叫打印函式
    }
}

// =================================================================
//   輔助函式 (Helper Functions)
// =================================================================

/**
 * @brief 處理從序列埠接收到的指令
 * @param command 從序列埠讀取到的字串指令
 */
void handleSerialCommand(String command) {
    if (command == "home") {
        Serial.println("--> Command received: [home]");
        myRobot.startHoming();

    } else if (command.startsWith("pos ")) {
        Serial.print("--> Command received: [");
        Serial.print(command);
        Serial.println("]");
        
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            float angle_rad = command.substring(space2 + 1).toFloat();

            myRobot.setTargetPosition_rad(motorID, angle_rad);

        } else {
            Serial.println("  [ERROR] Invalid format. Use 'pos <id> <angle_in_radians>'.");
        }
        
    } else if (command.startsWith("wiggle ")) {
        int motorID = command.substring(7).toInt();
        Serial.printf("--> Command received: [wiggle %d]\n", motorID);
        myRobot.startWiggleTest(motorID);

    } else if (command.startsWith("motor ")) {
        Serial.print("--> Command received: [");
        Serial.print(command);
        Serial.println("]");
        
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            int current = command.substring(space2 + 1).toInt();

            myRobot.setSingleMotorCurrent(motorID, current);

        } else {
            Serial.println("  [ERROR] Invalid format. Use 'motor <id> <current_in_mA>'.");
        }
    
    } else if (command == "stop") {
        Serial.println("--> Command received: [stop]");
        myRobot.setIdle();

    } else if (command == "reboot") {
        Serial.println("--> Command received: [reboot]. Rebooting now...");
        delay(100);
        #ifdef __arm__
        SCB_AIRCR = 0x05FA0004;
        #endif

    } else {
        Serial.print("  [ERROR] Unknown command: ");
        Serial.println(command);
    }
}

/**
 * @brief 打印所有相關的狀態和數據到序列埠
 */
void printRobotStatus() {
    char buf[100];

    Serial.println("---------------- ROBOT STATUS ----------------");

    // --- 打印機器人控制器狀態 ---
    Serial.print("Robot Mode: ");
    Serial.println(myRobot.getModeString());

    // --- 打印 IMU 數據 ---
    snprintf(buf, sizeof(buf), "IMU Acc(g) -> X:%+7.3f Y:%+7.3f Z:%+7.3f", myIMU.accG[0], myIMU.accG[1], myIMU.accG[2]);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "IMU Gyro(dps)-> X:%+7.3f Y:%+7.3f Z:%+7.3f", myIMU.gyroDPS[0], myIMU.gyroDPS[1], myIMU.gyroDPS[2]);
    Serial.println(buf);
    Serial.println("---");

    // --- 打印馬達數據 ---
    Serial.println("Motor Data (ID | Calib. Pos (rad) | Velocity (radps) | Offset (rad))"); 
    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        float pos_rad = myRobot.getMotorPosition_rad(i);
        float vel_rad = myRobot.getMotorVelocity_rad(i);
        float offset_rad = myMotorControl.getOffset_rad(i);

        snprintf(buf, sizeof(buf), "Motor %2d | Pos: %+9.4f | Vel: %+9.4f | Offset: %+9.4f", i, pos_rad, vel_rad, offset_rad);
        Serial.println(buf);
    }
    Serial.println("================================================\n");
}