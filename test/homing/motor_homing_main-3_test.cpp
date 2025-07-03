// --- START OF MODIFIED FILE src/main.cpp ---

#include <Arduino.h>
#include "DriveSystem.h"
#include "LSM6DSO_SPI.h" // 也需要直接包含，因為要存取 myRobotDrive.getIMU().accG 等

// --- 全域設定 ---
const uint8_t IMU_CS_PIN = 8; // 定義 IMU 的 Chip Select (CS) 腳位
DriveSystem myRobotDrive(IMU_CS_PIN); // 在建立物件時傳入 CS 腳位

const unsigned long PRINT_INTERVAL_MS = 1000; // ms 打印頻率
unsigned long lastPrintTime = 0;

// 將打印狀態的邏輯抽離成一個獨立函式，以便調用
void printDriveSystemStatus() {
    // 獲取 DriveSystem 內部 IMU 物件的參考 (透過 getIMU() 函式)
    LSM6DSO& imu = myRobotDrive.getIMU(); // 確保 DriveSystem.h 中有 getIMU()

    char buffer[120];
    Serial.println("--- IMU & Motor Data ---");
    sprintf(buffer, " Accel (g):   X=%+7.3f, Y=%+7.3f, Z=%+7.3f",
            imu.accG[0], imu.accG[1], imu.accG[2]);
    Serial.println(buffer);
    sprintf(buffer, " Gyro (dps):  X=%+8.3f, Y=%+8.3f, Z=%+8.3f",
            imu.gyroDPS[0], imu.gyroDPS[1], imu.gyroDPS[2]);
    Serial.println(buffer);
    Serial.println("---");

    // 使用 DriveSystem::kNumActuators 獲取馬達總數
    for (int i = 0; i < DriveSystem::kNumActuators; i++) {
        float pos = myRobotDrive.GetActuatorPosition(i);
        float vel = myRobotDrive.GetActuatorVelocity(i);

        char motor_buf[50];
        sprintf(motor_buf, "Motor %2d | Pos: %+9.4f | Vel: %+9.4f", i, pos, vel);
        Serial.println(motor_buf);
    }
    Serial.println("========================================\n");
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // 開機時先亮燈
    delay(500);

    Serial.begin(115200);
    while (!Serial && millis() < 4000); // 等待序列埠連接

    Serial.println("\n--- ROBOT CONTROL SYSTEM (SAFE STARTUP) ---");

    // --- 初始化 IMU ---
    // SetupIMU 內部會檢查 WHO_AM_I 並設定 IMU 範圍，如果失敗會自動禁用系統
    myRobotDrive.SetupIMU(100);

    // --- 設定 Homing 閾值 (可選，這裡設定的是範例值) ---
    std::array<float, 12> homing_thresholds;
    homing_thresholds.fill(1.0f); // 預設所有馬達 Homing 閾值為 5.0A
    // 可假設膝關節 (2, 5, 8, 11) 需要更大的電流閾值來偵測碰撞
    // homing_thresholds[2] = homing_thresholds[5] = homing_thresholds[8] = homing_thresholds[11] = 6.5f;
    myRobotDrive.SetHomingThresholds(homing_thresholds);

    Serial.println("System Initialized. Drive system is currently DISABLED.");
    Serial.println("Send 'e' to ENABLE motors.");
    Serial.println("Send 'd' to DISABLE motors (emergency stop).");
    Serial.println("Send 'h' to start homing (requires system to be ENABLED).");
    Serial.println("----------------------------------------");
    digitalWrite(LED_BUILTIN, LOW); // 初始化完成，關燈
}

void loop() {
    // 1. 持續輪詢所有硬體，更新內部狀態機
    myRobotDrive.CheckForCANMessages();
    myRobotDrive.Update();

    // 2. 處理序列埠指令
    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'e': case 'E': { // <<< 修正點 1：加上大括號建立作用域
                Serial.println("\n[COMMAND] Enabling system for Motor 9 only...");
                myRobotDrive.EnableSystem();
                myRobotDrive.SetPositionKp(2.0f);
                myRobotDrive.SetPositionKd(0.0f);
                myRobotDrive.SetMaxCurrent(3.0f);  // 使用較小的電流測試
    
                // 只激活第9號馬達
                ActuatorActivations single_motor; // 變數現在位於獨立作用域內
                single_motor.fill(false);  // 全部設為false
                single_motor[9] = true;    // 只激活第9號馬達
                myRobotDrive.SetActivations(single_motor);
    
                // 設定第9號馬達的目標位置（例如：0.5弧度）
                ActuatorPositionVector target_positions; // 變數現在位於獨立作用域內
                for (int i = 0; i < 12; i++) {
                    target_positions[i] = 0.0f;  // 其他馬達設為0
                }
                target_positions[9] = 0.5f;  // 第9號馬達設為0.5弧度
                myRobotDrive.SetJointPositions(target_positions);
                break;
            } // <<< 修正點 1：作用域結束

            case 'd': case 'D': {
                Serial.println("\n[COMMAND] EMERGENCY STOP! Disabling system...");
                myRobotDrive.DisableSystem(); // 禁用系統
                break;
            }

            case 'h': case 'H': {
                Serial.println("\n[COMMAND] Starting Homing Sequence...");
                myRobotDrive.ExecuteHomingSequence();
                break;
            }
            
            case 'z': {
                myRobotDrive.ZeroCurrentPosition(); // 這會讀取當前原始位置並設為零點
                Serial.println("Motor 9 position has been zeroed.");
                break;
            }

            case '9': { // <<< 修正點 2：加上大括號建立作用域
                // 讀取當前位置
                float current_pos = myRobotDrive.GetActuatorPosition(9); // 變數現在位於獨立作用域內
                // 設定新位置（當前位置 + 0.5弧度）
                float new_pos = current_pos + 0.1f; // 變數現在位於獨立作用域內
                
                ActuatorPositionVector new_targets; // 變數現在位於獨立作用域內
                for (int i = 0; i < 12; i++) {
                    new_targets[i] = 0.0f;  // 其他馬達保持不動
                }
                new_targets[9] = new_pos;  // 只移動第9號馬達
                
                myRobotDrive.SetJointPositions(new_targets);
                Serial.print("Moving Motor 9 to: ");
                Serial.println(new_pos, 4);
                break;
            } // <<< 修正點 2：作用域結束
            
            // 在 main.cpp 的 switch 中新增
            case 't': {
                myRobotDrive.EnableSystem();
                // 直接指令第 9 號馬達輸出一個很小的正電流，比如 0.2A
                myRobotDrive.SetCurrent(9, 0.2f); 
                Serial.println("Sending 0.2A to motor 9");
                break;
            }

            default:
                // 忽略未知指令，避免打印干擾
                break;
        }
    }

    // 3. 定時打印所有數據
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // 閃爍 LED
        printDriveSystemStatus(); // 打印狀態
    }

    // 4. 關鍵的延遲
    delay(1);
}

// --- END OF MODIFIED FILE src/main.cpp ---