// --- START OF FILE src/motor_homing_main.cpp ---

#include <Arduino.h>
#include "DriveSystem.h"

// --- 全域設定 ---
const uint8_t IMU_CS_PIN = 8;
DriveSystem myRobotDrive(IMU_CS_PIN);

const unsigned long PRINT_INTERVAL_MS = 500;
unsigned long lastPrintTime = 0;

void printStatus() {
    // 打印當前控制模式和馬達位置，用於監控
    Serial.print("Mode: ");
    // (注意: control_mode_ 是私有的，我們需要一個 getter 來讀取它，或者直接打印馬達數據)
    // 為了簡單起見，我們先只打印馬達數據
    
    Serial.print("Positions: ");
    for (int i = 0; i < 12; i++) {
        Serial.print(myRobotDrive.GetActuatorPosition(i), 2);
        if (i < 11) Serial.print(", ");
    }
    Serial.println();
}


void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- Motor Homing/Calibration Program ---");
    Serial.println("Initializing Drive System...");

    // 這裡我們不需要 IMU，但還是要呼叫 setup 函式來完成初始化流程
    myRobotDrive.SetupIMU(100); 
    
    // --- 在這裡調整可設定的參數 ---
    Serial.println("Setting custom parameters...");
    
    // 1. 設定獨立的 Homing 電流閾值
    // 假設 2, 5, 8, 11 是膝關節，需要更大電流
    // std::array<float, 12> thresholds = {
    //     1.0f, 1.0f, 1.5f,  // Front-Right
    //     1.0f, 1.0f, 1.5f,  // Front-Left
    //     1.0f, 1.0f, 1.5f,  // Rear-Right
    //     1.0f, 1.0f, 1.5f   // Rear-Left
    // };
    // constexpr std::array<float, 3> pattern = {1.0f, 1.0f, 1.5f};
    // std::array<float, 12> thresholds = {
    //     pattern[0], pattern[1], pattern[2],
    //     pattern[0], pattern[1], pattern[2],
    //     pattern[0], pattern[1], pattern[2],
    //     pattern[0], pattern[1], pattern[2]
    // };
    std::array<float, 12> thresholds;
    thresholds.fill(1.0f);  // 全部設為 1.0f


    myRobotDrive.SetHomingThresholds(thresholds);
    Serial.println(" - Custom homing thresholds set.");

    // 2. 設定位置控制的 PD 增益
    myRobotDrive.SetPositionKp(10.0f); // 比例增益
    myRobotDrive.SetPositionKd(0.2f);  // 微分增益 (阻尼)
    Serial.println(" - Position PD gains set.");

    // ------------------------------------

    myRobotDrive.SetIdle(); // 確保開機後處於安全閒置狀態

    Serial.println("System Initialized. Ready for commands.");
    Serial.println("Send 'h' to start the homing sequence.");
    Serial.println("----------------------------------------");
}

void loop() {
    // 1. 持續更新驅動系統的狀態機
    //    這對於 Homing 程序的推進至關重要！
    myRobotDrive.CheckForCANMessages(); // 假設需要輪詢 CAN bus
    myRobotDrive.Update();

    // 2. 檢查來自序列埠的命令
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'h' || command == 'H') {
            Serial.println("\n[COMMAND RECEIVED] Starting Homing Sequence...");
            Serial.println("!!! ENSURE ROBOT HAS CLEARANCE TO MOVE !!!");
            delay(1000); // 給使用者一點反應時間
            myRobotDrive.ExecuteHomingSequence();
        }
        if (command == 's' || command == 'S') {
            Serial.println("\n[COMMAND] Moving to standing pose...");
            // 定義一個站立姿態 (所有馬達都在零位)
            ActuatorPositionVector stand_pose;
            stand_pose.fill(0.0f); 
            myRobotDrive.SetJointPositions(stand_pose);
        }
    }

    // 3. 定期打印狀態信息
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        printStatus();
    }

    delay(10); 
}