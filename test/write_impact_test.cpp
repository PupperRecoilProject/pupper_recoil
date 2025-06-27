// --- START OF FILE src/write_impact_test_main.cpp ---

#include <Arduino.h>
#include "DriveSystem.h"

const uint8_t IMU_CS_PIN = 8;
DriveSystem myRobotDrive(IMU_CS_PIN);

const unsigned long PRINT_INTERVAL_MS = 1000;
unsigned long lastPrintTime = 0;

bool commandSent = false; // 用於確保指令只發送一次的旗標

void printStatus() {
    // ... (打印函式與唯讀測試中的相同)
    char buffer[150];
    Serial.println("--- Motor Positions (rad) ---");
    for (int i = 0; i < 12; i++) {
        sprintf(buffer, " M%02d: %+7.4f", i, myRobotDrive.GetActuatorPosition(i));
        Serial.print(buffer);
        if ((i + 1) % 4 == 0) {
            Serial.println();
        }
    }
    Serial.println("=================================");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- DriveSystem WRITE-IMPACT Test ---");
    myRobotDrive.SetupIMU(100); 
    myRobotDrive.SetIdle(); // 注意：這裡的 SetIdle 會呼叫 CommandIdle，我們把它移到 loop 中手動觸發

    Serial.println("System Initialized. Reading data should work now.");
    Serial.println("Turn motors by hand to verify.");
    Serial.println("Press 'c' to send a single command packet and observe the impact.");
    Serial.println("----------------------------------------");
}

void loop() {
    // 持續輪詢 CAN Bus
    myRobotDrive.CheckForCANMessages();

    // 檢查使用者輸入
    if (Serial.available() > 0) {
        char command = Serial.read();
        if ((command == 'c' || command == 'C') && !commandSent) {
            Serial.println("\n<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            Serial.println("   SENDING SINGLE COMMAND PACKET NOW!  ");
            Serial.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
            
            // 手動呼叫 CommandIdle，它內部會呼叫 CommandCurrents
            myRobotDrive.CommandIdle(); 
            
            commandSent = true; // 設置旗標，確保只發送一次
        }
    }

    // 定期打印狀態信息
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        printStatus();
    }
}