// --- START OF FILE src/read_test_main.cpp ---

#include <Arduino.h>
#include "DriveSystem.h"

const uint8_t IMU_CS_PIN = 8;
DriveSystem myRobotDrive(IMU_CS_PIN);

const unsigned long PRINT_INTERVAL_MS = 100; // 加快打印頻率
unsigned long lastPrintTime = 0;

void printStatus() {
    char buffer[150]; // 加大緩衝區
    Serial.println("--- Motor Positions (rad) ---");
    for (int i = 0; i < 12; i++) {
        sprintf(buffer, " M%02d: %+7.4f", i, myRobotDrive.GetActuatorPosition(i));
        Serial.print(buffer);
        if ((i + 1) % 4 == 0) { // 每 4 個換一行
            Serial.println();
        }
    }
    Serial.println("=================================");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- DriveSystem READ-ONLY Test ---");
    Serial.println("This test only reads motor data. Please turn motors by hand.");

    myRobotDrive.SetupIMU(100); 
    myRobotDrive.SetIdle(); 
    
    Serial.println("System Initialized. Reading data...");
    Serial.println("----------------------------------------");
}

void loop() {
    // 1. 只輪詢 CAN Bus，不發送任何指令
    myRobotDrive.CheckForCANMessages();

    // 2. 定期打印狀態信息
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        printStatus();
    }
}