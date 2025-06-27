// src/main.cpp

#include <Arduino.h>
#include "LSM6DSO_SPI.h"      // 引入 IMU 程式庫
#include "MotorController.h"  // 引入馬達控制器程式庫

// --- 建立全域物件 ---
LSM6DSO myIMU;
MotorController myMotorControl;

// --- 設定 ---
const int PRINT_INTERVAL_MILLIS = 200; // ms 打印頻率
long last_print_time = 0;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);

    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- Integrated IMU and Motor Reader ---");

    // --- 初始化 IMU ---
    if (myIMU.begin()) {
        Serial.println("[SUCCESS] IMU Initialized.");
    } else {
        Serial.println("[FAILURE] IMU Initialization Failed! Halting.");
        while(1);
    }
    
    // --- 初始化馬達控制器 ---
    myMotorControl.begin();
    Serial.println("[SUCCESS] Motor Controller Initialized.");
    Serial.println("----------------------------------------");
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // 1. 持續輪詢所有硬體
    myIMU.readSensor();
    myMotorControl.pollAll();

    // 2. 定時打印所有數據
    if (millis() - last_print_time >= PRINT_INTERVAL_MILLIS) {
        last_print_time = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // --- 打印 IMU 數據 ---
        char imu_buf[80];
        sprintf(imu_buf, "IMU Acc(g) -> X:%+7.3f Y:%+7.3f Z:%+7.3f", myIMU.accG[0], myIMU.accG[1], myIMU.accG[2]);
        Serial.println(imu_buf);
        sprintf(imu_buf, "IMU Gyro(dps)-> X:%+7.3f Y:%+7.3f Z:%+7.3f", myIMU.gyroDPS[0], myIMU.gyroDPS[1], myIMU.gyroDPS[2]);
        Serial.println(imu_buf);
        Serial.println("---");

        // --- 打印馬達數據 ---
        for (int i = 0; i < TOTAL_MOTORS; i++) {
            float pos = myMotorControl.getPosition_rad(i);
            float vel = myMotorControl.getVelocity(i);
            
            char motor_buf[50];
            // 使用 sprintf 進行格式化對齊
            sprintf(motor_buf, "Motor %2d | Pos: %+9.4f | Vel: %+9.4f", i, pos, vel);
            Serial.println(motor_buf);
        }
        Serial.println("========================================\n");
    }
}