// 這個main 試著使用SparkFunLSM6DSO.h 來控制加速度計但有問題，所以棄用

#include <Arduino.h>
#include <SPI.h>              // SparkFun 函式庫需要明確引入 SPI
#include <SparkFunLSM6DSO.h> // *** 變更 #1: 引入新的函式庫標頭檔 ***
#include "MotorController.h"  // 引入馬達控制器程式庫

// --- 建立全域物件 ---
// *** 變更 #2: 使用新的函式庫來建立 IMU 物件 ***
SparkFunLSM6DSO myIMU;
MotorController myMotorControl;

// --- 設定 ---
const int IMU_CS_PIN = 8;              // *** 新增: 明確定義 IMU 的 SPI 片選腳位 ***
const int PRINT_INTERVAL_MILLIS = 200; // ms 打印頻率
long last_print_time = 0;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);

    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- Integrated IMU and Motor Reader (Using SparkFun Lib) ---");

    // --- 初始化 IMU ---
    // *** 變更 #3: 修改 IMU 初始化流程 ***
    SPI.begin(); // 由於我們要用 SPI，先初始化 SPI 總線
    // 使用 .beginSPI() 來初始化，並傳入片選腳位
    if (myIMU.beginSPI(IMU_CS_PIN)) { 
        Serial.println("[SUCCESS] IMU Initialized via SPI.");
    } else {
        Serial.println("[FAILURE] IMU Initialization Failed! Halting.");
        while(1);
    }

    // 設定加速度計和陀螺儀的量測範圍 (Range)
    // 這裡的 API 和您自訂的有些不同，但功能一樣
    myIMU.setAccelRange(2);  // 設定為 ±2g
    myIMU.setGyroRange(2000); // 設定為 ±2000 dps
    Serial.println("[INFO] IMU Range set to Accel: +/- 2g, Gyro: +/- 2000 dps");
    
    // --- 初始化馬達控制器 ---
    myMotorControl.begin();
    Serial.println("[SUCCESS] Motor Controller Initialized.");
    Serial.println("----------------------------------------");
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // 1. 持續輪詢所有硬體
    // *** 變更 #4: SparkFun 函式庫不需要手動呼叫 readSensor() ***
    // 數據會在內部自動更新，我們只需在需要時讀取其成員變數即可
    myMotorControl.pollAll();

    // 2. 定時打印所有數據
    if (millis() - last_print_time >= PRINT_INTERVAL_MILLIS) {
        last_print_time = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // --- 打印 IMU 數據 (使用 milli 單位) ---
        // 使用 readRaw/readInt 函式來獲取整數的 milli-g 和 milli-dps 值
        int16_t accX_mg = myIMU.readRawAccelX();
        int16_t accY_mg = myIMU.readRawAccelY();
        int16_t accZ_mg = myIMU.readRawAccelZ();
        int16_t gyroX_mdps = myIMU.readRawGyroX();
        int16_t gyroY_mdps = myIMU.readRawGyroY();
        int16_t gyroZ_mdps = myIMU.readRawGyroZ();

        char imu_buf[80];
        // 使用 %d 格式化整數，並調整標題
        snprintf(imu_buf, sizeof(imu_buf), "IMU Acc(mg)  -> X:%+6d Y:%+6d Z:%+6d", accX_mg, accY_mg, accZ_mg);
        Serial.println(imu_buf);
        snprintf(imu_buf, sizeof(imu_buf), "IMU Gyro(mdps)-> X:%+6d Y:%+6d Z:%+6d", gyroX_mdps, gyroY_mdps, gyroZ_mdps);
        Serial.println(imu_buf);
        Serial.println("---");

        // --- 打印馬達數據 ---
        for (int i = 0; i < TOTAL_MOTORS; i++) {
            float pos = myMotorControl.getPosition(i);
            float vel = myMotorControl.getVelocity(i);
            
            char motor_buf[50];
            snprintf(motor_buf, sizeof(motor_buf), "Motor %2d | Pos: %+9.4f | Vel: %+9.4f", i, pos, vel);
            Serial.println(motor_buf);
        }
        Serial.println("========================================\n");
    }
}