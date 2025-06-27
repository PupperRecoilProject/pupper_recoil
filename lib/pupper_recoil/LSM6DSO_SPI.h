// lib/LSM6DSO_Driver/LSM6DSO_SPI.h

#ifndef LSM6DSO_SPI_H
#define LSM6DSO_SPI_H

#include <Arduino.h>
#include <SPI.h>

// 量測範圍選項的定義 (放在 .h 檔，這樣主程式也能用)
enum AccelScale { AFS_2G, AFS_4G, AFS_8G, AFS_16G };
enum GyroScale { GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };

class LSM6DSO {
public: // --- 公開介面 (Public Interface) ---
    // 這些是您可以從 main.cpp 中直接呼叫的函式和變數

    // 構造函式：初始化一些預設值
    LSM6DSO(uint8_t csPin = 8); 

    // 初始化函式：設定 SPI、檢查 WHO_AM_I、配置感測器
    bool begin(AccelScale aScale = AFS_2G, GyroScale gScale = GFS_2000DPS);

    // 讀取函式：從感測器讀取最新的 6 軸數據
    void readSensor();

    // 公開的數據變數，方便直接存取
    float accG[3];    // [x, y, z] 加速度 (g)
    float gyroDPS[3]; // [x, y, z] 角速度 (dps)
    int16_t accRaw[3];  // 原始數據
    int16_t gyroRaw[3]; // 原始數據

private: // --- 私有成員 (Private Members) ---
    // 這些是類別內部使用的變數和函式，外部無法直接存取，實現了 "封裝"

    uint8_t _csPin; // 儲存晶片選擇腳位
    float _accelSensitivity;
    float _gyroSensitivity;

    // 底層 SPI 讀寫函式
    void writeRegister(byte reg, byte value);
    byte readRegister(byte reg);
    void readRegisters(byte reg, uint8_t count, byte* dest);
};

#endif