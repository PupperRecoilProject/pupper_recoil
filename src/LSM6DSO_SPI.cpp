// lib/LSM6DSO_Driver/LSM6DSO_SPI.cpp

#include "LSM6DSO_SPI.h"

// --- 暫存器地址 (定義為私有常數，外部不需要知道) ---
namespace { // 使用匿名命名空間來限制這些常數只在此檔案中可見
    const byte WHO_AM_I_REG = 0x0F;
    const byte CTRL1_XL     = 0x10;
    const byte CTRL2_G      = 0x11;
    const byte OUTX_L_G     = 0x22;
    const byte READ_BIT     = 0x80;
}

LSM6DSO::LSM6DSO(uint8_t csPin) {
    _csPin = csPin;
}

bool LSM6DSO::begin(AccelScale aScale, GyroScale gScale) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();

    if (readRegister(WHO_AM_I_REG) != 0x6C) {
        return false;
    }

    // --- 設定加速度計 ---
    byte accConfig = 0x40; // ODR = 104 Hz
    switch (aScale) {
        // 使用正確的位元遮罩，對應 FS_XL[1:0] 在 bit 3 和 bit 2
        case AFS_2G:   _accelSensitivity = 0.061f; accConfig |= 0b0000; break; // FS_XL = 00
        case AFS_16G:  _accelSensitivity = 0.488f; accConfig |= 0b0100; break; // FS_XL = 01
        case AFS_4G:   _accelSensitivity = 0.122f; accConfig |= 0b1000; break; // FS_XL = 10
        case AFS_8G:   _accelSensitivity = 0.244f; accConfig |= 0b1100; break; // FS_XL = 11
    }
    writeRegister(CTRL1_XL, accConfig);

    // --- 設定陀螺儀 (原程式碼已正確，此處為保持一致性) ---
    byte gyroConfig = 0x40; // ODR = 104 Hz
    switch (gScale) {
        case GFS_125DPS:  
            _gyroSensitivity = 4.375f; 
            gyroConfig |= 0b0010; // 這是設定 FS_125 位元 (bit 1)
            break; 
        // 使用正確的位元遮罩，對應 FS_G[1:0] 在 bit 3 和 bit 2
        case GFS_250DPS:  _gyroSensitivity = 8.750f; gyroConfig |= 0b0000; break; // FS_G = 00
        case GFS_500DPS:  _gyroSensitivity = 17.50f; gyroConfig |= 0b0100; break; // FS_G = 01
        case GFS_1000DPS: _gyroSensitivity = 35.00f; gyroConfig |= 0b1000; break; // FS_G = 10
        case GFS_2000DPS: _gyroSensitivity = 70.00f; gyroConfig |= 0b1100; break; // FS_G = 11
    }
    writeRegister(CTRL2_G, gyroConfig);

    return true;
}


void LSM6DSO::readSensor() {
    byte buffer[12];
    readRegisters(OUTX_L_G, 12, buffer);

    gyroRaw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    gyroRaw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    gyroRaw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    accRaw[0] = (int16_t)((buffer[7] << 8) | buffer[6]);
    accRaw[1] = (int16_t)((buffer[9] << 8) | buffer[8]);
    accRaw[2] = (int16_t)((buffer[11] << 8) | buffer[10]);

    for (int i = 0; i < 3; i++) {
        accG[i] = accRaw[i] * _accelSensitivity / 1000.0f;
        gyroDPS[i] = gyroRaw[i] * _gyroSensitivity / 1000.0f;
    }
}

// --- 底層 SPI 函式 ---
void LSM6DSO::writeRegister(byte reg, byte value) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

byte LSM6DSO::readRegister(byte reg) {
    byte result;
    readRegisters(reg, 1, &result);
    return result;
}

void LSM6DSO::readRegisters(byte reg, uint8_t count, byte* dest) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | READ_BIT);
    SPI.transfer(dest, count);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}