#ifndef AHRS_H
#define AHRS_H

#include <MahonyAHRS.h>

class SimpleAHRS {
public:
    // --- 公開變數 ---
    float roll, pitch, yaw;           // 姿態角 (度)
    float linearAccel[3];             // [x, y, z] 線性加速度 (g)
    float velocity[3];                // [x, y, z] 估算的線速度 (m/s)
    float quaternion[4];              // [w, x, y, z] 姿態四元數

public:
    // --- 公開函式 ---
    
    // 構造函式
    SimpleAHRS();

    // 初始化函式，設定更新頻率
    void begin(float sample_freq_hz);

    /**
     * @brief 使用 IMU 數據更新姿態和線性加速度估計
     * @param gx 陀螺儀 X 軸數據 (單位: 度/秒 dps)
     * @param gy 陀螺儀 Y 軸數據 (單位: 度/秒 dps)
     * @param gz 陀螺儀 Z 軸數據 (單位: 度/秒 dps)
     * @param ax 加速度計 X 軸數據 (單位: g)
     * @param ay 加速度計 Y 軸數據 (單位: g)
     * @param az 加速度計 Z 軸數據 (單位: g)
     */
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    void resetVelocity(); // <<< 新增：一個可以重置速度估計的函式

    private:
    // --- 私有成員 ---
    Mahony _filter; // 內嵌一個 Mahony 濾波器物件
    float _dt; // <<< 新增：儲存每次更新的時間間隔 (秒)

    // --- 私有函式 ---

    // 根據最新的四元數計算線性加速度
    void calculateLinearAcceleration(float ax, float ay, float az);
    void estimateVelocity(); // <<< 新增：估算速度的函式
};

#endif // AHRS_H