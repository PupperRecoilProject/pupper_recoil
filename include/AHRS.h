#ifndef AHRS_H
#define AHRS_H

#include <MahonyAHRS.h>

class SimpleAHRS {
public:
    // --- 公開成員變數 (Public Member Variables) ---
    // 這些變數由 `update()` 函式在每個控制週期更新，
    // 代表了機器人基於身體座標系的姿態和運動狀態。

    // 姿態角，單位為度 (degrees)。
    // 遵循 ZYX 歐拉角順序。
    //   - roll: 繞身體 X 軸 (左/右) 的旋轉。
    //   - pitch: 繞身體 Y 軸 (前/後) 的旋轉。
    //   - yaw: 繞身體 Z 軸 (上/下) 的旋轉。
    float roll, pitch, yaw;
    
    // 從加速度計讀數中移除了重力分量後的身體線性加速度。
    // 單位為 g (標準重力加速度)。
    float linearAccel[3];             // [x, y, z]

    // 對線性加速度進行積分後估算出的身體線速度。
    // 單位為 m/s。
    float velocity[3];                // [x, y, z]

    // 描述姿態的四元數，無單位。
    float quaternion[4];              // [w, x, y, z]

    // [新增] 重力向量在當前身體座標系中的投影。
    // 例如，水平站立時，約為 [0, 0, -1]。
    // 單位為 g (標準重力加速度)。
    float gravityVector[3];           // [x, y, z]

public:
    // --- 公開函式 (Public Functions) ---
    
    // 構造函式
    SimpleAHRS();

    // 初始化函式，設定濾波器的更新頻率
    void begin(float sample_freq_hz);

    /**
     * @brief 使用 IMU 的原始數據更新姿態和運動估計。
     * @param gx 陀螺儀 X 軸數據 (單位: 度/秒 dps)
     * @param gy 陀螺儀 Y 軸數據 (單位: 度/秒 dps)
     * @param gz 陀螺儀 Z 軸數據 (單位: 度/秒 dps)
     * @param ax 加速度計 X 軸數據 (單位: g)
     * @param ay 加速度計 Y 軸數據 (單位: g)
     * @param az 加速度計 Z 軸數據 (單位: g)
     */
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    /**
     * @brief 重置內部估算的速度。
     *        在機器人被拿起或重新放置時呼叫，以避免速度積分漂移。
     */
    void resetVelocity();

private:
    // --- 私有成員 (Private Members) ---
    Mahony _filter; // 內嵌一個 Mahony 濾波器物件
    float _dt;      // 每次更新的時間間隔 (秒)

    // --- 私有函式 (Private Functions) ---
    void calculateLinearAcceleration(float ax, float ay, float az);
    void estimateVelocity();
};

#endif // AHRS_H