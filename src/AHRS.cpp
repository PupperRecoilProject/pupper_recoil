#include "AHRS.h"
#include <Arduino.h> // 為了使用 DEG_TO_RAD

SimpleAHRS::SimpleAHRS() {
    // 初始化所有公開變數為 0
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    for (int i = 0; i < 3; i++) {
        linearAccel[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
        quaternion[i] = 0.0f;
    }
    quaternion[0] = 1.0f; // 四元數的實部 w 初始化為 1
}

void SimpleAHRS::begin(float sample_freq_hz) {
    _filter.begin(sample_freq_hz);
}

void SimpleAHRS::update(float gx, float gy, float gz, float ax, float ay, float az) {
    // 1. 更新 Mahony 濾波器
    // 注意：Mahony 需要的陀螺儀單位是 rad/s，所以要轉換
    _filter.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az);

    // 2. 從濾波器獲取最新的姿態角和四元數
    roll  = _filter.getRoll();
    pitch = _filter.getPitch();
    yaw   = _filter.getYaw();
    _filter.getQuaternion(&quaternion[0], &quaternion[1], &quaternion[2], &quaternion[3]);

    // 3. 計算線性加速度
    calculateLinearAcceleration(ax, ay, az);
}

void SimpleAHRS::calculateLinearAcceleration(float ax, float ay, float az) {
    // 獲取四元數的各個分量，方便計算
    float q_w = quaternion[0];
    float q_x = quaternion[1];
    float q_y = quaternion[2];
    float q_z = quaternion[3];

    // 計算重力在感測器坐標系中的分量 (gx, gy, gz)
    // 這是透過將世界坐標系中的標準重力向量 [0, 0, 1] 進行四元數旋轉得到的
    // 使用旋轉矩陣的第三列來計算
    float gravity_x = 2 * (q_x * q_z - q_w * q_y);
    float gravity_y = 2 * (q_y * q_z + q_w * q_x);
    float gravity_z = q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z;

    // 從原始加速度中減去重力分量，得到線性加速度 (單位: g)
    linearAccel[0] = ax - gravity_x;
    linearAccel[1] = ay - gravity_y;
    linearAccel[2] = az - gravity_z;
}