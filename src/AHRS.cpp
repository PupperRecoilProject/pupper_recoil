#include "AHRS.h"
#include <Arduino.h> // 為了使用 DEG_TO_RAD

const float G_ACCEL = 9.80665f;

SimpleAHRS::SimpleAHRS() {
    // 初始化所有公開變數為 0 或單位值
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;

    for (int i = 0; i < 3; i++) {
        linearAccel[i] = 0.0f;
        velocity[i] = 0.0f;
        gravityVector[i] = 0.0f;
    }

    // 初始化四元數為單位四元數 (無旋轉)
    quaternion[0] = 1.0f; 
    for (int i = 1; i < 4; i++) {
        quaternion[i] = 0.0f;
    }
    
    _dt = 0.0f;
}

void SimpleAHRS::begin(float sample_freq_hz) {
    _filter.begin(sample_freq_hz);
    if (sample_freq_hz > 0) {
        _dt = 1.0f / sample_freq_hz; 
    }
}

void SimpleAHRS::resetVelocity() {
    for (int i = 0; i < 3; i++) {
        velocity[i] = 0.0f;
    }
}

// [修正] 將 SimpleAHPS::update(...) 的拼寫錯誤修正為 SimpleAHRS::update(...)
void SimpleAHRS::update(float gx, float gy, float gz, float ax, float ay, float az) {
    // =========================================================================
    // === [最終方案] 僅進行單位轉換 ===
    //
    // 結論: MuJoCo中的目標身體座標系 (+X:左, +Y:後, +Z:上) 與
    //       實體IMU的物理方向一致，故無需進行座標軸交換。
    // =========================================================================

    // 1. 單位轉換: Mahony 濾波器嚴格要求陀螺儀單位為 rad/s。
    //    此處的 gx, gy, gz 從外部傳入時的單位是 dps (度/秒)。
    _filter.updateIMU(gx * DEG_TO_RAD, 
                      gy * DEG_TO_RAD, 
                      gz * DEG_TO_RAD, 
                      ax, ay, az);

    // 2. 獲取姿態角: 從濾波器獲取的姿態角 (單位: 度)
    roll  = _filter.getRoll();
    pitch = _filter.getPitch();
    yaw   = _filter.getYaw();
    _filter.getQuaternion(&quaternion[0], &quaternion[1], &quaternion[2], &quaternion[3]);

    // 3. 計算並更新運動學變數
    calculateLinearAcceleration(ax, ay, az);
    estimateVelocity();
}

void SimpleAHRS::calculateLinearAcceleration(float ax, float ay, float az) {
    float q_w = quaternion[0];
    float q_x = quaternion[1];
    float q_y = quaternion[2];
    float q_z = quaternion[3];

    // [重構] 計算重力在當前身體座標系中的投影，並儲存為公開成員變數
    // 這個計算與 base.py 中的 get_gravity 實質上是等價的。
    // 世界重力向量為 [0, 0, -1]，透過姿態四元數的逆旋轉得到其在身體座標系中的表示。
    // 身體 Z 軸在世界中的表示是 (2(q_x*q_z + q_w*q_y), 2(q_y*q_z - q_w*q_x), q_w^2 - q_x^2 - q_y^2 + q_z^2)
    // 我們需要的是世界 Z 軸在身體中的投影，也就是旋轉矩陣的第三行 (c_31, c_32, c_33)
    // 註：此處的重力向量是指向世界座標系 +Z 的向量 [0,0,1] 在身體座標系的投影，
    //     以匹配 MuJoCo 的 up_vector。若要匹配重力 [0,0,-1]，則全體取負號。
    //     get_gravity 函式是 `... @ [0,0,-1]`，所以我們應該取負號。
    gravityVector[0] = -2 * (q_x * q_z + q_w * q_y);
    gravityVector[1] = -2 * (q_y * q_z - q_w * q_x);
    gravityVector[2] = -(q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z);
    
    // 從 IMU 原始加速度讀數中，減去重力分量的影響，得到純粹的線性加速度。
    linearAccel[0] = ax - gravityVector[0];
    linearAccel[1] = ay - gravityVector[1];
    linearAccel[2] = az - gravityVector[2];
}

void SimpleAHRS::estimateVelocity() {
    if (_dt > 0) {
        // 積分: 速度變化量 = 線性加速度 * 時間間隔
        // 注意單位轉換：linearAccel 的單位是 g，需要乘以 G_ACCEL 換算成 m/s^2
        velocity[0] += linearAccel[0] * G_ACCEL * _dt;
        velocity[1] += linearAccel[1] * G_ACCEL * _dt;
        velocity[2] += linearAccel[2] * G_ACCEL * _dt;
    }
}