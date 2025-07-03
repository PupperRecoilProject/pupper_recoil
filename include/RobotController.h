// include/RobotController.h (放在 include 資料夾)

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <MotorController.h>
#include <array> // 引入 C++ 標準陣列容器

const int NUM_ROBOT_MOTORS = 12;
const int CONTROL_FREQUENCY_HZ_H = 1000; 

class RobotController {
public:
    // --- 公開介面 (Public Interface) ---

    // 機器人控制模式的枚舉 (enum)
    enum class ControlMode {
        IDLE,             // 待機模式
        POSITION_CONTROL, // 位置控制模式
        WIGGLE_TEST,      // 擺動測試模式
        CURRENT_MANUAL_CONTROL,   // 手動電流控制模式
        ERROR             // 錯誤模式
    };

    // 構造函式
    RobotController(MotorController* motor_ctrl);

    // --- 核心生命週期函式 ---
    void begin();
    void update(); // 在高頻迴圈中被呼叫

    // --- 高階指令函式 (由 main 呼叫) ---
    void startWiggleTest(int motorID);
    void setTargetPosition_rad(int motorID, float angle_rad);
    void setSingleMotorCurrent(int motorID, int16_t current);
    void setIdle();
    void performManualCalibration();     // 手動校準的觸發函式

    // --- 狀態與數據獲取函式 ---
    const char* getModeString();
    bool isCalibrated(); // 這個函式現在的意義變為 "是否已校準"
    float getMotorPosition_rad(int motorID);
    float getMotorVelocity_rad(int motorID);
    
private:
    // --- 私有函式 (Private Methods) ---

    // 狀態機內部使用的更新函式
    void updatePositionControl();
    void updateWiggleTest();

    // 輔助函式
    void sendCurrents(int16_t currents[NUM_ROBOT_MOTORS], bool is_ideal); // 統一的指令出口
    void setAllMotorsIdle();

    // --- 成員變數 (Member Variables) ---

    // 核心組件
    MotorController* motors;
    ControlMode mode;
    
    // 機器人定義相關參數
    std::array<float, NUM_ROBOT_MOTORS> direction_multipliers; // 馬達方向係數

    // --- 歸零模式參數 ---
    std::array<bool, NUM_ROBOT_MOTORS> is_joint_calibrated;


    // --- 位置控制模式參數 (高增益 PD + 啟動補償) ---
    std::array<float, NUM_ROBOT_MOTORS> target_positions_rad;
    std::array<float, NUM_ROBOT_MOTORS> integral_error_rad_s;   // 儲存每個馬達的積分誤差
    const float POS_CONTROL_KP = 1000.0f;                       // 高 P 增益，提供主要驅動力
    const float POS_CONTROL_KD = 20.0f;                         // 高 D 增益 (Kp/50)，提供穩定性
    const float POS_CONTROL_KI = 500.0f;                        // << 新增：I 增益 (初始值，之後要調整)

    // 啟動補償 (Kickstart / Friction Compensation) 參數
    const int16_t FRICTION_STATIC_COMP_mA = 350;            // 啟動時的基礎電流
    const int16_t FRICTION_KINETIC_COMP_mA = 200;           // 動摩擦補償，用於補償馬達在運動中的持續摩擦損耗
    const float FRICTION_VEL_THRESHOLD_RAD_S = 0.05f;       // 判定為 "靜止" 的速度閾值
    const float FRICTION_ERR_THRESHOLD_RAD = 0.02f;         // 需要啟動的最小誤差閾值
    // 安全限制
    const int16_t POS_CONTROL_MAX_CURRENT = 2500;            // 適當提高總電流限制以容納高增益輸出
    const float POS_CONTROL_MAX_ERROR_RAD = 1.5f;
    const float POS_CONTROL_MAX_VELOCITY_RAD_S = 10.0f;
    const float INTEGRAL_MAX_CURRENT_mA = 1000.0f;           // 積分飽和保護 (Integral Windup Protection)

    // --- 擺動測試參數 ---
    int wiggle_motor_id;
    float wiggle_center_pos_rad;
    float wiggle_amplitude_rad;
    float wiggle_frequency_hz;
    float wiggle_kp;

    // --- 手動控制參數 ---
    std::array<int16_t, NUM_ROBOT_MOTORS> manual_current_commands;
    const int16_t MANUAL_MAX_CURRENT = 1000;
};

#endif