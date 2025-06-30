// include/RobotController.h (放在 include 資料夾)

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <MotorController.h>
#include <array> // 引入 C++ 標準陣列容器

const int NUM_ROBOT_MOTORS = 12;

class RobotController {
public:
    // --- 公開介面 (Public Interface) ---

    // 機器人控制模式的枚舉 (enum)
    enum class ControlMode {
        IDLE,             // 待機模式
        HOMING,           // 自動歸零模式
        POSITION_CONTROL, // 位置控制模式
        WIGGLE_TEST,      // 擺動測試模式
        MANUAL_CONTROL,   // 手動電流控制模式
        ERROR             // 錯誤模式
    };

    // 構造函式
    RobotController(MotorController* motor_ctrl);

    // --- 核心生命週期函式 ---
    void begin();
    void update(); // 在高頻迴圈中被呼叫

    // --- 高階指令函式 (由 main 呼叫) ---
    void startHoming();
    void startWiggleTest(int motorID);
    void setTargetPosition_rad(int motorID, float angle_rad); // 新增的位置控制指令
    void setSingleMotorCurrent(int motorID, int16_t current); // 手動覆寫電流
    void setIdle(); // 強制機器人進入安全的待機模式

    // --- 狀態與數據獲取函式 ---
    const char* getModeString();
    bool isHomed();
    float getMotorPosition_rad(int motorID);
    float getMotorVelocity_rad(int motorID);
    
private:
    // --- 私有函式 (Private Methods) ---

    // 狀態機內部使用的更新函式
    void updateHoming();
    void updatePositionControl();
    void updateWiggleTest();
    
    // 輔助函式
    void setAllMotorsIdle();
    void sendAllCurrentsCommand(int16_t currents[NUM_ROBOT_MOTORS]);

    // --- 成員變數 (Member Variables) ---

    // 核心組件
    MotorController* motors;
    ControlMode mode;
    
    // 機器人定義相關參數
    std::array<float, NUM_ROBOT_MOTORS> direction_multipliers; // 馬達方向係數

    // --- 歸零模式參數 ---
    int homing_phase;
    std::array<bool, NUM_ROBOT_MOTORS> homed_axes;      // 各軸是否已完成歸零
    std::array<bool, NUM_ROBOT_MOTORS> homing_axes;     // 各軸是否正在歸零
    std::array<float, NUM_ROBOT_MOTORS> homing_directions; // 歸零時的轉動方向
    std::array<float, NUM_ROBOT_MOTORS> homed_positions_rad; // 歸零目標的機械角度 (弧度)
    float homing_current_mA;
    float homing_current_threshold_mA;

    // --- 位置控制模式參數 ---
    std::array<float, NUM_ROBOT_MOTORS> target_positions_rad; // 目標位置 (弧度)
    // 位置控制的安全與調校參數 (PD控制器 + 前饋補償)
    const float POS_CONTROL_KP = 40.0f;                      // P 控制器增益 (可以適當調高)
    const float POS_CONTROL_KD = 0.8f;                       // D 控制器增益 (用於穩定)
    const int16_t FF_CURRENT_mA = 350;                       // 前饋電流 (mA), 用於克服靜摩擦力
    const int16_t POS_CONTROL_MAX_CURRENT = 2000;            // 此模式下的最大電流 (mA)
    const float POS_CONTROL_MAX_ERROR_RAD = 1.5f;            // 弧度, 位置誤差超過此值則觸發安全停機
    const float POS_CONTROL_MAX_VELOCITY_RAD_S = 10.0f;      // rad/s, 速度超過此值則觸發安全停機
    
    // --- 擺動測試參數 ---
    int wiggle_motor_id;
    float wiggle_center_pos_rad;
    float wiggle_amplitude_rad;
    float wiggle_frequency_hz;
    float wiggle_kp;

    // --- 手動控制參數 ---
    std::array<int16_t, NUM_ROBOT_MOTORS> manual_current_commands;
    const int16_t MANUAL_MAX_CURRENT = 1000; // 手動模式最大電流 (mA)
};

#endif