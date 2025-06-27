// include/RobotController.h (建議放在 include 資料夾)

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <MotorController.h>

const int NUM_ROBOT_MOTORS = 12;

enum class ControlMode {
    IDLE,
    HOMING,
    POSITION_CONTROL,
    WIGGLE_TEST,
    MANUAL_CONTROL,
    ERROR
};

class RobotController {
public:
    RobotController(MotorController* motor_ctrl);

    void begin();
    void update(); // 在主 loop 中被呼叫

    void startHoming(); // 觸發自動歸零
    bool isHomed();     // 檢查是否已完成歸零
    void setIdle(); // 新增: 讓外部可以呼叫切換到 IDLE 模式
    void setSingleMotorCurrent(int motorID, int16_t current); // 新增: 手動控制模式
    const char* getModeString(); // 新增: 方便打印狀態
    float getMotorPosition_rad(int motorID); // 新增: 獲取校準後的弧度位置
    float getMotorVelocity_rad(int motorID); // 新增: 獲取馬達速度

    void startWiggleTest(int motorID); // 宣告公開的觸發函式
    float homing_current_mA; // 歸零電流
    const int16_t MAX_COMMAND_CURRENT_mA = 1000; // 馬達電流限制 (毫安)
    
private:
    void updateHoming();
    void updateWiggleTest(); // 宣告私有的更新函式
    void setAllMotorsIdle();


    MotorController* motors;
    ControlMode mode;
    
    // --- 機器人參數 ---
    std::array<float, NUM_ROBOT_MOTORS> direction_multipliers; 
    int16_t manual_current_commands[NUM_ROBOT_MOTORS];

    // --- 歸零參數 ---
    std::array<float, NUM_ROBOT_MOTORS> homing_directions;
    std::array<float, NUM_ROBOT_MOTORS> homed_positions_rad;
    std::array<bool, NUM_ROBOT_MOTORS> homed_axes;
    std::array<bool, NUM_ROBOT_MOTORS> homing_axes;
    float homing_velocity_rad_s;
    float homing_current_threshold_mA;
    int homing_phase; 

    // --- 擺動測試參數 ---
    int wiggle_motor_id;
    float wiggle_center_pos_rad;
    float wiggle_amplitude_rad;
    float wiggle_frequency_hz;
    float wiggle_kp;

    // 位置控制模式
    void updatePositionControl();
    float target_positions_rad[NUM_ROBOT_MOTORS];
    float pos_kp, pos_kd; // P 和 D 控制器增益

};

#endif