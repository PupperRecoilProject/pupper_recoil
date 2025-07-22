#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <MotorController.h>
#include <array> // 引入 C++ 標準陣列容器
#include <cstdint> // 為了使用 uint8_t
#include <vector>
#include <string>

const int NUM_ROBOT_MOTORS = 12;
const int CONTROL_FREQUENCY_HZ_H = 1000;
extern const std::array<float, NUM_ROBOT_MOTORS> manual_calibration_pose_rad;

struct CascadeDebugInfo {
    float target_pos_rad;
    float current_pos_rad;
    float pos_error_rad;
    float target_vel_rad_s;
    float current_vel_rad_s;
    float vel_error_rad_s;
    int16_t target_current_mA;
};

// <<< ADDED: 新增一個結構體來封裝級聯控制參數，方便傳遞
struct CascadeParams {
    float c;
    float vel_kp;
    float vel_ki;
    float max_target_velocity_rad_s;
    float integral_max_error_rad;
};

// <<< ADDED: for get source 指令 >>>
struct ParamSourceInfo {
    std::string c_source;
    std::string vel_kp_source;
    std::string vel_ki_source;
    std::string max_vel_source;
    std::string max_err_source;
};

class RobotController {
public:
    // --- 公開介面 (Public Interface) ---

    // 機器人控制模式的枚舉 (enum)
    enum class ControlMode {
        IDLE,
        POSITION_CONTROL, // <<< DELETED: 不再使用
        CASCADE_CONTROL,
        WIGGLE_TEST,
        CURRENT_MANUAL_CONTROL,
        JOINT_ARRAY_CONTROL, // 說明：此模式也未被使用，但因其可能為未來預留，暫時保留。
        ERROR
    };
    // 用於分組控制的關節類型枚舉
    enum class JointGroup {
        HIP,
        UPPER,
        LOWER,
        LEG0,
        LEG1,
        LEG2,
        LEG3,
        LEG_FRONT,
        LEG_REAR
    };

    enum class ParamScope {
        GLOBAL,
        GROUP,
        MOTOR
    };

    enum ParamMask : uint8_t {
        MASK_NONE    = 0,
        MASK_C       = 1 << 0, // 1
        MASK_VEL_KP  = 1 << 1, // 2
        MASK_VEL_KI  = 1 << 2, // 4
        MASK_MAX_VEL = 1 << 3, // 8
        MASK_MAX_ERR = 1 << 4, // 16
        MASK_ALL     = 0xFF
    };

    // 構造函式
    RobotController(MotorController* motor_ctrl);

    // --- 核心生命週期函式 ---
    void begin();
    void update();

    // --- 高階指令函式 (由 main 呼叫) ---
    void startWiggleTest(int motorID);
    void setTargetPositionPID(int motorID, float angle_rad);
    void setSingleMotorCurrent(int motorID, int16_t current);
    void setIdle();
    void performManualCalibration();
    void setAllJointPositions_rad(const float* target_angles);
    void setJointGroupPosition_rad(JointGroup group, float angle_rad);

    // --- 基於串級控制的指令函式 ---
    void setRobotPoseCascade(const std::array<float, NUM_ROBOT_MOTORS>& pose_rad);

    void setTargetPositionCascade(int motorID, float angle_rad);
    void setJointGroupPositionCascade(JointGroup group, float angle_rad);
    void setAllJointsCascade(const std::array<float, NUM_ROBOT_MOTORS>& pose_rad);
    // ---新增關節組(整隻腿)---
    void setLegJointsCascade(int leg_id, float hip_rad, float upper_rad, float lower_rad);
    void setLegPairCascade(JointGroup group, float hip_rad, float upper_rad, float lower_rad);

    // 獲取最終生效的參數給 updateCascadeControl 和 get_params指令使用的
    CascadeParams getEffectiveParams(int motorID) const;

    void setParamOverride(ParamScope scope, int target_id, const std::string& param_name, float value);
    void resetParamOverride(ParamScope scope, int target_id, const std::string& param_name);

    // --- 狀態與數據獲取函式 ---
    const char* getModeString();
    bool isCalibrated();
    float getMotorPosition_rad(int motorID);
    float getMotorVelocity_rad(int motorID);
    float getTargetPosition_rad(int motorID) const;
    int16_t getTargetCurrent_mA(int motorID);
    CascadeDebugInfo getCascadeDebugInfo(int motorID);

    ParamSourceInfo getParamSourceInfo(int motorID) const;
    std::vector<int> getMotorIdsForGroup(const std::string& group_name_short);

private:
    // --- 私有函式 (Private Methods) ---
    void updatePositionControl();
    void updateWiggleTest();
    void updateCascadeControl();
    void sendCurrents(int16_t currents[NUM_ROBOT_MOTORS], bool is_ideal);
    void setAllMotorsIdle();

    // --- 成員變數 (Member Variables) ---
    MotorController* motors;
    ControlMode mode;
    std::array<float, NUM_ROBOT_MOTORS> direction_multipliers;
    std::array<bool, NUM_ROBOT_MOTORS> is_joint_calibrated;

    // --- 位置控制模式參數 (高增益 PD + 啟動補償) ---
    std::array<float, NUM_ROBOT_MOTORS> target_positions_rad;
    std::array<float, NUM_ROBOT_MOTORS> integral_error_rad_s;
    const float POS_CONTROL_KP = 3000.0f;
    const float POS_CONTROL_KD = 150.0f;
    const float POS_CONTROL_KI = 3000.0f;
    const int16_t FRICTION_STATIC_COMP_mA = 350;
    const int16_t FRICTION_KINETIC_COMP_mA = 0;
    const float FRICTION_VEL_THRESHOLD_RAD_S = 0.05f;
    const float FRICTION_ERR_THRESHOLD_RAD = 0.02f;
    const int16_t POS_CONTROL_MAX_CURRENT = 3000;
    const float POS_CONTROL_MAX_ERROR_RAD = 2.5f;
    const float POS_CONTROL_MAX_VELOCITY_RAD_S = 15.0f;
    const float INTEGRAL_MAX_CURRENT_mA = 2000.0f;

    // --- 擺動測試參數 ---
    int wiggle_motor_id;
    float wiggle_center_pos_rad;
    float wiggle_amplitude_rad;
    float wiggle_frequency_hz;
    float wiggle_kp;

    // --- 手動控制參數 ---
    std::array<int16_t, NUM_ROBOT_MOTORS> manual_current_commands;
    const int16_t MANUAL_MAX_CURRENT = 1000;
    std::array<int16_t, NUM_ROBOT_MOTORS> _target_currents_mA;

    // --- 新的參數數據結構---
    static const CascadeParams SYSTEM_DEFAULT_PARAMS;
    CascadeParams _global_params;
    uint8_t _global_override_mask = MASK_NONE;
    struct ParamOverrides {
        CascadeParams values;
        uint8_t override_mask = MASK_NONE;
    };
    std::array<ParamOverrides, NUM_ROBOT_MOTORS> _motor_overrides;


    // 級聯控制狀態變數
    std::array<float, NUM_ROBOT_MOTORS> integral_error_vel;
    std::array<float, NUM_ROBOT_MOTORS> _target_vel_rad_s;
    std::array<float, NUM_ROBOT_MOTORS> _vel_error_rad_s;
};

#endif