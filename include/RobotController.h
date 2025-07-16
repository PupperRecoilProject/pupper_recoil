#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <MotorController.h>
#include <array> // 引入 C++ 標準陣列容器

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

class RobotController {
public:
    // --- 公開介面 (Public Interface) ---

    // 機器人控制模式的枚舉 (enum)
    enum class ControlMode {
        IDLE,             // 待機模式
        POSITION_CONTROL, // 位置控制模式
        CASCADE_CONTROL,  // 新的級聯控制
        WIGGLE_TEST,      // 擺動測試模式
        CURRENT_MANUAL_CONTROL,   // 手動電流控制模式
        JOINT_ARRAY_CONTROL, // 關節陣列控制模式 (用於分組控制)
        ERROR             // 錯誤模式
    };
    // 用於分組控制的關節類型枚舉
    enum class JointGroup {
        HIP,   // 髖關節 (左右擺動)
        UPPER, // 大腿關節 (前後擺動)
        LOWER  // 小腿關節 (膝蓋彎曲)
    };

    // 構造函式
    RobotController(MotorController* motor_ctrl);

    // --- 核心生命週期函式 ---
    void begin();
    void update(); // 在高頻迴圈中被呼叫

    // --- 高階指令函式 (由 main 呼叫) ---
    void startWiggleTest(int motorID);
    void setTargetPositionPID(int motorID, float angle_rad);
    void setSingleMotorCurrent(int motorID, int16_t current);
    void setIdle();
    void performManualCalibration();     // 手動校準的觸發函式
    void setAllJointPositions_rad(const float* target_angles); // 設定所有關節的目標角度 (以弧度為單位)

    // 按關節類型分組設定目標角度
    void setJointGroupPosition_rad(JointGroup group, float angle_rad);

    // --- 狀態與數據獲取函式 ---
    const char* getModeString();
    bool isCalibrated(); // 這個函式現在的意義變為 "是否已校準"
    float getMotorPosition_rad(int motorID);
    float getMotorVelocity_rad(int motorID);
    int16_t getTargetCurrent_mA(int motorID);   // 獲取上一個控制週期計算出的目標電流 (mA)

    // 用於啟動級聯控制模式並設定完整姿態的函式
    void setRobotPoseCascade(const std::array<float, NUM_ROBOT_MOTORS>& pose_rad);
    // --- 基於串級控制的指令函式 ---
    void setTargetPositionCascade(int motorID, float angle_rad);
    void setJointGroupPositionCascade(JointGroup group, float angle_rad);

    CascadeDebugInfo getCascadeDebugInfo(int motorID);

private:
    // --- 私有函式 (Private Methods) ---

    // 狀態機內部使用的更新函式
    void updatePositionControl();
    void updateWiggleTest();
    void updateCascadeControl(); //級聯控制 
    
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
    const float POS_CONTROL_KP = 3000.0f;                       // 高 P 增益，提供主要驅動力 1000
    const float POS_CONTROL_KD = 150.0f;                         // 高 D 增益 (Kp/50)，提供穩定性 30
    const float POS_CONTROL_KI = 3000.0f;                        // I 增益 (初始值，之後要調整) 1500

    // 啟動補償 (Kickstart / Friction Compensation) 參數
    const int16_t FRICTION_STATIC_COMP_mA = 350;            // 啟動時的基礎電流
    const int16_t FRICTION_KINETIC_COMP_mA = 0;           // 動摩擦補償，用於補償馬達在運動中的持續摩擦損耗
    const float FRICTION_VEL_THRESHOLD_RAD_S = 0.05f;       // 判定為 "靜止" 的速度閾值
    const float FRICTION_ERR_THRESHOLD_RAD = 0.02f;         // 需要啟動的最小誤差閾值
    // 安全限制
    const int16_t POS_CONTROL_MAX_CURRENT = 3000;            // 適當提高總電流限制以容納高增益輸出
    const float POS_CONTROL_MAX_ERROR_RAD = 2.5f;
    const float POS_CONTROL_MAX_VELOCITY_RAD_S = 15.0f;
    const float INTEGRAL_MAX_CURRENT_mA = 2000.0f;           // 積分飽和保護 (Integral Windup Protection)

    // --- 擺動測試參數 ---
    int wiggle_motor_id;
    float wiggle_center_pos_rad;
    float wiggle_amplitude_rad;
    float wiggle_frequency_hz;
    float wiggle_kp;

    // --- 手動控制參數 ---
    std::array<int16_t, NUM_ROBOT_MOTORS> manual_current_commands;
    const int16_t MANUAL_MAX_CURRENT = 1000;

    // 將 ideal_currents 移到成員變數區域，以便在 update() 之外也能訪問
    std::array<int16_t, NUM_ROBOT_MOTORS> _target_currents_mA;

    // --- 級聯控制器參數與狀態 ---
    // 外環: 位置 -> 速度
    static constexpr float CASCADE_POS_KP = 16.0f;   //12.0
    // 內環: 速度 -> 電流
    static constexpr float CASCADE_VEL_KP = 500.0f;  //85.0
    static constexpr float CASCADE_VEL_KI = 0.0f;    //450.0
    // 級聯控制安全限制與狀態
    static constexpr float CASCADE_MAX_TARGET_VELOCITY_RAD_S = 8.0f; //8.0
    static constexpr float CASCADE_INTEGRAL_MAX_ERROR_RAD = 0.5f;    //0.5
    std::array<float, NUM_ROBOT_MOTORS> integral_error_vel; // 速度積分項

    std::array<float, NUM_ROBOT_MOTORS> _target_vel_rad_s;
    std::array<float, NUM_ROBOT_MOTORS> _vel_error_rad_s;
};

#endif