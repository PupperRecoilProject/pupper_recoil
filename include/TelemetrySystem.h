// include/TelemetrySystem.h

#ifndef TELEMETRY_SYSTEM_H
#define TELEMETRY_SYSTEM_H

#include <array>
#include "RobotController.h" // 需要知道 RobotController 的定義，以便獲取其狀態和詳細調試信息
#include "AHRS.h"            // 需要知道 SimpleAHRS 的定義，以便獲取姿態數據
#include "MotorController.h" // 需要知道 MotorController 的定義，以便獲取馬達的實際電流
#include "LSM6DSO_SPI.h"  // 包含 IMU 的頭文件


// 這是一個遙測與監控系統的模組化類別。
// 它的職責是從機器人的各個部分收集數據，並根據所選模式以不同的格式將其打印出來。
class TelemetrySystem {
public:
    // 定義了可用的輸出模式（格式）
    enum class PrintMode {
        HUMAN_STATUS,
        CSV_LOG,
        DASHBOARD
    };

    /**
     * @brief 建構式
     * @param robot 指向 RobotController 物件的指標。
     * @param ahrs 指向 SimpleAHRS 物件的指標。
     * @param motors 指向 MotorController 物件的指標。
     */
    TelemetrySystem(RobotController* robot, SimpleAHRS* ahrs, MotorController* motors, LSM6DSO* imu);

    // --- 核心生命週期函式 ---
    void begin();
    void updateAndPrint();

    // --- 公開控制接口 ---
    void setPrintMode(PrintMode mode);
    void setFocusMotor(int motor_id);
    const char* getModeString();

    /**
     * @brief 暫停遙測數據的打印。
     */
    void pause();

    /**
     * @brief 恢復遙測數據的打印。
     */
    void resume();

    /**
     * @brief 設定要在遙測介面中顯示的最後一條指令。
     * @param cmd 使用者輸入的完整指令字串。
     */
    void setLastCommand(const String& cmd);

private:
    // 內部使用的統一遙測數據結構體。
    // 在打印前，所有需要的數據都會被先收集到這個結構體中。
    struct RobotTelemetry {
        unsigned long timestamp_ms; // 時間戳 (毫秒)
        const char* robot_mode;     // 機器人當前的控制模式
        bool is_calibrated;         // 機器人是否已校準
        float roll, pitch, yaw;     // AHRS 姿態數據 (度)
        // IMU 加速度數據欄位
        std::array<float, 3> imu_acc_g; // IMU 加速度 (g)
        // 所有馬達的數據陣列
        std::array<float, NUM_ROBOT_MOTORS> motor_positions_rad;
        std::array<float, NUM_ROBOT_MOTORS> motor_velocities_rad_s;
        std::array<int16_t, NUM_ROBOT_MOTORS> target_currents_mA;
        std::array<int16_t, NUM_ROBOT_MOTORS> actual_currents_mA;
        std::array<float, 3> ahrs_linear_accel_g;   // linearAccel 是從加速度計讀數中移除重力分量後得到的身體線性加速度(g)
        std::array<float, 3> ahrs_velocity_ms;      // velocity 是對線性加速度進行積分後估算出的身體線速度(m/s)
        std::array<CascadeDebugInfo, NUM_ROBOT_MOTORS> cascade_debug_infos; // 這個結構體陣列儲存了每個馬達在控制迴圈中的詳細內部狀態，包括位置誤差、目標速度、速度誤差等，是深度除錯的關鍵。
    };

    // --- 私有輔助函式 ---
    void collectData();
    void printAsHumanStatus();
    void printAsCsvLog();
    void printAsDashboard();

    // --- 指向外部依賴物件的指標 ---
    RobotController* _robot;
    SimpleAHRS* _ahrs;
    MotorController* _motors;
    LSM6DSO* _imu; // 增加 _imu 指標成員
    
    // --- 內部狀態變數 ---
    PrintMode _current_mode;      // 當前的輸出模式
    int _focus_motor_id;          // 焦點馬達的ID，-1 代表無焦點
    RobotTelemetry _telemetry_data; // 儲存本輪遙測數據的實例
    bool _csv_header_printed;     // 確保 CSV 標題只打印一次的旗標

    // --- 用於增強使用者體驗的狀態變數 ---
    bool _is_paused;              // 遙測是否已暫停
    String _last_command;         // 儲存從 CommandHandler 傳來的最後一條指令
};

#endif // TELEMETRY_SYSTEM_H