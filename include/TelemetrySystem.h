// include/TelemetrySystem.h

#ifndef TELEMETRY_SYSTEM_H
#define TELEMETRY_SYSTEM_H

#include <array>
#include <optional>         // 引入 C++17 的 optional，用於處理可選參數
#include "RobotController.h" // 需要知道 RobotController 的定義，以便獲取其狀態和詳細調試信息
#include "AHRS.h"            // 需要知道 SimpleAHRS 的定義，以便獲取姿態數據
#include "MotorController.h" // 需要知道 MotorController 的定義，以便獲取馬達的實際電流

// 這是一個遙測與監控系統的模組化類別。
// 它的職責是從機器人的各個部分收集數據，並根據所選模式以不同的格式將其打印出來。
class TelemetrySystem {
public:
    // 定義了可用的輸出模式（格式）
    enum class PrintMode {
        HUMAN_STATUS,       // 模式1: 打印人類可讀的、格式化的全局狀態報告
        CSV_LOG,            // 模式2: 打印 CSV 格式的日誌，方便導入到外部軟體進行分析
        DASHBOARD           // 模式3: 專注於單一馬達，以儀表板形式顯示其詳細的內部控制狀態
    };

    /**
     * @brief 構造函式
     * @param robot 指向 RobotController 物件的指針
     * @param ahrs 指向 SimpleAHRS 物件的指針
     * @param motors 指向 MotorController 物件的指針
     *
     * 採用"依賴注入"的方式，將所有數據源作為參數傳入，而不是使用全域變數。
     * 這讓模組的依賴關係非常清晰，且更易於測試和重用。
     */
    TelemetrySystem(RobotController* robot, SimpleAHRS* ahrs, MotorController* motors);

    /**
     * @brief 初始化遙測系統
     * 在 setup() 中呼叫，為未來可能的初始化操作預留位置。
     */
    void begin();

    /**
     * @brief 更新並打印數據
     * 這是該模組在主迴圈中唯一的接口。它會自動完成數據收集和根據當前模式打印的全部工作。
     */
    void updateAndPrint();

    /**
     * @brief 設定輸出格式
     * @param mode 要切換到的新模式 (HUMAN_STATUS, CSV_LOG, DASHBOARD)
     */
    void setPrintMode(PrintMode mode);
    
    /**
     * @brief 設定焦點馬達
     * @param motor_id 要監控的馬達ID。傳入 -1 代表取消焦點。
     */
    void setFocusMotor(int motor_id);

    /**
     * @brief 獲取當前模式的字串描述
     * @return 一個描述當前模式的 C-style 字串
     */
    const char* getModeString();

private:
    // 內部使用的統一遙測數據結構體。
    // 在打印前，所有需要的數據都會被先收集到這個結構體中。
    struct RobotTelemetry {
        unsigned long timestamp_ms; // 時間戳 (毫秒)
        const char* robot_mode;     // 機器人當前的控制模式
        bool is_calibrated;         // 機器人是否已校準
        float roll, pitch, yaw;     // AHRS 姿態數據 (度)
        // 所有馬達的數據陣列
        std::array<float, NUM_ROBOT_MOTORS> motor_positions_rad;
        std::array<float, NUM_ROBOT_MOTORS> motor_velocities_rad_s;
        std::array<int16_t, NUM_ROBOT_MOTORS> target_currents_mA;
        std::array<int16_t, NUM_ROBOT_MOTORS> actual_currents_mA;
    };

    // 私有輔助函式

    /**
     * @brief 從所有數據源收集最新的數據，並填充到 _telemetry_data 成員中
     */
    void collectData();

    /**
     * @brief 以人類可讀的格式打印全局狀態
     */
    void printAsHumanStatus();

    /**
     * @brief 以 CSV 格式打印日誌數據 (會根據是否有焦點馬達自動調整行為)
     */
    void printAsCsvLog();

    /**
     * @brief 以儀表板形式打印單個焦點馬達的詳細信息
     */
    void printAsDashboard();

    // 私有成員變數

    // 指向外部依賴物件的指針
    RobotController* _robot;
    SimpleAHRS* _ahrs;
    MotorController* _motors;
    
    PrintMode _current_mode;     // 儲存當前的輸出模式
    int _focus_motor_id;         // 儲存焦點馬達的ID，-1 代表無焦點
    RobotTelemetry _telemetry_data; // 用於儲存本輪遙測數據的實例
    bool _csv_header_printed;    // 一個標記，用於確保 CSV 文件的標題行只打印一次
};

#endif // TELEMETRY_SYSTEM_H