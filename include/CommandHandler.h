// CommandHandler.h
#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include <vector>

// 前向宣告，避免互相引用標頭檔造成的編譯問題
class RobotController;
class TelemetrySystem;
struct CascadeParams; // 前向宣告 CascadeParams 結構體


/**
 * @brief 指令處理器，負責解析用戶輸入的文字指令並執行對應操作。
 *
 * 這個類別是使用者與機器人互動的主要介面。它將字串指令
 * (例如 "move m3 1.57") 轉換為對 RobotController 或
 * TelemetrySystem 的具體函式呼叫。
 */
class CommandHandler {
public:
    /**
     * @brief 預設建構式。
     */
    CommandHandler();

    /**
     * @brief 初始化 CommandHandler，注入依賴的核心模組。
     * @param robot 指向 RobotController 物件的指標。
     * @param telemetry 指向 TelemetrySystem 物件的指標。
     */
    void begin(RobotController* robot, TelemetrySystem* telemetry);

    /**
     * @brief 執行一條指令。這是該類別的主要入口點。
     * @param command 從序列埠讀取的完整指令字串。
     */
    void executeCommand(String command);

private:
    // 內部處理函式，根據指令動詞進行分派
    void handleMoveCommand(const std::vector<String>& args);
    void handleSetCommand(const std::vector<String>& args);
    void handleGetCommand(const std::vector<String>& args);
    void handleResetCommand(const std::vector<String>& args);
    void handleSystemCommand(const std::vector<String>& args);

    // 輔助打印函式
    void printParams(int motor_id, const CascadeParams& params);
    void printParamSources(int motor_id, bool single_line = false);

    // --- 核心模組指標 ---
    RobotController* _robot;     // 指向機器人主控制器
    TelemetrySystem* _telemetry; // 指向遙測系統
};

#endif // COMMAND_HANDLER_H