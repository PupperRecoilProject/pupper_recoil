// CommandHandler.h
#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include <vector>

// 前向宣告，避免互相引用標頭檔造成的編譯問題
class RobotController;
class TelemetrySystem;

class CommandHandler {
public:
    // 建構式
    CommandHandler();

    // 初始化函式，傳入核心物件的指標
    void begin(RobotController* robot, TelemetrySystem* telemetry);

    // 執行指令的公開介面
    void executeCommand(String command);

private:
    void handleSetCommand(const std::vector<String>& args);
    void handleGetCommand(const std::vector<String>& args);
    void handleResetCommand(const std::vector<String>& args);
    void printParams(int motor_id);    

    // 第三階段新增
    void handleMoveCommand(const std::vector<String>& args);
    void handleConfigCommand(const std::vector<String>& args); // 用於處理 mode, freq, focus
    void handleSystemCommand(const std::vector<String>& args); // 用於處理 cal, stop, reboot, status
    void handleTestCommand(const std::vector<String>& args); // 用於處理 raw, test wiggle

    // 指向核心控制物件的指標
    RobotController* _robot;
    TelemetrySystem* _telemetry;
};

#endif // COMMAND_HANDLER_H