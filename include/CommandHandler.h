// CommandHandler.h
#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>

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
    // 指向核心控制物件的指標
    RobotController* _robot;
    TelemetrySystem* _telemetry;

    // 用於控制是否打印額外數據的旗標
    // 這個旗標從 main 移到這裡，因為它與指令處理更相關
    bool g_enable_extra_prints = false; 
};

#endif // COMMAND_HANDLER_H