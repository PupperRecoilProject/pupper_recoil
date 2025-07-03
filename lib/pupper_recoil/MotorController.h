// lib/Motor_Controller/MotorController.h

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <C610Bus.h> // 注意這裡需要 C610Subbus 的定義

const int TOTAL_MOTORS = 12;
const int NUM_MOTORS_PER_BUS = 6; // 每個 Bus 負責 6 個馬達

class MotorController {
public:
    // 構造函式
    MotorController();

    // 初始化 CAN 硬體 (現在主要由 C610Bus 的構造函式處理)
    void begin();
    // 輪詢所有 CAN Bus 以更新數據，必須在 loop 中持續呼叫
    void pollAll();

    float getPosition_rad(int motorID);
    float getVelocity_rad(int motorID);
    
    // 新增：獲取未經校準的原始數據
    float getRawPosition_rad(int motorID);
    float getRawVelocity_rad(int motorID);
    int16_t getRawCurrent(int motorID);

    bool setAllCurrents(int16_t currents[TOTAL_MOTORS]); // 修改 setAllCurrents 的返回類型，讓它可以報告狀態。返回 true 表示成功，返回 false 表示發生嚴重錯誤並已停機
    void setOffset_rad(int motorID, float offset);

    // 將 getOffset_rad 移動到這裡
    float getOffset_rad(int motorID); // 新增：讓外部可以讀取偏移量

private:
    C610Bus<CAN1> bus1;
    C610Bus<CAN2> bus2;
    float _offsets_rad[TOTAL_MOTORS];
    const int16_t ABSOLUTE_MAX_CURRENT_mA = 2000; // 在 MotorController 內部定義安全限制 1000ma 逐步調高

};

#endif