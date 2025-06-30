// lib/Motor_Controller/MotorController.cpp

#include "MotorController.h"

MotorController::MotorController() {
    // 構造函式目前為空，因為 C610Bus 物件的初始化是自動的
    // 初始化馬達偏移量為 0
    for(int i = 0; i < TOTAL_MOTORS; i++) {
        _offsets_rad[i] = 0.0f;
    }
}

void MotorController::begin() {
    // 雖然 C610Bus 會自動初始化，但保留這個 begin 函式
    // 是良好的設計習慣，未來可以在這裡加入額外的初始化代碼。
    // 例如：檢查 CAN Bus 狀態。
    // bus1 和 bus2 的構造函式會自動呼叫 InitializeCAN()
}

void MotorController::pollAll() {
    bus1.PollCAN();
    bus2.PollCAN();
}


// 獲取校準後的位置 (弧度)
float MotorController::getPosition_rad(int motorID) {
    if (motorID >= 0 && motorID < TOTAL_MOTORS) {
        // 原始弧度 - 偏移量弧度
        return getRawPosition_rad(motorID) - _offsets_rad[motorID];
    }
    return 0.0f;
}

float MotorController::getVelocity_rad(int motorID) {
    if (motorID >= 0 && motorID < NUM_MOTORS_PER_BUS) {
        return bus1.Get(motorID).Velocity();
    } else if (motorID >= NUM_MOTORS_PER_BUS && motorID < TOTAL_MOTORS) {
        return bus2.Get(motorID - NUM_MOTORS_PER_BUS).Velocity();
    }
    return 0.0f;
}

// 獲取原始位置 (弧度)
float MotorController::getRawPosition_rad(int motorID) {
    if (motorID >= 0 && motorID < NUM_MOTORS_PER_BUS) {
        return bus1.Get(motorID).Position(); // 返回的是弧度
    } else if (motorID >= NUM_MOTORS_PER_BUS && motorID < TOTAL_MOTORS) {
        return bus2.Get(motorID - NUM_MOTORS_PER_BUS).Position(); // 返回的是弧度
    }
    return 0.0f;
}

//@brief 獲取未經校準的原始馬達速度
float MotorController::getRawVelocity_rad(int motorID) {
    if (motorID >= 0 && motorID < NUM_MOTORS_PER_BUS) {
        return bus1.Get(motorID).Velocity();
    } else if (motorID >= NUM_MOTORS_PER_BUS && motorID < TOTAL_MOTORS) {
        return bus2.Get(motorID - NUM_MOTORS_PER_BUS).Velocity();
    }
    return 0.0f;
}

//@brief 獲取馬達回傳的實際電流 (單位：mA)
int16_t MotorController::getRawCurrent(int motorID) {
    if (motorID >= 0 && motorID < NUM_MOTORS_PER_BUS) {
        return bus1.Get(motorID).Current();
    } else if (motorID >= NUM_MOTORS_PER_BUS && motorID < TOTAL_MOTORS) {
        return bus2.Get(motorID - NUM_MOTORS_PER_BUS).Current();
    }
    return 0;
}


// 這個函式才是與 C610Bus::CommandTorques 對應的關鍵
bool MotorController::setAllCurrents(int16_t currents[TOTAL_MOTORS]) {
    // 1. 進行安全檢查
    for (int i = 0; i < TOTAL_MOTORS; i++) {
        if (abs(currents[i]) > ABSOLUTE_MAX_CURRENT_mA) {
            // 發現超限，打印詳細錯誤，觸發停機，並報告失敗
            Serial.printf("\n\n!!! CRITICAL MOTOR CONTROLLER FAILURE !!!\n");
            Serial.printf("Motor %d current command %d mA exceeds ABSOLUTE LIMIT %d mA.\n", i, currents[i], ABSOLUTE_MAX_CURRENT_mA);
            Serial.printf("ALL MOTORS HALTED BY HARDWARE ABSTRACTION LAYER.\n\n");
            
            // 觸發緊急停機：向所有馬達發送零電流指令
            int16_t zero_currents[TOTAL_MOTORS] = {0};
            // 這裡直接呼叫底層的 C610Bus 命令來確保指令發出
            bus1.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
            bus1.CommandTorques(0, 0, 0, 0, C610Subbus::kIDFourToSeven);
            bus2.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
            bus2.CommandTorques(0, 0, 0, 0, C610Subbus::kIDFourToSeven);

            // 通知上層控制器發生了錯誤
            return false; 
        }
    }

    // 2. 如果所有檢查都通過，才發送正常指令
    bus1.CommandTorques(currents[0], currents[1], currents[2], currents[3], C610Subbus::kIDZeroToThree);
    bus1.CommandTorques(currents[4], currents[5], 0, 0, C610Subbus::kIDFourToSeven);
    bus2.CommandTorques(currents[6], currents[7], currents[8], currents[9], C610Subbus::kIDZeroToThree);
    bus2.CommandTorques(currents[10], currents[11], 0, 0, C610Subbus::kIDFourToSeven);

    // 通知上層控制器指令已成功發送
    return true;
}

// 設定偏移量 (弧度)
void MotorController::setOffset_rad(int motorID, float offset_rad) { // 參數名改為 offset_rad，更清晰
    if (motorID >= 0 && motorID < TOTAL_MOTORS) {
        _offsets_rad[motorID] = offset_rad;
    }
}

// 獲取偏移量 (弧度)
float MotorController::getOffset_rad(int motorID) {
    if (motorID >= 0 && motorID < TOTAL_MOTORS) {
        return _offsets_rad[motorID];
    }
    return 0.0f;
}
