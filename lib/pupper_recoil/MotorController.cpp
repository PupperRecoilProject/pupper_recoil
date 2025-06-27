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
void MotorController::setAllCurrents(int16_t currents[TOTAL_MOTORS]) {
    // === 控制 Bus 1 (對應全域馬達 ID 0-5) ===
    // 發送給 Bus 1 的 ID 0, 1, 2, 3 (對應全域馬達 ID 0, 1, 2, 3)
    bus1.CommandTorques(currents[0], currents[1], currents[2], currents[3], C610Subbus::kIDZeroToThree);
    
    // 發送給 Bus 1 的 ID 4, 5 (對應全域馬達 ID 4, 5)
    // 注意：C610Bus::CommandTorques 需要四個參數，不足的用 0 補足
    bus1.CommandTorques(currents[4], currents[5], 0, 0, C610Subbus::kIDFourToSeven);

    // === 控制 Bus 2 (對應全域馬達 ID 6-11) ===
    // 發送給 Bus 2 的 ID 0, 1, 2, 3 (對應全域馬達 ID 6, 7, 8, 9)
    bus2.CommandTorques(currents[6], currents[7], currents[8], currents[9], C610Subbus::kIDZeroToThree);

    // 發送給 Bus 2 的 ID 4, 5 (對應全域馬達 ID 10, 11)
    bus2.CommandTorques(currents[10], currents[11], 0, 0, C610Subbus::kIDFourToSeven);
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
