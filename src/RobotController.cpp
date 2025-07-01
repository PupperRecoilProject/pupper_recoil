// src/RobotController.cpp

#include "RobotController.h"
#include <Arduino.h>
#include <cstring> // 為了使用 memcpy

// =================================================================
//   構造函式 與 生命週期函式
// =================================================================

RobotController::RobotController(MotorController* motor_ctrl) : motors(motor_ctrl) {
    mode = ControlMode::IDLE;
    direction_multipliers = {-1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1};
    target_positions_rad.fill(0.0f);
    manual_current_commands.fill(0);
}

void RobotController::begin() {
    homing_directions = {-1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1};
    
    float abduction_homed_rad = 0.838;
    float hip_homed_rad = 3.456;
    float knee_homed_rad = 2.923;
    homed_positions_rad = {abduction_homed_rad, hip_homed_rad, knee_homed_rad, 
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad,
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad,
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad};

    homing_current_mA = 350;
    homing_current_threshold_mA = 300;

    wiggle_amplitude_rad = 0.17;
    wiggle_frequency_hz = 0.5;
    wiggle_kp = 20.0;
}

void RobotController::update() {
    switch(mode) {
        case ControlMode::IDLE:
            setAllMotorsIdle();
            break;
        case ControlMode::HOMING:
            updateHoming();
            break;
        case ControlMode::POSITION_CONTROL:
            updatePositionControl();
            break;
        case ControlMode::WIGGLE_TEST:
            updateWiggleTest();
            break;
        case ControlMode::CURRENT_MANUAL_CONTROL:
            // 手動模式被定義為直接控制物理馬達，因此 is_ideal=false
            sendCurrents(manual_current_commands.data(), false);
            break;
        case ControlMode::ERROR:
            // 錯誤模式下，持續發送零電流指令
            setAllMotorsIdle();
            break;
    }
}

// =================================================================
//   高階指令函式 (由 main 呼叫)
// =================================================================

void RobotController::startHoming() {
    if (mode == ControlMode::HOMING) return;
    Serial.println("啟動自動歸零程序...");
    setIdle();
    mode = ControlMode::HOMING;
    homing_phase = 0;
    is_joint_homed.fill(false);
    is_joint_homing_active.fill(false);
    int knee_axes[] = {2, 5, 8, 11};
    for(int i : knee_axes) {
        is_joint_homing_active[i] = true;
    }
}

void RobotController::startWiggleTest(int motorID) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) {
        Serial.println("[錯誤] 無效的馬達ID用於擺動測試。");
        return;
    }
    if (!isHomed()) {
        Serial.println("[警告] 機器人尚未歸零，擺動測試將使用未校準的位置。");
    }
    setIdle();
    mode = ControlMode::WIGGLE_TEST;
    wiggle_motor_id = motorID;
    wiggle_center_pos_rad = getMotorPosition_rad(wiggle_motor_id); 
    Serial.printf("開始為馬達 %d 進行擺動測試，中心位置: %.4f rad。\n", motorID, wiggle_center_pos_rad);
}

void RobotController::setTargetPosition_rad(int motorID, float angle_rad) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) {
        Serial.println("[錯誤] 無效的馬達ID用於位置控制。");
        return;
    }
    if (mode != ControlMode::POSITION_CONTROL) {
        Serial.println("切換至 POSITION_CONTROL 模式。");
        setIdle(); // 在切換前重置所有狀態
        mode = ControlMode::POSITION_CONTROL;
    }
    target_positions_rad[motorID] = angle_rad;
    Serial.printf("  設定馬達 %d 的目標位置為 %.4f rad。\n", motorID, angle_rad);
}

void RobotController::setSingleMotorCurrent(int motorID, int16_t current) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) {
        Serial.println("  [錯誤] 無效的馬達 ID。");
        return;
    }
    if (abs(current) > MANUAL_MAX_CURRENT) {
        Serial.printf("  [錯誤] 手動電流 %d mA 超出限制 %d mA。指令已拒絕。\n", current, MANUAL_MAX_CURRENT);
        return;
    }
    setIdle();
    mode = ControlMode::CURRENT_MANUAL_CONTROL;
    manual_current_commands[motorID] = current;
    Serial.printf("  設定馬達 %d 的電流為 %d mA。\n", motorID, current);
}

void RobotController::setIdle() {
    mode = ControlMode::IDLE;
    manual_current_commands.fill(0);
    for (int i=0; i<NUM_ROBOT_MOTORS; ++i) {
        target_positions_rad[i] = getMotorPosition_rad(i);
    }
    setAllMotorsIdle();
    Serial.println("機器人控制器已設為 IDLE 模式 (所有狀態已重置)。");
}

// =================================================================
//   狀態與數據獲取函式
// =================================================================

const char* RobotController::getModeString() {
    switch (mode) {
        case ControlMode::IDLE:             return "IDLE (待機)";
        case ControlMode::HOMING:           return "HOMING (歸零中)";
        case ControlMode::POSITION_CONTROL: return "POSITION_CONTROL (位置控制)";
        case ControlMode::WIGGLE_TEST:      return "WIGGLE_TEST (擺動測試)";
        case ControlMode::CURRENT_MANUAL_CONTROL:   return "CURRENT_MANUAL_CONTROL (手動控制)";
        case ControlMode::ERROR:            return "ERROR (錯誤)";
        default:                            return "UNKNOWN (未知)";
    }
}

bool RobotController::isHomed() { /* ... 內容不變 ... */ for(bool h : is_joint_homed) if(!h) return false; return true; }
float RobotController::getMotorPosition_rad(int motorID) { /* ... 內容不變 ... */ if(motorID<0||motorID>=NUM_ROBOT_MOTORS)return 0; return motors->getPosition_rad(motorID) * direction_multipliers[motorID]; }
float RobotController::getMotorVelocity_rad(int motorID) { /* ... 內容不變 ... */ if(motorID<0||motorID>=NUM_ROBOT_MOTORS)return 0; return motors->getRawVelocity_rad(motorID) * direction_multipliers[motorID]; }

// =================================================================
//   私有: 內部更新邏輯
// =================================================================

void RobotController::updateHoming() {
    int16_t physical_currents[NUM_ROBOT_MOTORS] = {0};
    bool all_done_in_phase = true;

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        if (is_joint_homing_active[i] && !is_joint_homed[i]) {
            all_done_in_phase = false;
            // Homing 直接定義物理世界的方向和電流
            physical_currents[i] = homing_directions[i] * homing_current_mA;

            float vel_rad_s = motors->getRawVelocity_rad(i);
            int16_t actual_current = motors->getRawCurrent(i);

            if (abs(vel_rad_s) < 0.05 && abs(actual_current) > homing_current_threshold_mA) {
                is_joint_homed[i] = true;
                physical_currents[i] = 0;
                float raw_pos_rad = motors->getRawPosition_rad(i);
                float offset_rad_to_set = raw_pos_rad - homed_positions_rad[i];
                motors->setOffset_rad(i, offset_rad_to_set);
                Serial.printf("馬達 %d 已歸零。偏移量設為: %.4f rad\n", i, offset_rad_to_set);
            }
        }
    }
    
    // Homing 直接控制物理電流，因此 is_ideal=false
    sendCurrents(physical_currents, false);

    if (all_done_in_phase && mode == ControlMode::HOMING) {
        Serial.println("歸零階段完成。");
        setIdle();
    }
}

void RobotController::updatePositionControl() {
    int16_t ideal_currents[NUM_ROBOT_MOTORS] = {0};

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        float current_pos = getMotorPosition_rad(i);
        float current_vel = getMotorVelocity_rad(i);

        if (abs(current_vel) > POS_CONTROL_MAX_VELOCITY_RAD_S) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 速度 %.2f 超出限制 %.2f rad/s。\n", i, current_vel, POS_CONTROL_MAX_VELOCITY_RAD_S);
            setIdle(); return;
        }
        
        float pos_error = target_positions_rad[i] - current_pos;
        
        if (abs(pos_error) > POS_CONTROL_MAX_ERROR_RAD) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 位置誤差 %.2f 超出限制 %.2f rad。\n", i, pos_error, POS_CONTROL_MAX_ERROR_RAD);
            setIdle(); return;
        }

        float feedback_current = (POS_CONTROL_KP * pos_error) - (POS_CONTROL_KD * current_vel);
        float friction_comp_current = 0;

        if (abs(pos_error) > KICKSTART_ERROR_THRESHOLD_RAD && abs(current_vel) < KICKSTART_VELOCITY_THRESHOLD_RAD_S) {
            friction_comp_current = copysignf(KICKSTART_CURRENT_mA, pos_error);
        }

        ideal_currents[i] = (int16_t)(feedback_current + friction_comp_current);
    }

    // 位置控制計算的是理想電流，因此 is_ideal=true
    sendCurrents(ideal_currents, true);
}

void RobotController::updateWiggleTest() {
    int16_t ideal_currents[NUM_ROBOT_MOTORS] = {0};
    
    float current_pos_rad = getMotorPosition_rad(wiggle_motor_id);
    float error_rad = (wiggle_center_pos_rad + wiggle_amplitude_rad * sin(millis() / 1000.0f * 2.0f * PI * wiggle_frequency_hz)) - current_pos_rad;

    if (abs(error_rad) > POS_CONTROL_MAX_ERROR_RAD) {
        Serial.printf("!!! 安全警告 !!! 擺動測試誤差 (%.4f rad) 超出限制。\n", error_rad);
        setIdle(); return;
    }

    int16_t ideal_command_current = (int16_t)(wiggle_kp * error_rad);
    ideal_currents[wiggle_motor_id] = ideal_command_current;

    // Wiggle Test 也是基於 "機器人座標系" 的誤差，因此 is_ideal=true
    sendCurrents(ideal_currents, true);
}

// =================================================================
//   私有: 輔助函式
// =================================================================

void RobotController::setAllMotorsIdle() {
    int16_t zero_currents[NUM_ROBOT_MOTORS] = {0};
    // 零電流無所謂是否理想，發送物理即可
    sendCurrents(zero_currents, false);
}

void RobotController::sendCurrents(int16_t currents[NUM_ROBOT_MOTORS], bool is_ideal) {
    if (mode == ControlMode::ERROR) {
        int16_t zero_currents[NUM_ROBOT_MOTORS] = {0};
        motors->setAllCurrents(zero_currents);
        return;
    }

    int16_t physical_currents[NUM_ROBOT_MOTORS];

    if (is_ideal) {
        // 如果是理想電流，則根據方向係數轉換為物理電流
        for(int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            physical_currents[i] = (int16_t)((float)currents[i] * direction_multipliers[i]);
        }
    } else {
        // 如果是物理電流，直接拷貝
        memcpy(physical_currents, currents, sizeof(physical_currents));
    }
    
    // 在這裡對最終的物理電流進行統一限幅
    for(int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
        physical_currents[i] = constrain(physical_currents[i], -POS_CONTROL_MAX_CURRENT, POS_CONTROL_MAX_CURRENT);
    }

    // 將最終計算好的物理電流發送到硬體抽象層
    if (!motors->setAllCurrents(physical_currents)) {
        mode = ControlMode::ERROR;
        Serial.println("!!! RobotController 因硬體層錯誤而進入 ERROR 狀態 !!!");
    }
}