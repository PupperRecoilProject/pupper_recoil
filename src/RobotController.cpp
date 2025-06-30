// src/RobotController.cpp

#include "RobotController.h"
#include <Arduino.h>

// =================================================================
//   構造函式 與 生命週期函式
// =================================================================

RobotController::RobotController(MotorController* motor_ctrl) : motors(motor_ctrl) {
    mode = ControlMode::IDLE;
    direction_multipliers = {-1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1};
    target_positions_rad.fill(0.0f); // 初始化目標位置陣列
    manual_current_commands.fill(0); // 初始化手動指令陣列
}

void RobotController::begin() {
    homing_directions = {-1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1};
    
    float abduction_homed_rad = 0.838;
    float hip_homed_rad = 3.456;
    float knee_homed_rad = 2.923;
    // 注意: 這裡只設定了前3個馬達的範例值，實際機器人應定義全部12個
    homed_positions_rad = {abduction_homed_rad, hip_homed_rad, knee_homed_rad, 
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad,
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad,
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad};

    homing_current_mA = 350;
    homing_current_threshold_mA = 300;

    wiggle_amplitude_rad = 0.17; // 約 10 度
    wiggle_frequency_hz = 0.5;
    wiggle_kp = 20.0;
}

void RobotController::update() {
    switch(mode) {
        case ControlMode::IDLE:
            // 待機模式下，確保馬達是關閉的。setIdle()已做過，此處為保險。
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
        case ControlMode::MANUAL_CONTROL:
            // 持續發送手動指令
            sendAllCurrentsCommand(manual_current_commands.data()); 
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
    if (mode == ControlMode::HOMING) return; // 避免重複觸發
    Serial.println("啟動自動歸零程序...");
    setIdle(); // 確保從一個乾淨的狀態開始
    mode = ControlMode::HOMING;
    homing_phase = 0;
    
    homed_axes.fill(false);
    homing_axes.fill(false);

    // 範例: 開始歸零所有膝關節
    int knee_axes[] = {2, 5, 8, 11};
    for(int i : knee_axes) {
        homing_axes[i] = true;
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

    setIdle(); // 先進入一個已知的安全狀態
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
    // 可選: 在這裡增加對 angle_rad 範圍的合理性檢查

    // 如果當前不是位置控制模式，則先切換到安全狀態並初始化目標
    if (mode != ControlMode::POSITION_CONTROL) {
        Serial.println("切換至 POSITION_CONTROL 模式。");
        // *** 修改: 不再直接呼叫 setIdle()，而是在這裡初始化位置目標 ***
        // 這是因為 setIdle 現在會做這件事，我們避免重複。
        for(int i=0; i<NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
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

    setIdle(); // <--- 現在這個呼叫會清理掉舊的位置目標！
    mode = ControlMode::MANUAL_CONTROL;
    // manual_current_commands.fill(0); // setIdle 已經做過了，但多做一次也無妨
    manual_current_commands[motorID] = current;
    Serial.printf("  設定馬達 %d 的電流為 %d mA。\n", motorID, current);
}

void RobotController::setIdle() {
    // 1. 設定模式為 IDLE
    mode = ControlMode::IDLE;
    
    // 2. 清除所有模式的狀態變數，這是防止意外行為的關鍵
    manual_current_commands.fill(0); // 清除手動模式的指令
    
    // 清除位置控制模式的目標，將其設定為當前馬達的實際位置
    // 這樣即使意外切回 POSITION_CONTROL 模式，馬達也不會亂動
    for (int i=0; i<NUM_ROBOT_MOTORS; ++i) {
        target_positions_rad[i] = getMotorPosition_rad(i);
    }

    // 3. 發送零電流指令，讓馬達物理上停止
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
        case ControlMode::MANUAL_CONTROL:   return "MANUAL_CONTROL (手動控制)";
        case ControlMode::ERROR:            return "ERROR (錯誤)";
        default:                            return "UNKNOWN (未知)";
    }
}

bool RobotController::isHomed() {
    for(bool homed : homed_axes) {
        if (!homed) return false;
    }
    return true;
}

float RobotController::getMotorPosition_rad(int motorID) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return 0.0f;
    // 獲取減去偏移後的弧度
    float pos_after_offset_rad = motors->getPosition_rad(motorID);
    // 乘以方向係數得到最終校準後的位置
    return pos_after_offset_rad * direction_multipliers[motorID];
}

float RobotController::getMotorVelocity_rad(int motorID) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return 0.0f;
    // 速度同樣需要考慮方向
    return motors->getRawVelocity_rad(motorID) * direction_multipliers[motorID];
}

// =================================================================
//   私有: 內部更新邏輯
// =================================================================

void RobotController::updateHoming() {
    int16_t command_currents[NUM_ROBOT_MOTORS] = {0};
    bool all_done_in_phase = true;

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        if (homing_axes[i]) {
            all_done_in_phase = false; // 至少有一個馬達還在嘗試歸零
            if (!homed_axes[i]) {
                command_currents[i] = homing_directions[i] * homing_current_mA;

                float vel_rad_s = motors->getRawVelocity_rad(i);
                int16_t actual_current = motors->getRawCurrent(i);

                // 堵轉檢測: 速度接近零且電流超過閾值
                if (abs(vel_rad_s) < 0.05 && abs(actual_current) > homing_current_threshold_mA) {
                    homed_axes[i] = true;
                    command_currents[i] = 0; // 停止這個馬達

                    float raw_pos_rad = motors->getRawPosition_rad(i);
                    float offset_rad_to_set = raw_pos_rad - homed_positions_rad[i];
                    motors->setOffset_rad(i, offset_rad_to_set);
                            
                    Serial.printf("馬達 %d 已歸零。偏移量設為: %.4f rad\n", i, offset_rad_to_set);
                }
            }
        }
    }
    
    sendAllCurrentsCommand(command_currents);

    if (all_done_in_phase) {
        // TODO: 推進到下一個歸零階段 (homing_phase) 的邏輯
        Serial.println("歸零階段完成。所有馬達已歸零。");
        setIdle();
    }
}

// *** 全新修改的 updatePositionControl ***
void RobotController::updatePositionControl() {
    int16_t command_currents[NUM_ROBOT_MOTORS] = {0};

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        float current_pos = getMotorPosition_rad(i);
        float current_vel = getMotorVelocity_rad(i);

        // --- 1. 安全檢查 (與之前相同) ---
        if (abs(current_vel) > POS_CONTROL_MAX_VELOCITY_RAD_S) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 速度 %.2f 超出限制 %.2f rad/s。\n", i, current_vel, POS_CONTROL_MAX_VELOCITY_RAD_S);
            setIdle();
            return;
        }
        
        float pos_error = target_positions_rad[i] - current_pos;
        
        if (abs(pos_error) > POS_CONTROL_MAX_ERROR_RAD) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 位置誤差 %.2f 超出限制 %.2f rad。\n", i, pos_error, POS_CONTROL_MAX_ERROR_RAD);
            setIdle();
            return;
        }

        // --- 2. 計算 PD 控制器的主驅動力 ---
        // 這是控制的核心，現在它的輸出值會大得多。
        float feedback_current = (POS_CONTROL_KP * pos_error) - (POS_CONTROL_KD * current_vel);

        // --- 3. 判斷是否需要 "Kickstart" 啟動補償 ---
        float friction_comp_current = 0; // 預設摩擦力補償為 0

        // 條件一: 馬達需要移動 (誤差大於閾值)
        bool needs_to_move = abs(pos_error) > KICKSTART_ERROR_THRESHOLD_RAD;
        // 條件二: 馬達當前處於或接近靜止
        bool is_stopped = abs(current_vel) < KICKSTART_VELOCITY_THRESHOLD_RAD_S;

        // 如果需要移動且當前是靜止的
        if (needs_to_move && is_stopped) {
            // 給一個基礎的啟動電流，方向與誤差方向一致
            friction_comp_current = copysignf(KICKSTART_CURRENT_mA, pos_error);
        }

        // --- 4. 合併電流 ---
        // 總電流 = PD主驅動力 + (可能的)啟動補償
        int16_t total_current = (int16_t)(feedback_current + friction_comp_current);
        
        // --- 5. 限制總電流大小 ---
        command_currents[i] = constrain(total_current, -POS_CONTROL_MAX_CURRENT, POS_CONTROL_MAX_CURRENT);
    }
    sendAllCurrentsCommand(command_currents);
}



void RobotController::updateWiggleTest() {
    float current_pos_rad = getMotorPosition_rad(wiggle_motor_id);
    // 計算目標位置與當前位置的誤差
    float error_rad = (wiggle_center_pos_rad + wiggle_amplitude_rad * sin(millis() / 1000.0f * 2.0f * PI * wiggle_frequency_hz)) - current_pos_rad;

    if (abs(error_rad) > POS_CONTROL_MAX_ERROR_RAD) { // 重複使用位置控制的安全常數
        Serial.printf("!!! 安全警告 !!! 擺動測試誤差 (%.4f rad) 超出限制。切換至 IDLE 模式。\n", error_rad);
        setIdle();
        return;
    }

    int16_t command_current = (int16_t)(wiggle_kp * error_rad);
    command_current = constrain(command_current, -MANUAL_MAX_CURRENT, MANUAL_MAX_CURRENT);

    int16_t all_currents[NUM_ROBOT_MOTORS] = {0};
    all_currents[wiggle_motor_id] = command_current;
    sendAllCurrentsCommand(all_currents);
}

// =================================================================
//   私有: 輔助函式
// =================================================================



void RobotController::setAllMotorsIdle() {
    int16_t zero_currents[NUM_ROBOT_MOTORS] = {0};
    sendAllCurrentsCommand(zero_currents);
}

void RobotController::sendAllCurrentsCommand(int16_t currents[NUM_ROBOT_MOTORS]) {
    if (mode == ControlMode::ERROR) {
        // 如果已處於錯誤狀態，阻止任何非零電流指令
        int16_t zero_currents[NUM_ROBOT_MOTORS] = {0};
        motors->setAllCurrents(zero_currents);
        return;
    }

    if (!motors->setAllCurrents(currents)) {
        // 如果硬體抽象層回報了嚴重錯誤 (例如電流超出絕對限制)，
        // 本控制器就進入 ERROR 狀態。
        mode = ControlMode::ERROR;
        Serial.println("!!! RobotController 因硬體層錯誤而進入 ERROR 狀態 !!!");
    }
}