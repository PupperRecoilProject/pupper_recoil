// src/RobotController.cpp

#include "RobotController.h"
#include <Arduino.h>
#include <cstring> // 為了使用 memcpy

// =================================================================
//   構造函式 與 生命週期函式
// =================================================================

RobotController::RobotController(MotorController* motor_ctrl) : motors(motor_ctrl) {
    mode = ControlMode::IDLE;
    direction_multipliers = { 1,  1, -1,
                             -1, -1,  1,
                              1,  1, -1,
                             -1, -1,  1};
    target_positions_rad.fill(0.0f);
    manual_current_commands.fill(0);
}

void RobotController::begin() {
    homing_directions = { 1,  1, -1,
                         -1, -1,  1,
                          1,  1, -1,
                         -1, -1,  1};
    
    float abduction_homed_rad = 0.838;
    float hip_homed_rad = 3.456;
    float knee_homed_rad = 2.923;
    homed_positions_rad = {abduction_homed_rad, hip_homed_rad, knee_homed_rad, 
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad,
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad,
                           abduction_homed_rad, hip_homed_rad, knee_homed_rad};

    homing_current_mA = 400;
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
//   手動校準參數
// =================================================================
// 定義手動校準時，機器人應該處於的姿態。
// 這裡我們假設是一個所有關節角度都為 0 的 "零位姿態"。
// 您可以根據您的機器人設計，將其修改為任何您選擇的、易於擺放的參考姿態。
const std::array<float, NUM_ROBOT_MOTORS> manual_calibration_pose_rad = {
    0.0f, -1.14f, -3.014f,  // Leg 0 (Front Right)
    0.0f, 0.0f, 0.0f,  // Leg 1 (Front Left)
    0.0f, 0.0f, 0.0f,  // Leg 2 (Rear Right)
    0.0f, 0.0f, 0.0f   // Leg 3 (Rear Left)
};

// =================================================================
//   高階指令函式 (由 main 呼叫)
// =================================================================

void RobotController::startHoming() {
    if (mode == ControlMode::HOMING) return;
    Serial.println("啟動自動歸零程序...");
    setIdle(); // 這會重置一些狀態，很好
    
    // **核心修正點**
    mode = ControlMode::HOMING; // 在設定好所有東西後再切換模式
    homing_phase = HomingPhase::KNEES; // 設定初始階段
    
    is_joint_homed.fill(false);
    is_joint_homing_active.fill(false); // 先全部清空
    homing_stall_start_time_ms.fill(0);

    // **【關鍵】必須在這裡就設定好第一階段要活動的關節**
    Serial.println("Homing Phase 1: KNEES");
    int knee_axes[] = {2, 5, 8, 11};
    for(int i : knee_axes) {
        is_joint_homing_active[i] = true; // 激活膝關節
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
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) { /* ... */ return; }
        // 在第一次切換到位置控制模式時，初始化所有目標位置
    if (mode != ControlMode::POSITION_CONTROL) {
        Serial.println("切換至 POSITION_CONTROL 模式。");
        // 先不要 setIdle()，因為它會立即發送零電流
        // 在切換的瞬間，將所有目標位置設為當前位置，以防止跳動
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        mode = ControlMode::POSITION_CONTROL;
    }
    
    // 然後再設定指定的目標
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

/**
 * @brief 在 HOMING 模式下，每個控制迴圈都會呼叫此函式。
 * 
 * 該函式負責執行多階段的自動歸零程序。它會按順序（膝關節 -> 髖關節 -> 外展關節）
 * 為每組關節施加一個小電流，使其撞向機械限位。通過檢測馬達速度持續為零的狀態
 * 來判斷是否已到達限位（堵轉），然後計算並設置位置偏移量，從而校準絕對位置。
 */
void RobotController::updateHoming() {
    // --------------------------------------------------------------------------
    // 1. 初始化：準備本次更新所需的變數
    // --------------------------------------------------------------------------

    // 用於儲存最終要發送給所有馬達的物理電流值，預設為0。
    int16_t physical_currents[NUM_ROBOT_MOTORS] = {0};
    
    // 建立一個 "樂觀" 的標誌位。我們先假設當前階段的所有關節都已經完成了歸零。
    // 在後續的迴圈中，只要發現任何一個關節還在運動，我們就將此標誌位設為 false。
    bool all_done_in_current_phase = true;

    // --------------------------------------------------------------------------
    // 2. 核心迴圈：遍歷所有馬達，處理正在歸零的馬達
    // --------------------------------------------------------------------------

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        // 檢查這個馬達是否是「當前階段的目標」並且「尚未完成歸零」
        if (is_joint_homing_active[i] && !is_joint_homed[i]) {
            
            // 如果進入此 if，代表我們找到了一個仍在活動的歸零目標。
            // 因此，當前階段肯定尚未完成。立即將標誌位設為 false。
            all_done_in_current_phase = false;
            
            // --- 驅動馬達 ---
            // 根據預設的方向和電流大小，計算要施加的物理電流。
            physical_currents[i] = homing_directions[i] * homing_current_mA;

            // --- 監測與判斷堵轉 ---
            // 獲取馬達當前的原始速度。
            float vel_rad_s = motors->getRawVelocity_rad(i);

            // 判斷速度是否足夠低，以至於可以認為它已經停止了。
            if (abs(vel_rad_s) < 0.05) { // 0.05 rad/s 是一個合理的低速閾值
                
                // 如果速度很低，我們需要確認這種狀態是否持續了一段時間，以避免誤判。
                // 檢查這個馬達的堵轉計時器是否已經啟動。'0' 代表尚未啟動。
                if (homing_stall_start_time_ms[i] == 0) {
                    // 如果計時器未啟動，這是我們第一次檢測到它停止。
                    // 記錄下當前的系統時間（毫秒），啟動計時器。
                    homing_stall_start_time_ms[i] = millis();
                }
                // 如果計時器已啟動，計算從啟動到現在經過了多長時間。
                else if (millis() - homing_stall_start_time_ms[i] > HOMING_STALL_TIME_THRESHOLD_MS) {
                    
                    // =======================================================
                    // ===            歸零成功！執校準程序               ===
                    // =======================================================
                    
                    // 低速狀態已持續超過閾值（例如 100ms），我們確認馬達已堵轉。
                    // 將此馬達標記為「已歸零」，這樣下個迴圈就不會再處理它了。
                    is_joint_homed[i] = true;
                    
                    // 立即停止對該馬達施加電流，避免持續硬頂。
                    physical_currents[i] = 0;
                    
                    // 讀取此刻馬達的原始、未校準的位置。
                    float raw_pos_rad = motors->getRawPosition_rad(i);
                    
                    // 核心校準步驟：計算偏移量。
                    // 偏移量 = (當前的原始位置) - (我們希望這個位置被稱作什麼)
                    float offset_rad_to_set = raw_pos_rad - homed_positions_rad[i];
                    
                    // 將計算出的偏移量設定到底層的馬達控制器中。
                    motors->setOffset_rad(i, offset_rad_to_set);
                    
                    // 打印日誌，告知使用者此馬達已完成校準。
                    Serial.printf("馬達 %d 已歸零 (基於速度檢測)。偏移量設為: %.4f rad\n", i, offset_rad_to_set);
                }

            } else {
                // 如果馬達速度不為零（即仍在轉動），說明它沒有堵轉。
                // 我們需要重置它的堵轉計時器，以防之前的低速是抖動或誤判。
                homing_stall_start_time_ms[i] = 0;
            }
        }
    }

    // --------------------------------------------------------------------------
    // 3. 發送電流指令
    // --------------------------------------------------------------------------
    // 在迴圈結束後，physical_currents 陣列中包含了所有馬達的最終指令。
    // 正在歸零的馬達會有電流，已完成或非目標的馬達電流為0。
    // Homing 直接控制物理電流，因此 is_ideal 參數為 false。
    sendCurrents(physical_currents, false);

    // --------------------------------------------------------------------------
    // 4. 檢查階段是否完成，並切換到下一階段
    // --------------------------------------------------------------------------
    // 在遍歷完所有馬達後，檢查我們在開頭設定的樂觀標誌位。
    // 如果它從頭到尾都沒有被設為 false，就說明本階段的所有目標都已完成歸零。
    if (all_done_in_current_phase && mode == ControlMode::HOMING) {
        
        // 既然舊階段已完成，清空 active 標記，為新階段做準備。
        is_joint_homing_active.fill(false); 

        switch (homing_phase) {
            case HomingPhase::KNEES:
                // 膝關節階段完成，切換到髖關節階段。
                homing_phase = HomingPhase::HIPS;
                Serial.println("Homing Phase 1 (Knees) Done. -> Starting Phase 2: HIPS");
                { // 使用花括號創建局部作用域，避免變數名衝突
                    int hip_axes[] = {1, 4, 7, 10};
                    for (int i : hip_axes) is_joint_homing_active[i] = true; // 激活髖關節
                }
                break;

            case HomingPhase::HIPS:
                // 髖關節階段完成，切換到外展關節階段。
                homing_phase = HomingPhase::ABDUCTIONS;
                Serial.println("Homing Phase 2 (Hips) Done. -> Starting Phase 3: ABDUCTIONS");
                {
                    int abd_axes[] = {0, 3, 6, 9};
                    for (int i : abd_axes) is_joint_homing_active[i] = true; // 激活外展關節
                }
                break;

            case HomingPhase::ABDUCTIONS:
                // 最後的外展關節階段也完成了。
                homing_phase = HomingPhase::DONE;
                Serial.println("Homing Phase 3 (Abductions) Done. All joints are homed!");
                setIdle(); // 所有階段都完成，將機器人切換到安全的 IDLE 模式。
                break;

            case HomingPhase::DONE:
                // 如果程式因為某些原因在 DONE 狀態下再次進入此處，
                // 為保證安全，直接切換到 IDLE 模式。
                setIdle();
                break;
        }
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
//   手動校準實現
// =================================================================
/**
 * @brief 執行一次性的手動校準程序。
 *
 * 此函式假設使用者已經手動將機器人擺放到 `manual_calibration_pose_rad` 所定義的姿態。
 * 它會讀取所有馬達的當前原始位置，計算並設置偏移量，從而完成校準。
 * 為了安全，此操作只能在 IDLE 模式下執行。
 */
void RobotController::performManualCalibration() {
    Serial.println("\n--- 開始手動校準程序 ---");

    if (mode != ControlMode::IDLE) {
        Serial.println("[錯誤] 校準失敗！機器人必須處於 IDLE 模式才能進行校準。");
        Serial.println("請先發送 'stop' 指令，手動擺好姿態後再試一次。");
        return;
    }

    Serial.println("正在讀取原始馬達角度並計算偏移量...");
    Serial.println("(ID | Raw Pos (rad) | Target Pos (rad) | Offset (rad))");
    Serial.println("---------------------------------------------------------");

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        // 1. 讀取馬達的原始、未校準位置
        float raw_pos_rad = motors->getRawPosition_rad(i);

        // 2. 獲取此關節在校準姿態下的目標角度
        float target_pos_rad = manual_calibration_pose_rad[i];

        // 3. 核心計算：偏移量 = 原始讀數 - 期望讀數
        float offset_rad_to_set = raw_pos_rad - target_pos_rad;

        // 4. 將計算出的偏移量設定到底層馬達控制器
        motors->setOffset_rad(i, offset_rad_to_set);

        // 5. 打印日誌，方便除錯和確認
        char buf[100];
        snprintf(buf, sizeof(buf), "Motor %2d | %+14.4f | %+17.4f | %+13.4f",
                 i, raw_pos_rad, target_pos_rad, offset_rad_to_set);
        Serial.println(buf);
    }

    // 6. 將所有關節標記為已校準 (is_homed)
    is_joint_homed.fill(true);

    Serial.println("---------------------------------------------------------");
    Serial.println("[成功] 所有馬達手動校準完成！偏移量已儲存。");
    Serial.println("機器人現在已校準，可以接收 'pos' 或 'wiggle' 指令。\n");
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