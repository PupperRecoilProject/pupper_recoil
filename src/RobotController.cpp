// src/RobotController.cpp (與 main.cpp 同級)

#include "RobotController.h"
#include <Arduino.h> // for PI

// 單位轉換輔助函式 (圈 -> 弧度)
float turnsToRad(float turns) {
    return turns * 2.0f * PI;
}
float radToTurns(float rad) {
    return rad / (2.0f * PI);
}


RobotController::RobotController(MotorController* motor_ctrl) : motors(motor_ctrl) {
    // 初始化
    mode = ControlMode::IDLE;

    // 在建構函式中初始化 direction_multipliers (比在 begin() 中更好)
    direction_multipliers = {-1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1};
}

// 馬達的電流控制跟安全狀態切換
void RobotController::sendAllCurrentsCommand(int16_t currents[NUM_ROBOT_MOTORS]) {
    // 如果控制器已經處於錯誤狀態，則不執行任何操作，避免狀態被覆蓋
    if (mode == ControlMode::ERROR) {
        return;
    }

    // 呼叫底層的 setAllCurrents 並檢查其返回值
    if (!motors->setAllCurrents(currents)) {
        // 如果底層報告失敗 (返回 false)，則將本控制器狀態設為 ERROR
        mode = ControlMode::ERROR;
    }
}

void RobotController::begin() {
    // --- 初始化從 DriveSystem 借鑒的參數 ---
    // 這些值需要根據您的機器狗實際情況調整
    homing_directions = {-1, 1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1};
    
    // DriveSystem 中的 homed_positions 是弧度，我們也用弧度
    float abduction_homed_rad = 0.838;
    float hip_homed_rad = 3.456;
    float knee_homed_rad = 2.923;
    homed_positions_rad = {abduction_homed_rad, hip_homed_rad, knee_homed_rad, /* ...以此類推 */};

    homing_velocity_rad_s = 0.1; // 示例值: 0.1 rad/s
    homing_current_threshold_mA = 300; // 示例值: 300mA

    // 初始化擺動參數
    wiggle_amplitude_rad = 0.17; // 擺動幅度，約 +/- 10 度 (0.17 rad)
    wiggle_frequency_hz = 0.5;   // 擺動頻率，每秒半個週期
    wiggle_kp = 20.0;          // P 控制器增益，這個值需要實驗微調
    homing_current_mA = 350; // 歸零時的電流，這個值需要根據實際情況調整
}

void RobotController::startHoming() {
    if (mode == ControlMode::HOMING) return; // 避免重複觸發

    Serial.println("Starting automatic homing sequence...");
    mode = ControlMode::HOMING;
    homing_phase = 0; // 從第一個階段開始
    
    // 重置歸零狀態
    homed_axes.fill(false);
    homing_axes.fill(false);

    // TODO: 根據 homing_phase 啟動第一批要歸零的馬達
    // 例如，先啟動所有膝關節
    int knee_axes[] = {2, 5, 8, 11};
    for(int i : knee_axes) {
        homing_axes[i] = true;
    }
}

void RobotController::update() {
    // 狀態機
    switch(mode) {
        case ControlMode::IDLE:
            setAllMotorsIdle();
            break;
        case ControlMode::HOMING:
            updateHoming();
            break;
        case ControlMode::WIGGLE_TEST:
            updateWiggleTest();
            break;
        case ControlMode::MANUAL_CONTROL:
            // 在手動模式下，持續發送手動指令
            sendAllCurrentsCommand(manual_current_commands);
            break;
        case ControlMode::POSITION_CONTROL:
            updatePositionControl();
            break;
        case ControlMode::ERROR:
            // 在錯誤模式下，持續確保所有馬達電流為零
            setAllMotorsIdle(); 
            // 可以加上 LED 閃爍等視覺警報
            break;

        // 其他模式...
    }
}

// 返回當前模式的字串描述
const char* RobotController::getModeString() {
    switch (mode) {
        case ControlMode::IDLE: return "IDLE";
        case ControlMode::HOMING: return "HOMING";
        case ControlMode::POSITION_CONTROL: return "POSITION_CONTROL";
        case ControlMode::MANUAL_CONTROL: return "MANUAL_CONTROL";
        case ControlMode::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void RobotController::updateHoming() {
    int16_t command_currents[NUM_ROBOT_MOTORS] = {0};

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        if (homing_axes[i] && !homed_axes[i]) {
            // 這個馬達正在歸零，但還沒完成
            
            // 1. 給一個小的恆定電流，讓馬達朝指定方向轉動
            // DriveSystem 的實現更複雜(用PD)，我們先用簡單的恆定電流法
            command_currents[i] = homing_directions[i] * homing_current_mA; // 驅動電流設定

            // 2. 監測速度，如果速度接近 0，則認為堵轉
            float vel_rad_s = motors->getRawVelocity_rad(i);

            int16_t actual_current = motors->getRawCurrent(i);
            if (abs(vel_rad_s) < 0.05 && abs(actual_current) > homing_current_threshold_mA) {
    
                homed_axes[i] = true; // 標記此軸已完成
                homing_axes[i] = false;
                command_currents[i] = 0; // 停止這個馬達

                // 計算並設定零點偏移
                float raw_pos_rad = motors->getRawPosition_rad(i);
                float homed_pos_rad = homed_positions_rad[i];
                float offset_rad_to_set = raw_pos_rad - homed_pos_rad;

                motors->setOffset_rad(i, offset_rad_to_set);
                        
                Serial.printf("Motor %d homed. Raw: %.4f rad, Target: %.4f rad, Offset set to: %.4f rad\n", 
                                i, raw_pos_rad, homed_positions_rad[i], offset_rad_to_set);
                
            }
        }
    }
    
    sendAllCurrentsCommand(command_currents);

    // TODO: 檢查 homing_phase 是否完成，並進入下一個 phase
    // if (homing_phase == 0 && knee_axes_all_homed) { ... }
}

bool RobotController::isHomed() {
    for(bool homed : homed_axes) {
        if (!homed) return false;
    }
    return true;
}


void RobotController::setAllMotorsIdle() {
    int16_t zero_currents[NUM_ROBOT_MOTORS] = {0};
    sendAllCurrentsCommand(zero_currents);
}

// 新增 updateWiggleTest() 函式
void RobotController::updateWiggleTest() {
    const float MAX_REASONABLE_ERROR_RAD = 0.5; // 約 30 度。如果誤差超過這個值，說明出問題了。
    const int16_t WIGGLE_MAX_CURRENT = MAX_COMMAND_CURRENT_mA;

    // 1. 計算目標位置
    float target_offset = wiggle_amplitude_rad * sin(millis() / 1000.0f * 2.0f * PI * wiggle_frequency_hz);
    float target_pos_rad = wiggle_center_pos_rad + target_offset;

    // 2. 獲取當前位置
    float current_pos_rad = getMotorPosition_rad(wiggle_motor_id);

    // 3. 計算位置誤差
    float error_rad = target_pos_rad - current_pos_rad;

    // ================== 新增安全檢查 ==================
    if (abs(error_rad) > MAX_REASONABLE_ERROR_RAD) {
        Serial.printf("!!! SAFETY WARNING !!! Wiggle test error (%.4f rad) exceeds limit. Switching to IDLE.\n", error_rad);
        setIdle(); // 立即切換到安全模式
        return; // 終止本次 update
    }
    // =================================================

    // 4. P 控制器：計算指令電流
    int16_t command_current = (int16_t)(wiggle_kp * error_rad);
    
    // ================== 中止機制 ==================
    if (abs(command_current) > WIGGLE_MAX_CURRENT) {
        Serial.printf("!!! WIGGLE TEST ERROR !!! Calculated current %d mA exceeds limit %d mA. Switching to IDLE.\n", command_current, WIGGLE_MAX_CURRENT);
        setIdle(); // 中止擺動測試，切換到安全的待機模式
        return;    // 立即退出本次 update，避免發送超限電流
    }
    // =================================================

    // 5. 限制最大電流
    command_current = constrain(command_current, -MAX_COMMAND_CURRENT_mA, MAX_COMMAND_CURRENT_mA);

    // 6. 發送指令
    int16_t all_currents[NUM_ROBOT_MOTORS] = {0};
    all_currents[wiggle_motor_id] = command_current;
    sendAllCurrentsCommand(all_currents);
}

// 切換到待機模式
void RobotController::setIdle() {
    mode = ControlMode::IDLE;
    for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {// 清理手動控制指令
        manual_current_commands[i] = 0;
    }
    setAllMotorsIdle(); // 確保馬達停止
    Serial.println("Robot controller set to IDLE mode.");
}

// 接收手動控制指令
void RobotController::setSingleMotorCurrent(int motorID, int16_t current) {
    const int16_t MANUAL_MAX_CURRENT = MAX_COMMAND_CURRENT_mA;

    if (mode == ControlMode::HOMING) {
        Serial.println("  [WARN] Cannot manually control motors during homing. Command ignored.");
        return;
    }
    // ================== 新增：指令拒絕機制 ==================
    if (abs(current) > MANUAL_MAX_CURRENT) {
        Serial.printf("  [ERROR] Manual current command %d mA exceeds limit %d mA. Command REJECTED.\n", current, MANUAL_MAX_CURRENT);
        return; // 直接拒絕指令，不進行任何操作
    }
    // ======================================================

    // 切換到手動模式 (如果我們新增了 MANUAL 模式) 或者直接在 IDLE/POSITION_CONTROL 模式下覆蓋指令
    mode = ControlMode::MANUAL_CONTROL;
    // 為簡化，我們直接修改一個陣列，並在 update 中應用它
    if (motorID >= 0 && motorID < NUM_ROBOT_MOTORS) {
        // 清空所有之前的指令
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            manual_current_commands[i] = 0;
        }
        // 設定目標馬達的電流
        manual_current_commands[motorID] = current;
        
        // 為了立即響應，直接發送一次
        sendAllCurrentsCommand(manual_current_commands);

        // 如果當前是 IDLE，可以考慮新增一個 MANUAL 模式
        // mode = ControlMode::MANUAL; 
        Serial.printf("  Set motor %d current to %d.\n", motorID, manual_current_commands[motorID]);
    } else {
        Serial.println("  [ERROR] Invalid motor ID.");
    }
}

// 獲取校準後的馬達位置 (弧度)
float RobotController::getMotorPosition_rad(int motorID) {
    // 1. 從 MotorController 獲取減去偏移量的位置 (單位：弧度)
    float pos_after_offset_rad = motors->getPosition_rad(motorID);

    // 2. 乘以方向係數，得到真正的校準後弧度
    float calibrated_pos_rad = pos_after_offset_rad * direction_multipliers[motorID];
    
    // 3. 直接返回弧度值，不再需要轉換！
    return calibrated_pos_rad;
}

// 獲取馬達速度
float RobotController::getMotorVelocity_rad(int motorID) {
    // 速度也需要乘以方向係數
    return motors->getRawVelocity_rad(motorID) * direction_multipliers[motorID];
}

// 新增一個函式來觸發擺動測試
void RobotController::startWiggleTest(int motorID) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) {
        Serial.println("Invalid motor ID for wiggle test.");
        return;
    }
    
    // 必須先歸零，才能知道當前位置
    if (!isHomed()) {
        Serial.println("Error: Robot must be homed before wiggle test.");
        // 或者，不依賴歸零，直接用原始位置
        // 為了安全，我們先要求歸零
        // setIdle();
        // return;
    }

    mode = ControlMode::WIGGLE_TEST;
    wiggle_motor_id = motorID;
    // 將當前位置設為擺動中心
    wiggle_center_pos_rad = getMotorPosition_rad(wiggle_motor_id); 
    
    Serial.printf("Starting wiggle test for motor %d around position %.4f rad.\n", motorID, wiggle_center_pos_rad);
}

// 位置控制模式
void RobotController::updatePositionControl() {
    int16_t command_currents[NUM_ROBOT_MOTORS] = {0};
    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        float current_pos = getMotorPosition_rad(i);
        float error = target_positions_rad[i] - current_pos;
        // 簡易 P 控制器 (未來可以加入 D 項)
        command_currents[i] = (int16_t)(pos_kp * error);
        command_currents[i] = constrain(command_currents[i], -2000, 2000); // 限制電流
    }
    sendAllCurrentsCommand(command_currents);
}
