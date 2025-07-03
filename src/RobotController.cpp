// src/RobotController.cpp

#include "RobotController.h"
#include <Arduino.h>
#include <cstring> // 為了使用 memcpy

// =================================================================
//   手動校準姿態定義
// =================================================================

// --- 步驟 1: 定義基礎角度 ---
// 在這裡定義您手動擺放時，單一 "右腿" 的目標關節角度。
// 這些值將被用來生成一個完整的、對稱的12關節姿態。
const float HIP_JOINT_RAD       = 0.0f; // 髖關節 (左右擺動, Abduction/Adduction)
const float UPPER_LEG_JOINT_RAD = -1.18f; // 大腿關節 (前後擺動, Hip Flexion/Extension)
const float LOWER_LEG_JOINT_RAD = -2.68f; // 小腿關節 (膝蓋彎曲, Knee Flexion/Extension)

// --- 步驟 2: 檢視完整的對稱姿態 ---
// 這是根據上面的基礎角度和標準的四足對稱性生成的最終校準姿態。
// 您通常不需要修改這裡，除非您的機器人有非標準的對稱結構。
//
const std::array<float, NUM_ROBOT_MOTORS> manual_calibration_pose_rad = {
    // 關節順序: hip, upper, lower

    // Leg 0: Front-Right (FR) - IDs 0, 1, 2
    HIP_JOINT_RAD,                  // FR hip
    UPPER_LEG_JOINT_RAD,            // FR upper leg
    LOWER_LEG_JOINT_RAD,            // FR lower leg

    // Leg 1: Front-Left (FL) - IDs 3, 4, 5
    -HIP_JOINT_RAD,                 // FL hip (髖關節鏡像對稱)
    -UPPER_LEG_JOINT_RAD,            // FL upper leg
    -LOWER_LEG_JOINT_RAD,            // FL lower leg

    // Leg 2: Rear-Right (RR) - IDs 6, 7, 8
    HIP_JOINT_RAD,                  // RR hip
    UPPER_LEG_JOINT_RAD,            // RR upper leg
    LOWER_LEG_JOINT_RAD,            // RR lower leg

    // Leg 3: Rear-Left (RL) - IDs 9, 10, 11
    -HIP_JOINT_RAD,                 // RL hip (髖關節鏡像對稱)
    -UPPER_LEG_JOINT_RAD,            // RL upper leg
    -LOWER_LEG_JOINT_RAD             // RL lower leg
};



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
    integral_error_rad_s.fill(0.0f);
}

void RobotController::begin() {
    wiggle_amplitude_rad = 0.17;
    wiggle_frequency_hz = 0.5;
    wiggle_kp = 20.0;
    // 所有與 Homing 相關的初始化都已移除
}


void RobotController::update() {
    switch(mode) {
        case ControlMode::IDLE:
            setAllMotorsIdle();
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


void RobotController::startWiggleTest(int motorID) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) {
        Serial.println("[錯誤] 無效的馬達ID用於擺動測試。");
        return;
    }
    if (!isCalibrated()) {
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

        integral_error_rad_s.fill(0.0f); // 重置所有積分項
        // 在切換的瞬間，將所有目標位置設為當前位置，以防止跳動
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        mode = ControlMode::POSITION_CONTROL;
    }
    
    // 然後再設定指定的目標
    target_positions_rad[motorID] = angle_rad;

    // 當一個馬達被賦予新的目標位置時，也應該重置其對應的積分項。
    // 這可以防止舊的累積誤差影響對新目標的響應，讓響應更乾淨。
    integral_error_rad_s[motorID] = 0.0f;
    
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
    integral_error_rad_s.fill(0.0f);
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
        case ControlMode::POSITION_CONTROL: return "POSITION_CONTROL (位置控制)";
        case ControlMode::WIGGLE_TEST:      return "WIGGLE_TEST (擺動測試)";
        case ControlMode::CURRENT_MANUAL_CONTROL:   return "CURRENT_MANUAL_CONTROL (手動控制)";
        case ControlMode::ERROR:            return "ERROR (錯誤)";
        default:                            return "UNKNOWN (未知)";
    }
}

bool RobotController::isCalibrated() { /* ... 內容不變 ... */ for(bool h : is_joint_calibrated) if(!h) return false; return true; }
float RobotController::getMotorPosition_rad(int motorID) { /* ... 內容不變 ... */ if(motorID<0||motorID>=NUM_ROBOT_MOTORS)return 0; return motors->getPosition_rad(motorID) * direction_multipliers[motorID]; }
float RobotController::getMotorVelocity_rad(int motorID) { /* ... 內容不變 ... */ if(motorID<0||motorID>=NUM_ROBOT_MOTORS)return 0; return motors->getRawVelocity_rad(motorID) * direction_multipliers[motorID]; }

// =================================================================
//   私有: 內部更新邏輯
// =================================================================


void RobotController::updatePositionControl() {
    int16_t ideal_currents[NUM_ROBOT_MOTORS] = {0};
    // 獲取控制週期的時間間隔 (秒)
    // 您的 CONTROL_FREQUENCY_HZ 是 1000，所以 dt 是 0.001
    const float dt = 1.0f / CONTROL_FREQUENCY_HZ_H; 

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

        // --- 核心 PID 計算 ---
        integral_error_rad_s[i] += pos_error * dt;
        float p_term = POS_CONTROL_KP * pos_error;
        float i_term = POS_CONTROL_KI * integral_error_rad_s[i];
        float d_term = -POS_CONTROL_KD * current_vel;
        integral_error_rad_s[i] = constrain(integral_error_rad_s[i], -INTEGRAL_MAX_CURRENT_mA, INTEGRAL_MAX_CURRENT_mA);
        if ( (i_term >= INTEGRAL_MAX_CURRENT_mA && pos_error > 0) ||
             (i_term <= -INTEGRAL_MAX_CURRENT_mA && pos_error < 0) )
        {
             // 積分已飽和且誤差方向會使其更飽和，所以不再累加
             integral_error_rad_s[i] -= pos_error * dt; // 撤銷本次的累加
        }
        
        // 總回饋電流
        float feedback_current = p_term + i_term + d_term;

        // --- 摩擦力補償 ---
        float friction_comp_current = 0;

        if (abs(current_vel) < FRICTION_VEL_THRESHOLD_RAD_S && abs(pos_error) > FRICTION_ERR_THRESHOLD_RAD) {
            // 狀態 1: 需要克服靜摩擦力 (Kickstart)
            // 使用 feedback_current 的方向來決定啟動力的方向，因為此時速度可能為零。
            friction_comp_current = copysignf(FRICTION_STATIC_COMP_mA, feedback_current);
        } else {
            // 狀態 2: 已經在運動，補償動摩擦力
            // 使用 current_vel 的方向來決定補償力的方向。
            friction_comp_current = copysignf(FRICTION_KINETIC_COMP_mA, current_vel);
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

    // 6. 將所有關節標記為已校準
    is_joint_calibrated.fill(true);

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