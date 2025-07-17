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
    integral_error_vel.fill(0.0f); // **** NEW **** 初始化速度積分項
    _target_currents_mA.fill(0);
    is_joint_calibrated.fill(false); // 明確地將所有關節的校準狀態初始化為 false。
}

void RobotController::begin() {
    wiggle_amplitude_rad = 0.17;
    wiggle_frequency_hz = 0.5;
    wiggle_kp = 20.0;
    // 所有與 Homing 相關的初始化都已移除

    if (_param_map.empty()) { // 加上判斷，防止重複初始化
        _param_map["pos_kp"] = &CascadeParams::pos_kp;
        _param_map["vel_kp"] = &CascadeParams::vel_kp;
        _param_map["vel_ki"] = &CascadeParams::vel_ki;
        _param_map["max_vel"] = &CascadeParams::max_target_vel;
        _param_map["max_int"] = &CascadeParams::max_integral_err;
    }

    // 【新增】初始化參數系統
    _global_params.reset(_system_default_params);
    for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
        _motor_params[i].reset(_global_params);
    }
}


void RobotController::update() {
    switch(mode) {
        case ControlMode::IDLE:
            setAllMotorsIdle();
            break;
        case ControlMode::POSITION_CONTROL:
        //case ControlMode::JOINT_ARRAY_CONTROL: //舊版功能棄用
            updatePositionControl();
            break;
        case ControlMode::CASCADE_CONTROL: // **** NEW ****
            updateCascadeControl();    // 呼叫新的級聯控制更新
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

void RobotController::setTargetPositionPID(int motorID, float angle_rad) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) { /* ... */ return; }   // 在第一次切換到位置控制模式時，初始化所有目標位置

    // 在第一次切換到位置控制模式時，進行初始化。
    if (mode != ControlMode::POSITION_CONTROL) {
        Serial.println("從 IDLE 切換至 POSITION_CONTROL 模式。");
        Serial.println("凍結所有馬達在當前位置，僅更新目標馬達。");
        
        // << 新增的關鍵邏輯 >>
        // 為了防止未被指令控制的馬達移動到它們舊的、可能為零的目標位置，
        // 我們在切換模式的這一刻，將所有馬達的目標位置都刷新為它們的當前位置。
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        
        mode = ControlMode::POSITION_CONTROL;
        integral_error_rad_s.fill(0.0f); // 重置所有積分項
    }

    // 然後再設定指定的目標
    target_positions_rad[motorID] = angle_rad;

    // 重置該馬達的積分項，以獲得更乾淨的響應
    // integral_error_rad_s[motorID] = 0.0f; // 先移除，讓附載的運動更平滑
    
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
    integral_error_rad_s.fill(0.0f); // 重置PID位置積分項
    integral_error_vel.fill(0.0f);   // 重置級聯速度積分項
    for (int i=0; i<NUM_ROBOT_MOTORS; ++i) {
        target_positions_rad[i] = getMotorPosition_rad(i);
    }
    setAllMotorsIdle();
    Serial.println("機器人控制器已設為 IDLE 模式 (所有狀態已重置)。");
}

void RobotController::setJointGroupPosition_rad(JointGroup group, float angle_rad) {
    // 檢查機器人是否已校準，這是安全執行姿態控制的前提
    if (!isCalibrated()) {
        Serial.println("[錯誤] 分組控制失敗：機器人必須先校準！");
        return;
    }

    // 在第一次進入位置控制模式時，進行必要的初始化
    if (mode != ControlMode::POSITION_CONTROL) {
        Serial.println("切換至 POSITION_CONTROL 模式，準備進行分組控制。");
        
        // 關鍵步驟：將所有馬達的目標位置凍結在當前位置，防止未被控制的馬達亂動
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        
        mode = ControlMode::POSITION_CONTROL;
        // 從非位置模式首次進入時，清除所有積分項，以一個乾淨的狀態開始
        integral_error_rad_s.fill(0.0f);
    }

    int start_index = -1;
    const char* group_name = "UNKNOWN";

    // 根據選擇的組別，確定要操作的馬達索引的起始值
    switch(group) {
        case JointGroup::HIP:
            start_index = 0; // 馬達ID 0, 3, 6, 9
            group_name = "HIP";
            break;
        case JointGroup::UPPER:
            start_index = 1; // 馬達ID 1, 4, 7, 10
            group_name = "UPPER";
            break;
        case JointGroup::LOWER:
            start_index = 2; // 馬達ID 2, 5, 8, 11
            group_name = "LOWER";
            break;
        
        // 這些複雜的組別不由舊的 PID 控制器支援。
        // 將 start_index 保持為 -1，後續邏輯會處理這個錯誤。
        case JointGroup::LEG0:
        case JointGroup::LEG1:
        case JointGroup::LEG2:
        case JointGroup::LEG3:
        case JointGroup::LEG_FRONT:
        case JointGroup::LEG_REAR:
        case JointGroup::ALL:
        default:
            group_name = "UNSUPPORTED_IN_PID_MODE";
            start_index = -1;
            break;
    }
    
    Serial.printf("--> 正在設定關節組: %s, 目標角度: %.4f rad\n", group_name, angle_rad);

    if (start_index != -1) {
        // 使用一個迴圈，步長為3，來更新屬於同一組的所有馬達的目標位置
        for (int i = start_index; i < NUM_ROBOT_MOTORS; i += 3) {
            target_positions_rad[i] = angle_rad;
        }
    } else {
        Serial.println("[錯誤] 無效的關節組別。");
    }
}

// **** NEW **** - 進入級聯控制模式的外部接口
void RobotController::setRobotPoseCascade(const std::array<float, NUM_ROBOT_MOTORS>& pose_rad) {
    if (!isCalibrated()) {
        Serial.println("[錯誤] 級聯姿態控制失敗：機器人必須先校準！");
        return;
    }

    if (mode != ControlMode::CASCADE_CONTROL) {
        Serial.println("切換至 CASCADE_CONTROL 模式。");
        // 從非級聯模式首次進入時，清除所有速度積分項，以一個乾淨的狀態開始
        integral_error_vel.fill(0.0f);
        mode = ControlMode::CASCADE_CONTROL;
    }
    
    // 更新目標姿態
    target_positions_rad = pose_rad;
    Serial.println("已設定新的級聯控制目標姿態。");
}

// 實現新的 setTargetPositionCascade
void RobotController::setTargetPositionCascade(int motorID, float angle_rad) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return;
    if (!isCalibrated()) { /* ... 錯誤提示 ... */ return; }

    // 檢查是否需要切換模式或初始化
    if (mode != ControlMode::CASCADE_CONTROL) {
        Serial.println("切換至 CASCADE_CONTROL 模式，準備進行單關節控制。");
        
        // 關鍵：將所有目標凍結在當前位置
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        
        mode = ControlMode::CASCADE_CONTROL;
        integral_error_vel.fill(0.0f); // 首次進入，重置速度積分項
    }

    // 更新指定馬達的目標位置
    target_positions_rad[motorID] = angle_rad;
    Serial.printf("  [Cascade] 設定馬達 %d 的目標位置為 %.4f rad。\n", motorID, angle_rad);
}

// 實現新的 setJointGroupPositionCascade (邏輯與上面類似)
void RobotController::setJointGroupPositionCascade(JointGroup group, float angle_rad) {
    // 步驟 1: 安全檢查 - 未校準則不執行
    if (!isCalibrated()) {
        Serial.println("[錯誤] 分組控制失敗：機器人必須先校準 (cal)！");
        return;
    }

    // 步驟 2 & 3: 模式管理 - 如果不是串級模式，則執行安全切換
    if (mode != ControlMode::CASCADE_CONTROL) {
        Serial.println("非串級模式，正在執行安全切換至 CASCADE_CONTROL...");
        
        // 步驟 3.1: 凍結 - 將所有目標位置刷新為當前實際位置，防止亂動
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        
        // 步驟 3.2: 切換 - 設定新模式
        mode = ControlMode::CASCADE_CONTROL;
        // 步驟 3.3: 重置 - 清除舊的積分誤差
        integral_error_vel.fill(0.0f);
        Serial.println("模式切換完成，已凍結所有關節。");
    }

    // 步驟 4: 確定目標 - 根據組別確定起始索引
    std::string group_str_name;
    switch(group) {
        case JointGroup::HIP:       group_str_name = "hip";       break;
        case JointGroup::UPPER:     group_str_name = "upper";     break;
        case JointGroup::LOWER:     group_str_name = "lower";     break;
        case JointGroup::LEG0:      group_str_name = "leg0";      break;
        case JointGroup::LEG1:      group_str_name = "leg1";      break;
        case JointGroup::LEG2:      group_str_name = "leg2";      break;
        case JointGroup::LEG3:      group_str_name = "leg3";      break;
        case JointGroup::LEG_FRONT: group_str_name = "leg_front"; break;
        case JointGroup::LEG_REAR:  group_str_name = "leg_rear";  break;
        case JointGroup::ALL:       group_str_name = "all";       break;
        // 我們不需要 default，因為我們假設傳入的 enum 是有效的
    }
    
    // 步驟 5: 使用 parseGroup 輔助函式獲取目標馬達 ID 列表
    std::vector<int> target_ids;
    if (parseGroup(group_str_name, target_ids)) {
        Serial.printf("--> [Cascade] 正在設定關節組: %s (%zu motors), 目標角度: %.4f rad\n", group_str_name.c_str(), target_ids.size(), angle_rad);
        
        // 步驟 6: 遍歷 ID 列表並設定目標位置
        for (int id : target_ids) {
            if (id >= 0 && id < NUM_ROBOT_MOTORS) {
                target_positions_rad[id] = angle_rad;
            }
        }
    } else {
        // 正常情況下，因為 group enum 是受控的，這裡不會執行
        Serial.printf("[錯誤] 內部錯誤：無法解析關節組 '%s'。\n", group_str_name.c_str());
    }
}



// =================================================================
//   狀態與數據獲取函式
// =================================================================

const char* RobotController::getModeString() {
    switch (mode) {
        case ControlMode::IDLE:             return "IDLE (待機)";
        case ControlMode::POSITION_CONTROL: return "POSITION_CONTROL (位置控制)";
        case ControlMode::CASCADE_CONTROL:  return "CASCADE_CONTROL (級聯控制)"; // **** NEW ****
        case ControlMode::WIGGLE_TEST:      return "WIGGLE_TEST (擺動測試)";
        case ControlMode::CURRENT_MANUAL_CONTROL:   return "CURRENT_MANUAL_CONTROL (手動控制)";
        //case ControlMode::JOINT_ARRAY_CONTROL: return "JOINT_ARRAY_CONTROL (高層陣列控制)";   // 舊版功能棄用
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

        // --- 積分抗飽和 (Integral Anti-Windup) ---
        integral_error_rad_s[i] = constrain(integral_error_rad_s[i], -INTEGRAL_MAX_CURRENT_mA / POS_CONTROL_KI, INTEGRAL_MAX_CURRENT_mA / POS_CONTROL_KI);

        // 條件性積分。
        if ( (i_term >= INTEGRAL_MAX_CURRENT_mA && pos_error > 0) ||
             (i_term <= -INTEGRAL_MAX_CURRENT_mA && pos_error < 0) )
        {
             // 條件不滿足，不進行積分累加。
             // 因為前面已經加過了，所以這裡減掉來撤銷操作。
             integral_error_rad_s[i] -= pos_error * dt;
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


        _target_currents_mA[i] = (int16_t)(feedback_current + friction_comp_current);
    }

    // 位置控制計算的是理想電流，因此 is_ideal=true
    sendCurrents(_target_currents_mA.data(), true);
}

void RobotController::updateWiggleTest() {
    _target_currents_mA.fill(0);
    
    float current_pos_rad = getMotorPosition_rad(wiggle_motor_id);
    float error_rad = (wiggle_center_pos_rad + wiggle_amplitude_rad * sin(millis() / 1000.0f * 2.0f * PI * wiggle_frequency_hz)) - current_pos_rad;

    if (abs(error_rad) > POS_CONTROL_MAX_ERROR_RAD) {
        Serial.printf("!!! 安全警告 !!! 擺動測試誤差 (%.4f rad) 超出限制。\n", error_rad);
        setIdle(); return;
    }

    int16_t ideal_command_current = (int16_t)(wiggle_kp * error_rad);
    _target_currents_mA[wiggle_motor_id] = ideal_command_current;

    // Wiggle Test 也是基於 "機器人座標系" 的誤差，因此 is_ideal=true
    sendCurrents(_target_currents_mA.data(), true);
}

// **** NEW **** - 級聯控制的完整更新邏輯
void RobotController::updateCascadeControl() {
    const float dt = 1.0f / CONTROL_FREQUENCY_HZ_H;

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        const auto& params = _motor_params[i]; // 【修改】從讀取單一全域變數，改為讀取對應馬達的參數結構
        // --- 數據採集 ---
        float current_pos = getMotorPosition_rad(i);
        float current_vel = getMotorVelocity_rad(i);

        // --- 安全檢查 ---
        if (abs(current_vel) > POS_CONTROL_MAX_VELOCITY_RAD_S) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 速度 %.2f 超出限制 %.2f rad/s。\n", i, current_vel, POS_CONTROL_MAX_VELOCITY_RAD_S);
            setIdle(); return;
        }
        float pos_error = target_positions_rad[i] - current_pos;
        if (abs(pos_error) > POS_CONTROL_MAX_ERROR_RAD) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 位置誤差 %.2f 超出限制 %.2f rad。\n", i, pos_error, POS_CONTROL_MAX_ERROR_RAD);
            setIdle(); return;
        }

        // --- 外環: 位置控制器 (P-Controller) ---
        // 計算目標速度，位置誤差越大，期望的修正速度就越快
        float target_vel = params.pos_kp * pos_error;
        target_vel = constrain(target_vel, -params.max_target_vel, params.max_target_vel);
        // --- 內環: 速度控制器 (PI-Controller) ---
        float vel_error = target_vel - current_vel;

        // 積分項累加
        integral_error_vel[i] += vel_error * dt;
        // 積分抗飽和 (Anti-Windup)
        integral_error_vel[i] = constrain(integral_error_vel[i], -params.max_integral_err, params.max_integral_err);

        // PI 計算
        float p_term = params.vel_kp * vel_error;
        float i_term = params.vel_ki * integral_error_vel[i];

        // 總回饋電流
        float feedback_current = p_term + i_term;
        
        // --- 摩擦力補償 (同樣適用於級聯控制) ---
        float friction_comp_current = 0;
        if (abs(current_vel) < FRICTION_VEL_THRESHOLD_RAD_S && abs(pos_error) > FRICTION_ERR_THRESHOLD_RAD) {
            friction_comp_current = copysignf(FRICTION_STATIC_COMP_mA, feedback_current);
        } else {
            friction_comp_current = copysignf(FRICTION_KINETIC_COMP_mA, current_vel);
        }

        _target_currents_mA[i] = (int16_t)(feedback_current + friction_comp_current);

        _target_vel_rad_s[i] = target_vel;
        _vel_error_rad_s[i] = vel_error;
    }

    sendCurrents(_target_currents_mA.data(), true);
}

CascadeDebugInfo RobotController::getCascadeDebugInfo(int motorID) {
    CascadeDebugInfo info = {0};
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return info;

    info.target_pos_rad    = target_positions_rad[motorID];
    info.current_pos_rad   = getMotorPosition_rad(motorID);
    info.pos_error_rad     = info.target_pos_rad - info.current_pos_rad;
    info.target_vel_rad_s  = _target_vel_rad_s[motorID];
    info.current_vel_rad_s = getMotorVelocity_rad(motorID);
    info.vel_error_rad_s   = _vel_error_rad_s[motorID];
    info.target_current_mA = _target_currents_mA[motorID];
    return info;
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

    // 為了確保每次校準都是從一個已知的乾淨狀態開始，
    // 我們首先將所有馬達的現有偏移量重置為零。
    // 這可以防止因重複執行校準而導致的偏移量錯誤疊加。
    Serial.println("正在重置現有偏移量...");
    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        motors->setOffset_rad(i, 0.0f);
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

int16_t RobotController::getTargetCurrent_mA(int motorID) {
    if (motorID >= 0 && motorID < NUM_ROBOT_MOTORS) {
        return _target_currents_mA[motorID];
    }
    return 0;
}



// =================================================================
//   【新增】參數調校系統的完整實現
// =================================================================

bool RobotController::parseGroup(const std::string& name, std::vector<int>& ids) {
    ids.clear();
    if (name == "hip") {
        for (int i = 0; i < 4; ++i) ids.push_back(i * 3 + 0);
    } else if (name == "upper") {
        for (int i = 0; i < 4; ++i) ids.push_back(i * 3 + 1);
    } else if (name == "lower") {
        for (int i = 0; i < 4; ++i) ids.push_back(i * 3 + 2);
    } else if (name == "leg0") {
        ids = {0, 1, 2};
    } else if (name == "leg1") {
        ids = {3, 4, 5};
    } else if (name == "leg2") {
        ids = {6, 7, 8};
    } else if (name == "leg3") {
        ids = {9, 10, 11};
    } else if (name == "leg_front") {
        ids = {0, 1, 2, 3, 4, 5};
    } else if (name == "leg_rear") {
        ids = {6, 7, 8, 9, 10, 11};
    } else if (name == "all") {
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) ids.push_back(i);
    } else {
        return false; // 無效的組名
    }
    return true;
}

void RobotController::setParameter(const std::string& scope_type, const std::string& scope_name, const std::string& param_name, float value) {
    // 1. 檢查參數名稱是否有效
    if (_param_map.find(param_name) == _param_map.end()) {
        Serial.printf("  [ERROR] Invalid parameter name: '%s'\n", param_name.c_str());
        return;
    }
    auto param_ptr = _param_map[param_name];

    // 2. 根據 scope 執行操作
    if (scope_type == "global") {
        _global_params.*param_ptr = value; // 使用成員指標來修改對應的參數
        Serial.printf("--> [OK] Global '%s' set to %.3f. Affects all non-custom motors.\n", param_name.c_str(), value);
        // 更新所有未被自訂的馬達
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            if (!_motor_params[i].is_custom) {
                _motor_params[i].*param_ptr = value;
            }
        }
    } else if (scope_type == "motor") {
        int motor_id = std::stoi(scope_name);
        if (motor_id >= 0 && motor_id < NUM_ROBOT_MOTORS) {
            _motor_params[motor_id].*param_ptr = value;
            _motor_params[motor_id].is_custom = true; // 標記為自訂
            Serial.printf("--> [OK] Motor %d '%s' set to %.3f.\n", motor_id, param_name.c_str(), value);
        } else {
            Serial.printf("  [ERROR] Invalid motor ID: %d\n", motor_id);
        }
    } else if (scope_type == "group") {
        std::vector<int> ids;
        if (parseGroup(scope_name, ids)) {
            for (int id : ids) {
                _motor_params[id].*param_ptr = value;
                _motor_params[id].is_custom = true; // 標記為自訂
            }
            Serial.printf("--> [OK] Group '%s' (%zu motors) '%s' set to %.3f.\n", scope_name.c_str(), ids.size(), param_name.c_str(), value);
        } else {
            Serial.printf("  [ERROR] Invalid group name: '%s'\n", scope_name.c_str());
        }
    }
}

void RobotController::resetParameter(const std::string& scope_type, const std::string& scope_name) {
    if (scope_type == "global") {
        _global_params.reset(_system_default_params);
        Serial.println("--> [OK] Global parameters have been reset to system defaults.");
        // 同步更新所有非自訂馬達
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            if (!_motor_params[i].is_custom) {
                _motor_params[i].reset(_global_params);
            }
        }
    } else if (scope_type == "motor") {
        int motor_id = std::stoi(scope_name);
        if (motor_id >= 0 && motor_id < NUM_ROBOT_MOTORS) {
            _motor_params[motor_id].reset(_global_params); // 重置為當前的全域值
            Serial.printf("--> [OK] Motor %d parameters have been reset to global.\n", motor_id);
        } else {
             Serial.printf("  [ERROR] Invalid motor ID: %d\n", motor_id);
        }
    } else if (scope_type == "group") {
        std::vector<int> ids;
        // 特別處理 "all"，它重置所有馬達和全域參數
        if (scope_name == "all") {
             _global_params.reset(_system_default_params);
             for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
                _motor_params[i].reset(_system_default_params);
             }
             Serial.println("--> [OK] ALL motor and global parameters have been reset to system defaults.");
        } else if (parseGroup(scope_name, ids)) {
            for (int id : ids) {
                _motor_params[id].reset(_global_params);
            }
            Serial.printf("--> [OK] Group '%s' (%zu motors) parameters reset to global.\n", scope_name.c_str(), ids.size());
        } else {
            Serial.printf("  [ERROR] Invalid group name: '%s'\n", scope_name.c_str());
        }
    }
}

void RobotController::printParameters(const std::string& scope_type, const std::string& scope_name) {
    auto print_single_param = [](const CascadeParams& p, int id = -1) {
        char buf[128];
        if (id != -1) { // 打印馬達
            snprintf(buf, sizeof(buf), "Motor %2d %s| P_KP:%-7.2f V_KP:%-7.2f V_KI:%-7.2f | MaxVel:%-5.2f MaxInt:%-5.2f",
                     id, p.is_custom ? "(*)" : "   ", p.pos_kp, p.vel_kp, p.vel_ki, p.max_target_vel, p.max_integral_err);
        } else { // 打印全域
            snprintf(buf, sizeof(buf), "Global    | P_KP:%-7.2f V_KP:%-7.2f V_KI:%-7.2f | MaxVel:%-5.2f MaxInt:%-5.2f",
                     p.pos_kp, p.vel_kp, p.vel_ki, p.max_target_vel, p.max_integral_err);
        }
        Serial.println(buf);
    };

    if (scope_type == "global") {
        Serial.println("--- Current Global Cascade Parameters ---");
        print_single_param(_global_params);
    } else if (scope_type == "motor") {
        int motor_id = std::stoi(scope_name);
        if (motor_id >= 0 && motor_id < NUM_ROBOT_MOTORS) {
            Serial.printf("--- Current Cascade Parameters for Motor %d ---\n", motor_id);
            print_single_param(_motor_params[motor_id], motor_id);
        } else {
            Serial.printf("  [ERROR] Invalid motor ID: %d\n", motor_id);
        }
    } else if (scope_type == "group") {
        std::vector<int> ids;
        if (parseGroup(scope_name, ids)) {
            Serial.printf("--- Current Cascade Parameters for Group '%s' ---\n", scope_name.c_str());
            if (scope_name == "all") { // all 的情況下，先打印全域
                print_single_param(_global_params);
                Serial.println("-----------------------------------------------------------------------------");
            }
            for (int id : ids) {
                print_single_param(_motor_params[id], id);
            }
        } else {
            Serial.printf("  [ERROR] Invalid group name: '%s'\n", scope_name.c_str());
        }
    }
}