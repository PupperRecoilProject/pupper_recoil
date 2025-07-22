// src/RobotController.cpp

#include "RobotController.h"
#include <Arduino.h>
#include <cstring> // 為了使用 memcpy
#include <cmath>
#include <map>


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

// 定義在 .h 中宣告的靜態常數系統預設參數
const CascadeParams RobotController::SYSTEM_DEFAULT_PARAMS = {
    .c = 16.0f,
    .vel_kp = 500.0f,
    .vel_ki = 0.0f,
    .max_target_velocity_rad_s = 8.0f,
    .integral_max_error_rad = 0.5f
};



// =================================================================
//   輔助資料結構
// =================================================================
const std::map<std::string, RobotController::ParamMask> param_name_to_mask = {
    {"c",       RobotController::ParamMask::MASK_C},
    {"kp",      RobotController::ParamMask::MASK_VEL_KP},
    {"vel_kp",  RobotController::ParamMask::MASK_VEL_KP}, // "kp" 和 "vel_kp" 是別名
    {"ki",      RobotController::ParamMask::MASK_VEL_KI},
    {"vel_ki",  RobotController::ParamMask::MASK_VEL_KI}, // "ki" 和 "vel_ki" 是別名
    {"max_vel", RobotController::ParamMask::MASK_MAX_VEL},
    {"max_err", RobotController::ParamMask::MASK_MAX_ERR}
};



// =================================================================
//   構造函式 與 生命週期函式
// =================================================================

RobotController::RobotController(MotorController* motor_ctrl) 
    : motors(motor_ctrl), mode(ControlMode::IDLE) 
{
    direction_multipliers = { 1,  1, -1,
                             -1, -1,  1,
                              1,  1, -1,
                             -1, -1,  1};
    target_positions_rad.fill(0.0f);
    manual_current_commands.fill(0);
    integral_error_rad_s.fill(0.0f);
    _target_currents_mA.fill(0);
    is_joint_calibrated.fill(false);
       
    // 初始化狀態變數
    integral_error_vel.fill(0.0f);
    _target_vel_rad_s.fill(0.0f);
    _vel_error_rad_s.fill(0.0f);
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
        case ControlMode::JOINT_ARRAY_CONTROL:
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
                // <<< ADDED: 為新群組增加錯誤提示 >>>
        case JointGroup::LEG0:
        case JointGroup::LEG1:
        case JointGroup::LEG2:
        case JointGroup::LEG3:
        case JointGroup::LEG_FRONT:
        case JointGroup::LEG_REAR:
             Serial.println("[錯誤] 腿部群組需要3個角度，請使用 'leg' 或 'leg_pair' 指令。");
             return; // 直接返回，不執行後續操作  
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

// 實現新的 setRobotPoseCascade (C. 簡化模式切換邏輯)
void RobotController::setRobotPoseCascade(const std::array<float, NUM_ROBOT_MOTORS>& pose_rad) {
    if (!isCalibrated()) {
        Serial.println("[錯誤] 級聯姿態控制失敗：機器人必須先校準！");
        return;
    }
    // 呼叫新的輔助函式來處理模式切換
    ensureCascadeMode();
    
    // 更新目標姿態
    target_positions_rad = pose_rad;
    Serial.println("已設定新的級聯控制目標姿態。");
}

// 實現新的 setTargetPositionCascade (C. 簡化模式切換邏輯)
void RobotController::setTargetPositionCascade(int motorID, float angle_rad) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return;
    if (!isCalibrated()) { /* ... 錯誤提示 ... */ return; }

    // 呼叫新的輔助函式來處理模式切換
    ensureCascadeMode();

    // 更新指定馬達的目標位置
    target_positions_rad[motorID] = angle_rad;
    Serial.printf("  [Cascade] 設定馬達 %d 的目標位置為 %.4f rad。\n", motorID, angle_rad);
}

// 實現新的 setJointGroupPositionCascade (C. 簡化模式切換邏輯)
void RobotController::setJointGroupPositionCascade(JointGroup group, float angle_rad) {
    if (!isCalibrated()) {
        Serial.println("[錯誤] 分組控制失敗：機器人必須先校準 (cal)！");
        return;
    }

    // 呼叫新的輔助函式來處理模式切換
    ensureCascadeMode();

    int start_index = -1;
    const char* group_name = "UNKNOWN";
    switch(group) {
        case JointGroup::HIP:   start_index = 0; group_name = "HIP";   break;
        case JointGroup::UPPER: start_index = 1; group_name = "UPPER"; break;
        case JointGroup::LOWER: start_index = 2; group_name = "LOWER"; break;
        default:
             Serial.println("[錯誤] 此函式僅適用於 HIP/UPPER/LOWER 組。");
             return;
    }
    
    Serial.printf("--> [Cascade] 正在設定關節組: %s, 目標角度: %.4f rad\n", group_name, angle_rad);

    if (start_index != -1) {
        for (int i = start_index; i < NUM_ROBOT_MOTORS; i += 3) {
            target_positions_rad[i] = angle_rad;
        }
    }
}

// 實現新的 setLegJointsCascade (C. 簡化模式切換邏輯)
void RobotController::setLegJointsCascade(int leg_id, float hip_rad, float upper_rad, float lower_rad) {
    if (leg_id < 0 || leg_id > 3) {
        Serial.println("[錯誤] 無效的腿部 ID。請使用 0-3。");
        return;
    }
    if (!isCalibrated()) {
        Serial.println("[錯誤] 腿部控制失敗：機器人必須先校準！");
        return;
    }

    // 呼叫新的輔助函式來處理模式切換
    ensureCascadeMode();
    
    int base_motor_id = leg_id * 3;
    target_positions_rad[base_motor_id + 0] = hip_rad;
    target_positions_rad[base_motor_id + 1] = upper_rad;
    target_positions_rad[base_motor_id + 2] = lower_rad;
    
    Serial.printf("--> [Cascade] 設定腿 %d 關節角度為 (H:%.2f, U:%.2f, L:%.2f) rad。\n", 
                  leg_id, hip_rad, upper_rad, lower_rad);
}


// 實現新的 setLegPairCascade (C. 簡化模式切換邏輯)
void RobotController::setLegPairCascade(JointGroup group, float hip_rad, float upper_rad, float lower_rad) {
    if (group != JointGroup::LEG_FRONT && group != JointGroup::LEG_REAR) {
        Serial.println("[錯誤] 此函式僅適用於 LEG_FRONT 或 LEG_REAR。");
        return;
    }
    if (!isCalibrated()) {
        Serial.println("[錯誤] 腿部控制失敗：機器人必須先校準！");
        return;
    }

    // 呼叫新的輔助函式來處理模式切換
    ensureCascadeMode();

    const char* group_name = (group == JointGroup::LEG_FRONT) ? "前腿對" : "後腿對";
    Serial.printf("--> [Cascade] 設定 %s 角度為 (H:%.2f, U:%.2f, L:%.2f) rad (右腿為基準)。\n", 
                  group_name, hip_rad, upper_rad, lower_rad);

    if (group == JointGroup::LEG_FRONT) {
        target_positions_rad[0] = hip_rad;    target_positions_rad[1] = upper_rad;   target_positions_rad[2] = lower_rad;
        target_positions_rad[3] = -hip_rad;   target_positions_rad[4] = -upper_rad;  target_positions_rad[5] = -lower_rad;
    } else { // LEG_REAR
        target_positions_rad[6] = hip_rad;    target_positions_rad[7] = upper_rad;   target_positions_rad[8] = lower_rad;
        target_positions_rad[9] = -hip_rad;   target_positions_rad[10] = -upper_rad; target_positions_rad[11] = -lower_rad;
    }
}

// 相對運動函式
void RobotController::moveMotorRelative_rad(int motorID, float delta_rad) {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) {
        Serial.println("[錯誤] 無效的馬達 ID。");
        return;
    }
    if (!isCalibrated()) {
        Serial.println("[錯誤] 相對運動失敗：機器人必須先校準！");
        return;
    }

    // 呼叫輔助函式來確保處於正確模式
    ensureCascadeMode();
    
    // 在當前目標位置的基礎上進行增減
    target_positions_rad[motorID] += delta_rad;
    
    Serial.printf("  [Cascade] 馬達 %d 相對移動 %.4f rad, 新目標: %.4f rad。\n", 
                  motorID, delta_rad, target_positions_rad[motorID]);
}



// =================================================================
//   參數管理後端 (Parameter Management Backend)
// =================================================================
RobotController::ParamMask getMaskFromName(const std::string& param_name) {
    auto it = param_name_to_mask.find(param_name);
    if (it != param_name_to_mask.end()) {
        return it->second;
    }
    return RobotController::ParamMask::MASK_NONE;
}

void RobotController::setParamOverride(ParamScope scope, int target_id, const std::string& param_name, float value) {
    ParamMask mask_to_set = getMaskFromName(param_name);
    if (mask_to_set == MASK_NONE) return; // 無效的參數名

    // 根據作用域選擇要修改的對象
    if (scope == ParamScope::GLOBAL) {
        _global_override_mask |= mask_to_set; // 設置對應的旗標位
        if (mask_to_set & MASK_C)       _global_params.c = value;
        if (mask_to_set & MASK_VEL_KP)  _global_params.vel_kp = value;
        if (mask_to_set & MASK_VEL_KI)  _global_params.vel_ki = value;
        if (mask_to_set & MASK_MAX_VEL) _global_params.max_target_velocity_rad_s = value;
        if (mask_to_set & MASK_MAX_ERR) _global_params.integral_max_error_rad = value;
    } else if (scope == ParamScope::MOTOR) {
        if (target_id < 0 || target_id >= NUM_ROBOT_MOTORS) return;
        _motor_overrides[target_id].override_mask |= mask_to_set; // 設置旗標
        if (mask_to_set & MASK_C)       _motor_overrides[target_id].values.c = value;
        if (mask_to_set & MASK_VEL_KP)  _motor_overrides[target_id].values.vel_kp = value;
        if (mask_to_set & MASK_VEL_KI)  _motor_overrides[target_id].values.vel_ki = value;
        if (mask_to_set & MASK_MAX_VEL) _motor_overrides[target_id].values.max_target_velocity_rad_s = value;
        if (mask_to_set & MASK_MAX_ERR) _motor_overrides[target_id].values.integral_max_error_rad = value;
    }
}

void RobotController::resetParamOverride(ParamScope scope, int target_id, const std::string& param_name) {
    // 如果 param_name 為空，代表重置所有參數
    ParamMask mask_to_clear = param_name.empty() ? MASK_ALL : getMaskFromName(param_name);
    if (mask_to_clear == MASK_NONE) return;

    if (scope == ParamScope::GLOBAL) {
        _global_override_mask &= ~mask_to_clear; // 清除對應的旗標位
    } else if (scope == ParamScope::MOTOR) {
        if (target_id < 0 || target_id >= NUM_ROBOT_MOTORS) return;
        _motor_overrides[target_id].override_mask &= ~mask_to_clear;
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
        case ControlMode::JOINT_ARRAY_CONTROL: return "JOINT_ARRAY_CONTROL (高層陣列控制)";
        case ControlMode::ERROR:            return "ERROR (錯誤)";
        default:                            return "UNKNOWN (未知)";
    }
}

bool RobotController::isCalibrated() { /* ... 內容不變 ... */ for(bool h : is_joint_calibrated) if(!h) return false; return true; }
float RobotController::getMotorPosition_rad(int motorID) { /* ... 內容不變 ... */ if(motorID<0||motorID>=NUM_ROBOT_MOTORS)return 0; return motors->getPosition_rad(motorID) * direction_multipliers[motorID]; }
float RobotController::getMotorVelocity_rad(int motorID) { /* ... 內容不變 ... */ if(motorID<0||motorID>=NUM_ROBOT_MOTORS)return 0; return motors->getRawVelocity_rad(motorID) * direction_multipliers[motorID]; }

// <<< ADDED: 新的 getEffectiveParams 函式實作 >>>
CascadeParams RobotController::getEffectiveParams(int motorID) const {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return {0};
    
    // 1. 從系統預設開始
    CascadeParams effective_params = SYSTEM_DEFAULT_PARAMS;

    // 2. 應用全域覆蓋
    if (_global_override_mask & MASK_C)       effective_params.c = _global_params.c;
    if (_global_override_mask & MASK_VEL_KP)  effective_params.vel_kp = _global_params.vel_kp;
    if (_global_override_mask & MASK_VEL_KI)  effective_params.vel_ki = _global_params.vel_ki;
    if (_global_override_mask & MASK_MAX_VEL) effective_params.max_target_velocity_rad_s = _global_params.max_target_velocity_rad_s;
    if (_global_override_mask & MASK_MAX_ERR) effective_params.integral_max_error_rad = _global_params.integral_max_error_rad;

    // 3. 應用個別馬達覆蓋
    const auto& motor_override = _motor_overrides[motorID];
    if (motor_override.override_mask & MASK_C)       effective_params.c = motor_override.values.c;
    if (motor_override.override_mask & MASK_VEL_KP)  effective_params.vel_kp = motor_override.values.vel_kp;
    if (motor_override.override_mask & MASK_VEL_KI)  effective_params.vel_ki = motor_override.values.vel_ki;
    if (motor_override.override_mask & MASK_MAX_VEL) effective_params.max_target_velocity_rad_s = motor_override.values.max_target_velocity_rad_s;
    if (motor_override.override_mask & MASK_MAX_ERR) effective_params.integral_max_error_rad = motor_override.values.integral_max_error_rad;

    return effective_params;
}

// <<< ADDED: 新增 getParamSourceInfo 函式 >>>
ParamSourceInfo RobotController::getParamSourceInfo(int motorID) const {
    if (motorID < 0 || motorID >= NUM_ROBOT_MOTORS) return {};

    ParamSourceInfo info;
    const auto& motor_override = _motor_overrides[motorID];

    info.c_source       = (motor_override.override_mask & MASK_C) ? "Motor" : ((_global_override_mask & MASK_C) ? "Global" : "Default");
    info.vel_kp_source  = (motor_override.override_mask & MASK_VEL_KP) ? "Motor" : ((_global_override_mask & MASK_VEL_KP) ? "Global" : "Default");
    info.vel_ki_source  = (motor_override.override_mask & MASK_VEL_KI) ? "Motor" : ((_global_override_mask & MASK_VEL_KI) ? "Global" : "Default");
    info.max_vel_source = (motor_override.override_mask & MASK_MAX_VEL) ? "Motor" : ((_global_override_mask & MASK_MAX_VEL) ? "Global" : "Default");
    info.max_err_source = (motor_override.override_mask & MASK_MAX_ERR) ? "Motor" : ((_global_override_mask & MASK_MAX_ERR) ? "Global" : "Default");

    return info;
}

// <<< ADDED: 新增 getMotorIdsForGroup 函式 >>>
std::vector<int> RobotController::getMotorIdsForGroup(const std::string& group_name_short) {
    if (group_name_short == "h") return {0, 3, 6, 9};
    if (group_name_short == "u") return {1, 4, 7, 10};
    if (group_name_short == "l") return {2, 5, 8, 11};
    if (group_name_short == "l0") return {0, 1, 2};
    if (group_name_short == "l1") return {3, 4, 5};
    if (group_name_short == "l2") return {6, 7, 8};
    if (group_name_short == "l3") return {9, 10, 11};
    if (group_name_short == "f") return {0, 1, 2, 3, 4, 5};
    if (group_name_short == "r") return {6, 7, 8, 9, 10, 11};
    return {}; // 返回空向量如果組名無效
}

// 實現新的私有輔助函式
void RobotController::ensureCascadeMode() {
    // 檢查當前模式是否為 CASCADE_CONTROL
    if (mode != ControlMode::CASCADE_CONTROL) {
        Serial.println("非串級模式，正在執行安全切換至 CASCADE_CONTROL...");
        
        // 步驟 1: 凍結 - 將所有目標位置刷新為當前實際位置，防止亂動
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            target_positions_rad[i] = getMotorPosition_rad(i);
        }
        
        // 步驟 2: 切換 - 設定新模式
        mode = ControlMode::CASCADE_CONTROL;
        
        // 步驟 3: 重置 - 清除舊的積分誤差，以一個乾淨的狀態開始
        integral_error_vel.fill(0.0f);
        
        Serial.println("模式切換完成，已凍結所有關節。");
    }
}



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

// 級聯控制的完整更新邏輯
void RobotController::updateCascadeControl() {
    const float dt = 1.0f / CONTROL_FREQUENCY_HZ_H;

    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        const CascadeParams params = getEffectiveParams(i); //在迴圈開始時，獲取該馬達當前生效的參數
        // --- 數據採集 ---
        float current_pos = getMotorPosition_rad(i);
        float current_vel = getMotorVelocity_rad(i);

        // --- 安全檢查 (與位置控制共用相同的安全限制) ---
        if (std::abs(current_vel) > POS_CONTROL_MAX_VELOCITY_RAD_S) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 速度 %.2f 超出限制 %.2f rad/s。\n", i, current_vel, POS_CONTROL_MAX_VELOCITY_RAD_S);
            setIdle(); return;
        }
        float pos_error = target_positions_rad[i] - current_pos;
        if (std::abs(pos_error) > POS_CONTROL_MAX_ERROR_RAD) {
            Serial.printf("!!! 安全停機 !!! 馬達 %d 位置誤差 %.2f 超出限制 %.2f rad。\n", i, pos_error, POS_CONTROL_MAX_ERROR_RAD);
            setIdle(); return;
        }

        // --- 外環: 位置控制器 (P-Controller) ---
        // 計算目標速度，使用每個馬達獨立的 'c' 參數
        float target_vel = pos_error * params.c;

        // 限制目標速度，使用每個馬達獨立的限速參數
        target_vel = std::max(-params.max_target_velocity_rad_s, 
                              std::min(params.max_target_velocity_rad_s, target_vel));

        // --- 內環: 速度控制器 (PI-Controller) ---
        float vel_error = target_vel - current_vel;

        // 積分項累加，帶條件性抗飽和 (Anti-Windup)
        // 只有當位置誤差在允許範圍內時，才累加積分項
        if (std::abs(pos_error) < params.integral_max_error_rad) {
             integral_error_vel[i] += vel_error * dt;
        } else {
             integral_error_vel[i] = 0.0f;
        }

        // PI 計算，使用每個馬達獨立的 'vel_kp' 和 'vel_ki'
        float p_term = params.vel_kp * vel_error;
        float i_term = params.vel_ki * integral_error_vel[i];

        // 總回饋電流
        float feedback_current = p_term + i_term;
        
        // --- 摩擦力補償 (同樣適用於級聯控制) ---
        float friction_comp_current = 0;
        if (std::abs(current_vel) < FRICTION_VEL_THRESHOLD_RAD_S && std::abs(pos_error) > FRICTION_ERR_THRESHOLD_RAD) {
            friction_comp_current = copysignf(FRICTION_STATIC_COMP_mA, feedback_current);
        } else {
            friction_comp_current = copysignf(FRICTION_KINETIC_COMP_mA, current_vel);
        }

        _target_currents_mA[i] = static_cast<int16_t>(feedback_current + friction_comp_current);

        // 保存除錯資訊
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
    info.vel_error_rad_s   = info.target_vel_rad_s - info.current_vel_rad_s; // Corrected this line
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
