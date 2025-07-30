// src/TelemetrySystem.cpp

#include "TelemetrySystem.h"
#include "LSM6DSO_SPI.h"
#include <Arduino.h>

const float G_ACCEL = 9.80665f;


// 構造函式，使用初始化列表來設定所有成員變數的初始值
TelemetrySystem::TelemetrySystem(RobotController* robot, SimpleAHRS* ahrs, MotorController* motors, LSM6DSO* imu)
    : _robot(robot), 
      _ahrs(ahrs), 
      _motors(motors),
      _imu(imu), // 初始化新增的 _imu 指標
      _current_mode(PrintMode::HUMAN_STATUS), // 預設為人類可讀模式
      _focus_motor_id(-1),                    // 預設無焦點馬達
      _csv_header_printed(false),             // CSV標頭尚未打印
      _is_paused(false),                      // 預設遙測不暫停
      _last_command("None")                   // 預設最後指令為"None"
{
}

void TelemetrySystem::begin() {
    // 目前不需要任何特別的初始化操作，保留此函式以備未來擴充
}


// --- 暫停/恢復/設定指令 的函式實現 ---
void TelemetrySystem::pause() {
    if (!_is_paused) {
        _is_paused = true;
        Serial.println("\n--> [Telemetry] Paused. Use 'monitor resume' to continue.");
    }
}

void TelemetrySystem::resume() {
    if (_is_paused) {
        _is_paused = false;
        Serial.println("--> [Telemetry] Resumed.");
    }
}

void TelemetrySystem::setLastCommand(const String& cmd) {
    _last_command = cmd;
}


// 設定新的打印模式 (格式)
void TelemetrySystem::setPrintMode(PrintMode mode) {
    if (_current_mode != mode) {
        _current_mode = mode;
        _csv_header_printed = false; // 切換模式時，重置CSV標頭打印標記
        Serial.printf("\n--> [遙測系統] 輸出格式已切換至: %s\n", getModeString());
    }
}

// 設定新的焦點馬達 (對象)
void TelemetrySystem::setFocusMotor(int motor_id) {
    // 檢查傳入的ID是否在有效範圍內 (-1代表取消焦點)
    if (motor_id >= -1 && motor_id < NUM_ROBOT_MOTORS) {
        if (_focus_motor_id != motor_id) {
            _focus_motor_id = motor_id;
            if (motor_id == -1) {
                Serial.println("--> [遙測系統] 已取消焦點，恢復全局視圖。");
            } else {
                Serial.printf("--> [遙測系統] 已設定焦點於馬達 %d。\n", motor_id);
            }
        }
    } else {
        Serial.printf("[錯誤] 無效的馬達ID: %d。請使用 0-%d 或 -1。\n", motor_id, NUM_ROBOT_MOTORS - 1);
    }
}

// 根據當前模式返回對應的描述字串
const char* TelemetrySystem::getModeString() {
    switch(_current_mode) {
        case PrintMode::HUMAN_STATUS: return "人類可讀狀態 (Human Status / Context-Aware)";
        case PrintMode::CSV_LOG:      return "CSV 日誌 (CSV Log)";
        case PrintMode::DASHBOARD:    return "儀表板 (Dashboard)";
        default:                      return "未知模式 (Unknown)";
    }
}

// 更新並打印數據的核心函式
void TelemetrySystem::updateAndPrint() {

    // 在函式開頭檢查暫停狀態
    if (_is_paused) {
        return; // 如果已暫停，則不執行任何打印操作
    }

    // 步驟 1: 從所有源頭收集最新數據
    collectData();

    // 步驟 2: 根據當前模式，呼叫對應的打印函式
    switch (_current_mode) {
        case PrintMode::HUMAN_STATUS:
            printAsHumanStatus();
            break;
        case PrintMode::CSV_LOG:
            // --- 修改：如果CSV標頭還沒打印過，就先打印新版標頭 ---
            if (!_csv_header_printed) {
                // --- Part 1: System & IMU Headers ---
                Serial.print("timestamp_ms,robot_mode,is_calibrated,roll_deg,pitch_deg,yaw_deg,acc_x_g,acc_y_g,acc_z_g,");
                
                // --- Part 2: Motor Position Headers ---
                for(int i=0; i < NUM_ROBOT_MOTORS; ++i) Serial.printf("pos_rad_%d,", i);
                
                // --- Part 3: Motor Velocity Headers ---
                for(int i=0; i < NUM_ROBOT_MOTORS; ++i) Serial.printf("vel_radps_%d,", i);

                // --- Part 4: Motor Target Current Headers ---
                for(int i=0; i < NUM_ROBOT_MOTORS; ++i) Serial.printf("target_curr_mA_%d,", i);

                // --- Part 5: Motor Actual Current Headers (last one with newline) ---
                for(int i=0; i < NUM_ROBOT_MOTORS - 1; ++i) Serial.printf("actual_curr_mA_%d,", i);
                Serial.printf("actual_curr_mA_%d\n", NUM_ROBOT_MOTORS - 1);

                _csv_header_printed = true; // 標記為已打印
            }
            printAsCsvLog();
            break;
        case PrintMode::DASHBOARD:
            printAsDashboard();
            break;

        // 將新模式掛載到主更新流程中
        case PrintMode::POLICY_STREAM:
            printAsPolicyStream();
            break;
    }
}


// 從各個模組收集數據並存儲到內部的 _telemetry_data 結構體中
void TelemetrySystem::collectData() {
    _telemetry_data.timestamp_ms = millis();
    _telemetry_data.robot_mode = _robot->getModeString();
    _telemetry_data.is_calibrated = _robot->isCalibrated();

    // 收集姿態數據
    _telemetry_data.roll = _ahrs->roll;
    _telemetry_data.pitch = _ahrs->pitch;
    _telemetry_data.yaw = _ahrs->yaw;

    // 收集運動學數據
    if (_ahrs) { // 防呆
        _telemetry_data.ahrs_linear_accel_g[0] = _ahrs->linearAccel[0];
        _telemetry_data.ahrs_linear_accel_g[1] = _ahrs->linearAccel[1];
        _telemetry_data.ahrs_linear_accel_g[2] = _ahrs->linearAccel[2];
        
        // 填充 AHRS 估算出的線速度
        _telemetry_data.ahrs_velocity_ms[0] = _ahrs->velocity[0];
        _telemetry_data.ahrs_velocity_ms[1] = _ahrs->velocity[1];
        _telemetry_data.ahrs_velocity_ms[2] = _ahrs->velocity[2];

        // [新增] 收集 AHRS 計算出的重力向量
        _telemetry_data.ahrs_gravity_vector[0] = _ahrs->gravityVector[0];
        _telemetry_data.ahrs_gravity_vector[1] = _ahrs->gravityVector[1];
        _telemetry_data.ahrs_gravity_vector[2] = _ahrs->gravityVector[2];
    }

    // 收集原始 IMU 數據
    if (_imu) { // 防呆
        _telemetry_data.imu_acc_g[0] = _imu->accG[0];
        _telemetry_data.imu_acc_g[1] = _imu->accG[1];
        _telemetry_data.imu_acc_g[2] = _imu->accG[2];

        // [新增] 收集原始的陀螺儀 dps 數據
        _telemetry_data.imu_gyro_dps[0] = _imu->gyroDPS[0];
        _telemetry_data.imu_gyro_dps[1] = _imu->gyroDPS[1];
        _telemetry_data.imu_gyro_dps[2] = _imu->gyroDPS[2];
    }
    
    // 收集所有馬達的數據
    for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
        _telemetry_data.motor_positions_rad[i] = _robot->getMotorPosition_rad(i);
        _telemetry_data.motor_velocities_rad_s[i] = _robot->getMotorVelocity_rad(i);
        _telemetry_data.target_currents_mA[i] = _robot->getTargetCurrent_mA(i);
        _telemetry_data.actual_currents_mA[i] = _motors->getRawCurrent_mA(i);

        // 填充每個馬達詳細的級聯控制資訊
        if (_robot) { // 防呆
             _telemetry_data.cascade_debug_infos[i] = _robot->getCascadeDebugInfo(i);
        }
    }
}

// 實現人類可讀的全局狀態打印 (會根據焦點自動調整)
void TelemetrySystem::printAsHumanStatus() {
    char buf[120];

    // --- Part 1: 通用系統狀態 (無論是否有焦點都會顯示) ---
    Serial.println("---------------- 機器人狀態 ----------------");
    snprintf(buf, sizeof(buf), "Last Cmd: %s", _last_command.c_str());
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "模式: %s | 校準: %s | 焦點: %s", 
             _telemetry_data.robot_mode, 
             _telemetry_data.is_calibrated ? "是" : "否",
             _focus_motor_id == -1 ? "全局" : String(_focus_motor_id).c_str());
    Serial.println(buf);
    
    // --- Part 2: 通用 AHRS 數據 (無論是否有焦點都會顯示) ---
    Serial.println("--- AHRS 姿態 (度) & 運動估計 ---");
    snprintf(buf, sizeof(buf), "姿態 (R/P/Y): %+7.2f, %+7.2f, %+7.2f",
             _telemetry_data.roll, _telemetry_data.pitch, _telemetry_data.yaw);
    Serial.println(buf);
    // 新增線性加速度和速度的顯示
    snprintf(buf, sizeof(buf), "線加速度(g) X/Y/Z: %+6.3f, %+6.3f, %+6.3f",
             _telemetry_data.ahrs_linear_accel_g[0], _telemetry_data.ahrs_linear_accel_g[1], _telemetry_data.ahrs_linear_accel_g[2]);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "線速度 (m/s) X/Y/Z: %+6.3f, %+6.3f, %+6.3f",
             _telemetry_data.ahrs_velocity_ms[0], _telemetry_data.ahrs_velocity_ms[1], _telemetry_data.ahrs_velocity_ms[2]);
    Serial.println(buf);


    // --- Part 3: 馬達數據 (情境感知) ---
    if (_focus_motor_id != -1) {
        // --- 情境 A: 有焦點馬達，顯示該馬達的超級詳細報告 ---
        int i = _focus_motor_id;
        const auto& info = _telemetry_data.cascade_debug_infos[i];

        Serial.printf("--- 焦點馬達 %d 詳細數據 ---\n", i);
        Serial.println("控制環      |    目標值    |    實際值    |     誤差     ");
        Serial.println("-------------------------------------------------------");
        snprintf(buf, sizeof(buf), "位置 [rad]  | %+12.4f | %+12.4f | %+12.4f",
                 info.target_pos_rad, info.current_pos_rad, info.pos_error_rad);
        Serial.println(buf);
        snprintf(buf, sizeof(buf), "速度 [rad/s]| %+12.4f | %+12.4f | %+12.4f",
                 info.target_vel_rad_s, info.current_vel_rad_s, info.vel_error_rad_s);
        Serial.println(buf);
        snprintf(buf, sizeof(buf), "電流 [mA]   | %-12d | %-12d |",
                 info.target_current_mA, _telemetry_data.actual_currents_mA[i]);
        Serial.println(buf);

    } else {
        // --- 情境 B: 無焦點馬達，顯示所有馬達的簡潔概覽 ---
        Serial.println("--- 馬達數據 (位置rad / 速度rad/s / 目標mA / 實際mA) ---");
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            snprintf(buf, sizeof(buf), "M%02d: %+6.2f, %+7.2f, %5d, %5d",
                     i, 
                     _telemetry_data.motor_positions_rad[i], 
                     _telemetry_data.motor_velocities_rad_s[i],
                     _telemetry_data.target_currents_mA[i], 
                     _telemetry_data.actual_currents_mA[i]);
            Serial.println(buf);
        }
    }
    Serial.println("=======================================================\n");
}

// 實現 CSV 格式的日誌打印 (會根據焦點自動調整)
void TelemetrySystem::printAsCsvLog() {
    // --- Part 1: System & IMU Data ---
    Serial.printf("%lu,", _telemetry_data.timestamp_ms);
    // 根據指南，去除 robot_mode 中的括號和中文，使其更易解析
    String mode_str = _telemetry_data.robot_mode;
    mode_str.replace(" (待機)", "");
    mode_str.replace(" (位置控制)", "");
    mode_str.replace(" (級聯控制)", "");
    mode_str.replace(" (擺動測試)", "");
    mode_str.replace(" (手動控制)", "");
    mode_str.replace(" (高層陣列控制)", "");
    mode_str.replace(" (錯誤)", "");
    Serial.print(mode_str + ",");
    Serial.printf("%d,", _telemetry_data.is_calibrated ? 1 : 0);
    
    // --- 修改：直接使用 Serial.print 打印浮點數，並手動添加逗號 ---
    Serial.print(_telemetry_data.roll, 4); Serial.print(",");
    Serial.print(_telemetry_data.pitch, 4); Serial.print(",");
    Serial.print(_telemetry_data.yaw, 4); Serial.print(",");

    Serial.print(_telemetry_data.imu_acc_g[0], 4); Serial.print(",");
    Serial.print(_telemetry_data.imu_acc_g[1], 4); Serial.print(",");
    Serial.print(_telemetry_data.imu_acc_g[2], 4); Serial.print(",");

    // --- Part 2: Motor Position Data ---
    for(int i=0; i < NUM_ROBOT_MOTORS; ++i) {
        Serial.print(_telemetry_data.motor_positions_rad[i], 4);
        Serial.print(",");
    }

    // --- Part 3: Motor Velocity Data ---
    for(int i=0; i < NUM_ROBOT_MOTORS; ++i) {
        Serial.print(_telemetry_data.motor_velocities_rad_s[i], 4);
        Serial.print(",");
    }
    
    // --- Part 4: Motor Target Current Data ---
    for(int i=0; i < NUM_ROBOT_MOTORS; ++i) {
        Serial.printf("%d,", _telemetry_data.target_currents_mA[i]);
    }

    // --- Part 5: Motor Actual Current Data (last one with newline) ---
    for(int i=0; i < NUM_ROBOT_MOTORS - 1; ++i) {
        Serial.printf("%d,", _telemetry_data.actual_currents_mA[i]);
    }
    Serial.printf("%d\n", _telemetry_data.actual_currents_mA[NUM_ROBOT_MOTORS - 1]);
}


// 實現單馬達焦點監控的儀表板打印
void TelemetrySystem::printAsDashboard() {
    // [保留] 此函式目前保持不變。
    // 雖然它的功能已被整合到新的 human 模式中，但保留它可以向下相容，
    // 或許未來有其他用途。
    if (_focus_motor_id == -1) {
        Serial.println("[提示] 儀表板模式需要一個焦點馬達。請使用 'focus <id>' 指令設定。");
        _current_mode = PrintMode::HUMAN_STATUS;
        return;
    }
    
    // 從遙測數據中直接獲取資訊，而不是再次呼叫 _robot->getCascadeDebugInfo
    const auto& info = _telemetry_data.cascade_debug_infos[_focus_motor_id];

    char buf[120];
    
    Serial.println("--- 單馬達儀表板 ---");
    snprintf(buf, sizeof(buf), "Last Cmd: %s", _last_command.c_str());
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "監控馬達 ID: %d | 機器人模式: %s", _focus_motor_id, _robot->getModeString());
    Serial.println(buf);
    Serial.println("-----------------------------------------------------------------");
    Serial.println("控制環      |    目標值    |    實際值    |     誤差     ");
    Serial.println("-----------------------------------------------------------------");
    snprintf(buf, sizeof(buf), "位置 [rad]  | %+12.4f | %+12.4f | %+12.4f",
             info.target_pos_rad, info.current_pos_rad, info.pos_error_rad);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "速度 [rad/s]| %+12.4f | %+12.4f | %+12.4f",
             info.target_vel_rad_s, info.current_vel_rad_s, info.vel_error_rad_s);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "電流 [mA]    | %-12d | %-12d |",
             info.target_current_mA, _telemetry_data.actual_currents_mA[_focus_motor_id]);
    Serial.println(buf);
    Serial.println("-----------------------------------------------------------------");
}

// 核心實現：為 AI 模型打印嚴格格式的決策輸入流
void TelemetrySystem::printAsPolicyStream() {
    // 該函式不打印任何標頭或非數字字元。
    // 嚴格按照 AI 輸入「契約」順序輸出，並在輸出時進行最終的單位轉換。
    // 所有數據都來自已填充的 _telemetry_data 結構體，確保數據同步。
    
    // ==============================================================
    // === [數據契約 v1.1] 根據 joystick.py 的 state_obs 更新 ===
    // ==============================================================
    // 1. Angular Velocity (3維, rad/s)
    // 2. Gravity Vector   (3維, g-normalized)
    // 3. Accelerometer    (3維, m/s^2)
    // 4. Pitch Angle      (1維, rad)
    // 5. Joint Positions  (12維, rad)
    // 6. Joint Velocities (12維, rad/s)
    // (總維度: 3 + 3 + 3 + 1 + 12 + 12 = 34 維)
    // ==============================================================

    // 1. 角速度 (3維, rad/s)
    // 從 dps 轉換為 rad/s
    Serial.print(_telemetry_data.imu_gyro_dps[0] * DEG_TO_RAD, 6); Serial.print(",");
    Serial.print(_telemetry_data.imu_gyro_dps[1] * DEG_TO_RAD, 6); Serial.print(",");
    Serial.print(_telemetry_data.imu_gyro_dps[2] * DEG_TO_RAD, 6); Serial.print(",");

    // 2. 重力向量 (3維, g-normalized)
    // 直接使用已在 AHRS 中計算好的標準重力向量
    Serial.print(_telemetry_data.ahrs_gravity_vector[0], 6); Serial.print(",");
    Serial.print(_telemetry_data.ahrs_gravity_vector[1], 6); Serial.print(",");
    Serial.print(_telemetry_data.ahrs_gravity_vector[2], 6); Serial.print(",");

    // 3. 加速度計 (3維, m/s^2)
    // 從 g 轉換為 m/s^2
    // 注意：這裡使用的是未經座標系修正的原始 IMU 加速度，
    // 因為 joystick.py 中也是這樣做的 (`self.get_accelerometer(data)`)。
    Serial.print(_telemetry_data.imu_acc_g[0] * G_ACCEL, 6); Serial.print(",");
    Serial.print(_telemetry_data.imu_acc_g[1] * G_ACCEL, 6); Serial.print(",");
    Serial.print(_telemetry_data.imu_acc_g[2] * G_ACCEL, 6); Serial.print(",");

    // 4. 俯仰角 (1維, rad)
    // 從 度 轉換為 rad
    Serial.print(_telemetry_data.pitch * DEG_TO_RAD, 6); Serial.print(",");

    // 5. 關節位置 (12維, rad)
    for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
        Serial.print(_telemetry_data.motor_positions_rad[i], 6);
        Serial.print(",");
    }
    
    // 6. 關節速度 (12維, rad/s)
    for (int i = 0; i < NUM_ROBOT_MOTORS - 1; ++i) {
        Serial.print(_telemetry_data.motor_velocities_rad_s[i], 6);
        Serial.print(",");
    }
    // 最後一筆數據使用 println 來結束一行，作為訊息的終結符
    Serial.println(_telemetry_data.motor_velocities_rad_s[NUM_ROBOT_MOTORS - 1], 6);
}
