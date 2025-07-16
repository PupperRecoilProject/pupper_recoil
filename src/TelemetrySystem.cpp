// src/TelemetrySystem.cpp

#include "TelemetrySystem.h"
#include <Arduino.h>

// 構造函式，使用初始化列表來設定所有成員變數的初始值
TelemetrySystem::TelemetrySystem(RobotController* robot, SimpleAHRS* ahrs, MotorController* motors)
    : _robot(robot), 
      _ahrs(ahrs), 
      _motors(motors),
      _current_mode(PrintMode::HUMAN_STATUS), // 預設啟動時為人類可讀模式
      _focus_motor_id(-1),                    // 焦點馬達ID初始化為無效值-1
      _csv_header_printed(false)              // CSV標頭尚未打印
{
}

void TelemetrySystem::begin() {
    // 目前不需要任何特別的初始化操作，保留此函式以備未來擴充
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
        case PrintMode::HUMAN_STATUS: return "人類可讀狀態 (Human Status)";
        case PrintMode::CSV_LOG:      return "CSV 日誌 (CSV Log)";
        case PrintMode::DASHBOARD:    return "儀表板 (Dashboard)";
        default:                      return "未知模式 (Unknown)";
    }
}

// 更新並打印數據的核心函式
void TelemetrySystem::updateAndPrint() {
    // 步驟 1: 從所有源頭收集最新數據
    collectData();

    // 步驟 2: 根據當前模式，呼叫對應的打印函式
    switch (_current_mode) {
        case PrintMode::HUMAN_STATUS:
            printAsHumanStatus();
            break;
        case PrintMode::CSV_LOG:
            // 如果CSV標頭還沒打印過，就先打印標頭
            if (!_csv_header_printed) {
                Serial.println("timestamp_ms,robot_mode,is_calibrated,roll,pitch,yaw,motor_id,pos_rad,vel_rad_s,target_curr_mA,actual_curr_mA");
                _csv_header_printed = true; // 標記為已打印
            }
            printAsCsvLog();
            break;
        case PrintMode::DASHBOARD:
            printAsDashboard();
            break;
    }
}

// 從各個模組收集數據並存儲到內部的 _telemetry_data 結構體中
void TelemetrySystem::collectData() {
    _telemetry_data.timestamp_ms = millis();
    _telemetry_data.robot_mode = _robot->getModeString();
    _telemetry_data.is_calibrated = _robot->isCalibrated();

    _telemetry_data.roll = _ahrs->roll;
    _telemetry_data.pitch = _ahrs->pitch;
    _telemetry_data.yaw = _ahrs->yaw;

    for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
        _telemetry_data.motor_positions_rad[i] = _robot->getMotorPosition_rad(i);
        _telemetry_data.motor_velocities_rad_s[i] = _robot->getMotorVelocity_rad(i);
        _telemetry_data.target_currents_mA[i] = _robot->getTargetCurrent_mA(i);
        _telemetry_data.actual_currents_mA[i] = _motors->getRawCurrent_mA(i);
    }
}

// 實現人類可讀的全局狀態打印 (會根據焦點自動調整)
void TelemetrySystem::printAsHumanStatus() {
    char buf[120];

    Serial.println("---------------- 機器人狀態 ----------------");
    snprintf(buf, sizeof(buf), "機器人模式: %s | 是否校準: %s | 焦點: %s", 
             _telemetry_data.robot_mode, 
             _telemetry_data.is_calibrated ? "是" : "否",
             _focus_motor_id == -1 ? "全局" : String(_focus_motor_id).c_str());
    Serial.println(buf);
    
    Serial.println("--- AHRS 姿態 (度) ---");
    snprintf(buf, sizeof(buf), "翻滾角: %+7.2f | 俯仰角: %+7.2f | 偏航角: %+7.2f",
             _telemetry_data.roll, _telemetry_data.pitch, _telemetry_data.yaw);
    Serial.println(buf);
    
    Serial.println("--- 馬達數據 (位置rad / 速度radps / 目標電流mA / 實際電流mA) ---");
    if (_focus_motor_id != -1) {
        // 如果有焦點，只打印該馬達的數據
        int i = _focus_motor_id;
        snprintf(buf, sizeof(buf), "M%02d: %+6.2f, %+6.2f, %5d, %5d",
                 i, _telemetry_data.motor_positions_rad[i], _telemetry_data.motor_velocities_rad_s[i],
                 _telemetry_data.target_currents_mA[i], _telemetry_data.actual_currents_mA[i]);
        Serial.println(buf);
    } else {
        // 如果沒有焦點，打印所有馬達的數據
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            snprintf(buf, sizeof(buf), "M%02d: %+6.2f, %+6.2f, %5d, %5d",
                     i, _telemetry_data.motor_positions_rad[i], _telemetry_data.motor_velocities_rad_s[i],
                     _telemetry_data.target_currents_mA[i], _telemetry_data.actual_currents_mA[i]);
            Serial.println(buf);
        }
    }
    Serial.println("========================================================\n");
}

// 實現 CSV 格式的日誌打印 (會根據焦點自動調整)
void TelemetrySystem::printAsCsvLog() {
    if (_focus_motor_id != -1) {
        // --- 單馬達 CSV 日誌 ---
        int i = _focus_motor_id;
        Serial.printf("%lu,%s,%d,%.2f,%.2f,%.2f,%d,%.4f,%.4f,%d,%d\n",
            _telemetry_data.timestamp_ms, _telemetry_data.robot_mode, _telemetry_data.is_calibrated,
            _telemetry_data.roll, _telemetry_data.pitch, _telemetry_data.yaw,
            i, _telemetry_data.motor_positions_rad[i], _telemetry_data.motor_velocities_rad_s[i],
            _telemetry_data.target_currents_mA[i], _telemetry_data.actual_currents_mA[i]);
    } else {
        // --- 全局馬達 CSV 日誌 ---
        for (int i = 0; i < NUM_ROBOT_MOTORS; ++i) {
            Serial.printf("%lu,%s,%d,%.2f,%.2f,%.2f,%d,%.4f,%.4f,%d,%d\n",
                _telemetry_data.timestamp_ms, _telemetry_data.robot_mode, _telemetry_data.is_calibrated,
                _telemetry_data.roll, _telemetry_data.pitch, _telemetry_data.yaw,
                i, _telemetry_data.motor_positions_rad[i], _telemetry_data.motor_velocities_rad_s[i],
                _telemetry_data.target_currents_mA[i], _telemetry_data.actual_currents_mA[i]);
        }
    }
}

// 實現單馬達焦點監控的儀表板打印
void TelemetrySystem::printAsDashboard() {
    // 儀表板模式必須有焦點馬達
    if (_focus_motor_id == -1) {
        Serial.println("[提示] 儀表板模式需要一個焦點馬達。請使用 'focus <id>' 指令設定。");
        // 為了避免一直刷屏，我們可以將模式臨時切換回 human
        _current_mode = PrintMode::HUMAN_STATUS;
        return;
    }

    // 從 RobotController 獲取該馬達詳細的、級聯控制相關的調試數據
    CascadeDebugInfo info = _robot->getCascadeDebugInfo(_focus_motor_id);

    char buf[120];
    
    // 使用 VT100/ANSI 轉義序列來清空終端屏幕並將光標移動到左上角
    Serial.println("--- 單馬達儀表板 ---");
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
             info.target_current_mA, _motors->getRawCurrent_mA(_focus_motor_id));
    Serial.println(buf);
    Serial.println("-----------------------------------------------------------------");
}