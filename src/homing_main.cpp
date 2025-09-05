// --- START OF FILE homing_main.cpp ---

// src/imu_motor_test_main.cpp

#include <Arduino.h>
#include <MotorController.h>
#include "RobotController.h"
#include <LSM6DSO_SPI.h>
#include "AHRS.h"
#include "TelemetrySystem.h"
#include "CommandHandler.h"

// --- 建立全域物件 ---
LSM6DSO         myIMU;
MotorController myMotorControl;
RobotController myRobot(&myMotorControl);
SimpleAHRS      myAHRS;
TelemetrySystem myTelemetry(&myRobot, &myAHRS, &myMotorControl, &myIMU); 
CommandHandler  myCommander;

// =================================================================
// --- 固定頻率控制設定 ---
// =================================================================
const int CONTROL_FREQUENCY_HZ = 1000;
const long CONTROL_INTERVAL_MICROS = 1000000 / CONTROL_FREQUENCY_HZ;
unsigned long last_control_time_micros = 0;

const int PRINT_FREQUENCY_HZ = 2;
unsigned long g_print_interval_millis = 1000 / 2;
unsigned long last_print_time_millis = 0;


// =================================================================
// --- 新的、可靠的序列埠指令讀取邏輯 ---
// =================================================================
const int SERIAL_BUFFER_SIZE = 128;   // 指令緩衝區最大長度
char serial_buffer[SERIAL_BUFFER_SIZE]; // 用來儲存傳入字元的緩衝區
int serial_buffer_pos = 0;            // 目前寫入到緩衝區的位置

void checkAndProcessSerial(); // 檢查並處理序列埠數據的新函式
//void printRobotStatus();

// =================================================================
//   SETUP - 程式啟動時只會執行一次
// =================================================================
void setup() {
    // ... (setup 函式的內容完全不變，從 Serial.begin() 到結束)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);

    Serial.begin(115200);
    while (!Serial && millis() < 4000);

    Serial.println("\n--- Pupper Robot Control System ---");

    if (myIMU.begin()) {
        Serial.println("[SUCCESS] IMU Initialized.");
        myAHRS.begin(CONTROL_FREQUENCY_HZ);
    } else {
        Serial.println("[FAILURE] IMU Initialization Failed! Halting.");
        while(1);
    }
    
    myMotorControl.begin();
    Serial.println("[SUCCESS] Motor Controller Initialized.");
    
    myRobot.begin();
    Serial.println("[SUCCESS] Robot Controller Initialized.");
    myTelemetry.begin();
    Serial.println("[SUCCESS] Telemetry System Initialized.");
    myCommander.begin(&myRobot, &myTelemetry);
    Serial.println("[SUCCESS] Command Handler Initialized.");

    Serial.println("========================================");
    Serial.println(" Pupper Robot :: Command Set v3.3");
    Serial.println("========================================");
    Serial.println("--- Movement ---");
    Serial.println("  stand                     - Enter stable standing pose (tested values).");
    Serial.println("  zero                      - Move to the calibrated zero pose (functional home).");
    Serial.println("  move m<id> <rad>          - Absolute move for a motor.");
    Serial.println("  move m<id> += <rad>       - Relative move for a motor.");
    Serial.println("  move g<h|u|l> <rad>       - Move a joint group (hip, upper, lower).");
    Serial.println("  move gl<0-3> <h> <u> <l>  - Set a single leg's joints.");
    Serial.println("  move g<f|r> <h> <u> <l>   - Set leg pair joints (front/rear).");
    Serial.println("  move all <rad*12>         - Set all 12 joint angles.");
    Serial.println("");
    Serial.println("--- Parameters ---");
    Serial.println("  set <target> <p> <v>      - Set a param (p: c,kp,ki,max_vel,max_err).");
    Serial.println("  get <target> [source]     - Get params for target.");
    Serial.println("  reset <target> [p]        - Reset params for target.");
    Serial.println("  (Target: all, global, m<id>, g<name>)");
    Serial.println("");
    Serial.println("--- Telemetry & Monitoring ---");
    Serial.println("  status                    - Print a one-time full status report.");
    Serial.println("  monitor <h|c|d>           - Set telemetry format (human, csv, dashboard).");
    Serial.println("  monitor freq <hz>         - Set telemetry frequency.");
    Serial.println("  monitor <pause|resume>    - Pause/Resume telemetry stream.");
    Serial.println("  focus <m<id>|off>         - Set/unset focus motor for telemetry.");
    Serial.println("");
    Serial.println("--- System & Testing ---");
    Serial.println("  cal                       - Perform manual calibration.");
    Serial.println("  stop                      - Stop all motors (enter IDLE mode).");
    Serial.println("  reboot                    - Reboot the microcontroller.");
    Serial.println("  raw m<id> <mA>            - Manually set raw current.");
    Serial.println("  test wiggle m<id>         - Start wiggle test for a motor.");
    Serial.println("========================================\n");


    
    digitalWrite(LED_BUILTIN, LOW);
}


// =================================================================
//   LOOP - 採用新的指令處理邏輯
// =================================================================
void loop() {
    // -------------------------------------------------
    // 1. 高頻任務 (每個迴圈都執行)
    // -------------------------------------------------
    myMotorControl.pollAll(); 
    
    // *** 修改: 呼叫新的指令處理函式，取代舊的邏輯 ***
    checkAndProcessSerial();

    // -------------------------------------------------
    // 2. 固定頻率的控制任務 (例如 1000Hz)
    // -------------------------------------------------
    unsigned long current_micros = micros();
    if (current_micros - last_control_time_micros >= CONTROL_INTERVAL_MICROS) {
        last_control_time_micros = current_micros;
        
        myIMU.readSensor(); //從 myAHRS 物件中獲取姿態角
        myAHRS.update(myIMU.gyroDPS[0], myIMU.gyroDPS[1], myIMU.gyroDPS[2], myIMU.accG[0], myIMU.accG[1], myIMU.accG[2]);
        
        myRobot.update();
    }

    // -------------------------------------------------
    // 3. 低頻率的數據打印任務 (例如 1Hz)
    // -------------------------------------------------
    unsigned long current_millis = millis();
    if (current_millis - last_print_time_millis >= g_print_interval_millis) {
        last_print_time_millis = current_millis;

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        myTelemetry.updateAndPrint();
    }
}


// =================================================================
//   輔助函式 (Helper Functions)
// =================================================================

/**
 * @brief 檢查序列埠是否有數據，並在接收到換行符時處理指令。
 *        這個函式取代了舊的 loop() 中的 Serial.readStringUntil() 邏輯。
 */
void checkAndProcessSerial() {
    while (Serial.available() > 0) {
        char incoming_char = Serial.read();

        // 如果收到換行符 '\n' (Enter 鍵)
        if (incoming_char == '\n') {
            // 在緩衝區末端加上字串結束符
            serial_buffer[serial_buffer_pos] = '\0';
            
            // 將緩衝區內容轉換為 String 物件來處理
            String command = String(serial_buffer);
            command.trim(); // 去掉前後的空白

            // 只有在指令非空時才處理
            if (command.length() > 0) {
                myCommander.executeCommand(command);
            }

            // 重置緩衝區，準備接收下一條指令
            serial_buffer_pos = 0;
            serial_buffer[0] = '\0';

        } 
        // 如果收到退格鍵 (Backspace)
        else if (incoming_char == '\b' || incoming_char == 127) {
            if (serial_buffer_pos > 0) {
                serial_buffer_pos--; // 緩衝區位置後退一格
            }
        }
        // 如果是普通字元，且緩衝區未滿
        else if (serial_buffer_pos < SERIAL_BUFFER_SIZE - 1) {
            // 將字元存入緩衝區，並移動位置
            serial_buffer[serial_buffer_pos] = incoming_char;
            serial_buffer_pos++;
        }
    }
}