// --- START OF FILE homing_main.cpp ---

// src/imu_motor_test_main.cpp

#include <Arduino.h>
#include <MotorController.h>
#include "RobotController.h"
#include <LSM6DSO_SPI.h>

// --- 建立全域物件 ---
LSM6DSO         myIMU;
MotorController myMotorControl;
RobotController myRobot(&myMotorControl);

// =================================================================
// --- 固定頻率控制設定 ---
// =================================================================
const int CONTROL_FREQUENCY_HZ = 1000;
const long CONTROL_INTERVAL_MICROS = 1000000 / CONTROL_FREQUENCY_HZ;
unsigned long last_control_time_micros = 0;

const int PRINT_FREQUENCY_HZ = 1;
const long PRINT_INTERVAL_MILLIS = 1000 / PRINT_FREQUENCY_HZ;
long last_print_time_millis = 0;

// =================================================================
// --- 新的、可靠的序列埠指令讀取邏輯 ---
// =================================================================
const int SERIAL_BUFFER_SIZE = 128;   // 指令緩衝區最大長度
char serial_buffer[SERIAL_BUFFER_SIZE]; // 用來儲存傳入字元的緩衝區
int serial_buffer_pos = 0;            // 目前寫入到緩衝區的位置

void checkAndProcessSerial(); // 檢查並處理序列埠數據的新函式
void handleSerialCommand(String command);
void printRobotStatus();

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
    } else {
        Serial.println("[FAILURE] IMU Initialization Failed! Halting.");
        while(1);
    }
    
    myMotorControl.begin();
    Serial.println("[SUCCESS] Motor Controller Initialized.");
    
    myRobot.begin();
    Serial.println("[SUCCESS] Robot Controller Initialized.");
    Serial.println("----------------------------------------");
    
    Serial.println("Commands (press Enter to execute):"); // 提示使用者要按 Enter
    Serial.println("  --- High-level ---");
    Serial.println("  home          - Start automatic homing sequence.");
    Serial.println("  pos <id> <rad> - Set target position for one motor (e.g., 'pos 0 1.57').");
    Serial.println("  wiggle <id>   - Start wiggle test for a motor.");
    Serial.println("  --- Manual & Stop ---");
    Serial.println("  motor <id> <mA> - Manually set current for one motor (e.g., 'motor 0 500').");
    Serial.println("  stop          - Stop all motors and enter IDLE mode.");
    Serial.println("  reboot        - Reboot the microcontroller.");
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
        
        myIMU.readSensor();
        myRobot.update();
    }

    // -------------------------------------------------
    // 3. 低頻率的數據打印任務 (例如 1Hz)
    // -------------------------------------------------
    unsigned long current_millis = millis();
    if (current_millis - last_print_time_millis >= PRINT_INTERVAL_MILLIS) {
        last_print_time_millis = current_millis;

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        printRobotStatus();
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
                handleSerialCommand(command);
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


/**
 * @brief 處理從序列埠接收到的指令 (此函式內容不變)
 * @param command 從序列埠讀取到的字串指令
 */
void handleSerialCommand(String command) {
    if (command == "home") {
        Serial.println("--> Command received: [home]");
        myRobot.startHoming();

    } else if (command.startsWith("pos ")) {
        Serial.print("--> Command received: [");
        Serial.print(command);
        Serial.println("]");
        
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            float angle_rad = command.substring(space2 + 1).toFloat();
            myRobot.setTargetPosition_rad(motorID, angle_rad);
        } else {
            Serial.println("  [ERROR] Invalid format. Use 'pos <id> <angle_in_radians>'.");
        }
        
    } else if (command.startsWith("wiggle ")) {
        int motorID = command.substring(7).toInt();
        Serial.printf("--> Command received: [wiggle %d]\n", motorID);
        myRobot.startWiggleTest(motorID);

    } else if (command.startsWith("motor ")) {
        Serial.print("--> Command received: [");
        Serial.print(command);
        Serial.println("]");
        
        int space1 = command.indexOf(' ');
        int space2 = command.indexOf(' ', space1 + 1);

        if (space1 != -1 && space2 != -1) {
            int motorID = command.substring(space1 + 1, space2).toInt();
            int current = command.substring(space2 + 1).toInt();
            myRobot.setSingleMotorCurrent(motorID, current);
        } else {
            Serial.println("  [ERROR] Invalid format. Use 'motor <id> <current_in_mA>'.");
        }
    
    } else if (command == "stop") {
        Serial.println("--> Command received: [stop]");
        myRobot.setIdle();

    } else if (command == "reboot") {
        Serial.println("--> Command received: [reboot]. Rebooting now...");
        delay(100);
        #ifdef __arm__
        SCB_AIRCR = 0x05FA0004;
        #endif

    } else {
        Serial.print("  [ERROR] Unknown command: ");
        Serial.println(command);
    }
}

/**
 * @brief 打印所有相關的狀態和數據到序列埠 (此函式內容不變)
 */
void printRobotStatus() {
    char buf[100];

    Serial.println("---------------- ROBOT STATUS ----------------");

    Serial.print("Robot Mode: ");
    Serial.println(myRobot.getModeString());

    snprintf(buf, sizeof(buf), "IMU Acc(g) -> X:%+7.3f Y:%+7.3f Z:%+7.3f", myIMU.accG[0], myIMU.accG[1], myIMU.accG[2]);
    Serial.println(buf);
    snprintf(buf, sizeof(buf), "IMU Gyro(dps)-> X:%+7.3f Y:%+7.3f Z:%+7.3f", myIMU.gyroDPS[0], myIMU.gyroDPS[1], myIMU.gyroDPS[2]);
    Serial.println(buf);
    Serial.println("---");

    Serial.println("Motor Data (ID | Calib. Pos (rad) | Velocity (radps) | Offset (rad))"); 
    for (int i = 0; i < NUM_ROBOT_MOTORS; i++) {
        float pos_rad = myRobot.getMotorPosition_rad(i);
        float vel_rad = myRobot.getMotorVelocity_rad(i);
        float offset_rad = myMotorControl.getOffset_rad(i);

        snprintf(buf, sizeof(buf), "Motor %2d | Pos: %+9.4f | Vel: %+9.4f | Offset: %+9.4f", i, pos_rad, vel_rad, offset_rad);
        Serial.println(buf);
    }
    Serial.println("================================================\n");
}