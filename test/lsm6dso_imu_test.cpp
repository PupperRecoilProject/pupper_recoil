// --- START OF FILE src/main.cpp ---

#include <Arduino.h>
#include "DriveSystem.h" // 引入我們修改好的 DriveSystem

// --- 全域設定 ---

// 1. 定義 IMU 的 Chip Select (CS) 腳位
//    請根據你的硬體接線修改此處
const uint8_t IMU_CS_PIN = 8; 

// 2. 建立 DriveSystem 物件
//    在建立物件時，將 CS 腳位傳遞給構造函式
DriveSystem myRobotDrive(IMU_CS_PIN);

// 3. 設定打印間隔 (單位: 毫秒)
const unsigned long PRINT_INTERVAL_MS = 200;
unsigned long lastPrintTime = 0;

void setup() {
  // 初始化內建 LED，用於視覺回饋
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // 開機時先亮燈

  // 初始化序列埠用於監控
  Serial.begin(115200);
  // 等待序列埠連接 (最多等4秒)
  while (!Serial && millis() < 4000); 

  Serial.println("\n--- DriveSystem with LSM6DSO Integration Test ---");

  // --- 初始化 DriveSystem ---
  // 這一步會自動呼叫內部的 SetupIMU()，我們來驗證它的輸出
  Serial.println("Initializing Drive System (which includes IMU)...");
  
  // 傳入的 filter_frequency 參數目前沒有作用，但為了保持 API 一致性，我們還是傳一個值
  myRobotDrive.SetupIMU(100); 
  
  // 檢查 IMU 是否初始化成功。如果 SetupIMU 內部偵測到錯誤，
  // 它會將 control_mode_ 設為 kError。我們這裡不做複雜檢查，
  // 而是直接看序列埠的輸出 ("IMU initialized successfully" 或 "failed")。
  
  Serial.println("Setup complete. Starting main loop...");
  Serial.println("----------------------------------------");
  digitalWrite(LED_BUILTIN, LOW); // 初始化完成，關燈
}

void loop() {
  // 1. 在每個迴圈中，更新 IMU 數據
  //    這會呼叫內部的 imu.readSensor()
  myRobotDrive.UpdateIMU();

  // (在一個完整的應用中，你還會在這裡呼叫 myRobotDrive.Update() 來執行馬達控制)
  // myRobotDrive.Update();

  // 2. 使用非阻塞式延遲，每隔一段時間打印一次狀態
  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    
    // 閃爍 LED，表示程式正在運行
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
    
    Serial.println("--- IMU Sensor Data ---");

    // 取得 DriveSystem 內部 IMU 物件的參考，方便存取。透過公開的 getter 函式取得
    LSM6DSO& imu = myRobotDrive.getIMU(); 

    // 使用 sprintf 格式化輸出，對齊更美觀
    char buffer[120];
    sprintf(buffer, " Accel (g):   X=%+7.3f, Y=%+7.3f, Z=%+7.3f", 
            imu.accG[0], imu.accG[1], imu.accG[2]);
    Serial.println(buffer);

    sprintf(buffer, " Gyro (dps):  X=%+8.3f, Y=%+8.3f, Z=%+8.3f", 
            imu.gyroDPS[0], imu.gyroDPS[1], imu.gyroDPS[2]);
    Serial.println(buffer);
    
    Serial.println("========================================");
  }
}

// --- END OF FILE src/main.cpp ---