#include <Arduino.h>
#include <Streaming.h>

#include "CommandInterpreter.h" // 需要它來解析指令
#include "DriveSystem.h" // 核心驅動系統
#include "Utils.h"

////////////////////// CONFIG ///////////////////////
// 校準模式下的電流可以設低一點，增加安全性
const float CALIBRATION_CURRENT = 2.0; 
////////////////////// END CONFIG ///////////////////////

// 建立 DriveSystem 物件，這是與硬體溝通的核心
DriveSystem drive;

// 建立 CommandInterpreter 物件來接收來自序列埠的指令
CommandInterpreter interpreter(true);

// 函式宣告，讓 setup 和 loop 可以呼叫
void waitForUserInput(const char* message);

void setup(void) {
  Serial.begin(500000);
  pinMode(LED_BUILTIN, OUTPUT); // 使用內建 LED 燈作為狀態指示

  // 閃爍LED燈，表示進入了校準模式
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  Serial.println("=========================================");
  Serial.println("==        PUPPER CALIBRATION MODE      ==");
  Serial.println("=========================================");
  Serial.println();

  // ------------------ 步驟 1: 等待使用者準備 ------------------
  waitForUserInput(">>> STEP 1: Please manually place the robot legs in the ZERO position. Press Enter in Serial Monitor when ready...");
  
  // ------------------ 步驟 2: 啟動馬達並設定安全電流 ------------------
  Serial.println("\n[INFO] Activating motors with low current to hold position...");
  drive.SetActivations({1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  drive.SetMaxCurrent(CALIBRATION_CURRENT);
  drive.SetIdle(); // 先設定為閒置模式，但不發送電流
  
  // 必須呼叫 Update() 幾次，讓CAN bus上的指令生效
  for (int i=0; i<10; ++i) {
    drive.CheckForCANMessages();
    drive.Update();
    delay(1);
  }
  
  Serial.println("[SUCCESS] Motors activated. Holding position.");
  Serial.println();

  // ------------------ 步驟 3: 等待最終確認並執行校準 ------------------
  waitForUserInput(">>> STEP 2: Robot is now holding the zero position. Final check. Press Enter to perform calibration...");
  
  Serial.println("\n[INFO] Performing zero-point calibration...");
  
  // 呼叫核心校準函式
  drive.ZeroCurrentPosition();
  
  Serial.println("[SUCCESS] Calibration complete! The new zero offsets have been stored.");
  Serial.println("[INFO] You can now power off the Teensy or flash the main firmware.");
  Serial.println();
  
  // 校準完成後，閃爍LED燈表示成功，然後進入無限迴圈
  // 避免執行任何其他動作
}

void loop() {
  // 校準完成後，loop 函式可以是空的，或者只做一些無害的事情，比如閃爍LED燈
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

// 輔助函式：等待使用者在序列監視器中輸入並按下Enter
void waitForUserInput(const char* message) {
    Serial.println(message);
    while (Serial.available() == 0) {
        // 閃爍LED，表示正在等待使用者
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
    }
    // 清空序列緩衝區，準備下一次輸入
    while(Serial.available() > 0) {
        Serial.read();
    }
}