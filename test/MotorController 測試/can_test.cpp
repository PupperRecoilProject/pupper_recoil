#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Can0.begin();
  Can0.setBaudRate(1000000); // 1 Mbps for C610
  
  Serial.println("Simple CAN Bus Test");
}

void loop() {
  CAN_message_t msg;
  msg.id = 0x201; // C610 的一個馬達 ID
  msg.len = 8;
  for (int i = 0; i < 8; i++) {
    msg.buf[i] = 0; // 發送全零的力矩指令
  }

  // 嘗試發送訊息
  int write_status = Can0.write(msg);

  // 打印發送狀態
  // 如果是 0 或 1，表示成功。如果是 -1 或其他負數，表示失敗。
  Serial.print("CAN write status: ");
  Serial.println(write_status);
  
  delay(500);
}