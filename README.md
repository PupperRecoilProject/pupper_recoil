# Pupper Robot Control System

這是一個為四足機器人設計的高性能、模組化控制系統韌體。它運行在 Arduino 相容的微控制器上，提供即時的姿態控制、參數調校和豐富的遙測功能。

<!-- 可選：在這裡放一張機器人的酷炫照片或 GIF 動畫 -->
![Pupper Robot](images/robot-image.jpg)

## ✨ 核心特性 (Features)

*   **高頻率控制迴圈**：基於 1000Hz 的固定頻率，實現穩定的級聯（位置-速度）控制器。
*   **動態參數系統**：支援三層級（預設、全域、單一馬達）的參數覆蓋，提供極致的調校彈性。
*   **互動式指令介面**：透過序列埠提供豐富的文字指令集，用於即時控制與除錯。
*   **多模式遙測系統**：內建人類可讀、CSV 日誌和即時儀表板三種監控模式。
*   **模組化設計**：基於 C++ 類別，實現高度內聚、低耦合的專業軟體架構。

---

## 🚀 快速入門 (Quick Start)

1.  **硬體連接**：將微控制器透過 USB 連接到電腦。
2.  **編譯與上傳**：使用 PlatformIO 或 Arduino IDE 編譯並上傳本專案。
3.  **打開序列監控視窗**：設定波特率為 `115200`。您將看到系統啟動訊息和完整的指令清單。
4.  **執行校準**：
    *   手動將機器人擺放成雙腿與地面垂直的標準站姿。
    *   在序列監控中輸入 `cal` 並按下 Enter。
5.  **站立與控制**：
    *   輸入 `stand` 讓機器人進入站立姿態。
    *   現在，您可以開始使用 `move` 等指令來控制機器人了！

---

## 📖 指令參考手冊 (Command Reference - v3.4)

### 語法提示 (Syntax Legend)
*   `<variable>` : 表示需要使用者替換的變數
*   `[optional]` : 表示可選的參數
*   `(a|b|c)`    : 表示從 a, b, c 中擇一選擇

---
### 核心運動 (Movement & Posing)
*   `stand`
    *   描述：進入手動校準時設定的站立姿態。
*   `move m<id> <rad>`
    *   描述：[絕對] 將馬達 `<id>` 移動到目標角度 `<rad>`。
    *   範例：`move m0 1.57`
*   `move m<id> += <rad>`
    *   描述：[相對] 將馬達 `<id>` 在當前目標上增加 `<rad>`。
    *   範例：`move m0 += -0.2`
*   `move all <rad*12>`
    *   描述：[絕對] 一次性設定所有 12 個關節的角度。
    *   範例：`move all 0 0 0 0 0 0 0 0 0 0 0 0`

---
### 關節組控制 (Joint Group Control)
*   `move g<h|u|l> <rad>`
    *   描述：移動一個關節功能組 (h:髖部, u:大腿, l:小腿)。
    *   範例：`move gu -1.2`
*   `move gl<0-3> <h> <u> <l>`
    *   描述：設定單一腿(0-3)的三個關節角度。
    *   範例：`move gl0 0 -1.2 -2.5`
*   `move g<f|r> <h> <u> <l>`
    *   描述：設定一對腿(f:前腿, r:後腿)的關節角度。
    *   範例：`move gf 0 -1.2 -2.5`

---
### 參數調校 (Parameter Tuning)
*   `set <target> <p> <v>`
    *   描述：設定目標 `<target>` 的參數 `<p>` 值為 `<v>`。
    *   參數 `<p>`: `c`, `kp`, `ki`, `max_vel`, `max_err`
    *   範例：`set m3 kp 600`
*   `get <target> [source]`
    *   描述：獲取目標 `<target>` 的當前生效參數。加上 `source` 可查看參數來源 (Default, Global, Motor)。
    *   範例：`get m3` 或 `get m3 source`
*   `reset <target> [p]`
    *   描述：重置目標 `<target>` 的參數 `<p>` (或全部) 為預設值。
    *   範例：`reset m3 kp` 或 `reset all`

> **目標對象 `<target>` 解釋**: `all`, `global`, `m<0-11>`, `g<h|u|l>`, `gl<0-3>`, `g<f|r>`

---
### 遙測與監控 (Telemetry & Monitoring)
*   `status`
    *   描述：打印一次性的完整狀態報告。
*   `monitor <h|c|d>`
    *   描述：設定遙測格式 (h:人類, c:CSV, d:儀表板)。
    *   **注意**: `monitor c` 的 CSV 格式為單行寬格式，專為數據分析設計。
*   `monitor freq <hz>`
    *   描述：設定遙測數據的更新頻率 (Hz)。
*   `monitor <pause|resume>`
    *   描述：暫停或恢復遙測數據流。
*   `focus <m<id>|off>`
    *   描述：設定儀表板或日誌的焦點馬達，或關閉焦點。

---
### 系統與除錯 (System & Debug)
*   `cal`
    *   描述：執行手動校準程序。
*   `stop`
    *   描述：停止所有馬達運動，進入 IDLE 模式。
*   `reboot`
    *   描述：重啟微控制器。
*   `raw m<id> <mA>`
    *   描述：[底層] 直接設定馬達的原始電流 (mA)。
*   `test wiggle m<id>`
    *   描述：為指定馬達啟動擺動測試。

---
## 🛠️ 開發者指南：指令系統設計 (Developer's Guide: Command System Design)

本節內容面向開發者，闡述了指令系統的底層設計哲學與規範。所有對 `CommandHandler` 模組的修改與擴充都應遵循此指南。

### 1. 核心設計哲學

*   **分層式解析 (Layered Parsing)**：指令解析遵循 `動詞 -> 目標 -> 參數` 的層次結構，確保邏輯清晰。
*   **明確性優先 (Explicitness over Implicitness)**：指令意圖應明確，避免需要複雜上下文理解的「魔術」指令。例如，`+=` 運算符明確觸發相對運動。
*   **一致性與可預測性 (Consistency & Predictability)**：相似的操作應有相似的語法。例如 `set`, `get`, `reset` 擁有高度一致的目標對象語法。

### 2. 語法結構規範

*   **基本結構**: `動詞 [目標] [參數...]`，由空格分隔，不區分大小寫。
*   **目標指示符 (Target Specifiers)**:
    *   `m<id>`: **M**otor
    *   `g<name>`: **G**roup
    *   `gl<id>`: **G**roup of **L**eg
    *   `all`: 特殊關鍵字，指所有馬達
    *   `global`: 特殊關鍵字，指全域參數作用域

### 3. 擴展與別名原則

*   **禁止歧義別名**: 為保證長期穩定性和可讀性，不鼓勵使用指令縮寫（如 `mv` for `move`）。
*   **新指令設計流程**: 應依循「確定動詞 -> 定義目標 -> 設計參數 -> 撰寫文檔 -> 實現測試」的標準流程。

### 4. 參數系統交互原則

*   **級聯覆蓋原則**: 參數生效優先級為 `單一馬達設定` > `全域設定` > `系統預設值`。
*   **層級化重置**: `reset` 指令的影響範圍依目標而定，從單一參數到整個系統。

---

## 🏗️ 專案結構 (Project Structure)
```
.
├── include/           # 頭文件
│   ├── AHRS.h
│   ├── CommandHandler.h
│   ├── MotorController.h
│   ├── RobotController.h
│   └── TelemetrySystem.h
├── src/               # 原始碼
│   ├── AHRS.cpp
│   ├── CommandHandler.cpp
│   ├── homing_main.cpp  # 主程式
│   ├── MotorController.cpp
│   ├── RobotController.cpp
│   └── TelemetrySystem.cpp
└── platformio.ini     # PlatformIO 配置文件
```
