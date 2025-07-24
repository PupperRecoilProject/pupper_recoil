### **指令集 (V3.4) - 專業文檔格式**

**輸入前請先改成英文輸入法**

========================================
 Pupper Robot :: Command Set v3.4
========================================

語法提示 (Syntax Legend):
  <variable> : 表示需要使用者替換的變數
  [optional] : 表示可選的參數
  (a|b|c)    : 表示從 a, b, c 中擇一選擇
----------------------------------------

--- 核心運動 (Movement & Posing) ---
  stand                     - 進入手動校準時設定的站立姿態。
  move m<id> <rad>          - [絕對] 將馬達 <id> 移動到目標角度 <rad>。
  move m<id> += <rad>       - [相對] 將馬達 <id> 在當前目標上增加 <rad>。
  move all <rad*12>         - [絕對] 一次性設定所有 12 個關節的角度。
  
--- 關節組控制 (Joint Group Control) ---
  move g<h|u|l> <rad>       - 移動一個關節功能組 (h:髖部, u:大腿, l:小腿)。
  move gl<0-3> <h> <u> <l>  - 設定單一腿(0-3)的三個關節角度。
  move g<f|r> <h> <u> <l>   - 設定一對腿(f:前腿, r:後腿)的關節角度。

--- 參數調校 (Parameter Tuning) ---
  set <target> <p> <v>      - 設定目標 <target> 的參數 <p> 值為 <v>。
                              (參數 p: c, kp, ki, max_vel, max_err)
  get <target> [source]     - 獲取目標 <target> 的當前生效參數。
                              (加上 source 可查看參數來源: Default, Global, Motor)
  reset <target> [p]        - 重置目標 <target> 的參數 <p> (或全部) 為預設值。
  (目標 target: all, global, m<id>, g<name>, gl<id>, ... )

--- 遙測與監控 (Telemetry & Monitoring) ---
  status                    - 打印一次性的完整狀態報告。
  monitor <h|c|d>           - 設定遙測格式 (h:人類, c:CSV, d:儀表板)。
  monitor freq <hz>         - 設定遙測數據的更新頻率 (Hz)。
  monitor <pause|resume>    - 暫停或恢復遙測數據流。
  focus <m<id>|off>         - 設定儀表板或日誌的焦點馬達，或關閉焦點。

--- 系統與除錯 (System & Debug) ---
  cal                       - 執行手動校準程序。
  stop                      - 停止所有馬達運動，進入 IDLE 模式。
  reboot                    - 重啟微控制器。
  raw m<id> <mA>            - [底層] 直接設定馬達的原始電流 (mA)。
  test wiggle m<id>         - 為指定馬達啟動擺動測試。

========================================