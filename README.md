# Vibe Racer 控制系統 (esp32c3-launcher)

![Project Status](https://img.shields.io/badge/Status-Stable-green)
![Platform](https://img.shields.io/badge/Platform-ESP32--C3-blue)
![Framework](https://img.shields.io/badge/Framework-Arduino-orange)

Vibe Racer 是一個專為高品質遙控車設計的嵌入式控制系統。基於 ESP32-C3 晶片開發，提供流暢的 Web 虛擬搖桿與 BLE 藍牙控制介面，並內建精密馬達物理模擬邏輯，確保遙控車在啟動、轉向與停止時具備真實且安全的動態表現。

---

## 🚀 核心功能

*   **雙模通訊控制**：支援透過手機瀏覽器（Web Server）或專用 App（BLE）進行即時搖桿控制。
*   **實時電壓監控與診斷**：(New ✨) 內建電池電壓採樣與濾波邏輯，並透過即時 Web UI 回傳。
*   **視覺化 Web 控制介面**：內建基於 Tailwind CSS 的自適應虛擬搖桿，具備電量、Ramping 狀態與診斷儀表板。
*   **進階馬達物理模擬**：
    *   **進階起步 (Ramping)**：模擬真實物理加速過程，避免電流瞬間過大造成的硬體損耗。
    *   **靜摩擦補償 (Kickstart)**：智慧型起始脈衝，確保馬達在低轉速下能順利起步。
*   **全面安全保護**：
    *   **控制超時自動煞車**：接收不到指令時於 500ms 內自動停止。
    *   **轉向過載保護**：轉向馬達連輸出時間限制（預設 800ms），防止打死燒毀。
    *   **H 橋直通保護**：硬體換向時強制加入換向死區時間。
*   **動態參數配置**：
    *   支提供 Web API 與 BLE 特徵點修改馬達校準參數（如加速步長、最高速限制、啟動電壓）。
    *   支援 NVS 非易失性存儲，參數修改後斷電不丟失。
*   **維護與更新**：
    *   **ArduinoOTA**：支援無線韌體升級。
    *   **mDNS**：可透過自定義域名（如 `esp32c3-xxxxxx.local`）直接訪問，無需記憶 IP。

---

## 🛠️ 硬體配置

### ESP32-C3 腳位定義 (GPIO)

| 功能 (Function) | GPIO 腳位 | 說明 |
| :--- | :---: | :--- |
| **AIN1** | 3 | 馬達 T (Throttle) 輸入 1 |
| **AIN2** | 2 | 馬達 T (Throttle) 輸入 2 |
| **BIN1** | 10 | 馬達 S (Steering) 輸入 1 |
| **BIN2_PIN** | 7 | 馬達 S (Steering) 輸入 2 |
| **NSLEEP** | 4 | 馬達驅動器致能 (High Active) |
| **BATT_ADC** | 5 | 電池分壓電壓採樣 (ADC1_CH5) |

### 指示燈與狀態
- **Serial**: 波特率定為 `115200`，輸出即時 Ramp 狀態與系統診斷資訊。

---

## 📡 通訊介面

### Web 控制 (Wi-Fi)
- **首頁**: `http://<IP_ADDRESS>/` - 啟動全螢幕虛擬搖桿。
- **控制接口**: `/control?t=[speed]&s=[steering]`
    - **回應**: 返回 JSON 格式診斷數據：`{"v":電壓, "t":目標T, "s":目標S, "rt":即時T, "rs":即時S, "to":超時標記}`。
- **配置接口**: `/config` (GET 讀取 / POST 寫入 JSON)

### BLE 藍牙介面
- **服務 (Service)**:
    - 配置服務: `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (SSID/Password 寫入)
    - 控制服務: `4fafc201-1fb5-459e-8fcc-c0ffee00dead` (馬達控制與參數配置)
- **特徵點 (Characteristic)**:
    - 實時控制: `4fafc202-...` (寫入 `T,S` 字串)
    - 參數配置: `4fafc203-...` (結構體二進位讀寫)

---

## ⚙️ 馬達校準參數

系統支援針對不同馬達特性進行動態調優，核心參數如下（可透過 Web/BLE 修改）：

| 參數名 | 預設值 | 說明 |
| :--- | :---: | :--- |
| `controlTimeoutMs` | 500 | 通訊中斷超過此時間後馬達自動停止。 |
| `pwmEffectiveLimit` | 255 | 輸出的最高 PWM 範圍限制。 |
| `rampAccelStep` | 10 | 每次 Ramp 檢查週期中速度增加的步長（數值越大加速越快）。 |
| `pwmStartKick` | 200 | 當馬達從靜止啟動時，瞬間給予的起始 PWM，用於克服靜摩擦。 |

> [!TIP]
> **電壓監控提示**：為確保安全，建議使用兩顆 100kΩ 電阻作為分壓電路連接至 **GPIO 5**。系統預設分壓比為 1/2，可監測 0~6V 範圍。
> **轉向保護機制**：轉向馬達 (S Motor) 若連續輸出超過 800ms，系統會自動將輸出功率降至 60% (PWM 150)。

---

## 📦 軟體依賴與環境

- **IDE**: [PlatformIO](https://platformio.org/)
- **Framework**: Arduino (espressif32)
- **主要依賴庫**:
    *   `h2zero/NimBLE-Arduino` (高性能低內存藍牙棧)
    *   `WiFiManager` (可選)
    *   `ArduinoOTA` / `ESPmDNS`

---

## 📝 開發者說明
本專案原始碼由 `src/main.cpp` 驅動，採用 FreeRTOS 任務概念實作非同步控制邏輯。若需修改 GPIO 腳位，請至 `include/esp32c3_gpio.h` 進行調整。
