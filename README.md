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
*   **馬達控制加速 (Diagonal Power/Square Mapping)**：(New 🚀) 優化搖桿映射邏輯，確保在斜向推動時仍能輸出 100% 扭力，解決圓形搖桿力道打折問題。
*   **全面安全保護**：
    *   **分開的控制超時自動煞車**：加速 (T) 與轉向 (S) 分開計時，預設分別於 500ms / 2000ms 內自動停止，兼顧安全與穩定。
    *   **轉向過載保護與持久力度**：(Updated 🦾) 轉向馬達連輸出時間限制延長至 **3000ms**，並於超時後維持 **230 PWM** 的強勁回正力，在保護馬達的同時確保轉向不鬆脫。
    *   **H 橋直通保護**：硬體換向時強制加入換向死區時間。
*   **動態參數配置**：
    *   支提供 Web API 與 BLE 特徵點修改馬達校準參數（如加速步長、最高速限制、啟動電壓）。
    *   支援 NVS 非易失性存儲，參數修改後斷電不丟失。
*   **分區分級更新策略 (Staged OTA)**：(New 🔄)
    *   **暫存區 (ota_0)**：所有雲端更新（官方或學生代碼）皆先下載至暫存區運行與驗證。
    *   **安全回寫 (Factory Copy)**：官方更新經 `ota_0` 驗證成功後，會自動回寫至 `factory` 分區，確保底層系統穩定。
    *   **學生開發區**：學生代碼可直接於 `ota_0` 運行，不影響 `factory` 安全分區。
*   **軟體自動回滾 (Soft Rollback)**：(New 🚨) 若新韌體在 `ota_0` 啟動後 10 秒內崩潰或斷電，系統將自動偵測並於下次開機回滾至 `factory` 分區。
*   **硬體強制救援 (Rescue Mode)**：(New 🔘) 於啟動時將 **GPIO 1** 接地，可強制切換開機分區至 `factory`，無視任何 OTA 設定，確保 100% 救磚。
*   **美化維護頁面**：(New ✨) 內建現代化維修中心 (`/update_factory`)，提供官方系統更新 (To Factory) 與學生代碼更新 (To OTA_0) 兩種模式。

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
| **BATT_ADC** | 0 | 電池分壓電壓採樣 (ADC1_CH0) |
| **FACTORY_RESCUE** | 1 | 強制回歸 Factory 分區 (地面觸發) |

### 指示燈與狀態
- **Serial**: 波特率定為 `115200`，輸出即時 Ramp 狀態與系統診斷資訊。

---

## 📡 通訊介面

### Web 控制 (Wi-Fi)
- **首頁**: `http://<IP_ADDRESS>/` - 啟動全螢幕虛擬搖桿。
- **控制接口**: `/control?t=[speed]&s=[steering]`
    - **回應**: 返回 JSON：`{"v":電壓, "t":目標T, "s":目標S, "rt":即時T, "rs":即時S, "to":超時T, "tos":超時S}`。
- **配置接口**: `/config` (GET 讀取 / POST 寫入 JSON)
- **維護中心**: `/update_factory` (提供官方/學生雙重更新模式)
- **官方更新**: `/update_official` (下載至 ota_0 並在成功後回寫 factory)
- **學生更新**: `/update_student` (僅下載至 ota_0 供測試)

### BLE 藍牙介面
- **服務 (Service)**:
    - 配置服務: `6e400001-b5a3-f393-e0a9-e50e24dcca9e` (SSID/Password 寫入)
    - 控制服務: `4fafc201-1fb5-459e-8fcc-c0ffee00dead` (馬達控制與參數配置)
- **特徵點 (Characteristic)**:
    - 實時控制: `4fafc202-1fb5-459e-8fcc-c0ffee01feed` (寫入 `T,S` 字串)
    - 參數配置: `4fafc203-1fb5-459e-8fcc-c0ffee02dead` (結構體二進位讀寫)
    - Wi-Fi SSID: `6e400002-b5a3-f393-e0a9-e50e24dcca9e` (寫入)
    - Wi-Fi PASS: `6e400003-b5a3-f393-e0a9-e50e24dcca9e` (寫入)

---

## ⚙️ 馬達校準參數

系統支援針對不同馬達特性進行動態調優，核心參數如下（可透過 Web/BLE 修改）：

| 參數名 | 預設值 | 說明 |
| :--- | :---: | :--- |
| `controlTimeoutT` | 500 | 加速馬達 (T) 通訊中斷自動停止時間。 |
| `controlTimeoutS` | 2000 | 轉向馬達 (S) 通訊中斷自動停止時間。 |
| `pwmEffectiveLimitS` | 255 | 轉向馬達 (S) 最高 PWM，確保能克服回正彈簧。 |
| `rampAccelStepS` | 20 | 轉向加速度（提高至 20 以獲得更為靈敏的反應）。 |
| `pwmStartKickS` | 120 | 轉向起步脈衝，確保能瞬間帶動輪胎。 |
| `autoUpdateEnabled` | 0 | (New) 開機自動更新開關 (0: 關閉, 1: 開啟)。 |

> [!TIP]
> **電壓監控提示**：為確保安全，建議使用兩顆 100kΩ 電阻作為分壓電路連接至 **GPIO 0**。系統預設分壓比為 1/2，可監測 0~6V 範圍。
> **轉向保護機制**：轉向馬達 (S Motor) 若連續輸出超過 **3000ms**，系統會自動將輸出功率稍微降至 **PWM 230** (維持最高扭矩並避免燒毀)。

---

## 📦 軟體依賴與環境

- **IDE**: [PlatformIO](https://platformio.org/)
- **Framework**: Arduino (espressif32)
- **主要依賴庫**:
    *   `h2zero/NimBLE-Arduino` (高性能低內存藍牙棧)
    *   `ArduinoOTA` / `ESPmDNS`

---

## 📝 開發者說明
本專案原始碼由 `src/main.cpp` 驅動，採用 FreeRTOS 任務概念實作非同步控制邏輯。若需修改 GPIO 腳位，請至 `include/esp32c3_gpio.h` 進行調整。
