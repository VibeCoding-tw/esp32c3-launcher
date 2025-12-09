// 包含必要的庫
#include <WiFi.h>
#include <WebServer.h>               // Web Server
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_ota_ops.h"             // OTA 相關操作
#include "esp_partition.h"           // 分區表操作
#include "esp_task_wdt.h"            // Watchdog Timer 函式庫
#include "esp32c3_gpio.h" // 假設此處定義了 AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN, NSLEEP_PIN

#include <NimBLEDevice.h>
#include <Preferences.h> // 用於儲存 BLE 接收到的 Wi-Fi 憑證

// --- 全域變數 ---
String globalHostname;              // 基於 MAC 位址的唯一 Hostname
WebServer server(80);                // 實例化同步 Web Server
// 【已移除】 WiFiManager wm;                      // 實例化同步 WiFiManager

// LEDC PWM 設定
const int PWM_FREQ = 20000;        // 頻率 (Hz)
const int PWM_RESOLUTION = 8;      // 解析度 8-bit (0-255)
const int PWM_MAX = 255;           // PWM 訊號最大值 (2^8 - 1)

// PWM 通道 (用於 DRV8833 的四個輸入腳)
const int LEDC_CH_A1 = 0;          // 馬達 T (速度) - AIN1
const int LEDC_CH_A2 = 1;          // 馬達 T (速度) - AIN2
const int LEDC_CH_B1 = 2;          // 馬達 S (轉向) - BIN1
const int LEDC_CH_B2 = 3;          // 馬達 S (轉向) - BIN2

// --- 馬達 Ramping 核心變數 ---
volatile int targetSpeedT = 0;     // 速度馬達的目標速度 (-255 到 255)
volatile int currentSpeedT = 0;    // 速度馬達的實際輸出速度 (-255 到 255)
volatile int targetSpeedS = 0;     // 轉向馬達的目標速度 (-255 到 255)
volatile int currentSpeedS = 0;    // 轉向馬達的實際輸出速度 (-255 到 255)

// --- 速度過渡配置 ---
const int RAMP_INTERVAL_MS = 10;    // 每 10ms 檢查一次 PWM 速度
unsigned long lastRampTime = 0;

// --- T 馬達 (速度/Throttle) Ramping 參數 ---
const int PWM_EFFECTIVE_LIMIT_T = 200; 
const int RAMP_ACCEL_STEP_T = 5; 
const int PWM_START_KICK_T = 128;

// --- S 馬達 (轉向/Steering) Ramping 參數 ---
const int PWM_EFFECTIVE_LIMIT_S = 250; 
const int RAMP_ACCEL_STEP_S = 20; 
const int PWM_START_KICK_S = 150;

// --- BLE UUID 定義 (使用 MAC 嵌入模板) ---
#define CONFIG_SERVICE_UUID_BASE  "0000FFFF-0000-1000-8000-ZZZZZZZZZZZZ" 
#define CONTROL_SERVICE_UUID_BASE "4FAFC201-1FB5-459E-8FCC-ZZZZZZZZZZZZ"

// 3. 特徵 (Characteristic) - 僅使用短 UUID，避免衝突
#define SSID_CHAR_UUID          "FF01"
#define PASS_CHAR_UUID          "FF02"
#define CONTROL_CHAR_UUID       "B26A" 

// --- 程式碼動態生成 ---
// 在全域變數中，定義實際的 UUID 字串變數：
String configServiceUUID;
String controlServiceUUID;

// 輔助函數：將 MAC Address 格式化並嵌入 UUID
void generateUniqueUuids() {
    // 獲取並清理 MAC Address (例如: "AABBCCDDEEFF")
    String macAddr = WiFi.macAddress(); 
    macAddr.replace(":", ""); 
    macAddr.toUpperCase(); 

    // 取 MAC Address 的後半部分 (DD EE FF)
    String mac_suffix = macAddr.substring(6); 
    
    // --- 構建服務 UUID ---
    
    // CONFIG_SERVICE_UUID
    configServiceUUID = CONFIG_SERVICE_UUID_BASE;
    // 將 BASE UUID 末尾的 ZZZ... 替換為 MAC Address
    configServiceUUID.replace("ZZZZZZZZZZZZ", macAddr);

    // CONTROL_SERVICE_UUID
    controlServiceUUID = CONTROL_SERVICE_UUID_BASE;
    // 將 BASE UUID 末尾的 ZZZ... 替換為 MAC Address
    controlServiceUUID.replace("ZZZZZZZZZZZZ", macAddr);
    
    Serial.printf("Unique Config Service UUID: %s\n", configServiceUUID.c_str());
    Serial.printf("Unique Control Service UUID: %s\n", controlServiceUUID.c_str());
}

// --- 全域變數 ---
Preferences preferences; // 用於儲存 Wi-Fi 憑證

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pControlCharacteristic = NULL;
NimBLECharacteristic *pSsidCharacteristic = NULL;
NimBLECharacteristic *pPassCharacteristic = NULL;

// Wi-Fi 憑證暫存
String ble_ssid;
String ble_pass;
bool wifi_config_received = false;

// --- BLE 連線狀態回調 ---
class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("BLE Client Connected.");
    }
    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("BLE Client Disconnected. Restarting Advertising...");
        vTaskDelay(pdMS_TO_TICKS(10));
        pServer->startAdvertising(); // 重新啟動廣告
    }
};

// --- BLE 特徵寫入回調 ---
class MyCharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    // 處理馬達控制指令
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String command = String(rxValue.c_str());
            Serial.printf("[BLE CMD] Received: %s\n", command.c_str());
            
            // 語義共用：將 BLE 指令轉換為與 Wi-Fi 相同的 (T, S) 參數
            // 假設 BLE 傳輸格式為 "T,S" (例如: "150,50" 或 "-100,-20")
            int commaIndex = command.indexOf(',');
            if (commaIndex > 0) {
                int rawT = command.substring(0, commaIndex).toInt();
                int rawS = command.substring(commaIndex + 1).toInt();

                // 導向與 Wi-Fi handleControl 相同的邏輯
                targetSpeedT = constrain(rawT, -PWM_EFFECTIVE_LIMIT_T, PWM_EFFECTIVE_LIMIT_T); 
                targetSpeedS = constrain(rawS, -PWM_EFFECTIVE_LIMIT_S, PWM_EFFECTIVE_LIMIT_S);
            }
        }
    }
};

// --- BLE 配置回調 (接收 Wi-Fi 憑證) ---
class ConfigCharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String data = String(rxValue.c_str());
            
            if (pCharacteristic == pSsidCharacteristic) {
                ble_ssid = data;
                Serial.printf("[BLE Config] Received SSID: %s\n", ble_ssid.c_str());
            } else if (pCharacteristic == pPassCharacteristic) {
                ble_pass = data;
                Serial.println("[BLE Config] Received Password.");
                
                // SSID 和 PASS 都收到，觸發 Wi-Fi 儲存邏輯
                if (ble_ssid.length() > 0) {
                    wifi_config_received = true; 
                    Serial.println("BLE 配置完成，準備儲存 Wi-Fi 憑證...");
                }
            }
        }
    }
};

// --- HTML 網頁內容 (內嵌虛擬搖桿) ---
const char* HTML_CONTENT = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 馬達搖桿控制</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        /* 確保全螢幕高度和柔軟的背景色 */
        body { 
            background-color: #1f2937; 
            color: #f9fafb; 
            font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, "Noto Sans", sans-serif;
            display: flex; 
            justify-content: center; 
            align-items: center; 
            min-height: 100vh; 
            margin: 0; 
            padding: 1rem;
        }
        .container { 
            max-width: 400px; 
            width: 100%; 
            padding: 20px; 
        }
        /* 搖桿圓盤樣式 */
        #joystick { 
            position: relative; 
            width: 100%;
            padding-top: 100%; /* 1:1 比例 */
            margin: 0 auto; 
            border-radius: 50%; 
            background: linear-gradient(145deg, #2d3748, #1a202c); 
            box-shadow: 10px 10px 20px #171d26, -10px -10px 20px #273142, inset 0 0 10px rgba(0,0,0,0.5);
            touch-action: none; /* 禁用瀏覽器預設的觸摸行為 */
        }
        /* 實際可拖曳區域 (內縮 5% 讓邊緣有陰影效果) */
        #joystick-inner {
            position: absolute;
            top: 5%; left: 5%; right: 5%; bottom: 5%;
            width: 90%;
            height: 90%;
        }
        /* 搖桿中心點 (Thumb) */
        #joystick-thumb {
            position: absolute;
            width: 70px; 
            height: 70px;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            border-radius: 50%;
            background: #4f46e5;
            box-shadow: 0 0 15px #4f46e5, inset 0 0 10px #7c3aed;
            cursor: grab;
            transition: box-shadow 0.1s;
        }
        #joystick-thumb.active { cursor: grabbing; box-shadow: 0 0 25px #7c3aed, inset 0 0 15px #4f46e5; }
        /* 狀態文字 */
        #status { font-weight: 700; text-shadow: 0 0 5px rgba(79, 70, 229, 0.5); }
    </style>
</head>
<body class="p-4">
    <div class="container bg-gray-800 rounded-xl shadow-2xl">
        <h1 class="text-3xl font-extrabold text-center text-indigo-400 mb-2">Vibe Racer</h1>
        <p class="text-center text-sm mb-6 text-gray-400">
            裝置名稱: <span id="hostname">%HOSTNAME%</span><br>
            IP: <span id="ipaddress">%IPADDRESS%</span>
        </p>

        <div id="joystick" class="mb-6">
            <div id="joystick-inner">
                 <div id="joystick-thumb"></div>
            </div>
        </div>

        <div class="text-center space-y-2">
            <p class="text-xl">狀態: <span id="status" class="text-green-400">靜止</span></p>
            <p class="text-xs text-gray-500">
                X (轉向): <span id="val_x">0</span> | Y (速度): <span id="val_y">0</span>
            </p>
        </div>
    </div>

    <script>
        const joystickContainer = document.getElementById('joystick'); 
        const joystick = document.getElementById('joystick-inner'); 
        const thumb = document.getElementById('joystick-thumb');
        const statusEl = document.getElementById('status');
        const valXEl = document.getElementById('val_x');
        const valYEl = document.getElementById('val_y');
        
        // Deadzone 設定 (PWM 值，範圍 0-255)
        const DEADZONE_PWM = 20; 

        // maxRadius 是實際拖曳區域 (joystick-inner) 的半徑
        const maxRadius = joystick.clientWidth / 2;
        let isDragging = false;
        let controlInterval;
        let lastMotorT = 0; // 上次發送的 T 馬達速度
        let lastMotorS = 0; // 上次發送的 S 馬達速度

        // 檢查當前 IP，用於 AP 模式下的絕對路徑
        const currentIP = document.getElementById('ipaddress').textContent;
        // 由於移除了 WiFiManager 的 AP 模式，這裡的檢查已簡化
        const baseIp = ''; 
        
        /**
         * @brief 根據搖桿位置 (Cartesian 座標) 計算並發送馬達速度。
         * @param rawX X 軸位移 (Cartesian: 右為正)
         * @param rawY Y 軸位移 (Cartesian: 上為正)
         */
        function updateMotorValues(rawX, rawY) {
            
            // 1. 計算幅度和角度
            const distance = Math.sqrt(rawX*rawX + rawY*rawY);
            const magnitude = Math.min(1.0, distance / maxRadius);
            const angle = Math.atan2(rawY, rawX);
            
            // 2. 計算歸一化後的 X, Y (範圍 -1.0 到 1.0)
            const normX = magnitude * Math.cos(angle); // 轉向 (Steering)
            const normY = magnitude * Math.sin(angle); // 速度 (Throttle)

            // 3. 轉換為 -255 到 255 的整數 (注意：ESP32 端會將 255 限制為 230)
            let speedT = Math.round(normY * 255);
            let speedS = Math.round(normX * 255);

            // --- 4. 關鍵：在 Web 端實作 Deadzone 邏輯 ---
            if (Math.abs(speedT) < DEADZONE_PWM) {
                speedT = 0;
            }
            if (Math.abs(speedS) < DEADZONE_PWM) {
                speedS = 0;
            }
            // ----------------------------------------------------

            // 更新顯示
            valYEl.textContent = speedT; // 顯示 T 馬達 (速度)
            valXEl.textContent = speedS; // 顯示 S 馬達 (轉向)
            
            // 更新狀態文字和顏色
            let currentStatus = "靜止";
            let statusColor = "text-green-400";
            if (Math.abs(speedT) > 0 || Math.abs(speedS) > 0) {
                 statusColor = "text-yellow-400";
                 if (speedT > 50 && Math.abs(speedS) < 50) currentStatus = "前進加速中";
                 else if (speedT < -50 && Math.abs(speedS) < 50) currentStatus = "後退減速中";
                 else if (speedS > 50) currentStatus = "右轉中";
                 else if (speedS < -50) currentStatus = "左轉中";
                 else currentStatus = "移動中";
            } else {
                 statusColor = "text-green-400";
            }
            statusEl.textContent = currentStatus;
            statusEl.className = statusColor;

            // 如果數值有變化，發送控制請求
            if (speedT !== lastMotorT || speedS !== lastMotorS) {
                lastMotorT = speedT;
                lastMotorS = speedS;
                // 發送 T 馬達速度 (t) 和 S 馬達速度 (s)
                sendControl(speedT, speedS); 
            }
        }

        function sendControl(T, S) {
            // 使用非同步請求發送馬達速度 (fetch 依然可用於同步 Web Server)
            fetch(`${baseIp}/control?t=${T}&s=${S}`, { method: 'GET' })
                .then(response => {
                    if (!response.ok) {
                        console.error('Server responded with an error:', response.status);
                    }
                })
                .catch(error => {
                    // console.error('Control command failed:', error);
                });
        }

        function resetThumbPosition() {
            thumb.style.left = '50%';
            thumb.style.top = '50%';
            thumb.style.transform = 'translate(-50%, -50%)';
            thumb.classList.remove('active');
        }

        function stopMotors() {
            isDragging = false;
            if (controlInterval) clearInterval(controlInterval);
            resetThumbPosition();
            // 發送 T=0, S=0，觸發 ESP32 端的即時停止
            updateMotorValues(0, 0); 
        }

        function handleMove(e) {
            e.preventDefault();
            if (!isDragging) return;

            // 取得觸摸或滑鼠位置
            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;

            // 取得搖桿容器 (joystick-inner) 的位置
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.left + maxRadius;
            const centerY = rect.top + maxRadius;

            // 1. 原始位移 (CSS 座標: X 向右為正, Y 向下為正)
            let offsetX = clientX - centerX;
            let offsetY = clientY - centerY; 
            
            // 2. 限制位移在搖桿圓盤內
            const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            if (distance > maxRadius) {
                const angle = Math.atan2(offsetY, offsetX);
                offsetX = maxRadius * Math.cos(angle);
                offsetY = maxRadius * Math.sin(angle);
            }
            
            // 3. 更新搖桿中心點位置 (使用 CSS 座標)
            const thumbX = maxRadius + offsetX;
            const thumbY = maxRadius + offsetY; 

            thumb.style.left = `${thumbX}px`;
            thumb.style.top = `${thumbY}px`;
            thumb.style.transform = 'translate(-50%, -50%)';

            // 4. 更新馬達值 (使用 Cartesian 座標: Y 軸向上為正)
            // 將 CSS Y 軸反轉: -offsetY
            updateMotorValues(offsetX, -offsetY);
        }

        function handleStart(e) {
            isDragging = true;
            thumb.classList.add('active');
            handleMove(e); // 立即更新一次位置和值

            // 設置間隔發送，確保命令持續性
            if (controlInterval) clearInterval(controlInterval);
            controlInterval = setInterval(() => {
                // 重新讀取上次計算的值並發送，確保命令持續性
                sendControl(lastMotorT, lastMotorS);
            }, 100); // 每 100ms 發送一次
        }

        function handleEnd() {
            stopMotors();
        }

        // --- 事件監聽 ---
        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);

        joystick.addEventListener('touchstart', handleStart);
        document.addEventListener('touchmove', handleMove);
        // 觸摸結束可能在搖桿外，監聽大容器確保停止命令發出
        joystickContainer.addEventListener('touchend', handleEnd); 

        // 初始化時發送一次停止命令
        stopMotors(); 
    </script>
</body>
</html>
)rawliteral";

// 產生基於 MAC 位址的 Hostname ---
void generateHostname() {    
    globalHostname = "esp32c3-" + WiFi.macAddress(); 
    globalHostname.replace(":", ""); 
    globalHostname.toLowerCase(); 
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}

// --- 輔助函數: 實際寫入 PWM 值 ---
void setMotorPwm(int speedT, int speedS) {
    // T 馬達 (速度)
    if (speedT > 0) { 
        // Forward
        ledcWrite(LEDC_CH_A1, speedT);
        ledcWrite(LEDC_CH_A2, 0);
    } else if (speedT < 0) { 
        // Reverse
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, -speedT); 
    } else { 
        // STOP: Coast mode (IN1=LOW, IN2=LOW)
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, 0);
    }

    // S 馬達 (轉向)
    if (speedS > 0) { // 正值: 右轉 (BIN1 LOW, BIN2 HIGH)
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, speedS);
    } else if (speedS < 0) { // 負值: 左轉 (BIN1 HIGH, BIN2 LOW)
        ledcWrite(LEDC_CH_B1, -speedS); 
        ledcWrite(LEDC_CH_B2, 0);
    } else { 
        // STOP: Coast mode (IN1=LOW, IN2=LOW)
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, 0);
    }
}

// --- 定時馬達 Ramping 任務 (T 和 S 獨立參數) ---
void motorRampTask() {
    if (millis() - lastRampTime < RAMP_INTERVAL_MS) return;
    lastRampTime = millis();
    
    // (Ramping 邏輯保持不變，因為它是核心控制邏輯)

    // ------------------------------------------------------------------
    // 速度馬達 (T Motor) 邏輯
    // ------------------------------------------------------------------
    if (targetSpeedT == 0) {
        currentSpeedT = 0; 
    } else {
        if (currentSpeedT == 0) {
            if (targetSpeedT > 0) {
                currentSpeedT = PWM_START_KICK_T;
            } else {
                currentSpeedT = -PWM_START_KICK_T;
            }
        }
        
        if (abs(currentSpeedT) > abs(targetSpeedT)) {
             currentSpeedT = targetSpeedT;
        }

        if (abs(targetSpeedT - currentSpeedT) > RAMP_ACCEL_STEP_T) {
            if (targetSpeedT > currentSpeedT) {
                currentSpeedT += RAMP_ACCEL_STEP_T;
            } else {
                currentSpeedT -= RAMP_ACCEL_STEP_T;
            }
        } else {
            currentSpeedT = targetSpeedT;
        }
    }

    // ------------------------------------------------------------------
    // 轉向馬達 (S Motor) 邏輯 
    // ------------------------------------------------------------------
    if (targetSpeedS == 0) {
        currentSpeedS = 0; 
    } else {
        if (currentSpeedS == 0) {
            if (targetSpeedS > 0) {
                currentSpeedS = PWM_START_KICK_S;
            } else {
                currentSpeedS = -PWM_START_KICK_S;
            }
        }

        if (abs(currentSpeedS) > abs(targetSpeedS)) {
             currentSpeedS = targetSpeedS;
        }

        if (abs(targetSpeedS - currentSpeedS) > RAMP_ACCEL_STEP_S) {
             if (targetSpeedS > currentSpeedS) {
                currentSpeedS += RAMP_ACCEL_STEP_S;
            } else {
                currentSpeedS -= RAMP_ACCEL_STEP_S;
            }
        } else {
            currentSpeedS = targetSpeedS;
        }
    }

    // 實際寫入 PWM
    setMotorPwm(currentSpeedT, currentSpeedS); 
}

// --- Web Server 處理函式 (同步版本) ---
void handleRoot() {
    
    String html = HTML_CONTENT; 
    // 由於移除了 WiFiManager，Web Server 僅在 STA 模式連線成功後才會啟動。
    String ipAddress = WiFi.localIP().toString();
    
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);

    server.send(200, "text/html", html);
}

void handleControl() {
    // *** 檢查參數替換為 server.hasArg ***
    if (server.hasArg("t") && server.hasArg("s")) {
        
        // 讀取原始搖桿輸入
        int rawT = server.arg("t").toInt();
        int rawS = server.arg("s").toInt();
        
        // 將目標速度分別約束在 T 和 S 的有效限制內
        targetSpeedT = constrain(rawT, -PWM_EFFECTIVE_LIMIT_T, PWM_EFFECTIVE_LIMIT_T); 
        targetSpeedS = constrain(rawS, -PWM_EFFECTIVE_LIMIT_S, PWM_EFFECTIVE_LIMIT_S);

        Serial.printf("WebControl (Target): T馬達(速度)=%d, S馬達(轉向)=%d\n", targetSpeedT, targetSpeedS);        
        server.send(200, "text/plain", "OK"); 
    } else {
        server.send(400, "text/plain", "Invalid arguments (Missing t or s)");
    }
}

void setupWebServer() {
    Serial.println("--- 啟動 Web Server (STA 模式) ---");

    // 處理根目錄請求 (虛擬搖桿頁面)
    server.on("/", HTTP_GET, handleRoot);

    // 處理馬達控制 API 請求
    server.on("/control", HTTP_GET, handleControl);

    // 處理所有未定義的請求
    server.onNotFound([](){
        server.send(404, "text/plain", "Not Found");
    });

    server.begin();
    Serial.println("HTTP 伺服器已啟動於 Port 80。");
}

// --- mDNS/OTA 設定 ---
void setupMdnsOtaSta() {
    Serial.println("--- 設定 mDNS 和 OTA (STA 模式) ---");

    // 1. Setup mDNS
    if (MDNS.begin(globalHostname.c_str())) {
        Serial.printf("mDNS (STA 模式) 啟動: %s.local -> %s\n", 
            globalHostname.c_str(), WiFi.localIP().toString().c_str());
    } else {
        Serial.println("mDNS (STA 模式) 啟動失敗。");
    }

    // 2. Setup OTA
    ArduinoOTA.setHostname(globalHostname.c_str());
    ArduinoOTA.setPassword("mysecurepassword"); 

    ArduinoOTA.onStart([]() { Serial.println("OTA 更新開始..."); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA 更新完成! 正在重啟..."); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA 錯誤碼 [%u]\n", error); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("進度: %u%%\r", (progress * 100) / total);
    });
    ArduinoOTA.begin();
    
    Serial.println("-------------------------------------------------");
}

void connectToSavedWiFi() {
    // 偵錯點 A：確認函數已進入
    Serial.println("--- 進入 connectToSavedWiFi 函數 ---"); 

    preferences.begin("wifi-config", true); // Open in read-only mode
    
    // 偵錯點 B：確認 preferences.begin 成功
    Serial.println("NVS: preferences.begin() 執行完畢。"); 

    String saved_ssid = preferences.getString("ssid", "");
    String saved_pass = preferences.getString("pass", "");
    preferences.end();
    
    // 偵錯點 C：確認 preferences.end 成功
    Serial.println("NVS: 憑證讀取完畢。"); 

    // ... (後續邏輯不變)
    if (saved_ssid.length() == 0) {
        Serial.println("無儲存的 Wi-Fi 憑證。裝置保持 BLE 廣告，等待配置...");
        return; 
    }
    
    // ... (嘗試連線邏輯不變)
}
/*
// --- 【替換函數】讀取 Preferences 並連線到 Wi-Fi ---
void connectToSavedWiFi() {

    preferences.begin("wifi-config", true); // Open in read-only mode
    String saved_ssid = preferences.getString("ssid", "");
    String saved_pass = preferences.getString("pass", "");
    preferences.end();
    
    // 檢查是否有儲存的憑證
    if (saved_ssid.length() == 0) {
        Serial.println("無儲存的 Wi-Fi 憑證。裝置保持 BLE 廣告，等待配置...");
        // 保持在空閒狀態，只運行 BLE 任務和 loop() 中的 motorRampTask()
        return; 
    }
    
    // 嘗試連線
    Serial.printf("嘗試連線到儲存的 Wi-Fi: %s\n", saved_ssid.c_str());
    WiFi.setHostname(globalHostname.c_str());
    WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
    
    // 設置 Wi-Fi 模式為 STA
    WiFi.mode(WIFI_STA); 
    
    // 避免長時間阻塞，連線狀態會在 loop() 中的 checkAndStartServices() 檢查
}
*/
bool servicesStarted = false;

void checkAndStartServices() {
    // 只有在連線成功且服務尚未啟動時才執行
    if (WiFi.status() == WL_CONNECTED && !servicesStarted) {
        Serial.println("✅ Wi-Fi 連線成功，啟動服務...");
        
        // 2. Setup mDNS and OTA (STA Mode)
        setupMdnsOtaSta();

        // 3. Setup Web Server (STA Mode)
        setupWebServer();
        
        servicesStarted = true;

        Serial.println("-------------------------------------------------------");
        Serial.printf("裝置已啟動: %s.local\n", globalHostname.c_str()); 
        Serial.println("-------------------------------------------------------");
    }
}

// --- BLE 配置處理迴圈 (在收到憑證後重啟) ---
void handleBleConfigLoop() {
    if (wifi_config_received) {
        Serial.println("🚨 儲存新憑證並重啟中...");
        preferences.begin("wifi-config", false);
        preferences.putString("ssid", ble_ssid);
        preferences.putString("pass", ble_pass);
        preferences.end();
        wifi_config_received = false;
        
        // 延遲以確保 Serial 輸出完成
        delay(100); 
        ESP.restart();
    }
}

// 替換您現有的 setupBLE() 函數
void setupBleServer() {
    // 0. 初始化 BLE 設備 (只需執行一次)
    NimBLEDevice::init(globalHostname.c_str());
    // 建議將功率設置為 MAX (ESP_PWR_LVL_P9) 以優化連線穩定性
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); 
    
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 1. 馬達控制服務 (使用動態 UUID)
    NimBLEService *pControlService = pServer->createService(controlServiceUUID.c_str());
    pControlCharacteristic = pControlService->createCharacteristic(
        CONTROL_CHAR_UUID, 
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    pControlCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pControlService->start();

    // 2. Wi-Fi 配置服務 (使用動態 UUID)
    NimBLEService *pConfigService = pServer->createService(configServiceUUID.c_str());
    
    // SSID 特徵 (寫入)
    pSsidCharacteristic = pConfigService->createCharacteristic(
        SSID_CHAR_UUID, 
        NIMBLE_PROPERTY::WRITE
    );
    pSsidCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());

    // Password 特徵 (寫入)
    pPassCharacteristic = pConfigService->createCharacteristic(
        PASS_CHAR_UUID, 
        NIMBLE_PROPERTY::WRITE
    );
    pPassCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    
    pConfigService->start();

    // 3. 啟動廣告
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(globalHostname.c_str());
    
    // 加入兩個服務的 UUID 到廣告封包中
    pAdvertising->addServiceUUID(controlServiceUUID.c_str());
    pAdvertising->addServiceUUID(configServiceUUID.c_str()); 

    // 啟動廣告
    pAdvertising->start(); 

    Serial.println("✅ BLE Advertising Started.");
}

void bleTask(void *p) {

    setupBleServer();

    while (true)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- 初始化馬達控制腳位 (DRV8833) ---
    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH); 
    Serial.printf("馬達驅動 (nSLEEP) 已致能於 GPIO%d\n", NSLEEP_PIN);

    // PWM 設定
    ledcSetup(LEDC_CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B2, PWM_FREQ, PWM_RESOLUTION);

    // PWM 腳位連接
    ledcAttachPin(AIN1_PIN, LEDC_CH_A1);
    ledcAttachPin(AIN2_PIN, LEDC_CH_A2);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1);
    ledcAttachPin(BIN2_PIN, LEDC_CH_B2);

    setMotorPwm(0, 0); // 確保馬達啟動時靜止
    
    // 0. 產生唯一的 Hostname
    generateHostname();
    generateUniqueUuids();

    // --- 啟動 BLE 任務 (最高優先級以確保穩定性) ---
    xTaskCreate(
        bleTask,          // 任務函數
        "BLE_Task",       // 任務名稱
        8192,             // 堆疊大小 
        NULL,             // 傳遞給任務的參數
        1,                // 任務優先級
        NULL              // 任務句柄
    );

    delay(50); 
    Serial.println("--- 延遲 50ms 確保 BLE 穩定 ---");
    // --- 啟動器核心邏輯 ---
    // 1. 嘗試連線 Wi-Fi (取代 WiFiManager 的 autoConnect)
    connectToSavedWiFi();
}

// --- Loop ---
void loop() {
    
    // 【已移除】 wm.process(); 

    // 2. 檢查連線狀態，並啟動後續服務 (只執行一次)
    checkAndStartServices();

    // 3. Web Server 和 OTA 僅在服務啟動 (即 STA 連線成功) 後才處理
    if (servicesStarted) {
        server.handleClient();
        ArduinoOTA.handle();
    }
    
    // 4. 處理 BLE 接收到的 Wi-Fi 配置並重啟
    handleBleConfigLoop();
    
    // 5. 馬達 Ramping 邏輯
    motorRampTask();
    
    yield();
}