// 包含必要的庫
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>               // Web Server
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_ota_ops.h"             // OTA 相關操作
#include "esp_partition.h"           // 分區表操作
#include "esp_task_wdt.h"            // Watchdog Timer 函式庫
#include "esp32c3_gpio.h"            // 假設此處定義了 GPIO 腳位 (例如: AIN1_PIN, NSLEEP_PIN)

// --- 【BLE 核心變更】替換為 Bluedroid 庫 ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>             // 用於儲存 Wi-Fi 憑證
#include <freertos/FreeRTOS.h>       // 繼續使用 FreeRTOS 任務功能
#include <freertos/task.h>

// --- 全域變數 ---
String globalHostname;              // 基於 MAC 位址的唯一 Hostname
WebServer server(80);                // 實例化同步 Web Server

// LEDC PWM 設定 (保持不變)
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
const int LEDC_CH_A1 = 0;
const int LEDC_CH_A2 = 1;
const int LEDC_CH_B1 = 2;
const int LEDC_CH_B2 = 3;

// --- 馬達 Ramping 核心變數 ---
volatile unsigned long lastControlTime = 0; // 上次接收到控制命令的時間戳記
const unsigned long CONTROL_TIMEOUT_MS = 1000; // 1000 毫秒 = 1 秒超時
volatile int targetSpeedT = 0;
volatile int currentSpeedT = 0;
volatile int targetSpeedS = 0;
volatile int currentSpeedS = 0;
const int RAMP_INTERVAL_MS = 10;
unsigned long lastRampTime = 0;
const int PWM_EFFECTIVE_LIMIT_T = 250; //200; 
const int RAMP_ACCEL_STEP_T = 5; 
const int PWM_START_KICK_T = 200; //128;
const int PWM_EFFECTIVE_LIMIT_S = 250; 
const int RAMP_ACCEL_STEP_S = 5; //20; 
const int PWM_START_KICK_S = 200; //150;

// --- BLE UUID 定義 ---
const char* CONFIG_SERVICE_UUID_BASE  = "0000FFFF-0000-1000-8000-000000000000"; 
const char* CONTROL_SERVICE_UUID_BASE = "4FAFC201-1FB5-459E-8FCC-000000000000";

// 特徵 (Characteristic)
#define SSID_CHAR_UUID          "0000FFFF-0000-1000-8000-000000000001"
#define PASS_CHAR_UUID          "0000FFFF-0000-1000-8000-000000000002"
#define CONTROL_CHAR_UUID       "4FAFC201-1FB5-459E-8FCC-000000000000"

// --- 程式碼動態生成 ---
String configServiceUUID;
String controlServiceUUID;

// --- 全域變數：BLE / NVS ---
Preferences preferences; 
BLEServer *pServer = NULL;
BLECharacteristic *pControlCharacteristic = NULL;
BLECharacteristic *pSsidCharacteristic = NULL;
BLECharacteristic *pPassCharacteristic = NULL;
String ble_ssid;
String ble_pass;
bool wifi_config_received = false;
volatile bool should_restart_advertising = false;
bool servicesStarted = false;

// --- HTML 網頁內容 (保持不變) ---
const char* HTML_CONTENT = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VibeRcer 搖桿控制</title>
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
        const baseIp = ''; 
        
        /**
         * @brief 根據搖桿位置 (Cartesian 座標) 計算並發送馬達速度。
         */
        function updateMotorValues(rawX, rawY) {
            
            // 1. 計算幅度和角度
            const distance = Math.sqrt(rawX*rawX + rawY*rawY);
            const magnitude = Math.min(1.0, distance / maxRadius);
            const angle = Math.atan2(rawY, rawX);
            
            // 2. 計算歸一化後的 X, Y (範圍 -1.0 到 1.0)
            const normX = magnitude * Math.cos(angle); // 轉向 (Steering)
            const normY = magnitude * Math.sin(angle); // 速度 (Throttle)

            // 3. 轉換為 -255 到 255 的整數
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
            // 使用非同步請求發送馬達速度
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

// 輔助函數：將 MAC Address 格式化並嵌入 UUID
void generateUniqueUuids() {
    String macAddr = WiFi.macAddress(); 
    macAddr.replace(":", ""); 
    macAddr.toUpperCase(); 

    // CONFIG_SERVICE_UUID
    configServiceUUID = CONFIG_SERVICE_UUID_BASE;
    configServiceUUID.replace("ZZZZZZZZZZZZ", macAddr);

    // CONTROL_SERVICE_UUID
    controlServiceUUID = CONTROL_SERVICE_UUID_BASE;
    controlServiceUUID.replace("ZZZZZZZZZZZZ", macAddr);
    
    Serial.printf("Config Service UUID: %s\n", configServiceUUID.c_str());
    Serial.printf("Control Service UUID: %s\n", controlServiceUUID.c_str());
}

// --- 輔助函數: 實際寫入 PWM 值 ---
void setMotorPwm(int speedT, int speedS) {
    // T 馬達 (速度)
    if (speedT > 0) { 
        ledcWrite(LEDC_CH_A1, speedT);
        ledcWrite(LEDC_CH_A2, 0);
    } else if (speedT < 0) { 
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, -speedT); 
    } else { 
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, 0);
    }

    // S 馬達 (轉向)
    if (speedS > 0) { 
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, speedS);
    } else if (speedS < 0) { 
        ledcWrite(LEDC_CH_B1, -speedS); 
        ledcWrite(LEDC_CH_B2, 0);
    } else { 
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, 0);
    }
}

// --- 定時馬達 Ramping 任務 ---
void motorRampTask() {
    if (millis() - lastRampTime < RAMP_INTERVAL_MS) return;
    lastRampTime = millis();
    
    // 速度馬達 (T Motor) 邏輯
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

    // 轉向馬達 (S Motor) 邏輯 
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
    setMotorPwm(currentSpeedT, currentSpeedS); 

    // <-- 新增: 輸出實際驅動的 PWM 值 (僅在馬達運轉時輸出，避免洗版)
    if (currentSpeedT != 0 || currentSpeedS != 0) {
        Serial.printf("[Ramp] Current PWM: T=%d, S=%d\n", currentSpeedT, currentSpeedS);
    }
}

// --- Web Server 處理函式 ---
void handleRoot() {
    String html = HTML_CONTENT; 
    String ipAddress = WiFi.localIP().toString();
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);
    server.send(200, "text/html", html);
}

void handleControl() {
    if (server.hasArg("t") && server.hasArg("s")) {
        int rawT = server.arg("t").toInt();
        int rawS = server.arg("s").toInt();
        targetSpeedT = constrain(rawT, -PWM_EFFECTIVE_LIMIT_T, PWM_EFFECTIVE_LIMIT_T); 
        targetSpeedS = constrain(rawS, -PWM_EFFECTIVE_LIMIT_S, PWM_EFFECTIVE_LIMIT_S);
        // 【新增】更新控制時間戳記
        lastControlTime = millis();
        Serial.printf("WebControl (Target): T馬達(速度)=%d, S馬達(轉向)=%d\n", targetSpeedT, targetSpeedS);   

        // 組裝您想回傳給 BLE 客戶端的字串
        String response = "T:" + String(targetSpeedT) + ",S:" + String(targetSpeedS);    
        // 透過特徵發送通知
        if (pControlCharacteristic) {
            pControlCharacteristic->setValue(response.c_str());
            pControlCharacteristic->notify(); 
        }

        server.send(200, "text/plain", "OK"); 
    } else {
        server.send(400, "text/plain", "Invalid arguments (Missing t or s)");
    }
}

void setupWebServer() {
    Serial.println("--- 啟動 Web Server (STA 模式) ---");
    server.on("/", HTTP_GET, handleRoot);
    server.on("/control", HTTP_GET, handleControl);
    server.onNotFound([](){
        server.send(404, "text/plain", "Not Found");
    });
    server.begin();
    Serial.println("HTTP 伺服器已啟動於 Port 80。");
}

// --- mDNS/OTA 設定 ---
void setupMdnsOtaSta() {
    Serial.println("--- 設定 mDNS 和 OTA (STA 模式) ---");
    if (MDNS.begin(globalHostname.c_str())) {
        Serial.printf("mDNS (STA 模式) 啟動: %s.local -> %s\n", 
            globalHostname.c_str(), WiFi.localIP().toString().c_str());
    } else {
        Serial.println("mDNS (STA 模式) 啟動失敗。");
    }

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

// --- 讀取 Preferences 並連線到 Wi-Fi ---
void connectToSavedWiFi() {
    preferences.begin("wifi-config", true); 
    String saved_ssid = preferences.getString("ssid", "");
    String saved_pass = preferences.getString("pass", "");
    preferences.end();
    
    if (saved_ssid.length() == 0) {
        Serial.println("無儲存的 Wi-Fi 憑證。裝置保持 BLE 廣告，等待配置...");
        return; 
    }
    
    Serial.printf("嘗試連線到儲存的 Wi-Fi: %s\n", saved_ssid.c_str());
    WiFi.setHostname(globalHostname.c_str());
    WiFi.mode(WIFI_STA); 
    WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
}

// --- BLE 連線狀態回調 ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        vTaskDelay(pdMS_TO_TICKS(1)); 
        Serial.println("✅ BLE Client Connected.");
    }
    void onDisconnect(BLEServer* pServer) {
        Serial.println("❌ BLE Client Disconnected. Setting flag to restart...");
        should_restart_advertising = true; 
        vTaskDelay(pdMS_TO_TICKS(5));
    }
};

// --- BLE 特徵寫入回調 ---
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String command = String(rxValue.c_str());
            Serial.printf("[BLE CMD] Received RAW: %s\n", command.c_str()); // <-- 新增: 顯示收到的原始字串
            
            int commaIndex = command.indexOf(',');
            if (commaIndex > 0) {
                int rawT = command.substring(0, commaIndex).toInt();
                int rawS = command.substring(commaIndex + 1).toInt();
                
                // 設定目標速度，並進行約束 (Constrain)
                targetSpeedT = constrain(rawT, -PWM_EFFECTIVE_LIMIT_T, PWM_EFFECTIVE_LIMIT_T); 
                targetSpeedS = constrain(rawS, -PWM_EFFECTIVE_LIMIT_S, PWM_EFFECTIVE_LIMIT_S);
                // 【新增】更新控制時間戳記
                lastControlTime = millis(); 
                Serial.printf("  -> Target Set: T=%d, S=%d\n", targetSpeedT, targetSpeedS);
            }
        }
    }
};

// --- BLE 配置回調 (接收 Wi-Fi 憑證) ---
class ConfigCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String data = String(rxValue.c_str());
            
            if (pCharacteristic == pSsidCharacteristic) {
                ble_ssid = data;
                Serial.printf("[BLE Config] Received SSID: %s\n", ble_ssid.c_str());
            } else if (pCharacteristic == pPassCharacteristic) {
                ble_pass = data;
                Serial.println("[BLE Config] Received Password.");
                
                if (ble_ssid.length() > 0) {
                    wifi_config_received = true; 
                    Serial.println("BLE 配置完成，準備儲存 Wi-Fi 憑證...");
                }
            }
        }
    }
};

// --- BLE 配置處理迴圈 (在收到憑證後重啟) ---
void handleBleConfigLoop() {
    if (wifi_config_received) {
        Serial.println("🚨 儲存新憑證並重啟中...");
        preferences.begin("wifi-config", false);
        preferences.putString("ssid", ble_ssid);
        preferences.putString("pass", ble_pass);
        preferences.end();
        wifi_config_received = false;
        
        delay(100); 
        ESP.restart();
    }
}

// --- 【BLE 核心修改】 setupBleServer_Bluedroid ---
void setupBleServer_Bluedroid() {
    BLEDevice::init(globalHostname.c_str());
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 1. 馬達控制服務 (使用動態 UUID)
    BLEService *pControlService = pServer->createService(controlServiceUUID.c_str());
    pControlCharacteristic = pControlService->createCharacteristic(
        CONTROL_CHAR_UUID, 
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pControlCharacteristic->addDescriptor(new BLE2902());
    pControlCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // 2. Wi-Fi 配置服務 (使用動態 UUID)
    BLEService *pConfigService = pServer->createService(configServiceUUID.c_str());
    
    pSsidCharacteristic = pConfigService->createCharacteristic(
        SSID_CHAR_UUID, 
        BLECharacteristic::PROPERTY_WRITE
    );
    pSsidCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());

    pPassCharacteristic = pConfigService->createCharacteristic(
        PASS_CHAR_UUID, 
        BLECharacteristic::PROPERTY_WRITE
    );
    pPassCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    
    // 啟動服務 (必須在創建完所有特徵後呼叫)
    pConfigService->start();
    pControlService->start(); 

    // 3. 啟動廣告 (使用 Bluedroid)
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(configServiceUUID.c_str()); 
    pAdvertising->addServiceUUID(controlServiceUUID.c_str());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    pAdvertising->start(); 

    Serial.println("✅ BLE Advertising Started (Bluedroid).");
}


// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- 初始化馬達控制腳位 (DRV8833) ---
    // 註：請確保 esp32c3_gpio.h 中定義了 NSLEEP_PIN, AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN
    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH); 
    Serial.printf("馬達驅動 (nSLEEP) 已致能於 GPIO%d\n", NSLEEP_PIN);

    // PWM 設定和連接
    ledcSetup(LEDC_CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(AIN1_PIN, LEDC_CH_A1);
    ledcAttachPin(AIN2_PIN, LEDC_CH_A2);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1);
    ledcAttachPin(BIN2_PIN, LEDC_CH_B2);

    setMotorPwm(0, 0); 
    
    // 0. 產生唯一的 Hostname
    generateHostname();
    generateUniqueUuids();

    // --- 【BLE 核心修改】直接在 setup() 中啟動 Bluedroid ---
    setupBleServer_Bluedroid();

    // 1. 嘗試連線 Wi-Fi
    connectToSavedWiFi();
}

// --- Loop ---
void loop() {    
    // 1. 超時安全停止檢查
    if (targetSpeedT != 0 || targetSpeedS != 0) {
        if (millis() - lastControlTime > CONTROL_TIMEOUT_MS) {
            Serial.println("🚨 控制超時! 強制設定目標速度為零。");
            targetSpeedT = 0;
            targetSpeedS = 0;
            
            // 可選：發送通知給 BLE 客戶端，告知已停止
            if (pControlCharacteristic) {
                pControlCharacteristic->setValue("TIMEOUT:0,0");
                pControlCharacteristic->notify(); 
            }
        }
    }
    // 2. 處理延遲的廣告重啟請求
    if (should_restart_advertising) {
        Serial.print("Loop: Executing delayed advertising restart... ");
        pServer->startAdvertising(); 
        Serial.println("✅ SUCCESS");
        should_restart_advertising = false;
    }

    // 3. 檢查連線狀態，並啟動後續服務 (只執行一次)
    if (WiFi.status() == WL_CONNECTED && !servicesStarted) {
        Serial.println("✅ Wi-Fi 連線成功，啟動服務...");
        setupMdnsOtaSta();
        setupWebServer();
        servicesStarted = true;
        Serial.println("-------------------------------------------------------");
        Serial.printf("裝置已啟動: %s.local\n", globalHostname.c_str()); 
        Serial.println("-------------------------------------------------------");
    }

    // 4. Web Server 和 OTA 僅在服務啟動後才處理
    if (servicesStarted) {
        server.handleClient();
        ArduinoOTA.handle();
    }
    
    // 5. 處理 BLE 接收到的 Wi-Fi 配置並重啟
    handleBleConfigLoop();
    
    // 6. 馬達 Ramping 邏輯
    motorRampTask();
    
    // 保持 loop() 有機會切換任務
    delay(1); 
}