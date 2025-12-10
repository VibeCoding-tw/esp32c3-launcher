// 包含必要的庫
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>               // Web Server
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_task_wdt.h"            // Watchdog Timer 函式庫
#include <BLEDevice.h>               // Bluedroid 庫
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>             // 用於儲存 Wi-Fi / 馬達參數
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32c3_gpio.h"            // 假設此處定義了 GPIO 腳位 (例如: AIN1_PIN, NSLEEP_PIN)

// --- 全域變數 ---
String globalHostname;              // 基於 MAC 位址的唯一 Hostname
WebServer server(80);                // 實例化同步 Web Server
Preferences preferences;             // NVS 偏好設定實例

// LEDC PWM 設定
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
const int LEDC_CH_A1 = 0; // T Motor 正轉通道
const int LEDC_CH_A2 = 1; // T Motor 反轉通道
const int LEDC_CH_B1 = 2; // S Motor 正轉通道
const int LEDC_CH_B2 = 3; // S Motor 反轉通道

// --- 馬達 Ramping 參數結構體 ---
typedef struct {
    unsigned long controlTimeoutMs; 
    int pwmEffectiveLimitT; 
    int rampAccelStepT; 
    int pwmStartKickT; 
    int pwmEffectiveLimitS; 
    int rampAccelStepS; 
    int pwmStartKickS;
} MotorConfig_t;

// --- 馬達 Ramping 核心變數 ---
MotorConfig_t motorConfig; // 實例化結構體來存儲當前參數
volatile unsigned long lastControlTime = 0; // 上次接收到控制命令的時間戳記
volatile int targetSpeedT = 0; // 遙控器送來的目標速度 (T Motor)
volatile int currentSpeedT = 0; // 當前實際輸出給 PWM 的速度
volatile int targetSpeedS = 0; // 遙控器送來的目標速度 (S Motor)
volatile int currentSpeedS = 0; // 當前實際輸出給 PWM 的速度
const int RAMP_INTERVAL_MS = 10; // 馬達斜坡控制間隔 (100Hz)
unsigned long lastRampTime = 0;

// --- BLE UUID 定義 (已修正，確保唯一性) ---
// 服務 (Service)
const char* CONFIG_SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"; // 常用 UART 服務 (Wi-Fi Config)
const char* CONTROL_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c0ffee00dead"; // 自定義控制服務

// 特徵 (Characteristic) - **關鍵：修正為唯一的 UUIDs**
#define SSID_CHAR_UUID          "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Wi-Fi SSID (Write)
#define PASS_CHAR_UUID          "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Wi-Fi PASS (Write)
#define MOTOR_CONFIG_CHAR_UUID  "6e400004-b5a3-f393-e0a9-e50e24dcca9e" // 馬達配置參數 (Read/Notify)
// 使用不同的 UUID 命名空間
#define CONTROL_CHAR_UUID       "4fafc202-1fb5-459e-8fcc-c0ffee01feed" // 馬達即時控制 (Write/Notify)

// --- 全域變數：BLE / Wi-Fi Config ---
BLEServer *pServer = NULL;
BLECharacteristic *pControlCharacteristic = NULL;
BLECharacteristic *pSsidCharacteristic = NULL;
BLECharacteristic *pPassCharacteristic = NULL;
BLECharacteristic *pMotorConfigCharacteristic = NULL; 
String ble_ssid;
String ble_pass;
bool wifi_config_received = false;
volatile bool should_restart_advertising = false;
bool servicesStarted = false; // 追蹤 Web 和 OTA 是否已啟動 (STA 模式)

// --- 輔助函數: 實際寫入 PWM 值 ---
void setMotorPwm(int speedT, int speedS) {
    // 限制速度在 PWM_MAX 範圍內
    speedT = constrain(speedT, -PWM_MAX, PWM_MAX);
    speedS = constrain(speedS, -PWM_MAX, PWM_MAX);

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
        // 假設 BIN1/BIN2 為正反轉控制，根據硬體接線調整
        ledcWrite(LEDC_CH_B1, speedS);
        ledcWrite(LEDC_CH_B2, 0);
    } else if (speedS < 0) { 
        ledcWrite(LEDC_CH_B1, 0); 
        ledcWrite(LEDC_CH_B2, -speedS);
    } else { 
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, 0);
    }
}


// --- 網頁 HTML 內容 (單一檔案，包含所有 CSS/JS) ---
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
        /* 【重要】設定表單的顯示/隱藏 */
        #configModal {
            position: fixed;
            top: 0; left: 0; right: 0; bottom: 0;
            background: rgba(0, 0, 0, 0.8);
            z-index: 100;
            display: none; /* 預設隱藏 */
        }
        .config-card {
            max-height: 90vh;
            overflow-y: auto;
        }
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

        <div class="text-center space-y-2 mb-6">
            <p class="text-xl">狀態: <span id="status" class="text-green-400">靜止</span></p>
            <p class="text-xs text-gray-500">
                X (轉向): <span id="val_x">0</span> | Y (速度): <span id="val_y">0</span>
            </p>
        </div>
        
        <!-- 【新增】組態設定按鈕 -->
        <button id="btnOpenConfig" class="w-full bg-gray-700 text-gray-300 py-3 rounded-xl font-bold transition duration-150 hover:bg-gray-600">
            馬達組態設定
        </button>
    </div>

    <!-- 【新增】組態設定彈窗 -->
    <div id="configModal" class="flex justify-center items-center">
        <div class="config-card bg-gray-900 p-6 rounded-xl shadow-2xl w-full max-w-sm">
            <h2 class="text-2xl font-bold text-indigo-400 mb-4">馬達參數設定</h2>
            
            <form id="configForm" class="space-y-4">
                <!-- PWM_EFFECTIVE_LIMIT_T -->
                <div>
                    <label for="pwmEffectiveLimitT" class="block text-sm font-medium text-gray-400">速度上限 (T Limit)</label>
                    <input type="number" id="pwmEffectiveLimitT" name="pwmEffectiveLimitT" min="1" max="255" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>

                <!-- RAMP_ACCEL_STEP_T -->
                <div>
                    <label for="rampAccelStepT" class="block text-sm font-medium text-gray-400">T 加速步長 (Accel Step)</label>
                    <input type="number" id="rampAccelStepT" name="rampAccelStepT" min="1" max="100" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>

                <!-- PWM_START_KICK_T -->
                <div>
                    <label for="pwmStartKickT" class="block text-sm font-medium text-gray-400">T 啟動突波 (Start Kick)</label>
                    <input type="number" id="pwmStartKickT" name="pwmStartKickT" min="0" max="255" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>
                
                <!-- PWM_EFFECTIVE_LIMIT_S -->
                 <div>
                    <label for="pwmEffectiveLimitS" class="block text-sm font-medium text-gray-400">轉向上限 (S Limit)</label>
                    <input type="number" id="pwmEffectiveLimitS" name="pwmEffectiveLimitS" min="1" max="255" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>
                
                <!-- RAMP_ACCEL_STEP_S -->
                <div>
                    <label for="rampAccelStepS" class="block text-sm font-medium text-gray-400">S 加速步長 (Accel Step)</label>
                    <input type="number" id="rampAccelStepS" name="rampAccelStepS" min="1" max="100" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>

                <!-- PWM_START_KICK_S -->
                <div>
                    <label for="pwmStartKickS" class="block text-sm font-medium text-gray-400">S 啟動突波 (Start Kick)</label>
                    <input type="number" id="pwmStartKickS" name="pwmStartKickS" min="0" max="255" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>
                
                <!-- CONTROL_TIMEOUT_MS -->
                <div>
                    <label for="controlTimeoutMs" class="block text-sm font-medium text-gray-400">控制超時 (ms)</label>
                    <input type="number" id="controlTimeoutMs" name="controlTimeoutMs" min="100" max="5000" required class="mt-1 block w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white">
                </div>

                <button type="submit" class="w-full bg-indigo-600 text-white p-3 rounded-xl font-bold transition duration-150 hover:bg-indigo-700 mt-6">
                    儲存參數並重啟
                </button>
            </form>

            <button id="btnCloseConfig" class="w-full bg-red-600 text-white p-3 rounded-xl font-bold transition duration-150 hover:bg-red-700 mt-3">
                關閉
            </button>
            <div id="configStatus" class="text-center mt-3 text-sm text-yellow-400"></div>
        </div>
    </div>

    <script>
        // 核心搖桿控制邏輯 (與舊版相同，確保 Web/Wi-Fi 控制正常)
        const joystickContainer = document.getElementById('joystick'); 
        const joystick = document.getElementById('joystick-inner'); 
        const thumb = document.getElementById('joystick-thumb');
        const statusEl = document.getElementById('status');
        const valXEl = document.getElementById('val_x');
        const valYEl = document.getElementById('val_y');
        
        const DEADZONE_PWM = 20; 
        let maxRadius = 0; 
        let isDragging = false;
        let controlInterval;
        let lastMotorT = 0; 
        let lastMotorS = 0; 
        const baseIp = ''; 
        
        // 初始化搖桿尺寸
        window.addEventListener('load', () => {
             // 確保在元素渲染完成後獲取正確的尺寸
            const containerRect = joystick.getBoundingClientRect();
            maxRadius = containerRect.width / 2;
        });
        
        // 在視窗大小改變時更新 maxRadius，確保響應性
        window.addEventListener('resize', () => {
            const containerRect = joystick.getBoundingClientRect();
            maxRadius = containerRect.width / 2;
        });


        function updateMotorValues(rawX, rawY) {
            if (maxRadius === 0) return;

            const distance = Math.sqrt(rawX*rawX + rawY*rawY);
            const magnitude = Math.min(1.0, distance / maxRadius);
            const angle = Math.atan2(rawY, rawX);
            
            const normX = magnitude * Math.cos(angle); 
            const normY = magnitude * Math.sin(angle); 

            let speedT = Math.round(normY * 255);
            let speedS = Math.round(normX * 255);

            if (Math.abs(speedT) < DEADZONE_PWM) {
                speedT = 0;
            }
            if (Math.abs(speedS) < DEADZONE_PWM) {
                speedS = 0;
            }

            valYEl.textContent = speedT; 
            valXEl.textContent = speedS; 
            
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

            if (speedT !== lastMotorT || speedS !== lastMotorS) {
                lastMotorT = speedT;
                lastMotorS = speedS;
                sendControl(speedT, speedS); 
            }
        }

        function sendControl(T, S) {
            // 使用非同步請求發送馬達速度
            fetch(`${baseIp}/control?t=${T}&s=${S}`, { method: 'GET' })
                .then(response => {
                    // if (!response.ok) { console.error('Server responded with an error:', response.status); }
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
            updateMotorValues(0, 0); 
        }

        function handleMove(e) {
            e.preventDefault();
            if (!isDragging || maxRadius === 0) return;

            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;

            const rect = joystick.getBoundingClientRect();
            const centerX = rect.left + maxRadius;
            const centerY = rect.top + maxRadius;

            let offsetX = clientX - centerX;
            let offsetY = clientY - centerY; 
            
            const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            if (distance > maxRadius) {
                const angle = Math.atan2(offsetY, offsetX);
                offsetX = maxRadius * Math.cos(angle);
                offsetY = maxRadius * Math.sin(angle);
            }
            
            const thumbX = maxRadius + offsetX;
            const thumbY = maxRadius + offsetY; 

            thumb.style.left = `${thumbX}px`;
            thumb.style.top = `${thumbY}px`;
            thumb.style.transform = 'translate(-50%, -50%)';

            // 將 CSS Y 軸反轉 (向上為正)
            updateMotorValues(offsetX, -offsetY);
        }

        function handleStart(e) {
            // 避免觸發多次，只處理第一個觸摸點或滑鼠點擊
            if(isDragging) return; 

            isDragging = true;
            thumb.classList.add('active');
            
            // 確保只處理單點觸摸或滑鼠
            if (e.touches && e.touches.length > 1) return;
            
            handleMove(e); // 立即更新一次位置和值

            if (controlInterval) clearInterval(controlInterval);
            // 每 100ms 發送一次控制命令，維持連線和馬達狀態
            controlInterval = setInterval(() => {
                sendControl(lastMotorT, lastMotorS);
            }, 100); 
        }

        function handleEnd() {
            stopMotors();
        }

        // --- 搖桿事件監聽 ---
        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);

        joystick.addEventListener('touchstart', handleStart);
        document.addEventListener('touchmove', handleMove);
        joystickContainer.addEventListener('touchend', handleEnd); 

        // 初始化時發送一次停止命令
        stopMotors(); 


        // ===============================================
        // 【新增】馬達組態設定邏輯
        // ===============================================

        const configModal = document.getElementById('configModal');
        const configForm = document.getElementById('configForm');
        const btnOpenConfig = document.getElementById('btnOpenConfig');
        const btnCloseConfig = document.getElementById('btnCloseConfig');
        const configStatusEl = document.getElementById('configStatus');

        btnOpenConfig.addEventListener('click', () => {
            fetchConfigAndOpenModal();
        });

        btnCloseConfig.addEventListener('click', () => {
            configModal.style.display = 'none';
        });

        async function fetchConfigAndOpenModal() {
            configStatusEl.textContent = '正在讀取裝置設定...';
            try {
                // 1. 讀取設定 API (GET /config)
                const response = await fetch(`${baseIp}/config`, { method: 'GET' });
                if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
                
                const data = await response.json();
                
                // 2. 將讀到的值填入表單
                document.getElementById('controlTimeoutMs').value = data.CONTROL_TIMEOUT_MS;
                document.getElementById('pwmEffectiveLimitT').value = data.PWM_EFFECTIVE_LIMIT_T;
                document.getElementById('rampAccelStepT').value = data.RAMP_ACCEL_STEP_T;
                document.getElementById('pwmStartKickT').value = data.PWM_START_KICK_T;
                document.getElementById('pwmEffectiveLimitS').value = data.PWM_EFFECTIVE_LIMIT_S;
                document.getElementById('rampAccelStepS').value = data.RAMP_ACCEL_STEP_S;
                document.getElementById('pwmStartKickS').value = data.PWM_START_KICK_S;

                configStatusEl.textContent = '讀取成功。';
                configModal.style.display = 'flex';

            } catch (error) {
                console.error('Failed to fetch config:', error);
                configStatusEl.textContent = `讀取失敗: ${error.message}`;
            }
        }
        
        configForm.addEventListener('submit', async (e) => {
            e.preventDefault();
            configStatusEl.textContent = '正在儲存設定...';
            
            const formData = new FormData(configForm);
            const data = {};
            formData.forEach((value, key) => { data[key] = parseInt(value); }); // 確保轉換為數字

            // 3. 儲存設定 API (POST /config)
            try {
                const response = await fetch(`${baseIp}/config`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(data)
                });

                if (response.ok) {
                    configStatusEl.textContent = '✅ 儲存成功！裝置正在重啟。';
                    setTimeout(() => {
                        configModal.style.display = 'none';
                        window.location.reload(); 
                    }, 2000);
                } else {
                    const errorText = await response.text();
                    throw new Error(`儲存失敗: ${errorText}`);
                }

            } catch (error) {
                console.error('Failed to save config:', error);
                configStatusEl.textContent = `❌ 儲存失敗: ${error.message}`;
            }
        });
    </script>
</body>
</html>
)rawliteral";


// 產生基於 MAC 位址的 Hostname
void generateHostname() {    
    uint8_t mac[6];
    WiFi.macAddress(mac);
    // 產生格式: esp32c3-XXYYZZ
    globalHostname.reserve(15);
    globalHostname = "esp32c3-";
    for(int i=0; i<6; i++) {
        char buf[3];
        sprintf(buf, "%02x", mac[i]);
        globalHostname += String(buf);
    }
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}


// --- 馬達參數處理函式 ---
// 設定預設參數
void setDefaultMotorConfig() {
    motorConfig.controlTimeoutMs = 1000; 
    motorConfig.pwmEffectiveLimitT = 250; 
    motorConfig.rampAccelStepT = 20; 
    motorConfig.pwmStartKickT = 180; 
    motorConfig.pwmEffectiveLimitS = 250; 
    motorConfig.rampAccelStepS = 20; 
    motorConfig.pwmStartKickS = 180;
}

// 從 NVS 載入參數 (如果沒有，使用預設值)
void loadMotorConfig() {
    setDefaultMotorConfig(); // 先設定預設值
    
    preferences.begin("motor-cfg", true); // 以唯讀模式開啟命名空間
    size_t size = preferences.getBytesLength("config_data");
    
    if (size == sizeof(MotorConfig_t)) {
        preferences.getBytes("config_data", &motorConfig, size);
        Serial.println("✅ 馬達參數已從 NVS 載入。");
    } else {
        Serial.printf("⚠️ NVS 中無馬達參數 (Size: %u), 使用預設值。\n", size);
    }
    preferences.end();
}

// 儲存參數到 NVS
void saveMotorConfig() {
    preferences.begin("motor-cfg", false); // 以讀寫模式開啟命名空間
    preferences.putBytes("config_data", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();
    Serial.println("💾 馬達參數已儲存到 NVS。");
}

/**
 * @brief 定時馬達斜坡控制任務 (修復了減速邏輯)
 */
void motorRampTask() {
    if (millis() - lastRampTime < RAMP_INTERVAL_MS) return;
    lastRampTime = millis();
    
    const int RAMP_ACCEL_STEP_T = motorConfig.rampAccelStepT;
    const int PWM_START_KICK_T = motorConfig.pwmStartKickT;
    const int RAMP_ACCEL_STEP_S = motorConfig.rampAccelStepS;
    const int PWM_START_KICK_S = motorConfig.pwmStartKickS;
    
    // --- 速度馬達 (T Motor) 邏輯 ---
    // 目標速度 (targetSpeedT) 由 Web/BLE 控制函式設定
    
    // 1. 如果目標速度為零，且當前速度不為零，進行平穩減速
    if (targetSpeedT == 0) {
        if (currentSpeedT > 0) {
            currentSpeedT = max(0, currentSpeedT - RAMP_ACCEL_STEP_T);
        } else if (currentSpeedT < 0) {
            currentSpeedT = min(0, currentSpeedT + RAMP_ACCEL_STEP_T);
        }
    } 
    // 2. 如果目標速度不為零
    else {
        int dirT = (targetSpeedT > 0) ? 1 : -1;
        
        if (abs(currentSpeedT) < PWM_START_KICK_T && abs(targetSpeedT) >= PWM_START_KICK_T && currentSpeedT == 0) {
            // 啟動突波 (Jump-start to kick value)
            currentSpeedT = dirT * PWM_START_KICK_T;
            // 確保啟動突波不會超過目標速度
            if (abs(currentSpeedT) > abs(targetSpeedT)) {
                 currentSpeedT = targetSpeedT;
            }
        } 
        // 斜坡加速/減速
        else if (abs(targetSpeedT) > abs(currentSpeedT)) {
            // 加速
            currentSpeedT += dirT * RAMP_ACCEL_STEP_T;
        } else if (abs(targetSpeedT) < abs(currentSpeedT)) {
            // 減速 (例如從 200 降到 100)
            currentSpeedT -= dirT * RAMP_ACCEL_STEP_T;
        }

        // 確保不會超過目標值 (防止過衝)
        if (dirT > 0 && currentSpeedT > targetSpeedT) currentSpeedT = targetSpeedT;
        if (dirT < 0 && currentSpeedT < targetSpeedT) currentSpeedT = targetSpeedT;
    }


    // --- 轉向馬達 (S Motor) 邏輯 (同上，進行修復) --- 
    
    if (targetSpeedS == 0) {
        if (currentSpeedS > 0) {
            currentSpeedS = max(0, currentSpeedS - RAMP_ACCEL_STEP_S);
        } else if (currentSpeedS < 0) {
            currentSpeedS = min(0, currentSpeedS + RAMP_ACCEL_STEP_S);
        }
    } else {
        int dirS = (targetSpeedS > 0) ? 1 : -1;

        if (abs(currentSpeedS) < PWM_START_KICK_S && abs(targetSpeedS) >= PWM_START_KICK_S && currentSpeedS == 0) {
            // 啟動突波
            currentSpeedS = dirS * PWM_START_KICK_S;
            if (abs(currentSpeedS) > abs(targetSpeedS)) {
                 currentSpeedS = targetSpeedS;
            }
        } 
        // 斜坡加速/減速
        else if (abs(targetSpeedS) > abs(currentSpeedS)) {
            currentSpeedS += dirS * RAMP_ACCEL_STEP_S;
        } else if (abs(targetSpeedS) < abs(currentSpeedS)) {
            currentSpeedS -= dirS * RAMP_ACCEL_STEP_S;
        }
        
        if (dirS > 0 && currentSpeedS > targetSpeedS) currentSpeedS = targetSpeedS;
        if (dirS < 0 && currentSpeedS < targetSpeedS) currentSpeedS = targetSpeedS;
    }

    setMotorPwm(currentSpeedT, currentSpeedS); 
}

// --- Web Server 處理函式 ---
void handleRoot() {
    String html = HTML_CONTENT; 
    String ipAddress = WiFi.localIP().toString();
    // 替換佔位符
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);
    server.send(200, "text/html", html);
}

void handleControl() {
    if (server.hasArg("t") && server.hasArg("s")) {
        int rawT = server.arg("t").toInt();
        int rawS = server.arg("s").toInt();
        
        // 使用 motorConfig 的極限值來約束
        targetSpeedT = constrain(rawT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT); 
        targetSpeedS = constrain(rawS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
        
        lastControlTime = millis();
        // Serial.printf("WebControl (Target): T馬達(速度)=%d, S馬達(轉向)=%d\n", targetSpeedT, targetSpeedS);   

        String response = "T:" + String(targetSpeedT) + ",S:" + String(targetSpeedS);    
        
        // 如果有 BLE 客戶端連線，也發送通知
        if (pControlCharacteristic && pControlCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902)) != nullptr &&
            pControlCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902))->getValue()[0] == 0x01) {
            pControlCharacteristic->setValue(response.c_str());
            pControlCharacteristic->notify(); 
        }

        server.send(200, "text/plain", "OK"); 
    } else {
        server.send(400, "text/plain", "Invalid arguments (Missing t or s)");
    }
}

// --- 處理 GET 請求: 讀取馬達參數 ---
void handleGetConfig() {
    // 將 motorConfig 結構體打包成 JSON 格式
    String json = "{";
    json += "\"CONTROL_TIMEOUT_MS\":" + String(motorConfig.controlTimeoutMs) + ",";
    json += "\"PWM_EFFECTIVE_LIMIT_T\":" + String(motorConfig.pwmEffectiveLimitT) + ",";
    json += "\"RAMP_ACCEL_STEP_T\":" + String(motorConfig.rampAccelStepT) + ",";
    json += "\"PWM_START_KICK_T\":" + String(motorConfig.pwmStartKickT) + ",";
    json += "\"PWM_EFFECTIVE_LIMIT_S\":" + String(motorConfig.pwmEffectiveLimitS) + ",";
    json += "\"RAMP_ACCEL_STEP_S\":" + String(motorConfig.rampAccelStepS) + ",";
    json += "\"PWM_START_KICK_S\":" + String(motorConfig.pwmStartKickS);
    json += "}";

    server.send(200, "application/json", json);
}

// --- 處理 POST 請求: 寫入馬達參數 ---
void handlePostConfig() {
    if (server.hasArg("plain")) {
        String jsonStr = server.arg("plain");
        Serial.printf("Received Config JSON: %s\n", jsonStr.c_str());

        // --- 簡易 JSON 解析 (僅適用於固定且單層的結構) ---
        // 這是為了避免引入複雜的 ArduinoJson 庫而採用的臨時方案
        #define FIND_AND_PARSE(key, targetVar) \
            { \
                int start = jsonStr.indexOf('\"' + String(key) + '\"'); \
                if (start != -1) { \
                    start = jsonStr.indexOf(':', start); \
                    int end = jsonStr.indexOf(',', start); \
                    if (end == -1) end = jsonStr.indexOf('}', start); \
                    if (start != -1 && end != -1) { \
                        targetVar = jsonStr.substring(start + 1, end).toInt(); \
                    } \
                } \
            }
        
        // 執行解析並更新 motorConfig
        FIND_AND_PARSE("controlTimeoutMs", motorConfig.controlTimeoutMs);
        FIND_AND_PARSE("pwmEffectiveLimitT", motorConfig.pwmEffectiveLimitT);
        FIND_AND_PARSE("rampAccelStepT", motorConfig.rampAccelStepT);
        FIND_AND_PARSE("pwmStartKickT", motorConfig.pwmStartKickT);
        FIND_AND_PARSE("pwmEffectiveLimitS", motorConfig.pwmEffectiveLimitS);
        FIND_AND_PARSE("rampAccelStepS", motorConfig.rampAccelStepS);
        FIND_AND_PARSE("pwmStartKickS", motorConfig.pwmStartKickS);
        
        // 確保數值在合理範圍內 (例如, 步長不能為 0)
        motorConfig.rampAccelStepT = max(1, motorConfig.rampAccelStepT);
        motorConfig.rampAccelStepS = max(1, motorConfig.rampAccelStepS);
        // ----------------------------------------------------

        // 儲存並重啟
        saveMotorConfig();
        server.send(200, "text/plain", "Config saved. Restarting.");
        delay(100);
        ESP.restart();

    } else {
        server.send(400, "text/plain", "Invalid request body.");
    }
}


void setupWebServer() {
    Serial.println("--- 啟動 Web Server (STA 模式) ---");
    server.on("/", HTTP_GET, handleRoot);
    server.on("/control", HTTP_GET, handleControl);
    server.on("/config", HTTP_GET, handleGetConfig);    // 讀取參數
    server.on("/config", HTTP_POST, handlePostConfig);  // 寫入參數
    server.onNotFound([](){
        server.send(404, "text/plain", "Not Found");
    });
    server.begin();
    Serial.println("HTTP 伺服器已啟動於 Port 80。");
}

void setupMdnsOtaSta() {
    // 啟動 mDNS (讓您可以透過 hostanme.local 訪問)
    if (!MDNS.begin(globalHostname.c_str())) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.printf("MDNS responder started: %s.local\n", globalHostname.c_str());
        MDNS.addService("http", "tcp", 80);
    }
    
    // 啟動 OTA 服務
    ArduinoOTA
        .onStart([]() {
            Serial.println("Start updating...");
        })
        .onEnd([]() {
            Serial.println("\nEnd OTA Update.");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("OTA 服務已啟動。");
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

    // 阻塞等待連線 (最多 10 秒)
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
        // 這裡需要 WDT 重設，否則可能在等待 Wi-Fi 時超時
        esp_task_wdt_reset(); 
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n成功連線! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWi-Fi 連線失敗，進入 BLE 配置模式。");
        WiFi.disconnect(true); // 斷開失敗的連線
    }
}

// --- BLE 伺服器連線回調 ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("BLE Client Connected.");
        BLEDevice::stopAdvertising(); // 連線後停止廣告
    }
    void onDisconnect(BLEServer* pServer) {
        Serial.println("BLE Client Disconnected. Restarting advertising...");
        // 延遲重啟廣告，確保資源已釋放
        should_restart_advertising = true; 
    }
};

// --- BLE 特徵寫入回調 (馬達控制) ---
class MotorCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String command = String(rxValue.c_str());
            Serial.printf("[BLE CMD] Received RAW: %s\n", command.c_str()); 
            
            // 預期格式: T,S (例如: 200,-100)
            int commaIndex = command.indexOf(',');
            if (commaIndex > 0) {
                int rawT = command.substring(0, commaIndex).toInt();
                int rawS = command.substring(commaIndex + 1).toInt();
                
                // 設定目標速度，並進行約束 (Constrain)
                targetSpeedT = constrain(rawT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT); 
                targetSpeedS = constrain(rawS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);

                lastControlTime = millis();
                // Serial.printf("BLEControl (Target): T馬達(速度)=%d, S馬達(轉向)=%d\n", targetSpeedT, targetSpeedS); 
            } else {
                Serial.printf("[BLE CMD] Invalid format: %s\n", command.c_str());
            }
        }
    }
};

// --- BLE 特徵寫入回調 (Wi-Fi Config) ---
class ConfigCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String uuid = pCharacteristic->getUUID().toString().c_str();

            if (uuid.equals(SSID_CHAR_UUID)) {
                ble_ssid = String(rxValue.c_str());
                Serial.printf("Received BLE SSID: %s\n", ble_ssid.c_str());
            } else if (uuid.equals(PASS_CHAR_UUID)) {
                ble_pass = String(rxValue.c_str());
                Serial.println("Received BLE PASS (Stored but not shown)");
                
                // 確保同時收到 SSID 和 PASS 後才儲存
                if (ble_ssid.length() > 0) {
                    wifi_config_received = true;
                }
            }
        }
    }
};

void setupBle() {
    Serial.println("--- 啟動 BLE 服務器 ---");
    BLEDevice::init(globalHostname.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 1. 設置 Wi-Fi 配置服務
    BLEService *pConfigService = pServer->createService(CONFIG_SERVICE_UUID);
    
    // SSID Characteristic (Write)
    pSsidCharacteristic = pConfigService->createCharacteristic(
                                          SSID_CHAR_UUID,
                                          BLECharacteristic::PROPERTY_WRITE
                                      );
    pSsidCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    
    // PASS Characteristic (Write)
    pPassCharacteristic = pConfigService->createCharacteristic(
                                          PASS_CHAR_UUID,
                                          BLECharacteristic::PROPERTY_WRITE
                                      );
    pPassCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    
    // Motor Config Characteristic (Read/Notify)
    pMotorConfigCharacteristic = pConfigService->createCharacteristic(
                                              MOTOR_CONFIG_CHAR_UUID,
                                              BLECharacteristic::PROPERTY_READ |
                                              BLECharacteristic::PROPERTY_NOTIFY
                                          );
    pMotorConfigCharacteristic->addDescriptor(new BLE2902());

    // 2. 設置控制服務
    BLEService *pControlService = pServer->createService(CONTROL_SERVICE_UUID);
    
    // Control Characteristic (Write/Notify)
    pControlCharacteristic = pControlService->createCharacteristic(
                                                CONTROL_CHAR_UUID,
                                                BLECharacteristic::PROPERTY_WRITE |
                                                BLECharacteristic::PROPERTY_NOTIFY
                                            );
    pControlCharacteristic->setCallbacks(new MotorCharacteristicCallbacks()); // 注意這裡使用新的類別名稱
    pControlCharacteristic->addDescriptor(new BLE2902());
    
    // 啟動服務
    pConfigService->start();
    pControlService->start();

    // 開始廣告
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
    pAdvertising->addServiceUUID(CONTROL_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE 廣告已開始，等待客戶端連線...");
}

void setupPwm() {
    // T Motor (速度)
    ledcSetup(LEDC_CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(AIN1_PIN, LEDC_CH_A1);
    ledcSetup(LEDC_CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(AIN2_PIN, LEDC_CH_A2);

    // S Motor (轉向)
    ledcSetup(LEDC_CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1);
    ledcSetup(LEDC_CH_B2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BIN2_PIN, LEDC_CH_B2);
    
    // 啟用馬達驅動 (NSLEEP)
    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH);
}


void setup() {
    // 啟動 WDT 並設定超時為 10 秒
    // ⚠️ 如果系統在 loop() 中沒有規律重設 WDT，將會重啟
    esp_task_wdt_init(10, true); 
    esp_task_wdt_add(NULL); // 將主任務加入 WDT 監控

    Serial.begin(115200);
    delay(500);

    // 載入馬達參數
    loadMotorConfig();
    
    // 初始化 PWM/GPIO
    setupPwm();

    // 產生 Hostname
    generateHostname();

    // 嘗試連線到儲存的 Wi-Fi
    connectToSavedWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        // 如果連線成功，啟動 Web Server 和 OTA
        setupWebServer();
        setupMdnsOtaSta();
        servicesStarted = true;
    } else {
        // 如果連線失敗，啟動 BLE 進行配置
        setupBle();
        servicesStarted = false;
    }
}

void loop() {
    // 核心：重設 WDT，確保系統不會因為超時而重啟
    esp_task_wdt_reset(); 

    if (servicesStarted) {
        // Wi-Fi 模式下的服務
        server.handleClient();
        ArduinoOTA.handle();
    } else {
        // BLE 模式下的服務
        // 處理 BLE 廣告重啟
        if (should_restart_advertising) {
            delay(100); // 確保連線資源釋放
            BLEDevice::startAdvertising();
            Serial.println("BLE 廣告已重啟。");
            should_restart_advertising = false;
        }

        // 處理 Wi-Fi 配置接收
        if (wifi_config_received) {
            // 儲存新的 Wi-Fi 憑證
            preferences.begin("wifi-config", false);
            preferences.putString("ssid", ble_ssid);
            preferences.putString("pass", ble_pass);
            preferences.end();
            Serial.println("✅ 新的 Wi-Fi 憑證已儲存。裝置即將重啟...");
            delay(100); 
            ESP.restart();
        }
    }
    
    // --- 【核心】馬達控制與保護邏輯 (不論 Wi-Fi 或 BLE 模式皆執行) ---
    // 1. Ramping 控制
    motorRampTask();

    // 2. 超時保護
    if (millis() - lastControlTime > motorConfig.controlTimeoutMs) {
        if (targetSpeedT != 0 || targetSpeedS != 0) {
            Serial.println("⚠️ 控制超時！馬達停止。");
            // 將目標速度設為 0，讓 motorRampTask 進行平穩減速
            targetSpeedT = 0;
            targetSpeedS = 0;
            // 由於 motorRampTask 會處理，這裡只需更新時間，避免重複警告
            lastControlTime = millis(); 
        }
    }
}