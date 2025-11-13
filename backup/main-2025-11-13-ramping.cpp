// 包含必要的庫
#include <WiFi.h>
#include <ESPAsync_WiFiManager.h>    // 替換為非同步 WiFiManager 庫
#include <ESPAsyncDNSServer.h>       // 用於 Captive Portal 的 DNS 伺服器
#include <ESPAsyncWebServer.h>       // 替換為非同步 Web Server 庫
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_ota_ops.h"             // OTA 相關操作
#include "esp_partition.h"           // 分區表操作
#include "esp_task_wdt.h"            // Watchdog Timer 函式庫
#include "esp32c3_gpio.h" 

// --- 全域變數 ---
String globalHostname;              // 基於 MAC 位址的唯一 Hostname
bool isConfigurationMode = false;   // 標記是否處於 Wi-Fi 配置模式

AsyncWebServer server(80);          // 實例化 Async Web Server
ESPAsync_WiFiManager *wm;           // 實例化 Async WiFiManager
AsyncDNSServer dns;

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
// PWM_EFFECTIVE_LIMIT_T: 限制速度馬達的最高輸出 PWM。
const int PWM_EFFECTIVE_LIMIT_T = 200; 
// RAMP_ACCEL_STEP_T: 速度馬達的加速步長 (越小越平穩，越能保護電源)
const int RAMP_ACCEL_STEP_T = 5; 
// PWM_START_KICK_T: 速度馬達的啟動推力 (128 = 約 50% PWM)
const int PWM_START_KICK_T = 128;

// --- S 馬達 (轉向/Steering) Ramping 參數 ---
// PWM_EFFECTIVE_LIMIT_S: 限制轉向馬達的最高輸出 PWM。
const int PWM_EFFECTIVE_LIMIT_S = 250; 
// RAMP_ACCEL_STEP_S: 轉向馬達的加速步長 (越大越靈敏)
const int RAMP_ACCEL_STEP_S = 20; 
// PWM_START_KICK_S: 轉向馬達的啟動推力 (150 = 約 59% PWM, 略高於速度馬達以提高靈敏度)
const int PWM_START_KICK_S = 150;


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

        <!-- 搖桿區域 -->
        <div id="joystick" class="mb-6">
            <div id="joystick-inner">
                 <div id="joystick-thumb"></div>
            </div>
        </div>

        <!-- 狀態顯示區 -->
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
        const baseIp = currentIP.startsWith('192.168.4.1') ? 'http://192.168.4.1' : '';
        
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
    
    // ------------------------------------------------------------------
    // 速度馬達 (T Motor) 邏輯
    // ------------------------------------------------------------------
    if (targetSpeedT == 0) {
        // 優化修正：當目標為 0 時，強制當前速度立即為 0
        currentSpeedT = 0; 
    } else {
        // 目標不為 0 (加速或減速到新的非零目標)

        // *** 啟動推力邏輯 (Kick Start) ***
        if (currentSpeedT == 0) {
            // 如果當前速度為 0，且目標速度非 0，則跳轉到 PWM_START_KICK_T
            if (targetSpeedT > 0) {
                currentSpeedT = PWM_START_KICK_T;
            } else {
                currentSpeedT = -PWM_START_KICK_T;
            }
        }
        
        // 確保 Kick Start 後的速度不會超過目標速度
        if (abs(currentSpeedT) > abs(targetSpeedT)) {
             currentSpeedT = targetSpeedT;
        }

        // *** 繼續 Ramping 邏輯 ***
        if (abs(targetSpeedT - currentSpeedT) > RAMP_ACCEL_STEP_T) {
            if (targetSpeedT > currentSpeedT) {
                // 平穩加速
                currentSpeedT += RAMP_ACCEL_STEP_T;
            } else {
                // 平穩減速 
                currentSpeedT -= RAMP_ACCEL_STEP_T;
            }
        } else {
            // 距離目標值很近，直接設定為目標值
            currentSpeedT = targetSpeedT;
        }
    }

    // ------------------------------------------------------------------
    // 轉向馬達 (S Motor) 邏輯 (使用更激進的參數)
    // ------------------------------------------------------------------
    if (targetSpeedS == 0) {
        // 優化修正：當目標為 0 時，強制當前速度立即為 0
        currentSpeedS = 0; 
    } else {
        // 目標不為 0 (轉向加速/減速)
        
        // *** 啟動推力邏輯 (Kick Start) ***
        if (currentSpeedS == 0) {
            // 如果當前速度為 0，且目標速度非 0，則跳轉到 PWM_START_KICK_S
            if (targetSpeedS > 0) {
                currentSpeedS = PWM_START_KICK_S;
            } else {
                currentSpeedS = -PWM_START_KICK_S;
            }
        }

        // 確保 Kick Start 後的速度不會超過目標速度
        if (abs(currentSpeedS) > abs(targetSpeedS)) {
             currentSpeedS = targetSpeedS;
        }

        // *** 繼續 Ramping 邏輯 ***
        if (abs(targetSpeedS - currentSpeedS) > RAMP_ACCEL_STEP_S) {
             if (targetSpeedS > currentSpeedS) {
                currentSpeedS += RAMP_ACCEL_STEP_S;
            } else {
                currentSpeedS -= RAMP_ACCEL_STEP_S;
            }
        } else {
            // 距離目標值很近，直接設定為目標值
            currentSpeedS = targetSpeedS;
        }
    }

    // 實際寫入 PWM
    setMotorPwm(currentSpeedT, currentSpeedS); 
    
    // Serial.printf("Ramp: T(Curr/Targ)=%d/%d, S(Curr/Targ)=%d/%d\n", 
    //               currentSpeedT, targetSpeedT, currentSpeedS, targetSpeedS);
}

// --- Web Server 處理函式 (Async 版本) ---
void handleRoot(AsyncWebServerRequest *request) {
    
    String html = HTML_CONTENT; 
    // 根據當前模式顯示正確的 IP 位址
    String ipAddress = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);

    request->send(200, "text/html", html);
}

void handleControl(AsyncWebServerRequest *request) {
    if (request->hasParam("t") && request->hasParam("s")) {
        
        // 讀取原始搖桿輸入
        int rawT = request->arg("t").toInt();
        int rawS = request->arg("s").toInt();
        
        // *** 關鍵修正：將目標速度分別約束在 T 和 S 的有效限制內 ***
        targetSpeedT = constrain(rawT, -PWM_EFFECTIVE_LIMIT_T, PWM_EFFECTIVE_LIMIT_T); 
        targetSpeedS = constrain(rawS, -PWM_EFFECTIVE_LIMIT_S, PWM_EFFECTIVE_LIMIT_S);

        Serial.printf("WebControl (Target): T馬達(速度)=%d, S馬達(轉向)=%d\n", targetSpeedT, targetSpeedS);        
        request->send(200, "text/plain", "OK"); 
    } else {
        request->send(400, "text/plain", "Invalid arguments (Missing t or s)");
    }
}

void setupWebServer() {
    Serial.println("--- 啟動 Async Web Server ---");

    // 處理根目錄請求 (虛擬搖桿頁面)
    server.on("/", HTTP_GET, handleRoot);

    // 處理馬達控制 API 請求
    server.on("/control", HTTP_GET, handleControl);

    // 處理所有未定義的請求 (選用)
    server.onNotFound([](AsyncWebServerRequest *request){
        request->send(404, "text/plain", "Not Found");
    });

    server.begin();
    Serial.println("HTTP 伺服器已啟動於 Port 80 (Async)。");
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
    ArduinoOTA.setPassword("mysecurepassword"); // 請替換為您的密碼

    ArduinoOTA.onStart([]() { Serial.println("OTA 更新開始..."); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA 更新完成! 正在重啟..."); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA 錯誤碼 [%u]\n", error); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("進度: %u%%\r", (progress * 100) / total);
    });
    ArduinoOTA.begin();
    
    Serial.println("-------------------------------------------------");
}

// --- 連線或啟動 Wi-Fi 配置入口網站 (Async 版) ---
void connectToWiFi() {

    WiFi.setHostname(globalHostname.c_str());

    // 設置配置入口網站的回調函式 (AP Mode 啟動時)
    wm->setAPCallback([](ESPAsync_WiFiManager *wm) {
        isConfigurationMode = true;
        Serial.println("進入配置 AP 模式 (ESP32-Setup)。");
        Serial.println("連線 AP: " + WiFi.softAPSSID());
        Serial.println("IP 位址: " + WiFi.softAPIP().toString());
    });
    
    // 設置連線成功的處理函式
    wm->setSaveConfigCallback([](){
        isConfigurationMode = false;
        Serial.println("✅ Wi-Fi 成功配置並連線!");
    });

    Serial.println("正在啟動 AsyncWiFiManager autoConnect...");

    // *** 禁用 WDT ***
    Serial.println("⛔ 暫時禁用 Core 0 WDT...");
    disableCore0WDT();
    
    if (!wm->autoConnect("ESP32-Setup")) {        
        Serial.println("AutoConnect 失敗，設定為配置模式旗標。");
        isConfigurationMode = true;
    } else {
        isConfigurationMode = false;
        Serial.println("✅ Wi-Fi 連線成功!");
    }

    // *** 重新啟用 WDT ***
    Serial.println("✅ 重新啟用 Core 0 WDT...");
    enableCore0WDT();

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
    
    // --- 啟動器核心邏輯 ---
    wm = new ESPAsync_WiFiManager(&server, &dns, "ESP32-Setup");
    
    // 0. 產生唯一的 Hostname
    generateHostname();

    // 1. 連線 Wi-Fi (AsyncWiFiManager 的 autoConnect)
    connectToWiFi();

    if (!isConfigurationMode) {
        // 2. Setup mDNS and OTA (STA Mode)
        setupMdnsOtaSta();

        // 3. Setup Web Server (STA Mode)
        setupWebServer();

        Serial.println("-------------------------------------------------------");
        // 這段訊息通常是 Launcher 的 Log，保持不變
        Serial.println("⚠️ otadata 未指向有效的 OTA 應用程式。停留在啟動器模式。"); 
        Serial.println("-------------------------------------------------------");

    } else {
        Serial.println("⚠️ 進入 AP 配置模式。只啟用 AsyncWiFiManager Portal。");
        // 在 AP 模式下，AsyncWiFiManager 的 Web Portal 會自動運行。
    }
}

// --- Loop ---
void loop() {
    // 由於使用了 AsyncWebServer，我們只需要處理 OTA
    ArduinoOTA.handle();
    // *** 關鍵：定時執行馬達 Ramping 任務 ***
    motorRampTask();
    // AsyncWebServer 在內部 FreeRTOS 任務中運行，無需 server.handleClient()
    yield();
}

// app_main 保持不變
extern "C" void app_main()
{
    initArduino();   
    setup();         
    for (;;) {
        loop();      
        delay(1); // 減少阻塞時間
    }
}