// 包含必要的庫
#include <WiFi.h>
#include <WiFiManager.h>      // 使用標準同步 (Blocking) 的 WiFiManager 庫
#include <ArduinoOTA.h>       // 透過網路進行韌體更新
#include <ESPmDNS.h>          // 區域網路名稱解析
#include <WebServer.h>        // Web Server 庫
#include "esp_ota_ops.h"      // OTA 相關操作
#include "esp_partition.h"    // 分區表操作
#include "esp32c3_gpio.h" 

// --- 全域變數 ---
String globalHostname; // 儲存基於 MAC 位址的唯一 Hostname

// --- Configuration 設定 ---
const int JUMP_DELAY_SEC = 5;       // 跳轉到用戶應用程式前的倒數秒數
bool isConfigurationMode = false;   // 標記是否處於 Wi-Fi 配置模式

WiFiManager wm;       // 使用標準同步 (Blocking) 的 WiFiManager
WebServer server(80); // Web Server 實例，監聽 Port 80

// LEDC PWM 設定
const int PWM_FREQ = 20000;        // 頻率 (Hz)
const int PWM_RESOLUTION = 8;      // 解析度 8-bit (0-255)

// PWM 通道 (用於 DRV8833 的四個輸入腳)
const int LEDC_CH_A1 = 0;       // 馬達 A (速度) - AIN1
const int LEDC_CH_A2 = 1;       // 馬達 A (速度) - AIN2
const int LEDC_CH_B1 = 2;       // 馬達 B (轉向) - BIN1
const int LEDC_CH_B2 = 3;       // 馬達 B (轉向) - BIN2

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
                X (轉向): <span id="val_x">0.00</span> | Y (速度): <span id="val_y">0.00</span>
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
        
        // maxRadius 是實際拖曳區域 (joystick-inner) 的半徑
        const maxRadius = joystick.clientWidth / 2;
        let isDragging = false;
        let controlInterval;
        let lastMotorA = 0; // 上次發送的 A 馬達速度
        let lastMotorB = 0; // 上次發送的 B 馬達速度

        // 檢查當前 IP，用於 AP 模式下的絕對路徑
        const currentIP = document.getElementById('ipaddress').textContent;
        const baseIp = currentIP.startsWith('192.168.4.1') ? 'http://192.168.4.1' : '';
        
        /**
         * @brief 根據搖桿位置 (Cartesian 座標) 計算並發送馬達速度。
         * @param rawX X 軸位移 (Cartesian: 右為正)
         * @param rawY Y 軸位移 (Cartesian: 上為正)
         */
        function updateMotorValues(rawX, rawY) {
            
            // 1. 計算幅度和角度 (這裡用於歸一化，並不需要角度本身)
            const distance = Math.sqrt(rawX*rawX + rawY*rawY);
            const magnitude = Math.min(1.0, distance / maxRadius);
            const angle = Math.atan2(rawY, rawX);
            
            // 2. 計算歸一化後的 X, Y (範圍 -1.0 到 1.0)
            const normX = magnitude * Math.cos(angle); // 轉向 (Steering)
            const normY = magnitude * Math.sin(angle); // 速度 (Throttle)

            // --- 3. 獨立馬達控制邏輯 (A=速度/Y, B=轉向/X) ---
            // A 馬達 (L參數) 速度 = Y 軸輸入 (油門)
            let motorA_float = normY; 
            // B 馬達 (R參數) 速度 = X 軸輸入 (轉向)
            let motorB_float = normX; 

            // 4. 轉換為 -255 到 255 的整數
            const speedA = Math.round(motorA_float * 255);
            const speedB = Math.round(motorB_float * 255);

            // 更新顯示
            valYEl.textContent = speedA; // 顯示 A 馬達 (速度)
            valXEl.textContent = speedB; // 顯示 B 馬達 (轉向)
            
            // 更新狀態文字和顏色
            let currentStatus = "靜止";
            let statusColor = "text-green-400";
            if (Math.abs(speedA) > 5 || Math.abs(speedB) > 5) {
                 statusColor = "text-yellow-400";
                 if (speedA > 50 && Math.abs(speedB) < 50) currentStatus = "前進加速中";
                 else if (speedA < -50 && Math.abs(speedB) < 50) currentStatus = "後退減速中";
                 else if (speedB > 50) currentStatus = "右轉中";
                 else if (speedB < -50) currentStatus = "左轉中";
                 else currentStatus = "移動中";
            } else {
                 statusColor = "text-green-400";
            }
            statusEl.textContent = currentStatus;
            statusEl.className = statusColor;

            // 如果數值有變化，發送控制請求
            if (speedA !== lastMotorA || speedB !== lastMotorB) {
                lastMotorA = speedA;
                lastMotorB = speedB;
                // 發送 A 馬達速度 (l) 和 B 馬達速度 (r)
                sendControl(speedA, speedB); 
            }
        }

        function sendControl(A, B) {
            // 使用非同步請求發送馬達速度
            fetch(`${baseIp}/control?a=${A}&b=${B}`, { method: 'GET' })
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
            updateMotorValues(0, 0); // 設置 A=0, B=0 (這會調用 sendControl(0, 0))
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
                sendControl(lastMotorA, lastMotorB);
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
    globalHostname = "esp32c3-" + WiFi.macAddress(); // 產生基於 MAC 位址的唯一 Hostname
    globalHostname.replace(":", ""); // 移除冒號以獲得乾淨名稱
    globalHostname.toLowerCase(); // 轉換為小寫，利於 Hostname 規範
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}

// --- 馬達控制邏輯 (LEDC PWM) ---
void setMotorA(int speed) {
    
    speed = constrain(speed, -255, 255); // 確保速度在有效範圍內

    if (speed > 0) { // 正轉 (AIN1 HIGH, AIN2 LOW)
        ledcWrite(LEDC_CH_A1, speed);
        ledcWrite(LEDC_CH_A2, 0);
    } else if (speed < 0) { // 反轉 (AIN1 LOW, AIN2 HIGH)
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, -speed); // 使用絕對值
    } else { // 停止
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, 0);
    }
}

void setMotorB(int speed) {
    
    speed = constrain(speed, -255, 255); // 確保速度在有效範圍內

    if (speed > 0) { // 反轉 (BIN1 LOW, BIN2 HIGH)
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, speed);
    } else if (speed < 0) { // 正轉 (BIN1 HIGH, BIN2 LOW)
        ledcWrite(LEDC_CH_B1, -speed); // 使用絕對值
        ledcWrite(LEDC_CH_B2, 0);
    } else { // 停止
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, 0);
    }
}

// --- Web Server 處理函式 ---
void handleRoot() {
    
    String html = HTML_CONTENT; // 替換 HTML 中的變數
    // 根據當前模式顯示正確的 IP 位址
    String ipAddress = WiFi.getMode() == WIFI_MODE_AP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%IPADDRESS%", ipAddress);

    server.send(200, "text/html", html);
}

void handleControl() {
    if (server.hasArg("a") && server.hasArg("b")) {
        // 搖桿的 Y 軸 (速度) 值透過 'a' 傳給 Motor A
        int speedA_Throttle = server.arg("a").toInt(); 
        
        // 搖桿的 X 軸 (轉向) 值透過 'b' 傳給 Motor B
        int speedB_Steering = server.arg("b").toInt();

        // Motor A: 速度
        setMotorA(speedA_Throttle);
        // Motor B: 轉向
        setMotorB(speedB_Steering);

        Serial.printf("WebControl: A馬達(速度)=%d, B馬達(轉向)=%d\n", speedA_Throttle, speedB_Steering);
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Invalid arguments (Missing a or b)");
    }
}

void setupWebServer() {
    Serial.println("--- 啟動 Web Server ---");

    // 處理根目錄請求 (虛擬搖桿頁面)
    server.on("/", HTTP_GET, handleRoot);

    // 處理馬達控制 API 請求
    server.on("/control", HTTP_GET, handleControl);

    server.begin();
    Serial.println("HTTP 伺服器已啟動於 Port 80。");
}

// --- mDNS/OTA 設定 ---
void setupMdnsAp() {
    Serial.println("--- 設定 mDNS (AP 模式) ---");
    if (MDNS.begin(globalHostname.c_str())) {
        Serial.printf("mDNS (AP 模式) 啟動: %s.local -> 192.168.4.1\n", globalHostname.c_str());
    } else {
        Serial.println("mDNS (AP 模式) 啟動失敗。");
    }
}

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

// --- 連線或啟動 Wi-Fi 配置入口網站 ---
void connectToWiFi() {

    // 設置配置入口網站的回調函式 (AP Mode 啟動時)
    wm.setAPCallback([](WiFiManager *wm) {
        isConfigurationMode = true;
        Serial.println("進入配置 AP 模式 (ESP32-Setup)。");
        Serial.println("連線 AP: " + WiFi.softAPSSID());
        Serial.println("IP 位址: " + WiFi.softAPIP().toString());
        
        // 在 AP Mode 中啟動 mDNS 和 Web Server
        //setupMdnsAp(); 
        //setupWebServer();
    });

    wm.setHostname(globalHostname);

    Serial.println("正在啟動 WiFiManager autoConnect...");

    // autoConnect 會阻塞直到連線成功或進入配置 AP
    if (!wm.autoConnect("ESP32-Setup")) {
        // 如果 autoConnect 失敗，且沒有進入配置模式 (通常是因為連線 AP 失敗或超時)
        if (!isConfigurationMode) {
            Serial.println("AutoConnect 失敗，設定為配置模式旗標。");
            isConfigurationMode = true;
        }
    } else {
        isConfigurationMode = false;
        Serial.println("✅ Wi-Fi 連線成功!");
    }

    if(WiFi.status() == WL_CONNECTED) {
        //Serial.println("本地 IP: " + WiFi.localIP().toString());
    }
}

// --- Utility: 輸出分區資訊 ---
void printPartitionInfo(const esp_partition_t *p, const char *tag) {
    if (p) {
        Serial.printf("[%s] 標籤(Label): %s | 類型(Type): %d | 子類型(Subtype): %d | 位址(Address): 0x%X | 大小(Size): %u\n",
                      tag, p->label, p->type, p->subtype, p->address, p->size);
    } else {
        Serial.printf("[%s] 找不到分區!\n", tag);
    }
}

// --- 尋找最新的有效 OTA 應用程式 ---
const esp_partition_t* findNextBootApp() {
    const esp_partition_t *next_partition = esp_ota_get_boot_partition();
    const esp_partition_t *running_partition = esp_ota_get_running_partition();

    if (!next_partition) {
        Serial.println("❌ 無法取得下一個啟動分區資訊。");
        return nullptr;
    }

    // 檢查下一個啟動分區是否是當前正在運行的分區 (即 factory 分區)
    if (next_partition->address == running_partition->address) {
        Serial.println("⚠️ otadata 指向當前運行的分區 (Factory)。不執行跳轉。");
        // 為了安全起見，再檢查一次是否有有效的 OTA 槽位
        
        // 尋找 otadata 指定的啟動分區是否為 ota_0 或 ota_1
        if (next_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 || 
            next_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
            
            // 如果 Factory 分區被設置為 Boot Partition，且正在運行，這是一個異常情況
            // 除非是第一次啟動，但通常 Factory 不會有 otadata 標記。
            // 我們將只在 next_partition 是 ota_0/ota_1 且不等於 running_partition 時才跳轉。
            return nullptr; 
        }

        // 如果 next_partition 是 factory 且等於 running_partition
        // 且它不是 ota_0/ota_1，則表示沒有有效的 OTA 應用程式可跳轉。
        return nullptr;
    }

    // 檢查 next_partition 是否為有效的 OTA 應用程式分區
    if (next_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 || 
        next_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
        
        Serial.printf("✅ otadata 指向有效的 OTA 分區: %s\n", next_partition->label);
        return next_partition;
    }

    Serial.printf("❌ otadata 指向的分區 (%s) 不是 OTA 應用程式分區。不執行跳轉。\n", next_partition->label);
    return nullptr;
}


// --- 啟動最新的 OTA 應用程式（修訂版） ---
void startLatestApp() {
    const esp_partition_t *user_partition = findNextBootApp(); // 使用新的檢查函式
    
    if (user_partition) {
        Serial.println("-------------------------------------------------------");
        Serial.printf("在分區 %s 中找到有效的用戶應用程式 (由 otadata 指定)。\n", user_partition->label);
        Serial.printf("將在 %d 秒後跳轉到用戶應用程式...\n", JUMP_DELAY_SEC);
        Serial.println("-------------------------------------------------------");

        for (int i = 0; i < JUMP_DELAY_SEC; i++) {
            // 在倒數期間保持 WebServer 和 OTA 運行
            //server.handleClient();
            //ArduinoOTA.handle();
            
            Serial.printf("正在跳轉: %d 秒...\n", JUMP_DELAY_SEC - i);
            yield(); 
            delay(1000);
        }

        Serial.println("✅ otadata 已指定啟動分區。正在重啟...");
        delay(500);
        esp_restart();

    } else {
        Serial.println("-------------------------------------------------------");
        Serial.println("⚠️ otadata 未指向有效的 OTA 應用程式。停留在啟動器模式。");
        Serial.println("-------------------------------------------------------");
    }
}

// --- Utility: 擦除所有 OTA 應用程式分區 ---
void eraseAllOtaApps() {
    Serial.println("--- 🚨 執行救援模式：擦除所有 OTA 應用程式分區 🚨 ---");
    
    const esp_partition_t *running_partition = esp_ota_get_running_partition();

    // 尋找所有 OTA 應用程式類型分區 (ota_0, ota_1, ...)
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    
    int erased_count = 0;
    while (it != NULL) {
        const esp_partition_t *p = esp_partition_get(it);
        
        // 僅檢查 OTA 應用程式分區
        if (p->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 || p->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
            
            // 檢查是否是當前正在運行的分區
            if (p->address == running_partition->address) {
                Serial.printf("⚠️ 警告: 分區 %s 正在運行，無法擦除。\n", p->label);
            } else {
                Serial.printf("✅ 正在擦除分區: %s (位址: 0x%X, 大小: %u)\n", p->label, p->address, p->size);
                
                // 執行擦除操作
                esp_err_t err = esp_partition_erase_range(p, 0, p->size);
                
                if (err == ESP_OK) {
                    Serial.println("    擦除成功。");
                    erased_count++;
                } else {
                    Serial.printf("    ❌ 擦除失敗 (錯誤碼=%d)。\n", err);
                }
            }
        }
        it = esp_partition_next(it);
    }
    
    // 釋放迭代器
    if (it != NULL) {
        esp_partition_iterator_release(it);
    }

    if (erased_count > 0) {
        Serial.println("--- 🚨 OTA 應用程式已清除完成。系統將保持在 Factory 模式 🚨 ---");
    } else {
        Serial.println("--- 檢查完成：未發現可清除的 OTA 應用程式。 ---");
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- 初始化馬達控制腳位 (DRV8833) ---
    // 設置 nSLEEP 為輸出並拉高以致能 DRV8833
    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH); 
    Serial.printf("馬達驅動 (nSLEEP) 已致能於 GPIO%d\n", NSLEEP_PIN);

    // 配置 PWM (LEDC) 通道
    ledcSetup(LEDC_CH_A1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_A2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B2, PWM_FREQ, PWM_RESOLUTION);

    // 將 PWM 通道連結到實際的 GPIO 腳位
    ledcAttachPin(AIN1_PIN, LEDC_CH_A1);
    ledcAttachPin(AIN2_PIN, LEDC_CH_A2);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1);
    ledcAttachPin(BIN2_PIN, LEDC_CH_B2);

    // 確保馬達啟動時靜止
    setMotorA(0);
    setMotorB(0);
    
    // --- 啟動器核心邏輯 ---
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();

    Serial.println("\n=======================================================");
    Serial.println("         ESP32 工廠啟動器與控制介面已啟動             ");
    Serial.println("=======================================================");
    printPartitionInfo(running, "正在運行分區 (Running)");
    printPartitionInfo(boot, "初始啟動分區 (Boot)");
    //eraseAllOtaApps();

    // 0. 產生唯一的 Hostname
    generateHostname();

    // 1. 連線 Wi-Fi (會阻塞直到連線成功或進入配置模式)
    connectToWiFi();

    if (isConfigurationMode) { // 如果進入配置 AP 模式
        Serial.println("⚠️ 進入 AP 配置模式。只啟用 WiFiManager Portal。");
        // **不啟動 setupMdnsAp() 和 setupWebServer()**
        // 系統將只依賴 WiFiManager 內建的 Web 服務進行配置。
    } else { // 成功連網狀態下 (STA Mode)

        // 2. Setup mDNS and OTA (STA Mode)
        setupMdnsOtaSta();

        // 3. Setup Web Server (STA Mode)
        setupWebServer();

        // 4. 嘗試跳轉到用戶應用程式 (倒數期間 WebServer/OTA 仍運作)
        //startLatestApp();

        // 如果 startLatestApp 沒有跳轉，程式會繼續在 loop 中執行。
        Serial.println("-------------------------------------------------------");
        Serial.println("⚠️ otadata 未指向有效的 OTA 應用程式。停留在啟動器模式。");
        Serial.println("-------------------------------------------------------");

    }
}

// --- Loop ---
void loop() {
    if (isConfigurationMode) { // 如果進入配置 AP 模式
        // 確保 WiFiManager 在 AP 模式下能持續監聽，並處理連線維持
        wm.process(); 
    } else { // 成功連網狀態下 (STA Mode)
        server.handleClient(); // 處理 Web Server 客戶端請求
        ArduinoOTA.handle();  // 處理 OTA 任務
    }
    yield();
}

// 在 Arduino core 裡 (esp32-hal-main.c)
extern "C" void app_main()
{
    initArduino();   // 初始化硬體/系統
    setup();         // 呼叫使用者定義的 setup()
    for (;;) {
        loop();      // 不斷呼叫使用者的 loop()
        delay(1);
    }
}
