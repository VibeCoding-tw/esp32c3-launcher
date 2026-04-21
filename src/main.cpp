// 包含必要的庫
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>               // Web Server
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_ota_ops.h"             // OTA 相關操作
#include "esp_partition.h"           // 分區表操作
#include "esp_task_wdt.h"            // Watchdog Timer 函式庫
#include "esp32c3_gpio.h"            // 此處定義了 GPIO 腳位
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>             // 用於儲存 Wi-Fi 憑證及馬達參數
#include <freertos/FreeRTOS.h>       // 繼續使用 FreeRTOS 任務功能
#include <freertos/task.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <WebSocketsServer.h>

// --- 全域變數 ---
String globalHostname;               // 基於 MAC 位址的唯一 Hostname
#define CURRENT_VERSION "2026.04.21.05" 
WebServer server(80);                
WebSocketsServer webSocket(81);      
WiFiUDP udp;                         
const unsigned int UDP_PORT = 4210;  
volatile bool isUpdating = false;    
TaskHandle_t updateTaskHandle = NULL;

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
const int LEDC_CH_A1 = 0;
const int LEDC_CH_A2 = 1;
const int LEDC_CH_B1 = 2;
const int LEDC_CH_B2 = 3;

typedef struct {
    uint16_t controlTimeoutT; 
    uint16_t controlTimeoutS; 
    int pwmEffectiveLimitT; 
    int rampAccelStepT; 
    int pwmStartKickT; 
    int pwmEffectiveLimitS; 
    int rampAccelStepS; 
    int pwmStartKickS;
    uint8_t autoUpdateEnabled; 
    uint8_t padding;           
} MotorConfig_t;

Preferences preferences;             
MotorConfig_t motorConfig;           

volatile unsigned long lastControlTime = 0; 
volatile int targetSpeedT = 0;             
volatile int currentSpeedT = 0;            
volatile int targetSpeedS = 0;             
volatile int currentSpeedS = 0;            
const int RAMP_INTERVAL_MS = 10;           
unsigned long lastRampTime = 0;

const char* CONFIG_SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"; 
const char* MOTOR_SERVICE_UUID   = "4fafc201-1fb5-459e-8fcc-c0ffee00dead"; 
#define SSID_CHAR_UUID          "6e400002-b5a3-f393-e0a9-e50e24dcca9e" 
#define PASS_CHAR_UUID          "6e400003-b5a3-f393-e0a9-e50e24dcca9e" 
#define MOTOR_CONTROL_CHAR_UUID "4fafc202-1fb5-459e-8fcc-c0ffee01feed" 
#define MOTOR_CONFIG_CHAR_UUID  "4fafc203-1fb5-459e-8fcc-c0ffee02dead" 

BLEServer *pServer = NULL;
BLECharacteristic *pControlCharacteristic = NULL;
BLECharacteristic *pSsidCharacteristic = NULL;
BLECharacteristic *pPassCharacteristic = NULL;
BLECharacteristic *pMotorConfigCharacteristic = NULL; 
String ble_ssid;
String ble_pass;
bool wifi_config_received = false;
volatile bool should_restart_advertising = false;
bool servicesStarted = false;
float batteryVoltage = 0.0;          
const float ADC_VOLT_REF = 3.1;      
const float DIVIDER_RATIO = 2.0;     
unsigned long lastBatteryCheck = 0;
const int BATT_CHECK_INTERVAL = 500; 

// --- 函式前置宣告 ---
void loadMotorConfig();
void saveMotorConfig();
void performGitHubCloudUpdate();
void checkAndPerformAutoUpdate();
void handleRoot();
void handleJoystick();
void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void checkUdpControl();
void vTaskUpdate(void *pvParameters);
void setupWebServer();
void setupMdnsOtaSta();
void connectToSavedWiFi();
void generateHostname();
void motorRampTask();
void updateBatteryVoltage();
void setMotorPwm(int t, int s);

// --- HTML 網頁內容 ---
static const char* JOYSTICK_PAGE_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Vibe Racer 控制 (搖桿)</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { background: #0f172a; color: #f9fafb; font-family: system-ui; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 1rem; }
        .container { max-width: 400px; width: 100%; padding: 20px; }
        #joystick { 
            position: relative; width: 100%; padding-top: 100%; margin: 0 auto; 
            border-radius: 50%; background: linear-gradient(145deg, #2d3748, #1a202c); 
            box-shadow: 10px 10px 20px #171d26, -10px -10px 20px #273142, inset 0 0 10px rgba(0,0,0,0.5);
            touch-action: none;
        }
        #joystick-thumb {
            position: absolute; width: 70px; height: 70px; top: 50%; left: 50%;
            transform: translate(-50%, -50%); border-radius: 50%; background: #4f46e5;
            box-shadow: 0 0 15px #4f46e5, inset 0 0 10px #7c3aed;
        }
    </style>
</head>
<body class="p-4">
    <div class="container bg-gray-800 rounded-xl shadow-2xl">
        <h1 class="text-3xl font-extrabold text-center text-indigo-400 mb-2">Vibe Racer</h1>
        <p class="text-center text-[10px] mb-4 text-gray-500 uppercase tracking-widest">模式: 虛擬搖桿 | <a href="/" class="text-indigo-400 underline">D-Pad</a></p>
        <div class="bg-gray-900/50 rounded-lg p-3 mb-6 border border-gray-700">
            <div class="flex justify-between items-center text-left">
                <div><p class="text-[10px] text-gray-500 uppercase">電池</p><p class="text-xl font-mono font-bold text-green-400"><span id="val_v">0.00</span>V</p></div>
                <div class="text-right"><p class="text-[10px] text-gray-500 uppercase">即時</p><p class="text-sm font-mono text-indigo-300">T:<span id="val_rt">0</span> S:<span id="val_rs">0</span></p></div>
            </div>
        </div>
        <div id="joystick" class="mb-6"><div id="joystick-thumb"></div></div>
        <p id="status" class="text-center text-sm font-bold text-green-400 mb-4 uppercase tracking-widest">● 靜止 ●</p>
        <div class="p-4 border-t border-gray-700 text-center text-[10px] text-gray-500"><a href="/update_factory" class="hover:text-indigo-400">韌體維修中心</a> | Ver %VERSION%</div>
    </div>
    <script>
        const joystick = document.getElementById('joystick'); const thumb = document.getElementById('joystick-thumb');
        const valVEl = document.getElementById('val_v'); const valRTEl = document.getElementById('val_rt'); const valRSELEl = document.getElementById('val_rs');
        const statusEl = document.getElementById('status');
        const maxRadius = joystick.clientWidth/2;
        let isDragging = false, lastMotorT = 0, lastMotorS = 0;
        let ws = new WebSocket(`ws://${location.hostname}:81`);
        
        function send(t,s) { if(ws.readyState===1) ws.send(`${t},${s}`); }
        function fetchTelemetry() { fetch('/control?t='+lastMotorT+'&s='+lastMotorS).then(r=>r.json()).then(d=>{ valVEl.textContent=d.v.toFixed(2); valRTEl.textContent=d.rt; valRSELEl.textContent=d.rs; }); }
        setInterval(fetchTelemetry, 1000);

        function stop() { isDragging = false; thumb.style.left = '50%'; thumb.style.top = '50%'; send(0,0); statusEl.textContent="● 靜止 ●"; }
        joystick.addEventListener('touchstart', e => { isDragging=true; });
        document.addEventListener('touchmove', e => {
            if(!isDragging) return; e.preventDefault();
            const rect = joystick.getBoundingClientRect();
            let ox = e.touches[0].clientX - (rect.left+maxRadius);
            let oy = e.touches[0].clientY - (rect.top+maxRadius);
            const ds = Math.sqrt(ox*ox+oy*oy); if(ds>maxRadius){ const an=Math.atan2(oy,ox); ox=maxRadius*Math.cos(an); oy=maxRadius*Math.sin(an); }
            thumb.style.left=`${maxRadius+ox}px`; thumb.style.top=`${maxRadius+oy}px`;
            lastMotorS = Math.round((ox/maxRadius)*255); lastMotorT = Math.round((-oy/maxRadius)*255);
            send(lastMotorT, lastMotorS);
            statusEl.textContent="● 運行中 ●";
        }, {passive:false});
        document.addEventListener('touchend', stop);
    </script>
</body>
</html>)rawliteral";

static const char* DPAD_PAGE_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Vibe Racer 控制 (D-Pad)</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { background: #0f172a; color: #f9fafb; font-family: system-ui; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 1rem; }
        .btn-dpad { aspect-ratio: 1/1; border-radius: 16px; background: #1e293b; border: 1px solid rgba(255,255,255,0.1); display: flex; justify-content: center; align-items: center; cursor: pointer; transition: 0.1s; -webkit-tap-highlight-color: transparent; }
        .btn-dpad:active { background: #4f46e5; transform: scale(0.95); }
    </style>
</head>
<body class="p-4">
    <div class="container bg-gray-800 rounded-2xl shadow-2xl p-6 max-w-sm w-full">
        <h1 class="text-2xl font-bold text-center text-indigo-400 mb-6">Vibe Racer</h1>
        <div class="bg-gray-900/50 rounded-xl p-4 border border-gray-700 mb-8 flex justify-between">
            <div><p class="text-[10px] text-gray-500 uppercase">電池</p><p class="text-lg font-mono font-bold text-green-400"><span id="val_v">0.00</span>V</p></div>
            <div class="text-right"><p class="text-[10px] text-gray-500 uppercase">即時</p><p class="text-xs font-mono text-indigo-300">T:<span id="val_rt">0</span> S:<span id="val_rs">0</span></p></div>
        </div>
        <div class="grid grid-cols-3 gap-3 mb-10">
            <div></div><div class="btn-dpad" ontouchstart="mt(255,0)" ontouchend="sp()"><svg class="w-8 h-8" viewBox="0 0 24 24" fill="currentColor"><path d="M12 4l-8 8h16z"/></svg></div><div></div>
            <div class="btn-dpad" ontouchstart="mt(0,-255)" ontouchend="sp()"><svg class="w-8 h-8" viewBox="0 0 24 24" fill="currentColor"><path d="M4 12l8-8v16z"/></svg></div>
            <div class="btn-dpad bg-red-500/10" ontouchstart="sp()"><div class="w-3 h-3 bg-red-500 rounded-sm"></div></div>
            <div class="btn-dpad" ontouchstart="mt(0,255)" ontouchend="sp()"><svg class="w-8 h-8" viewBox="0 0 24 24" fill="currentColor"><path d="M20 12l-8-8v16z"/></svg></div>
            <div></div><div class="btn-dpad" ontouchstart="mt(-255,0)" ontouchend="sp()"><svg class="w-8 h-8" viewBox="0 0 24 24" fill="currentColor"><path d="M12 20l-8-8h16z"/></svg></div><div></div>
        </div>
        <div class="text-center text-[10px] text-gray-600"><a href="/joystick" class="underline">切換至搖桿</a> | <a href="/update_factory" class="underline">維護中心</a> | Ver %VERSION%</div>
    </div>
    <script>
        let ws = new WebSocket(`ws://${location.hostname}:81`);
        function mt(t,s){ if(ws.readyState===1) ws.send(`${t},${s}`); }
        function sp(){ if(ws.readyState===1) ws.send(`0,0`); }
        setInterval(()=>{ fetch('/control?t=0&s=0').then(r=>r.json()).then(d=>{ document.getElementById('val_v').textContent=d.v.toFixed(2); document.getElementById('val_rt').textContent=d.rt; document.getElementById('val_rs').textContent=d.rs; }); }, 2000);
    </script>
</body>
</html>)rawliteral";

const char UPDATE_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head><meta charset="UTF-8"><title>Updating...</title><script src="https://cdn.tailwindcss.com"></script></head>
<body class="bg-slate-900 text-white flex items-center justify-center min-h-screen">
    <div class="p-8 bg-slate-800 rounded-3xl shadow-2xl text-center">
        <h1 class="text-2xl font-bold text-indigo-400 mb-4">系統更新中...</h1>
        <p class="text-sm text-gray-400">正在下載韌體並釋放內存，請勿斷電。</p>
        <div class="mt-6 flex justify-center"><div class="animate-spin rounded-full h-10 w-10 border-4 border-indigo-500 border-t-transparent"></div></div>
        <p class="mt-8 text-xs text-gray-600">預計 40 秒後自動重啟並返回首頁</p>
    </div>
    <script>setTimeout(()=>location.href='/', 45000);</script>
</body>
</html>)rawliteral";

const char MAINTENANCE_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head><meta charset="UTF-8"><title>Maintenance</title><script src="https://cdn.tailwindcss.com"></script></head>
<body class="bg-slate-900 text-white p-6 flex justify-center items-center min-h-screen">
    <div class="bg-slate-800 p-8 rounded-3xl shadow-2xl w-full max-w-md">
        <h1 class="text-2xl font-bold text-indigo-400 mb-6">維修與更新</h1>
        <div class="space-y-6">
            <div class="p-4 bg-slate-900 rounded-2xl flex justify-between items-center">
                <span>自動更新</span>
                <button onclick="location.href='/toggle_auto_update'" class="px-4 py-2 bg-indigo-600 rounded-xl text-xs font-bold uppercase">切換狀態</button>
            </div>
            <button onclick="if(confirm('啟動雲端更新？')) location.href='/update_github';" class="w-full p-4 bg-indigo-600 rounded-2xl font-bold shadow-lg">🚀 啟動 GitHub 雲端更新</button>
            <div class="pt-6 border-t border-slate-700 text-center"><a href="/" class="text-xs text-gray-500 underline">返回控制頁</a></div>
        </div>
    </div>
</body>
</html>)rawliteral";

// --- OTA 邏輯 ---
void vTaskUpdate(void *pvParameters) {
    isUpdating = true;
    Serial.println("\n[Update] Background OTA Task Started.");
    BLEDevice::deinit(true); 
    delay(1000);
    
    char* url = (char*)pvParameters;
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    if (http.begin(client, url)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            int len = http.getSize();
            const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
            if (len > 0 && part) {
                esp_ota_handle_t update_handle = 0;
                if (esp_ota_begin(part, OTA_SIZE_UNKNOWN, &update_handle) == ESP_OK) {
                    WiFiClient* stream = http.getStreamPtr();
                    size_t written = 0;
                    uint8_t buff[2048];
                    while (http.connected() && (written < len)) {
                        size_t avail = stream->available();
                        if (avail) {
                            int c = stream->readBytes(buff, min(avail, (size_t)2048));
                            esp_ota_write(update_handle, buff, c);
                            written += c;
                            if (written % 51200 == 0) Serial.printf("Progress: %u%%\r", (written * 100) / len);
                        }
                        vTaskDelay(1);
                    }
                    if (written == len) {
                        esp_ota_end(update_handle);
                        esp_ota_set_boot_partition(part);
                        Serial.println("\n✅ Update Success! Restarting...");
                        delay(1000); ESP.restart();
                    }
                }
            }
        }
        http.end();
    }
    isUpdating = false;
    vTaskDelete(NULL);
}

void performGitHubCloudUpdate() {
    if (isUpdating) return;
    static char url[] = "https://github.com/VibeCoding-tw/esp32c3-launcher/releases/latest/download/firmware.bin";
    xTaskCreatePinnedToCore(vTaskUpdate, "OTA", 8192, (void*)url, 1, NULL, 1);
}

void checkAndPerformAutoUpdate() {
    if (motorConfig.autoUpdateEnabled != 1) return;
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http; http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (http.begin(client, "https://github.com/VibeCoding-tw/esp32c3-launcher/releases/latest/download/version.txt")) {
        if (http.GET() == HTTP_CODE_OK) {
            String remote = http.getString(); remote.trim();
            if (remote != CURRENT_VERSION && remote.length() > 0) performGitHubCloudUpdate();
        }
        http.end();
    }
}

// --- 馬達與配置 ---
void loadMotorConfig() {
    preferences.begin("motor-config", true);
    if (preferences.getBytes("config", &motorConfig, sizeof(MotorConfig_t)) != sizeof(MotorConfig_t)) {
        motorConfig = {500, 2000, 200, 3, 60, 255, 20, 120, 1, 0};
    }
    preferences.end();
}
void saveMotorConfig() {
    preferences.begin("motor-config", false);
    preferences.putBytes("config", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();
}

void setMotorPwm(int t, int s) {
    if (t > 0) { ledcWrite(LEDC_CH_A2, 0); ledcWrite(LEDC_CH_A1, t); }
    else if (t < 0) { ledcWrite(LEDC_CH_A1, 0); ledcWrite(LEDC_CH_A2, -t); }
    else { ledcWrite(LEDC_CH_A1, 0); ledcWrite(LEDC_CH_A2, 0); }
    if (s > 0) { ledcWrite(LEDC_CH_B1, 0); ledcWrite(LEDC_CH_B2, s); }
    else if (s < 0) { ledcWrite(LEDC_CH_B2, 0); ledcWrite(LEDC_CH_B1, -s); }
    else { ledcWrite(LEDC_CH_B1, 0); ledcWrite(LEDC_CH_B2, 0); }
}

void motorRampTask() {
    if (millis() - lastRampTime < 10) return;
    lastRampTime = millis();
    if (targetSpeedT == 0) currentSpeedT = 0;
    else {
        if (currentSpeedT == 0) currentSpeedT = (targetSpeedT > 0) ? motorConfig.pwmStartKickT : -motorConfig.pwmStartKickT;
        int diff = targetSpeedT - currentSpeedT;
        if (abs(diff) <= motorConfig.rampAccelStepT) currentSpeedT = targetSpeedT;
        else currentSpeedT += (diff > 0) ? motorConfig.rampAccelStepT : -motorConfig.rampAccelStepT;
    }
    if (targetSpeedS == 0) currentSpeedS = 0;
    else {
        if (currentSpeedS == 0) currentSpeedS = (targetSpeedS > 0) ? motorConfig.pwmStartKickS : -motorConfig.pwmStartKickS;
        int diff = targetSpeedS - currentSpeedS;
        if (abs(diff) <= motorConfig.rampAccelStepS) currentSpeedS = targetSpeedS;
        else currentSpeedS += (diff > 0) ? motorConfig.rampAccelStepS : -motorConfig.rampAccelStepS;
    }
    currentSpeedT = constrain(currentSpeedT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT);
    currentSpeedS = constrain(currentSpeedS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
    setMotorPwm(currentSpeedT, currentSpeedS);
}

void updateBatteryVoltage() {
    if (millis() - lastBatteryCheck < 500) return;
    lastBatteryCheck = millis();
    float v = (analogRead(BATT_ADC_PIN) / 4095.0) * 3.1 * 2.0;
    if (batteryVoltage < 0.1) batteryVoltage = v;
    else batteryVoltage = batteryVoltage * 0.9 + v * 0.1;
}

// --- Web Handlers ---
void handleRoot() { server.sendHeader("Connection", "close"); String h=DPAD_PAGE_HTML; h.replace("%VERSION%", CURRENT_VERSION); server.send(200, "text/html", h); }
void handleJoystick() { server.sendHeader("Connection", "close"); String h=JOYSTICK_PAGE_HTML; h.replace("%VERSION%", CURRENT_VERSION); server.send(200, "text/html", h); }
void handleControl() {
    if (server.hasArg("t") && server.hasArg("s")) {
        targetSpeedT = server.arg("t").toInt(); targetSpeedS = server.arg("s").toInt();
        lastControlTime = millis();
        String json = "{\"v\":"+String(batteryVoltage,2)+",\"rt\":"+String(currentSpeedT)+",\"rs\":"+String(currentSpeedS)+"}";
        server.send(200, "application/json", json);
    }
}
void handleFactoryUpdate() { server.sendHeader("Connection", "close"); server.send(200, "text/html", MAINTENANCE_PAGE_HTML); }
void handleGitHubUpdate() { server.send(200, "text/html", UPDATE_PAGE_HTML); delay(500); performGitHubCloudUpdate(); }
void handleToggleAutoUpdate() { motorConfig.autoUpdateEnabled = !motorConfig.autoUpdateEnabled; saveMotorConfig(); server.sendHeader("Location", "/update_factory"); server.send(303); }

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* p) { Serial.println("BLE Connected."); }
    void onDisconnect(BLEServer* p) { should_restart_advertising = true; }
};
class MyCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* p) {
        String cmd = String(p->getValue().c_str());
        int idx = cmd.indexOf(',');
        if (idx > 0) { targetSpeedT = cmd.substring(0,idx).toInt(); targetSpeedS = cmd.substring(idx+1).toInt(); lastControlTime = millis(); }
    }
};

void setupBleServer_Bluedroid() {
    BLEDevice::init(globalHostname.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pSvc = pServer->createService(MOTOR_SERVICE_UUID);
    pControlCharacteristic = pSvc->createCharacteristic(MOTOR_CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pControlCharacteristic->setCallbacks(new MyCharCallbacks());
    pSvc->start();
    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(MOTOR_SERVICE_UUID);
    pAdv->start();
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        String msg = String((char*)payload);
        int idx = msg.indexOf(',');
        if (idx > 0) { targetSpeedT = msg.substring(0,idx).toInt(); targetSpeedS = msg.substring(idx+1).toInt(); lastControlTime = millis(); }
    }
}

void checkUdpControl() {
    if (udp.parsePacket()) {
        char b[32]; int l = udp.read(b,31); b[l]=0; String msg=String(b);
        int idx = msg.indexOf(',');
        if (idx > 0) { targetSpeedT = msg.substring(0,idx).toInt(); targetSpeedS = msg.substring(idx+1).toInt(); lastControlTime = millis(); }
    }
}

void generateHostname() {    
    globalHostname = "esp32c3-" + WiFi.macAddress(); 
    globalHostname.replace(":", ""); 
    globalHostname.toLowerCase(); 
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}
void connectToSavedWiFi() {
    preferences.begin("wifi-config", true); String s = preferences.getString("ssid",""); String p = preferences.getString("pass",""); preferences.end();
    if (s.length()>0) { WiFi.mode(WIFI_STA); WiFi.begin(s.c_str(), p.c_str()); }
}

void setup() {
    Serial.begin(115200); loadMotorConfig();
    pinMode(NSLEEP_PIN, OUTPUT); digitalWrite(NSLEEP_PIN, HIGH);
    ledcSetup(LEDC_CH_A1, 20000, 8); ledcSetup(LEDC_CH_A2, 20000, 8);
    ledcSetup(LEDC_CH_B1, 20000, 8); ledcSetup(LEDC_CH_B2, 20000, 8);
    ledcAttachPin(AIN1_PIN, LEDC_CH_A1); ledcAttachPin(AIN2_PIN, LEDC_CH_A2);
    ledcAttachPin(BIN1_PIN, LEDC_CH_B1); ledcAttachPin(BIN2_PIN, LEDC_CH_B2);
    analogReadResolution(12); generateHostname(); setupBleServer_Bluedroid(); connectToSavedWiFi();
}

void loop() {
    if (servicesStarted) {
        webSocket.loop();
        checkUdpControl();
        server.handleClient();
    }
    if (WiFi.status() == WL_CONNECTED && !servicesStarted) {
        MDNS.begin(globalHostname.c_str());
        server.on("/", handleRoot); server.on("/joystick", handleJoystick); server.on("/control", handleControl);
        server.on("/update_factory", handleFactoryUpdate); server.on("/update_github", handleGitHubUpdate);
        server.on("/toggle_auto_update", handleToggleAutoUpdate);
        server.begin(); webSocket.begin(); webSocket.onEvent(onWsEvent); udp.begin(UDP_PORT);
        servicesStarted = true; checkAndPerformAutoUpdate();
    }
    if (servicesStarted) server.handleClient();
    if (!isUpdating) { motorRampTask(); updateBatteryVoltage(); if(millis()-lastControlTime>1000){targetSpeedT=0;targetSpeedS=0;} }
    if (should_restart_advertising && pServer) { pServer->startAdvertising(); should_restart_advertising = false; }
    delay(1);
}
