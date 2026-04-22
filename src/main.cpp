#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp32c3_gpio.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>

// --- 全域變數 ---
String globalHostname;
#define CURRENT_VERSION "2026.04.21.15.1"
WebServer server(80);
WebSocketsServer webSocket(81);
volatile bool isUpdating = false;

typedef struct {
    uint16_t controlTimeoutT; uint16_t controlTimeoutS;
    int pwmEffectiveLimitT; int rampAccelStepT; int pwmStartKickT;
    int pwmEffectiveLimitS; int rampAccelStepS; int pwmStartKickS;
    uint8_t autoUpdateEnabled; uint8_t padding;
} MotorConfig_t;

Preferences preferences;
MotorConfig_t motorConfig;

volatile unsigned long lastThrottleTime = 0;
volatile unsigned long lastSteerTime = 0;
unsigned long steerStartTime = 0;
volatile int targetSpeedT = 0, currentSpeedT = 0;
volatile int targetSpeedS = 0, currentSpeedS = 0;
unsigned long lastRampTime = 0;

const char* CONFIG_SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const char* MOTOR_SERVICE_UUID   = "4fafc201-1fb5-459e-8fcc-c0ffee00dead";
#define SSID_CHAR_UUID          "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define PASS_CHAR_UUID          "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define MOTOR_CONTROL_CHAR_UUID "4fafc202-1fb5-459e-8fcc-c0ffee01feed"
#define MOTOR_CONFIG_CHAR_UUID  "4fafc203-1fb5-459e-8fcc-c0ffee02dead"

BLEServer *pServer = NULL;
BLECharacteristic *pControlCharacteristic = NULL, *pSsidCharacteristic = NULL, *pPassCharacteristic = NULL, *pMotorConfigCharacteristic = NULL;
String ble_ssid, ble_pass;
bool servicesStarted = false;
float batteryVoltage = 0.0;
unsigned long lastBatteryCheck = 0;

// --- 介面代碼 ---
const char JOYSTICK_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Vibe Racer</title><script src="https://cdn.tailwindcss.com"></script>
<style>body{background:#0f172a;color:#fff;touch-action:none}#joystick{width:250px;height:250px;background:#1e293b;border-radius:50%;position:relative;margin:0 auto}#thumb{width:60px;height:60px;background:#4f46e5;border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);box-shadow:0 0 15px #4f46e5}</style></head>
<body class="flex flex-col items-center justify-center min-h-screen p-4">
    <div class="bg-gray-800 p-6 rounded-3xl shadow-2xl w-full max-w-sm text-center">
        <h1 class="text-2xl font-bold text-indigo-400 mb-4">Vibe Racer</h1>
        <div id="joystick"><div id="thumb"></div></div>
        <div class="mt-8 pt-4 border-t border-gray-700 text-[10px] text-gray-500">Ver %VERSION% | <a href="/update_factory" class="text-red-400">MAINTENANCE</a></div>
    </div>
    <script>
        const j=document.getElementById('joystick'), t=document.getElementById('thumb');
        let ws=new WebSocket(`ws://${location.hostname}:81`), dr=false;
        function send(t,s){ if(ws.readyState===1) ws.send(`${t},${s}`); }
        j.addEventListener('touchstart',()=>dr=true);
        document.addEventListener('touchmove',e=>{
            if(!dr)return; e.preventDefault(); const r=j.getBoundingClientRect(), m=125;
            let ox=e.touches[0].clientX-(r.left+m), oy=e.touches[0].clientY-(r.top+m);
            const ds=Math.sqrt(ox*ox+oy*oy); if(ds>m){ const a=Math.atan2(oy,ox); ox=m*Math.cos(a); oy=m*Math.sin(a); }
            t.style.left=`${m+ox}px`; t.style.top=`${m+oy}px`;
            let nx=ox/m, ny=-oy/m, mag=Math.sqrt(nx*nx+ny*ny);
            if(mag>0){ let sc=1.0/Math.max(Math.abs(nx),Math.abs(ny)); nx*=sc; ny*=sc; }
            send(Math.round(ny*255), Math.round(nx*255));
        },{passive:false});
        document.addEventListener('touchend',()=>{ dr=false; t.style.left=t.style.top='50%'; send(0,0); });
    </script>
</body></html>)rawliteral";

const char MAINTENANCE_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>Maintenance</title><script src="https://cdn.tailwindcss.com"></script></head>
<body class="bg-slate-900 text-white p-6 flex justify-center items-center min-h-screen">
    <div class="bg-slate-800 p-8 rounded-3xl shadow-2xl w-full max-w-md text-center">
        <h1 class="text-2xl font-bold text-indigo-400 mb-6">維護中心 (%VERSION%)</h1>
        <button onclick="if(confirm('啟動官方更新？\n這將更新工廠分區，請確保電源穩定。')) location.href='/update_official';" class="w-full py-5 bg-indigo-600 rounded-2xl font-bold shadow-lg mb-4">🛡️ 官方系統更新 (To Factory)</button>
        <button onclick="if(confirm('啟動學生練習區更新？')) location.href='/update_student';" class="w-full py-5 bg-emerald-600 rounded-2xl font-bold shadow-lg mb-4">🚀 學生代碼更新 (To OTA_0)</button>
        <div class="text-[10px] text-gray-500 mb-6 font-mono border p-2 border-white/10 rounded-lg">救援提醒：若開機失敗，請按住 GPIO 1 重新上電以回歸官方系統。</div>
        <a href="/" class="text-xs text-indigo-400 underline">返回控制介面</a>
    </div>
</body></html>)rawliteral";

// --- 核心邏輯 ---
void copyRunningToFactory() {
    Serial.println("\n[SYSTEM] !!! STARTING COPY TO FACTORY !!!");
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (!running || !factory) { Serial.println("[ERROR] Partitions not found!"); return; }

    Serial.printf("[SYSTEM] Erasing factory partition (%d bytes)...\n", factory->size);
    esp_partition_erase_range(factory, 0, factory->size);

    uint8_t buf[4096];
    for (size_t offset = 0; offset < running->size; offset += sizeof(buf)) {
        esp_partition_read(running, offset, buf, sizeof(buf));
        esp_partition_write(factory, offset, buf, sizeof(buf));
        if ((offset % (128 * 1024)) == 0) Serial.printf("[SYSTEM] Progress: %d%%\n", (int)((offset * 100) / running->size));
    }
    Serial.println("[SYSTEM] Copy complete! Factory is now updated.");
}

void checkRescueKey() {
    pinMode(1, INPUT_PULLUP);
    if (digitalRead(1) == LOW) {
        Serial.println("\n[RESCUE] !!! GPIO 1 TRIGGERED - RESCUE MODE !!!");
        const esp_partition_t* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
        if (factory) {
            esp_ota_set_boot_partition(factory);
            Serial.println("[RESCUE] Boot partition set to FACTORY. Restarting...");
            delay(1000); ESP.restart();
        }
    }
}

void vTaskUpdate(void *pvParameters) {
    bool isOfficial = (bool)pvParameters;
    isUpdating = true; 
    Serial.printf("\n[OTA] Targeting Staging Area (OTA_0) | Official Update: %s\n", isOfficial ? "YES" : "NO");
    
    WiFiClientSecure client; client.setInsecure(); HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (http.begin(client, "https://github.com/vibe-coding-tw/esp32c3-launcher/releases/latest/download/firmware.bin")) {
        if (http.GET() == 200) {
            int len = http.getSize();
            const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
            esp_ota_handle_t h = 0;
            if (part && esp_ota_begin(part, len, &h) == ESP_OK) {
                WiFiClient* s = http.getStreamPtr(); size_t w = 0; uint8_t b[4096];
                while (http.connected() && w < len) {
                    if (size_t c = s->readBytes(b, 4096)) { esp_ota_write(h, b, c); w += c; }
                }
                if (w == len) {
                    esp_ota_end(h);
                    if (isOfficial) {
                        preferences.begin("ota-state", false);
                        preferences.putBool("pending_factory", true);
                        preferences.end();
                    }
                    esp_ota_set_boot_partition(part);
                    Serial.println("[OTA] Success! Rebooting to OTA_0 for verification...");
                    delay(1000); ESP.restart();
                }
            }
        }
        http.end();
    }
    isUpdating = false; vTaskDelete(NULL);
}

void setMotorPwm(int t, int s) {
    if (t > 0) { analogWrite(AIN2_PIN, 0); analogWrite(AIN1_PIN, t); }
    else if (t < 0) { analogWrite(AIN1_PIN, 0); analogWrite(AIN2_PIN, -t); }
    else { analogWrite(AIN1_PIN, 0); analogWrite(AIN2_PIN, 0); }
    if (s > 0) { analogWrite(BIN1_PIN, 0); analogWrite(BIN2_PIN, s); }
    else if (s < 0) { analogWrite(BIN2_PIN, 0); analogWrite(BIN1_PIN, -s); }
    else { analogWrite(BIN1_PIN, 0); analogWrite(BIN2_PIN, 0); }
}

class MotorControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* p) {
        std::string v = p->getValue();
        if (v.length() > 0) {
            String cmd = String(v.c_str()); int i = cmd.indexOf(',');
            int t = cmd.substring(0, i).toInt(); int s = cmd.substring(i + 1).toInt();
            if (t != 0) lastThrottleTime = millis();
            if (s != 0) { lastSteerTime = millis(); if (steerStartTime == 0) steerStartTime = millis(); }
            else { steerStartTime = 0; }
            targetSpeedT = t; targetSpeedS = s;
        }
    }
};

class WiFiSsidCallbacks: public BLECharacteristicCallbacks { void onWrite(BLECharacteristic* p) { ble_ssid = String(p->getValue().c_str()); } };
class WiFiPassCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* p) {
        ble_pass = String(p->getValue().c_str());
        preferences.begin("wifi-config", false); preferences.putString("ssid", ble_ssid); preferences.putString("pass", ble_pass); preferences.end();
        delay(2000); ESP.restart();
    }
};

void setup() {
    Serial.begin(115200); delay(1000);
    checkRescueKey();

    // --- Soft Rollback Logic ---
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running->subtype != ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        preferences.begin("ota-state", false);
        bool boot_success = preferences.getBool("boot_success", false);
        if (!boot_success) {
            int boot_count = preferences.getInt("boot_count", 0);
            if (boot_count >= 1) {
                Serial.println("\n[ROLLBACK] !!! BOOT FAILED MULTIPLE TIMES. REVERTING TO FACTORY !!!");
                preferences.putBool("boot_success", true); // Reset
                preferences.putInt("boot_count", 0);
                preferences.end();
                const esp_partition_t* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
                esp_ota_set_boot_partition(factory);
                delay(500); ESP.restart();
            } else {
                preferences.putInt("boot_count", boot_count + 1);
            }
        }
        preferences.end();
    }

    Serial.println("\n🚀 !!! VIBE RACER " CURRENT_VERSION " BOOTING !!!");
    
    // --- Official Update Staging Check ---
    preferences.begin("ota-state", true);
    bool pending = preferences.getBool("pending_factory", false);
    preferences.end();

    if (pending && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
        Serial.println("[SYSTEM] Official update staging detected. Verifying & Copying to Factory...");
        // In a real product, we would check SHA256 here.
        delay(2000); // Wait for stability
        copyRunningToFactory();
        preferences.begin("ota-state", false);
        preferences.putBool("pending_factory", false);
        preferences.putBool("boot_success", true); 
        preferences.end();
        Serial.println("[SYSTEM] Factory updated successfully. Rebooting to Factory...");
        const esp_partition_t* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
        esp_ota_set_boot_partition(factory);
        delay(1000); ESP.restart();
    }

    // Mark boot success after initialization
    xTaskCreate([](void*){ vTaskDelay(pdMS_TO_TICKS(10000)); preferences.begin("ota-state", false); preferences.putBool("boot_success", true); preferences.putInt("boot_count", 0); preferences.end(); Serial.println("[SYSTEM] Boot marked as SUCCESS."); vTaskDelete(NULL); }, "SuccessTask", 2048, NULL, 1, NULL);

    preferences.begin("motor-config", true);
    if (preferences.getBytes("config", &motorConfig, sizeof(MotorConfig_t)) != sizeof(MotorConfig_t)) motorConfig = {500, 2000, 255, 20, 60, 255, 20, 60, 1, 0};
    preferences.end();
    pinMode(NSLEEP_PIN, OUTPUT); digitalWrite(NSLEEP_PIN, HIGH);
    analogReadResolution(12); globalHostname = "esp32c3-" + WiFi.macAddress(); globalHostname.replace(":",""); globalHostname.toLowerCase();
    
    BLEDevice::init(globalHostname.c_str());
    pServer = BLEDevice::createServer();
    BLEService *pM = pServer->createService(MOTOR_SERVICE_UUID);
    pControlCharacteristic = pM->createCharacteristic(MOTOR_CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
    pControlCharacteristic->setCallbacks(new MotorControlCallbacks());
    pMotorConfigCharacteristic = pM->createCharacteristic(MOTOR_CONFIG_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pMotorConfigCharacteristic->setValue((uint8_t*)&motorConfig, sizeof(MotorConfig_t));
    pM->start();

    BLEService *pC = pServer->createService(CONFIG_SERVICE_UUID);
    pSsidCharacteristic = pC->createCharacteristic(SSID_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pSsidCharacteristic->setCallbacks(new WiFiSsidCallbacks());
    pPassCharacteristic = pC->createCharacteristic(PASS_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pPassCharacteristic->setCallbacks(new WiFiPassCallbacks());
    pC->start();

    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(MOTOR_SERVICE_UUID); pAdv->addServiceUUID(CONFIG_SERVICE_UUID);
    pAdv->start();

    preferences.begin("wifi-config", true); String s = preferences.getString("ssid",""), p = preferences.getString("pass",""); preferences.end();
    if(s.length()>0) { WiFi.mode(WIFI_STA); WiFi.begin(s.c_str(), p.c_str()); }
}

void loop() {
    if (WiFi.status() == WL_CONNECTED && !servicesStarted) {
        MDNS.begin(globalHostname.c_str());
        server.on("/", [](){ String h=JOYSTICK_PAGE_HTML; h.replace("%VERSION%", CURRENT_VERSION); server.send(200, "text/html", h); });
        server.on("/update_factory", [](){ String h=MAINTENANCE_PAGE_HTML; h.replace("%VERSION%", CURRENT_VERSION); server.send(200, "text/html", h); });
        server.on("/update_official", [](){ server.send(200, "text/html", "<h1>Starting Official Update (to Factory)...</h1><p>Device will reboot to staging area first.</p>"); xTaskCreatePinnedToCore(vTaskUpdate, "OTA", 8192, (void*)true, 1, NULL, 1); });
        server.on("/update_student", [](){ server.send(200, "text/html", "<h1>Starting Student Update (to OTA_0)...</h1>"); xTaskCreatePinnedToCore(vTaskUpdate, "OTA", 8192, (void*)false, 1, NULL, 1); });
        server.begin(); webSocket.begin(); 
        webSocket.onEvent([](uint8_t n, WStype_t t, uint8_t* pl, size_t l){
            if(t==WStype_TEXT){
                String msg=String((char*)pl); int i=msg.indexOf(',');
                int tt=msg.substring(0,i).toInt(); int ss=msg.substring(i+1).toInt();
                if(tt!=0) lastThrottleTime=millis();
                if(ss!=0){ lastSteerTime=millis(); if(steerStartTime==0) steerStartTime=millis(); } else steerStartTime=0;
                targetSpeedT=tt; targetSpeedS=ss;
            }
        });
        servicesStarted = true;
    }
    if (servicesStarted) { server.handleClient(); webSocket.loop(); }
    if (!isUpdating) {
        unsigned long now = millis();
        if(now - lastRampTime >= 10){
            lastRampTime = now;
            auto ramp = [](volatile int &cur, int tar, int st, int ki){
                if(tar==0) cur=0; else { if(cur==0) cur=(tar>0)?ki:-ki; int d=tar-cur; if(abs(d)<=st) cur=tar; else cur+=(d>0)?st:-st; }
            };
            ramp(currentSpeedT, targetSpeedT, motorConfig.rampAccelStepT, motorConfig.pwmStartKickT);
            ramp(currentSpeedS, targetSpeedS, motorConfig.rampAccelStepS, motorConfig.pwmStartKickS);
            if(currentSpeedS != 0 && steerStartTime != 0 && (now - steerStartTime > 3000)) currentSpeedS = constrain(currentSpeedS, -230, 230);
            setMotorPwm(currentSpeedT, currentSpeedS);
        }
        if(now - lastThrottleTime > 500) targetSpeedT = 0;
        if(now - lastSteerTime > 2000) targetSpeedS = 0;
        ArduinoOTA.handle();
    }
    delay(1);
}
