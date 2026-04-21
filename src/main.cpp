#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp32c3_gpio.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>

// --- 緊急救援分區定義 ---
#define CURRENT_VERSION "2026.04.21.16-REPAIR-BRIDGE"
WebServer server(80);
volatile bool isUpdating = false;

// --- 救援介面 ---
const char REPAIR_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>EMERGENCY REPAIR</title><script src="https://cdn.tailwindcss.com"></script></head>
<body class="bg-red-900 text-white flex items-center justify-center min-h-screen">
    <div class="bg-black/50 p-10 rounded-3xl border-4 border-yellow-500 shadow-2xl text-center max-w-lg">
        <h1 class="text-4xl font-black mb-4">⚠️ 救援跳板模式 ⚠️</h1>
        <p class="mb-8 text-yellow-500 font-bold uppercase tracking-widest text-sm">正在從 OTA_1 (練習區) 運行</p>
        <div class="bg-red-800/50 p-6 rounded-xl mb-8 text-left text-xs font-mono space-y-2">
            <p>1. 本韌體為臨時修復工具。</p>
            <p>2. 按下紅鈕後將下載最新版並【覆蓋工廠區】。</p>
            <p>3. 失敗可能導致無法開機 (需插 USB 救回)。</p>
        </div>
        <button onclick="if(confirm('確定覆蓋工廠區？')) location.href='/repair_factory';" 
                class="w-full py-4 bg-yellow-500 hover:bg-yellow-400 text-black font-black rounded-full shadow-2xl transition-all active:scale-95">
            🛠️ 執行工廠區反向修復 (Target: Factory)
        </button>
        <p class="mt-8 text-gray-400 text-[10px]">Version: 2026.04.21.16-BRIDGE</p>
    </div>
</body></html>)rawliteral";

// --- 反向修復邏輯 (定向至 Factory) ---
void vTaskRepair(void *pvParameters) {
    isUpdating = true; Serial.println("\n[REPAIR] CRITICAL: Targeting FACTORY Area (0x10000)...");
    WiFiClientSecure client; client.setInsecure(); HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (http.begin(client, "https://github.com/VibeCoding-tw/esp32c3-launcher/releases/latest/download/firmware.bin")) {
        if (http.GET() == 200) {
            int len = http.getSize();
            // [CRITICAL] 這裡強行指向 FACTORY
            const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
            esp_ota_handle_t h = 0;
            if (part && esp_ota_begin(part, len, &h) == ESP_OK) {
                WiFiClient* s = http.getStreamPtr(); size_t w = 0; uint8_t b[2048];
                while (http.connected() && w < len) {
                    if (size_t c = s->readBytes(b, 2048)) { esp_ota_write(h, b, c); w += c; }
                }
                if (w == len) { esp_ota_end(h); esp_ota_set_boot_partition(part); Serial.println("Repair Success! Rebooting to Factory..."); delay(1000); ESP.restart(); }
            }
        }
        http.end();
    }
    isUpdating = false; vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200); Serial.println("\n--- EMERGENCY BRIDGE STARTED ---");
    pinMode(NSLEEP_PIN, OUTPUT); digitalWrite(NSLEEP_PIN, HIGH);
    
    // 自動連網 (延習上一版的 Preferences)
    Preferences pref; pref.begin("wifi-config", true);
    String s = pref.getString("ssid",""), p = pref.getString("pass","");
    pref.end();
    if(s.length()>0) { WiFi.begin(s.c_str(), p.c_str()); }
    
    // Web Server
    server.on("/", [](){ server.send(200, "text/html", REPAIR_PAGE_HTML); });
    server.on("/repair_factory", [](){ 
        server.send(200, "text/html", "<h1>Repairing Factory...</h1><p>DO NOT POWER OFF.</p>");
        xTaskCreatePinnedToCore(vTaskRepair, "Repair", 8192, NULL, 1, NULL, 1);
    });
    server.begin();
}

void loop() {
    server.handleClient();
    delay(10);
}
