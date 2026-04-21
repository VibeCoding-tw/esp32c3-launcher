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
#define CURRENT_VERSION "2026.04.21.16-REPAIR-BRIDGE-V2"
WebServer server(80);
volatile bool isUpdating = false;

// --- 救援介面 (含進度條與 UTF-8) ---
const char REPAIR_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>EMERGENCY REPAIR</title><script src="https://cdn.tailwindcss.com"></script>
<style>
    .progress-bar { width: 0%; transition: width 0.5s ease; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
    .loading { animation: pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite; }
</style></head>
<body class="bg-red-950 text-white flex items-center justify-center min-h-screen font-sans">
    <div class="bg-black/60 p-10 rounded-[3rem] border-4 border-yellow-500 shadow-2xl text-center max-w-lg w-full mx-4">
        <div class="text-6xl mb-6">🛠️</div>
        <h1 class="text-3xl font-black mb-2 uppercase tracking-tighter">救援跳板模式</h1>
        <p class="mb-8 text-yellow-500 font-bold uppercase tracking-widest text-xs">Running from OTA_1 (Practice Area)</p>
        
        <div id="setup-view">
            <div class="bg-red-900/30 p-6 rounded-2xl mb-8 text-left text-xs font-mono border border-red-500/30">
                <p class="text-red-400 mb-2 font-bold underline">⚠ 本處為臨時救援環境：</p>
                <p>• 即將下載最新韌體並強行【覆蓋工廠區】</p>
                <p>• 請確保供電穩定，請勿中途移動小車</p>
            </div>
            <button onclick="startRepair()" class="w-full py-5 bg-gradient-to-r from-yellow-500 to-amber-600 hover:from-yellow-400 hover:to-amber-500 text-black font-black rounded-2xl shadow-xl transition-all active:scale-95 text-lg">
                🚀 執行工廠區修復 (Restore Factory)
            </button>
        </div>

        <div id="progress-view" class="hidden">
            <p class="text-sm mb-4 loading font-bold text-yellow-500">正在下載並寫入工廠區...</p>
            <div class="w-full bg-gray-800 rounded-full h-4 mb-4 overflow-hidden border border-white/10">
                <div id="bar" class="progress-bar bg-yellow-500 h-full"></div>
            </div>
            <p id="status" class="text-[10px] text-gray-500 font-mono">請待約 45 秒，完成後將自動重啟</p>
        </div>

        <p class="mt-10 text-gray-600 text-[10px] uppercase">Build: 2026.04.21.16-V2</p>
    </div>

    <script>
        function startRepair() {
            document.getElementById('setup-view').classList.add('hidden');
            document.getElementById('progress-view').classList.remove('hidden');
            let p = 0;
            const b = document.getElementById('bar');
            const itv = setInterval(() => { p += 2; if(p<=95) b.style.width = p + '%'; }, 1000);
            fetch('/do_repair').then(r => {
                if(r.ok) { 
                    b.style.width = '100%';
                    document.getElementById('status').innerText = '✅ 修復完成！正在重啟回工廠區...';
                    setTimeout(() => location.href='/', 5000);
                } else {
                    document.getElementById('status').innerText = '❌ 修復失敗，請查看 Serial 日誌';
                    document.getElementById('status').classList.add('text-red-500');
                    clearInterval(itv);
                }
            });
        }
    </script>
</body></html>)rawliteral";

// --- 反向修復核心任務 ---
void vTaskRepair(void *pvParameters) {
    isUpdating = true; 
    Serial.println("\n[REPAIR] Starting Factory Recovery...");
    WiFiClientSecure client; client.setInsecure(); HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    // 使用明確的大小寫與連結
    if (http.begin(client, "https://github.com/vibe-coding-tw/esp32c3-launcher/releases/latest/download/firmware.bin")) {
        int code = http.GET();
        if (code == 200) {
            int len = http.getSize();
            Serial.printf("[REPAIR] Binary size: %d bytes\n", len);
            const esp_partition_t* part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
            esp_ota_handle_t h = 0;
            if (part && esp_ota_begin(part, len, &h) == ESP_OK) {
                WiFiClient* s = http.getStreamPtr(); size_t total_read = 0; uint8_t b[2048];
                while (http.connected() && total_read < len) {
                    if (size_t c = s->readBytes(b, 2048)) { 
                        esp_ota_write(h, b, c); total_read += c; 
                        if (total_read % 10240 == 0) Serial.printf("[REPAIR] Progress: %d%%\n", (total_read * 100) / len);
                    }
                    vTaskDelay(1);
                }
                if (total_read == len) {
                    esp_ota_end(h); esp_ota_set_boot_partition(part);
                    Serial.println("✅ [REPAIR] SUCCESS! Partition switched to Factory."); 
                    delay(2000); ESP.restart(); 
                } else {
                    Serial.printf("❌ [REPAIR] Download Incomplete: %d/%d\n", total_read, len);
                }
            } else { Serial.println("❌ [REPAIR] OTA Begin Failed! Partition busy or small."); }
        } else { Serial.printf("❌ [REPAIR] HTTP Error: %d\n", code); }
        http.end();
    }
    isUpdating = false; vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200); Serial.println("\n--- EMERGENCY BRIDGE V2 STARTED ---");
    pinMode(NSLEEP_PIN, OUTPUT); digitalWrite(NSLEEP_PIN, HIGH);
    
    Preferences pref; pref.begin("wifi-config", true);
    String s = pref.getString("ssid",""), p = pref.getString("pass","");
    pref.end();
    if(s.length()>0) { WiFi.begin(s.c_str(), p.c_str()); }
    
    server.on("/", [](){ server.send(200, "text/html", REPAIR_PAGE_HTML); });
    server.on("/do_repair", [](){ 
        Serial.println("[WEB] Repair Command Received.");
        xTaskCreatePinnedToCore(vTaskRepair, "Repair", 8192, NULL, 1, NULL, 1);
        server.send(200, "text/plain", "OK");
    });
    server.begin();
}

void loop() {
    server.handleClient();
    delay(10);
}
