#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
#include "esp32c3_gpio.h"
#include "esp_ota_ops.h"

// --- v13 Emergency Bridge ---
#define CURRENT_VERSION "2026.04.21.13"
static const char* JOYSTICK_PAGE_HTML = "<html><body><h1>V13 Emergency Bridge (v13)</h1><p>Web UI disabled for survival. Use BLE or Serial.</p><a href='/update_github'>Cloud Update</a></body></html>";
static const char* DPAD_PAGE_HTML = "v13 Active.";
const char UPDATE_PAGE_HTML[] PROGMEM = "<html><body><h1>System Updating...</h1><p>Please wait 40 seconds.</p></body></html>";

WebServer server(80);
volatile bool isUpdating = false;
volatile int targetSpeedT=0, currentSpeedT=0, targetSpeedS=0, currentSpeedS=0;
volatile unsigned long lastControlTime=0;
float batteryVoltage=0.0;
bool servicesStarted=false;

typedef struct {
    uint16_t controlTimeoutT; uint16_t controlTimeoutS;
    int pwmEffectiveLimitT; int rampAccelStepT; int pwmStartKickT;
    int pwmEffectiveLimitS; int rampAccelStepS; int pwmStartKickS;
    uint8_t autoUpdateEnabled; uint8_t padding;
} MotorConfig_t;
MotorConfig_t motorConfig;
Preferences preferences;

void setMotorPwm(int t, int s) {
    if (t != 0 || s != 0) Serial.printf("DRV -> T:%d, S:%d\n", t, s);
    if (t > 0) { analogWrite(AIN2_PIN, 0); analogWrite(AIN1_PIN, t); }
    else if (t < 0) { analogWrite(AIN1_PIN, 0); analogWrite(AIN2_PIN, -t); }
    else { analogWrite(AIN1_PIN, 0); analogWrite(AIN2_PIN, 0); }
    if (s > 0) { analogWrite(BIN1_PIN, 0); analogWrite(BIN2_PIN, s); }
    else if (s < 0) { analogWrite(BIN2_PIN, 0); analogWrite(BIN1_PIN, -s); }
    else { analogWrite(BIN1_PIN, 0); analogWrite(BIN2_PIN, 0); }
}

void motorRampTask() {
    static unsigned long lastRamp = 0;
    if (millis() - lastRamp < 10) return; lastRamp = millis();
    auto ramp = [](volatile int &cur, int tar, int step, int kick) {
        if (tar == 0) cur = 0;
        else {
            if (cur == 0) cur = (tar > 0) ? kick : -kick;
            int d = tar - cur;
            if (abs(d) <= step) cur = tar; else cur += (d > 0) ? step : -step;
        }
    };
    ramp(currentSpeedT, targetSpeedT, motorConfig.rampAccelStepT, motorConfig.pwmStartKickT);
    ramp(currentSpeedS, targetSpeedS, motorConfig.rampAccelStepS, motorConfig.pwmStartKickS);
    setMotorPwm(currentSpeedT, currentSpeedS);
}

void vTaskUpdate(void *p) {
    isUpdating = true; BLEDevice::deinit(true); delay(1000);
    WiFiClientSecure client; client.setInsecure(); HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (http.begin(client, (char*)p)) {
        if (http.GET() == 200) {
            int len = http.getSize();
            // [CRITICAL] V13 使用更精確的切換邏輯
            const esp_partition_t* update_part = esp_ota_get_next_update_partition(NULL);
            esp_ota_handle_t h = 0;
            if (esp_ota_begin(update_part, len, &h) == ESP_OK) {
                uint8_t b[2048]; WiFiClient* s = http.getStreamPtr(); size_t w = 0;
                while (http.connected() && w < len) {
                    if (int c = s->readBytes(b, 2048)) { esp_ota_write(h, b, c); w += c; }
                    vTaskDelay(1);
                }
                if (w == len) { esp_ota_end(h); esp_ota_set_boot_partition(update_part); ESP.restart(); }
            }
        }
    }
    isUpdating = false; vTaskDelete(NULL);
}

void performGitHubCloudUpdate() {
    static char u[] = "https://github.com/VibeCoding-tw/esp32c3-launcher/releases/latest/download/firmware.bin";
    xTaskCreate(vTaskUpdate, "OTA", 8192, (void*)u, 1, NULL);
}

void setup() {
    Serial.begin(115200); 
    preferences.begin("motor-config", true);
    if (preferences.getBytes("config", &motorConfig, sizeof(MotorConfig_t)) != sizeof(MotorConfig_t))
        motorConfig = {1000, 1000, 255, 20, 60, 255, 20, 60, 1, 0};
    preferences.end();
    pinMode(NSLEEP_PIN, OUTPUT); digitalWrite(NSLEEP_PIN, HIGH);
    analogReadResolution(12);
    BLEDevice::init("Bridge-V13");
    
    preferences.begin("wifi-config", true);
    String s = preferences.getString("ssid",""), p = preferences.getString("pass","");
    preferences.end();
    if (s.length()>0) { WiFi.mode(WIFI_STA); WiFi.begin(s.c_str(), p.c_str()); }
}

void loop() {
    if (WiFi.status() == WL_CONNECTED && !servicesStarted) {
        server.on("/", [](){ server.send(200, "text/html", JOYSTICK_PAGE_HTML); });
        server.on("/control", [](){ targetSpeedT=server.arg("t").toInt();targetSpeedS=server.arg("s").toInt();lastControlTime=millis();server.send(200); });
        server.on("/update_github", [](){ server.send(200, "text/html", UPDATE_PAGE_HTML); performGitHubCloudUpdate(); });
        server.begin(); servicesStarted = true;
    }
    if (servicesStarted) server.handleClient();
    if (!isUpdating) { motorRampTask(); if(millis()-lastControlTime>1000){targetSpeedT=0;targetSpeedS=0;} }
    delay(1);
}
