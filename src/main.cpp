#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp32c3_gpio.h"

// --- Version / system constants ---
static const char *CURRENT_VERSION = "2026.04.22.1";
static const uint16_t PWM_MAX = 255;
static const uint32_t RAMP_INTERVAL_MS = 10;
static const uint32_t STEER_MAX_ON_TIME_MS = 3000;
static const int STEER_HOLD_PWM = 230;
static const uint32_t BATTERY_SAMPLE_INTERVAL_MS = 100;
static const float ADC_REFERENCE_VOLTAGE = 3.3f;
static const float ADC_COUNTS = 4095.0f;
static const float BATTERY_DIVIDER_RATIO = 2.0f;   // 100k / 100k divider => measured voltage * 2
static const float BATTERY_FILTER_ALPHA = 0.20f;
static const float FACTORY_COPY_MIN_BATTERY = 3.6f;

// --- Global state ---
String globalHostname;
WebServer server(80);
Preferences preferences;

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

MotorConfig_t motorConfig;

volatile int targetSpeedT = 0;
volatile int currentSpeedT = 0;
volatile int targetSpeedS = 0;
volatile int currentSpeedS = 0;
volatile unsigned long lastThrottleTime = 0;
volatile unsigned long lastSteerTime = 0;
unsigned long lastRampTime = 0;
unsigned long steerStartTime = 0;
bool throttleTimedOut = false;
bool steerTimedOut = false;
bool servicesStarted = false;
volatile bool isUpdating = false;
float batteryVoltage = 0.0f;
float batteryVoltageMin = 0.0f;
unsigned long lastBatterySample = 0;

// --- BLE UUIDs ---
static const char *CONFIG_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *MOTOR_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c0ffee00dead";
#define SSID_CHAR_UUID          "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define PASS_CHAR_UUID          "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define MOTOR_CONTROL_CHAR_UUID "4fafc202-1fb5-459e-8fcc-c0ffee01feed"
#define MOTOR_CONFIG_CHAR_UUID  "4fafc203-1fb5-459e-8fcc-c0ffee02dead"

BLEServer *pServer = nullptr;
BLECharacteristic *pControlCharacteristic = nullptr;
BLECharacteristic *pSsidCharacteristic = nullptr;
BLECharacteristic *pPassCharacteristic = nullptr;
BLECharacteristic *pMotorConfigCharacteristic = nullptr;
String bleSsid;

const char JOYSTICK_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Vibe Racer</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { background: #0f172a; color: #fff; touch-action: none; }
        #joystick {
            width: 280px; height: 280px; position: relative; margin: 0 auto;
            border-radius: 9999px; background: radial-gradient(circle at 35% 35%, #334155, #0f172a);
            box-shadow: inset 0 0 18px rgba(0,0,0,.45), 0 18px 45px rgba(15,23,42,.4);
        }
        #thumb {
            width: 72px; height: 72px; position: absolute; top: 50%; left: 50%;
            transform: translate(-50%, -50%); border-radius: 9999px;
            background: linear-gradient(145deg, #818cf8, #4f46e5);
            box-shadow: 0 0 24px rgba(99,102,241,.7);
        }
        .card { background: rgba(30,41,59,.92); border: 1px solid rgba(148,163,184,.15); }
    </style>
</head>
<body class="min-h-screen flex items-center justify-center p-4">
    <div class="w-full max-w-md space-y-4">
        <div class="card rounded-3xl p-6 shadow-2xl text-center">
            <h1 class="text-2xl font-bold text-indigo-400">Vibe Racer</h1>
            <p class="text-xs text-slate-400 mt-1">Diag + square-mapped throttle/steering</p>
            <div id="joystick" class="mt-6"><div id="thumb"></div></div>
            <div class="grid grid-cols-2 gap-3 text-left text-sm mt-6">
                <div class="bg-slate-900/70 rounded-2xl p-3">
                    <div class="text-slate-400 text-xs">Battery</div>
                    <div id="battery" class="text-xl font-semibold">--.- V</div>
                    <div id="battery-min" class="text-xs text-slate-500">Min --.- V</div>
                </div>
                <div class="bg-slate-900/70 rounded-2xl p-3">
                    <div class="text-slate-400 text-xs">Timeout</div>
                    <div id="timeout" class="text-xl font-semibold">OK</div>
                    <div class="text-xs text-slate-500">T / S watchdog</div>
                </div>
                <div class="bg-slate-900/70 rounded-2xl p-3">
                    <div class="text-slate-400 text-xs">Target</div>
                    <div id="target" class="font-mono">T:0 S:0</div>
                </div>
                <div class="bg-slate-900/70 rounded-2xl p-3">
                    <div class="text-slate-400 text-xs">Realtime</div>
                    <div id="realtime" class="font-mono">T:0 S:0</div>
                </div>
            </div>
            <div class="mt-6 text-[10px] text-slate-500">
                Ver %VERSION% |
                <a href="/update_factory" class="text-amber-400">MAINTENANCE</a>
            </div>
        </div>
    </div>

    <script>
        const joystick = document.getElementById("joystick");
        const thumb = document.getElementById("thumb");
        const batteryEl = document.getElementById("battery");
        const batteryMinEl = document.getElementById("battery-min");
        const timeoutEl = document.getElementById("timeout");
        const targetEl = document.getElementById("target");
        const realtimeEl = document.getElementById("realtime");

        let dragging = false;
        let lastT = 0;
        let lastS = 0;

        function clampToDisc(x, y, radius) {
            const distance = Math.sqrt(x * x + y * y);
            if (distance <= radius) return { x, y };
            const angle = Math.atan2(y, x);
            return { x: radius * Math.cos(angle), y: radius * Math.sin(angle) };
        }

        function squareMap(nx, ny) {
            const maxAxis = Math.max(Math.abs(nx), Math.abs(ny));
            if (maxAxis === 0) return { x: 0, y: 0 };
            return { x: nx / maxAxis, y: ny / maxAxis };
        }

        function sendControl(t, s) {
            lastT = t;
            lastS = s;
            fetch(`/control?t=${t}&s=${s}`).catch(() => {});
        }

        function resetThumb() {
            thumb.style.left = "50%";
            thumb.style.top = "50%";
        }

        function updateFromPoint(clientX, clientY) {
            const rect = joystick.getBoundingClientRect();
            const radius = rect.width / 2;
            const centerX = rect.left + radius;
            const centerY = rect.top + radius;
            const clamped = clampToDisc(clientX - centerX, clientY - centerY, radius);

            thumb.style.left = `${radius + clamped.x}px`;
            thumb.style.top = `${radius + clamped.y}px`;

            const normalized = squareMap(clamped.x / radius, -clamped.y / radius);
            sendControl(Math.round(normalized.y * 255), Math.round(normalized.x * 255));
        }

        function pointerPoint(event) {
            if (event.touches && event.touches.length > 0) {
                return { x: event.touches[0].clientX, y: event.touches[0].clientY };
            }
            return { x: event.clientX, y: event.clientY };
        }

        function startDrag(event) {
            dragging = true;
            const point = pointerPoint(event);
            updateFromPoint(point.x, point.y);
        }

        function moveDrag(event) {
            if (!dragging) return;
            event.preventDefault();
            const point = pointerPoint(event);
            updateFromPoint(point.x, point.y);
        }

        function endDrag() {
            if (!dragging) return;
            dragging = false;
            resetThumb();
            sendControl(0, 0);
        }

        function renderTelemetry(data) {
            batteryEl.textContent = `${data.v.toFixed(2)} V`;
            batteryMinEl.textContent = `Min ${data.vmin.toFixed(2)} V`;
            targetEl.textContent = `T:${data.t} S:${data.s}`;
            realtimeEl.textContent = `T:${data.rt} S:${data.rs}`;
            timeoutEl.textContent = data.to || data.tos ? `T:${data.to ? "!" : "OK"} S:${data.tos ? "!" : "OK"}` : "OK";
            timeoutEl.className = data.to || data.tos ? "text-xl font-semibold text-amber-400" : "text-xl font-semibold text-emerald-400";
        }

        async function refreshTelemetry() {
            try {
                const response = await fetch("/telemetry");
                if (!response.ok) return;
                const data = await response.json();
                renderTelemetry(data);
            } catch (_) {}
        }

        joystick.addEventListener("mousedown", startDrag);
        joystick.addEventListener("touchstart", startDrag, { passive: true });
        document.addEventListener("mousemove", moveDrag);
        document.addEventListener("touchmove", moveDrag, { passive: false });
        document.addEventListener("mouseup", endDrag);
        document.addEventListener("touchend", endDrag);

        resetThumb();
        refreshTelemetry();
        setInterval(refreshTelemetry, 250);
    </script>
</body>
</html>
)rawliteral";

const char MAINTENANCE_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Maintenance</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-slate-950 text-white min-h-screen flex items-center justify-center p-4">
    <div class="bg-slate-800/90 border border-slate-700 rounded-3xl shadow-2xl max-w-md w-full p-8 text-center">
        <h1 class="text-2xl font-bold text-indigo-400">維護中心</h1>
        <p class="text-xs text-slate-400 mt-2">Version %VERSION%</p>
        <div class="space-y-4 mt-8">
            <button onclick="if(confirm('啟動官方更新？裝置會先進入 ota_0 驗證，成功後才回寫 factory。')) location.href='/update_official';" class="w-full rounded-2xl bg-indigo-600 py-5 font-bold shadow-lg">
                官方系統更新 (To Factory)
            </button>
            <button onclick="if(confirm('啟動學生更新？裝置會寫入 ota_0 供測試。')) location.href='/update_student';" class="w-full rounded-2xl bg-emerald-600 py-5 font-bold shadow-lg">
                學生代碼更新 (To OTA_0)
            </button>
        </div>
        <div class="mt-6 rounded-2xl border border-white/10 bg-slate-900/70 p-4 text-xs text-slate-300">
            救援模式：開機時將 GPIO 1 接地，可強制切回 factory。
        </div>
        <a href="/" class="inline-block mt-6 text-sm text-indigo-400 underline">返回控制介面</a>
    </div>
</body>
</html>
)rawliteral";

// --- Helpers ---
String telemetryJson() {
    String json = "{";
    json += "\"v\":" + String(batteryVoltage, 2) + ",";
    json += "\"vmin\":" + String(batteryVoltageMin, 2) + ",";
    json += "\"t\":" + String(targetSpeedT) + ",";
    json += "\"s\":" + String(targetSpeedS) + ",";
    json += "\"rt\":" + String(currentSpeedT) + ",";
    json += "\"rs\":" + String(currentSpeedS) + ",";
    json += "\"to\":" + String(throttleTimedOut ? 1 : 0) + ",";
    json += "\"tos\":" + String(steerTimedOut ? 1 : 0);
    json += "}";
    return json;
}

void saveMotorConfig() {
    preferences.begin("motor-config", false);
    preferences.putBytes("config", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();
    if (pMotorConfigCharacteristic) {
        pMotorConfigCharacteristic->setValue((uint8_t *)&motorConfig, sizeof(MotorConfig_t));
    }
}

void loadMotorConfig() {
    const MotorConfig_t defaults = {
        500, 2000,
        255, 20, 60,
        255, 20, 120,
        0, 0
    };

    preferences.begin("motor-config", true);
    if (preferences.getBytes("config", &motorConfig, sizeof(MotorConfig_t)) != sizeof(MotorConfig_t)) {
        motorConfig = defaults;
    }
    preferences.end();
}

void generateHostname() {
    globalHostname = "esp32c3-" + WiFi.macAddress();
    globalHostname.replace(":", "");
    globalHostname.toLowerCase();
}

void setMotorPwm(int throttle, int steer) {
    throttle = constrain(throttle, -PWM_MAX, PWM_MAX);
    steer = constrain(steer, -PWM_MAX, PWM_MAX);

    if (throttle > 0) {
        analogWrite(AIN2_PIN, 0);
        analogWrite(AIN1_PIN, throttle);
    } else if (throttle < 0) {
        analogWrite(AIN1_PIN, 0);
        analogWrite(AIN2_PIN, -throttle);
    } else {
        analogWrite(AIN1_PIN, 0);
        analogWrite(AIN2_PIN, 0);
    }

    if (steer > 0) {
        analogWrite(BIN1_PIN, 0);
        analogWrite(BIN2_PIN, steer);
    } else if (steer < 0) {
        analogWrite(BIN2_PIN, 0);
        analogWrite(BIN1_PIN, -steer);
    } else {
        analogWrite(BIN1_PIN, 0);
        analogWrite(BIN2_PIN, 0);
    }
}

void applyControlCommand(int rawT, int rawS) {
    targetSpeedT = constrain(rawT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT);
    targetSpeedS = constrain(rawS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);

    const unsigned long now = millis();
    if (targetSpeedT != 0) {
        lastThrottleTime = now;
        throttleTimedOut = false;
    }
    if (targetSpeedS != 0) {
        lastSteerTime = now;
        steerTimedOut = false;
        if (steerStartTime == 0) {
            steerStartTime = now;
        }
    } else {
        steerStartTime = 0;
    }
}

void sampleBatteryVoltage() {
    const unsigned long now = millis();
    if (now - lastBatterySample < BATTERY_SAMPLE_INTERVAL_MS) {
        return;
    }
    lastBatterySample = now;

    const int raw = analogRead(BATT_ADC_PIN);
    const float measured = (static_cast<float>(raw) / ADC_COUNTS) * ADC_REFERENCE_VOLTAGE * BATTERY_DIVIDER_RATIO;
    if (batteryVoltage <= 0.01f) {
        batteryVoltage = measured;
        batteryVoltageMin = measured;
        return;
    }

    batteryVoltage = (batteryVoltage * (1.0f - BATTERY_FILTER_ALPHA)) + (measured * BATTERY_FILTER_ALPHA);
    if (batteryVoltageMin <= 0.01f || batteryVoltage < batteryVoltageMin) {
        batteryVoltageMin = batteryVoltage;
    }
}

int extractJsonInt(const String &json, const char *key, int fallback) {
    const String quotedKey = "\"" + String(key) + "\"";
    int keyPos = json.indexOf(quotedKey);
    if (keyPos < 0) {
        return fallback;
    }
    int colonPos = json.indexOf(':', keyPos + quotedKey.length());
    if (colonPos < 0) {
        return fallback;
    }
    int start = colonPos + 1;
    while (start < json.length() && (json[start] == ' ' || json[start] == '\t' || json[start] == '\n' || json[start] == '\r')) {
        ++start;
    }
    int end = start;
    while (end < json.length() && (json[end] == '-' || isDigit(json[end]))) {
        ++end;
    }
    if (end == start) {
        return fallback;
    }
    return json.substring(start, end).toInt();
}

bool otaStateGetBool(const char *key, bool defaultValue) {
    preferences.begin("ota-state", true);
    const bool value = preferences.getBool(key, defaultValue);
    preferences.end();
    return value;
}

void otaStateSetBool(const char *key, bool value) {
    preferences.begin("ota-state", false);
    preferences.putBool(key, value);
    preferences.end();
}

int otaStateGetInt(const char *key, int defaultValue) {
    preferences.begin("ota-state", true);
    const int value = preferences.getInt(key, defaultValue);
    preferences.end();
    return value;
}

void otaStateSetInt(const char *key, int value) {
    preferences.begin("ota-state", false);
    preferences.putInt(key, value);
    preferences.end();
}

// --- OTA / boot flow ---
bool copyRunningToFactory() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_FACTORY,
        nullptr
    );

    if (!running || !factory) {
        Serial.println("[OTA] Missing running/factory partition.");
        return false;
    }

    if (batteryVoltage > 0.01f && batteryVoltage < FACTORY_COPY_MIN_BATTERY) {
        Serial.printf("[OTA] Battery %.2fV below safe copy threshold.\n", batteryVoltage);
        return false;
    }

    Serial.printf("[OTA] Copying running partition to factory (%u bytes).\n", static_cast<unsigned>(running->size));
    esp_err_t err = esp_partition_erase_range(factory, 0, factory->size);
    if (err != ESP_OK) {
        Serial.printf("[OTA] Factory erase failed: %d\n", static_cast<int>(err));
        return false;
    }

    uint8_t buffer[4096];
    for (size_t offset = 0; offset < running->size; offset += sizeof(buffer)) {
        const size_t remaining = static_cast<size_t>(running->size) - offset;
        const size_t chunkSize = (remaining < sizeof(buffer)) ? remaining : sizeof(buffer);
        err = esp_partition_read(running, offset, buffer, chunkSize);
        if (err != ESP_OK) {
            Serial.printf("[OTA] Partition read failed at %u: %d\n", static_cast<unsigned>(offset), static_cast<int>(err));
            return false;
        }

        err = esp_partition_write(factory, offset, buffer, chunkSize);
        if (err != ESP_OK) {
            Serial.printf("[OTA] Partition write failed at %u: %d\n", static_cast<unsigned>(offset), static_cast<int>(err));
            return false;
        }
    }

    Serial.println("[OTA] Factory copy complete.");
    return true;
}

void checkRescueKey() {
    pinMode(1, INPUT_PULLUP);
    if (digitalRead(1) != LOW) {
        return;
    }

    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_FACTORY,
        nullptr
    );
    if (!factory) {
        return;
    }

    Serial.println("[RESCUE] GPIO1 grounded, forcing boot back to factory.");
    esp_ota_set_boot_partition(factory);
    delay(300);
    ESP.restart();
}

void markBootSuccessTask(void *) {
    vTaskDelay(pdMS_TO_TICKS(10000));
    otaStateSetBool("boot_success", true);
    otaStateSetInt("boot_count", 0);
    Serial.println("[OTA] Boot marked as successful.");
    vTaskDelete(nullptr);
}

void finalizeOfficialUpdateTask(void *) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    sampleBatteryVoltage();
    if (!copyRunningToFactory()) {
        otaStateSetBool("pending_factory", false);
        Serial.println("[OTA] Factory copy skipped/failed; staying on current image.");
        vTaskDelete(nullptr);
        return;
    }

    otaStateSetBool("pending_factory", false);
    otaStateSetBool("boot_success", true);

    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_FACTORY,
        nullptr
    );
    if (factory) {
        Serial.println("[OTA] Official update verified; rebooting into factory.");
        esp_ota_set_boot_partition(factory);
        delay(300);
        ESP.restart();
    }

    vTaskDelete(nullptr);
}

void performOtaTask(void *pvParameters) {
    const bool isOfficial = reinterpret_cast<uintptr_t>(pvParameters) != 0;
    isUpdating = true;

    const char *url = "https://github.com/vibe-coding-tw/esp32c3-launcher/releases/latest/download/firmware.bin";
    Serial.printf("[OTA] Downloading %s update to ota_0.\n", isOfficial ? "official" : "student");

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!http.begin(client, url)) {
        Serial.println("[OTA] HTTP begin failed.");
        isUpdating = false;
        vTaskDelete(nullptr);
        return;
    }

    const int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[OTA] HTTP GET failed: %d\n", httpCode);
        http.end();
        isUpdating = false;
        vTaskDelete(nullptr);
        return;
    }

    const esp_partition_t *ota0 = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_OTA_0,
        nullptr
    );
    if (!ota0) {
        Serial.println("[OTA] ota_0 partition not found.");
        http.end();
        isUpdating = false;
        vTaskDelete(nullptr);
        return;
    }

    esp_ota_handle_t otaHandle = 0;
    esp_err_t err = esp_ota_begin(ota0, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle);
    if (err != ESP_OK) {
        Serial.printf("[OTA] esp_ota_begin failed: %d\n", static_cast<int>(err));
        http.end();
        isUpdating = false;
        vTaskDelete(nullptr);
        return;
    }

    WiFiClient *stream = http.getStreamPtr();
    uint8_t buffer[4096];
    while (http.connected()) {
        const int availableBytes = stream->available();
        if (availableBytes <= 0) {
            delay(1);
            continue;
        }

        const size_t readSize = stream->readBytes(buffer, min(static_cast<int>(sizeof(buffer)), availableBytes));
        if (readSize == 0) {
            break;
        }

        err = esp_ota_write(otaHandle, buffer, readSize);
        if (err != ESP_OK) {
            Serial.printf("[OTA] esp_ota_write failed: %d\n", static_cast<int>(err));
            esp_ota_abort(otaHandle);
            http.end();
            isUpdating = false;
            vTaskDelete(nullptr);
            return;
        }
    }

    err = esp_ota_end(otaHandle);
    http.end();
    if (err != ESP_OK) {
        Serial.printf("[OTA] esp_ota_end failed: %d\n", static_cast<int>(err));
        isUpdating = false;
        vTaskDelete(nullptr);
        return;
    }

    if (isOfficial) {
        otaStateSetBool("pending_factory", true);
    }
    otaStateSetBool("boot_success", false);
    otaStateSetInt("boot_count", 0);
    esp_ota_set_boot_partition(ota0);
    Serial.println("[OTA] Download complete; rebooting into ota_0.");
    delay(300);
    ESP.restart();
}

void runRollbackChecks() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running || running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        return;
    }

    const bool bootSuccess = otaStateGetBool("boot_success", false);
    if (bootSuccess) {
        return;
    }

    const int bootCount = otaStateGetInt("boot_count", 0) + 1;
    otaStateSetInt("boot_count", bootCount);

    if (bootCount < 2) {
        Serial.printf("[OTA] Pending verification boot count: %d\n", bootCount);
        return;
    }

    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_FACTORY,
        nullptr
    );
    if (!factory) {
        return;
    }

    Serial.println("[OTA] Soft rollback triggered; reverting to factory.");
    otaStateSetBool("pending_factory", false);
    otaStateSetBool("boot_success", true);
    otaStateSetInt("boot_count", 0);
    esp_ota_set_boot_partition(factory);
    delay(300);
    ESP.restart();
}

// --- Web handlers ---
void handleRoot() {
    String html = JOYSTICK_PAGE_HTML;
    html.replace("%VERSION%", CURRENT_VERSION);
    server.send(200, "text/html", html);
}

void handleTelemetry() {
    server.send(200, "application/json", telemetryJson());
}

void handleControl() {
    if (!server.hasArg("t") || !server.hasArg("s")) {
        server.send(400, "application/json", "{\"error\":\"missing t or s\"}");
        return;
    }

    applyControlCommand(server.arg("t").toInt(), server.arg("s").toInt());
    server.send(200, "application/json", telemetryJson());
}

void updateConfigFromArgsOrJson() {
    String plain = server.arg("plain");

    auto readValue = [&](const char *argName, const char *jsonName, int currentValue) -> int {
        if (server.hasArg(argName)) {
            return server.arg(argName).toInt();
        }
        if (plain.length() > 0) {
            return extractJsonInt(plain, jsonName, currentValue);
        }
        return currentValue;
    };

    motorConfig.controlTimeoutT = static_cast<uint16_t>(readValue("timeoutT", "controlTimeoutT", motorConfig.controlTimeoutT));
    motorConfig.controlTimeoutS = static_cast<uint16_t>(readValue("timeoutS", "controlTimeoutS", motorConfig.controlTimeoutS));
    motorConfig.pwmEffectiveLimitT = readValue("limitT", "pwmEffectiveLimitT", motorConfig.pwmEffectiveLimitT);
    motorConfig.rampAccelStepT = readValue("stepT", "rampAccelStepT", motorConfig.rampAccelStepT);
    motorConfig.pwmStartKickT = readValue("kickT", "pwmStartKickT", motorConfig.pwmStartKickT);
    motorConfig.pwmEffectiveLimitS = readValue("limitS", "pwmEffectiveLimitS", motorConfig.pwmEffectiveLimitS);
    motorConfig.rampAccelStepS = readValue("stepS", "rampAccelStepS", motorConfig.rampAccelStepS);
    motorConfig.pwmStartKickS = readValue("kickS", "pwmStartKickS", motorConfig.pwmStartKickS);
    motorConfig.autoUpdateEnabled = static_cast<uint8_t>(readValue("autoUpdateEnabled", "autoUpdateEnabled", motorConfig.autoUpdateEnabled));

    motorConfig.pwmEffectiveLimitT = constrain(motorConfig.pwmEffectiveLimitT, 0, PWM_MAX);
    motorConfig.rampAccelStepT = constrain(motorConfig.rampAccelStepT, 1, PWM_MAX);
    motorConfig.pwmStartKickT = constrain(motorConfig.pwmStartKickT, 0, PWM_MAX);
    motorConfig.pwmEffectiveLimitS = constrain(motorConfig.pwmEffectiveLimitS, 0, PWM_MAX);
    motorConfig.rampAccelStepS = constrain(motorConfig.rampAccelStepS, 1, PWM_MAX);
    motorConfig.pwmStartKickS = constrain(motorConfig.pwmStartKickS, 0, PWM_MAX);
}

void handleConfig() {
    if (server.method() == HTTP_GET) {
        String json = "{";
        json += "\"controlTimeoutT\":" + String(motorConfig.controlTimeoutT) + ",";
        json += "\"controlTimeoutS\":" + String(motorConfig.controlTimeoutS) + ",";
        json += "\"pwmEffectiveLimitT\":" + String(motorConfig.pwmEffectiveLimitT) + ",";
        json += "\"rampAccelStepT\":" + String(motorConfig.rampAccelStepT) + ",";
        json += "\"pwmStartKickT\":" + String(motorConfig.pwmStartKickT) + ",";
        json += "\"pwmEffectiveLimitS\":" + String(motorConfig.pwmEffectiveLimitS) + ",";
        json += "\"rampAccelStepS\":" + String(motorConfig.rampAccelStepS) + ",";
        json += "\"pwmStartKickS\":" + String(motorConfig.pwmStartKickS) + ",";
        json += "\"autoUpdateEnabled\":" + String(motorConfig.autoUpdateEnabled);
        json += "}";
        server.send(200, "application/json", json);
        return;
    }

    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"method not allowed\"}");
        return;
    }

    updateConfigFromArgsOrJson();
    saveMotorConfig();
    server.send(200, "application/json", "{\"ok\":true}");
}

void handleMaintenance() {
    String html = MAINTENANCE_PAGE_HTML;
    html.replace("%VERSION%", CURRENT_VERSION);
    server.send(200, "text/html", html);
}

void setupWebServices() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/control", HTTP_GET, handleControl);
    server.on("/config", HTTP_ANY, handleConfig);
    server.on("/update_factory", HTTP_GET, handleMaintenance);
    server.on("/update_official", HTTP_GET, []() {
        server.send(200, "text/html", "<h1>Starting official OTA update...</h1><p>Device will reboot into ota_0, verify, then copy back to factory.</p>");
        xTaskCreate(performOtaTask, "ota-official", 8192, reinterpret_cast<void *>(1), 1, nullptr);
    });
    server.on("/update_student", HTTP_GET, []() {
        server.send(200, "text/html", "<h1>Starting student OTA update...</h1><p>Device will reboot into ota_0.</p>");
        xTaskCreate(performOtaTask, "ota-student", 8192, reinterpret_cast<void *>(0), 1, nullptr);
    });
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not Found");
    });
    server.begin();

    if (MDNS.begin(globalHostname.c_str())) {
        Serial.printf("[NET] mDNS ready at %s.local\n", globalHostname.c_str());
    }
    ArduinoOTA.setHostname(globalHostname.c_str());
    ArduinoOTA.begin();
}

// --- BLE callbacks ---
class MotorControlCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        const std::string value = pCharacteristic->getValue();
        if (value.empty()) {
            return;
        }

        const String command(value.c_str());
        const int commaIndex = command.indexOf(',');
        if (commaIndex <= 0) {
            return;
        }

        applyControlCommand(command.substring(0, commaIndex).toInt(), command.substring(commaIndex + 1).toInt());
    }
};

class MotorConfigCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) override {
        pCharacteristic->setValue((uint8_t *)&motorConfig, sizeof(MotorConfig_t));
    }

    void onWrite(BLECharacteristic *pCharacteristic) override {
        const std::string value = pCharacteristic->getValue();
        if (value.size() != sizeof(MotorConfig_t)) {
            Serial.printf("[BLE] Invalid config write size: %u\n", static_cast<unsigned>(value.size()));
            return;
        }

        memcpy(&motorConfig, value.data(), sizeof(MotorConfig_t));
        motorConfig.pwmEffectiveLimitT = constrain(motorConfig.pwmEffectiveLimitT, 0, PWM_MAX);
        motorConfig.rampAccelStepT = constrain(motorConfig.rampAccelStepT, 1, PWM_MAX);
        motorConfig.pwmStartKickT = constrain(motorConfig.pwmStartKickT, 0, PWM_MAX);
        motorConfig.pwmEffectiveLimitS = constrain(motorConfig.pwmEffectiveLimitS, 0, PWM_MAX);
        motorConfig.rampAccelStepS = constrain(motorConfig.rampAccelStepS, 1, PWM_MAX);
        motorConfig.pwmStartKickS = constrain(motorConfig.pwmStartKickS, 0, PWM_MAX);
        saveMotorConfig();
    }
};

class WifiSsidCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        bleSsid = String(pCharacteristic->getValue().c_str());
    }
};

class WifiPassCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        const String password = String(pCharacteristic->getValue().c_str());
        preferences.begin("wifi-config", false);
        preferences.putString("ssid", bleSsid);
        preferences.putString("pass", password);
        preferences.end();
        delay(200);
        ESP.restart();
    }
};

void setupBle() {
    BLEDevice::init(globalHostname.c_str());
    pServer = BLEDevice::createServer();

    BLEService *motorService = pServer->createService(MOTOR_SERVICE_UUID);
    pControlCharacteristic = motorService->createCharacteristic(
        MOTOR_CONTROL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pControlCharacteristic->setCallbacks(new MotorControlCallbacks());

    pMotorConfigCharacteristic = motorService->createCharacteristic(
        MOTOR_CONFIG_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pMotorConfigCharacteristic->setCallbacks(new MotorConfigCallbacks());
    pMotorConfigCharacteristic->addDescriptor(new BLE2902());
    pMotorConfigCharacteristic->setValue((uint8_t *)&motorConfig, sizeof(MotorConfig_t));
    motorService->start();

    BLEService *configService = pServer->createService(CONFIG_SERVICE_UUID);
    pSsidCharacteristic = configService->createCharacteristic(
        SSID_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pSsidCharacteristic->setCallbacks(new WifiSsidCallbacks());

    pPassCharacteristic = configService->createCharacteristic(
        PASS_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pPassCharacteristic->setCallbacks(new WifiPassCallbacks());
    configService->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(MOTOR_SERVICE_UUID);
    advertising->addServiceUUID(CONFIG_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();
}

void connectSavedWifi() {
    preferences.begin("wifi-config", true);
    const String ssid = preferences.getString("ssid", "");
    const String password = preferences.getString("pass", "");
    preferences.end();

    if (ssid.isEmpty()) {
        Serial.println("[NET] No saved Wi-Fi credentials.");
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(globalHostname.c_str());
    WiFi.begin(ssid.c_str(), password.c_str());
}

void motorRampTask() {
    const unsigned long now = millis();
    if (now - lastRampTime < RAMP_INTERVAL_MS) {
        return;
    }
    lastRampTime = now;

    auto rampOne = [](volatile int &current, int target, int step, int kick) {
        if (target == 0) {
            current = 0;
            return;
        }
        if (current == 0) {
            current = (target > 0) ? kick : -kick;
        }
        const int delta = target - current;
        if (abs(delta) <= step) {
            current = target;
        } else {
            current += (delta > 0) ? step : -step;
        }
    };

    rampOne(currentSpeedT, targetSpeedT, motorConfig.rampAccelStepT, motorConfig.pwmStartKickT);
    rampOne(currentSpeedS, targetSpeedS, motorConfig.rampAccelStepS, motorConfig.pwmStartKickS);

    if (targetSpeedS == 0) {
        steerStartTime = 0;
    } else if (steerStartTime == 0) {
        steerStartTime = now;
    } else if (now - steerStartTime > STEER_MAX_ON_TIME_MS) {
        currentSpeedS = constrain(currentSpeedS, -STEER_HOLD_PWM, STEER_HOLD_PWM);
    }

    currentSpeedT = constrain(currentSpeedT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT);
    currentSpeedS = constrain(currentSpeedS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
    setMotorPwm(currentSpeedT, currentSpeedS);
}

void enforceTimeouts() {
    const unsigned long now = millis();

    if (targetSpeedT != 0 && now - lastThrottleTime > motorConfig.controlTimeoutT) {
        targetSpeedT = 0;
        throttleTimedOut = true;
    }
    if (targetSpeedS != 0 && now - lastSteerTime > motorConfig.controlTimeoutS) {
        targetSpeedS = 0;
        steerTimedOut = true;
        steerStartTime = 0;
    }
}

void maybeStartServices() {
    if (servicesStarted || WiFi.status() != WL_CONNECTED) {
        return;
    }
    Serial.printf("[NET] Connected: %s\n", WiFi.localIP().toString().c_str());
    setupWebServices();
    servicesStarted = true;
}

void setup() {
    Serial.begin(115200);
    delay(300);

    checkRescueKey();
    runRollbackChecks();
    loadMotorConfig();

    pinMode(NSLEEP_PIN, OUTPUT);
    digitalWrite(NSLEEP_PIN, HIGH);
    analogReadResolution(12);
    analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);
    generateHostname();

    sampleBatteryVoltage();

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running && running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 && otaStateGetBool("pending_factory", false)) {
        xTaskCreate(finalizeOfficialUpdateTask, "finalize-official", 6144, nullptr, 1, nullptr);
    }

    xTaskCreate(markBootSuccessTask, "mark-boot-ok", 4096, nullptr, 1, nullptr);

    setupBle();
    connectSavedWifi();

    Serial.printf("[BOOT] Vibe Racer %s on %s\n", CURRENT_VERSION, globalHostname.c_str());
    Serial.printf("[CFG] Timeout T/S: %u / %u ms\n", motorConfig.controlTimeoutT, motorConfig.controlTimeoutS);
    Serial.printf("[CFG] Steer limit/step/kick: %d / %d / %d\n", motorConfig.pwmEffectiveLimitS, motorConfig.rampAccelStepS, motorConfig.pwmStartKickS);
}

void loop() {
    sampleBatteryVoltage();
    maybeStartServices();

    if (servicesStarted) {
        server.handleClient();
        ArduinoOTA.handle();
    }

    if (!isUpdating) {
        enforceTimeouts();
        motorRampTask();
    }

    delay(1);
}
