// 包含必要的庫
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>               // Web Server
#include <ArduinoOTA.h>              // 透過網路進行韌體更新
#include <ESPmDNS.h>                 // 區域網路名稱解析
#include "esp_ota_ops.h"             // OTA 相關操作
#include "esp_partition.h"           // 分區表操作
#include "esp_task_wdt.h"            // Watchdog Timer 函式庫
#include "esp32c3_gpio.h"            // 此處定義了 GPIO 腳位 (例如: AIN1_PIN, NSLEEP_PIN, BIN1_PIN, BIN2_PIN)

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>             // 用於儲存 Wi-Fi 憑證及馬達參數
#include <freertos/FreeRTOS.h>       // 繼續使用 FreeRTOS 任務功能
#include <freertos/task.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

// --- 全域變數 ---
String globalHostname;               // 基於 MAC 位址的唯一 Hostname
#define CURRENT_VERSION "2026.04.21.04" // UI 拆分：首頁 D-Pad，次頁 Joystick
WebServer server(80);                // 實例化同步 Web Server

// LEDC PWM 設定 (保持不變)
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;
const int LEDC_CH_A1 = 0;
const int LEDC_CH_A2 = 1;
const int LEDC_CH_B1 = 2;
const int LEDC_CH_B2 = 3;

// --- 馬達 Ramping 參數結構體 (與需求一致) ---
typedef struct {
    uint16_t controlTimeoutT; 
    uint16_t controlTimeoutS; 
    int pwmEffectiveLimitT; 
    int rampAccelStepT; 
    int pwmStartKickT; 
    int pwmEffectiveLimitS; 
    int rampAccelStepS; 
    int pwmStartKickS;
    uint8_t autoUpdateEnabled; // 是否開啟開機自動檢查更新 (0:關閉, 1:開啟)
    uint8_t padding;           // 對齊用
} MotorConfig_t;

// 【修正點 2】馬達 Ramping 核心變數
Preferences preferences;             // 全局 NVS 實例
MotorConfig_t motorConfig;           // 實例化結構體來存儲當前參數

volatile unsigned long lastControlTime = 0; // 上次接收到控制命令的時間戳記
volatile int targetSpeedT = 0;             // 遙控器送來的目標速度 (T Motor)
volatile int currentSpeedT = 0;            // 當前實際輸出給 PWM 的速度
volatile int targetSpeedS = 0;             // 遙控器送來的目標速度 (S Motor)
volatile int currentSpeedS = 0;            // 當前實際輸出給 PWM 的速度
const int RAMP_INTERVAL_MS = 10;           // 馬達斜坡控制間隔 (100Hz)
unsigned long lastRampTime = 0;

// --- BLE UUID 定義 (使用固定且唯一的 UUIDs) ---
// 服務 (Service)
const char* CONFIG_SERVICE_UUID  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"; 
const char* MOTOR_SERVICE_UUID   = "4fafc201-1fb5-459e-8fcc-c0ffee00dead"; 

// 特徵 (Characteristic) - **確保唯一性**
#define SSID_CHAR_UUID          "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Wi-Fi SSID (Write)
#define PASS_CHAR_UUID          "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Wi-Fi PASS (Write)
#define MOTOR_CONTROL_CHAR_UUID "4fafc202-1fb5-459e-8fcc-c0ffee01feed" // 馬達即時控制 (Write/Notify)
#define MOTOR_CONFIG_CHAR_UUID  "4fafc203-1fb5-459e-8fcc-c0ffee02dead" // 馬達配置參數 (Read/Write/Notify)

// --- 全域變數：BLE / NVS ---
BLEServer *pServer = NULL;
BLECharacteristic *pControlCharacteristic = NULL;
BLECharacteristic *pSsidCharacteristic = NULL;
BLECharacteristic *pPassCharacteristic = NULL;
BLECharacteristic *pMotorConfigCharacteristic = NULL; // 新增馬達配置特徵
String ble_ssid;
String ble_pass;
bool wifi_config_received = false;
volatile bool should_restart_advertising = false;
bool servicesStarted = false;
float batteryVoltage = 0.0;          // 電池電壓 ( filtered )
const float ADC_VOLT_REF = 3.1;      // ESP32-C3 ADC 參考電壓 ( 依實際情況微調 )
const float DIVIDER_RATIO = 2.0;     // 分壓比例 ( 100k+100k )
unsigned long lastBatteryCheck = 0;
const int BATT_CHECK_INTERVAL = 500; // 每 500ms 檢查一次

// --- 函式前置宣告 (Forward Declarations) ---
void loadMotorConfig();
void saveMotorConfig();
void performGitHubCloudUpdate();
void checkAndPerformAutoUpdate();
void handleRoot();
void handleJoystick();

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
        #joystick-inner { position: absolute; top: 5%; left: 5%; right: 5%; bottom: 5%; width: 90%; height: 90%; }
        #joystick-thumb {
            position: absolute; width: 70px; height: 70px; top: 50%; left: 50%;
            transform: translate(-50%, -50%); border-radius: 50%; background: #4f46e5;
            box-shadow: 0 0 15px #4f46e5, inset 0 0 10px #7c3aed;
        }
        #joystick-thumb.active { box-shadow: 0 0 25px #7c3aed, inset 0 0 15px #4f46e5; }
    </style>
</head>
<body class="p-4">
    <div class="container bg-gray-800 rounded-xl shadow-2xl">
        <h1 class="text-3xl font-extrabold text-center text-indigo-400 mb-2">Vibe Racer</h1>
        <p class="text-center text-[10px] mb-4 text-gray-500 uppercase tracking-widest">
            模式: 虛擬搖桿 | <a href="/" class="text-indigo-400 underline decoration-indigo-400/30">切換至 D-Pad</a>
        </p>
        <div class="bg-gray-900/50 rounded-lg p-3 mb-6 border border-gray-700">
            <div class="flex justify-between items-center mb-3">
                <div class="text-left"><p class="text-[10px] text-gray-500 uppercase">電池電壓</p><p class="text-xl font-mono font-bold text-green-400"><span id="val_v">0.00</span>V</p></div>
                <div class="text-right"><p class="text-[10px] text-gray-500 uppercase">即時功率</p><p class="text-sm font-mono text-indigo-300">T: <span id="val_rt">0</span> | S: <span id="val_rs">0</span></p></div>
            </div>
            <div class="grid grid-cols-2 gap-2 border-t border-gray-700 pt-3">
                <div class="space-y-1"><p class="text-[10px] text-gray-500 uppercase">加速檔位</p><div class="flex space-x-1"><span class="bg-indigo-500/10 text-indigo-300 text-[9px] px-1 rounded">L:<span id="cfg_lt"></span></span><span class="bg-indigo-500/10 text-indigo-300 text-[9px] px-1 rounded">S:<span id="cfg_st"></span></span></div></div>
                <div class="space-y-1"><p class="text-[10px] text-gray-500 uppercase">轉向檔位</p><div class="flex space-x-1"><span class="bg-indigo-500/10 text-indigo-300 text-[9px] px-1 rounded">L:<span id="cfg_ls"></span></span><span class="bg-indigo-500/10 text-indigo-300 text-[9px] px-1 rounded">S:<span id="cfg_ss"></span></span></div></div>
            </div>
        </div>
        <div id="joystick" class="mb-6"><div id="joystick-inner"><div id="joystick-thumb"></div></div></div>
        <p id="status" class="text-center text-sm font-bold text-green-400 mb-4 tracking-widest">● 靜止 ●</p>
        <div class="p-4 border-t border-gray-700 text-center text-[10px] text-gray-500"><a href="/update_factory" class="hover:text-indigo-400">韌體維修中心</a> | Ver 2026.04.21.04</div>
    </div>
    <script>
        const joystick = document.getElementById('joystick'); const thumb = document.getElementById('joystick-thumb'); const statusEl = document.getElementById('status');
        const valVEl = document.getElementById('val_v'); const valRTEl = document.getElementById('val_rt'); const valRSELEl = document.getElementById('val_rs');
        const DEADZONE_PWM = 20, maxRadius = joystick.clientWidth/2;
        let isDragging = false, controlInterval, lastMotorT = 0, lastMotorS = 0;
        function fetchConfig() { fetch('/config').then(r => r.json()).then(d => {
            document.getElementById('cfg_lt').textContent=d.pwmEffectiveLimitT; document.getElementById('cfg_st').textContent=d.rampAccelStepT;
            document.getElementById('cfg_ls').textContent=d.pwmEffectiveLimitS; document.getElementById('cfg_ss').textContent=d.rampAccelStepS;
        }); }
        fetchConfig();
        function updateMotorValues(ox, oy) {
            const ds = Math.sqrt(ox*ox + oy*oy); const mg = Math.min(1.0, ds/maxRadius);
            let rx = ox/maxRadius, ry = oy/maxRadius; let maxAxial = Math.max(Math.abs(rx), Math.abs(ry));
            if (maxAxial > 0) { let sc = mg/maxAxial; rx *= sc; ry *= sc; }
            lastMotorS = Math.round(rx*255); lastMotorT = Math.round(ry*255);
            if (Math.abs(lastMotorT) < DEADZONE_PWM) lastMotorT = 0;
            if (Math.abs(lastMotorS) < DEADZONE_PWM) lastMotorS = 0;
            statusEl.textContent = (lastMotorT===0 && lastMotorS===0) ? "● 靜止 ●" : "● 運行中 ●";
        }
        function sendControl(T, S) { fetch(`/control?t=${T}&s=${S}`).then(r => r.json()).then(data => { valVEl.textContent = data.v.toFixed(2); valRTEl.textContent = data.rt; valRSELEl.textContent = data.rs; }); }
        function stop() { isDragging = false; clearInterval(controlInterval); thumb.style.left = '50%'; thumb.style.top = '50%'; thumb.style.transform = 'translate(-50%, -50%)'; lastMotorT = 0; lastMotorS = 0; sendControl(0,0); }
        joystick.addEventListener('mousedown', e => { isDragging=true; thumb.classList.add('active'); startComm(); });
        joystick.addEventListener('touchstart', e => { isDragging=true; thumb.classList.add('active'); startComm(); });
        document.addEventListener('mousemove', e => {
            if(!isDragging) return; const rect = joystick.getBoundingClientRect();
            let ox = (e.touches?e.touches[0].clientX:e.clientX) - (rect.left+maxRadius);
            let oy = (e.touches?e.touches[0].clientY:e.clientY) - (rect.top+maxRadius);
            const ds = Math.sqrt(ox*ox+oy*oy); if(ds>maxRadius){ const an=Math.atan2(oy,ox); ox=maxRadius*Math.cos(an); oy=maxRadius*Math.sin(an); }
            thumb.style.left=`${maxRadius+ox}px`; thumb.style.top=`${maxRadius+oy}px`; thumb.style.transform='translate(-50%,-50%)';
            updateMotorValues(ox, -oy);
        });
        document.addEventListener('touchmove', e => { if(isDragging) e.preventDefault(); }, {passive:false});
        document.addEventListener('mouseup', stop); document.addEventListener('touchend', stop);
        function startComm() { if(controlInterval) clearInterval(controlInterval); controlInterval = setInterval(()=>sendControl(lastMotorT, lastMotorS), 50); }
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
        .container { max-width: 400px; width: 100%; padding: 20px; }
        .dpad-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; width: 210px; margin: 0 auto; }
        .btn-dpad { aspect-ratio: 1/1; border-radius: 16px; background: #1e293b; border: 1px solid rgba(255,255,255,0.1); display: flex; justify-content: center; align-items: center; cursor: pointer; transition: 0.1s; -webkit-tap-highlight-color: transparent; }
        .btn-dpad:active { background: #4f46e5; transform: scale(0.95); box-shadow: 0 0 15px rgba(79, 70, 229, 0.4); }
        .btn-dpad svg { width: 32px; height: 32px; color: #94a3b8; }
        .btn-dpad:active svg { color: white; }
    </style>
</head>
<body class="p-4">
    <div class="container bg-gray-800 rounded-2xl shadow-2xl overflow-hidden">
        <div class="p-6 bg-indigo-600/10 border-b border-indigo-500/20 mb-6 text-center">
            <h1 class="text-2xl font-bold text-indigo-400">Vibe Racer</h1>
            <p class="text-[10px] mt-1 text-gray-500 uppercase tracking-widest">測試模式: D-Pad | <a href="/joystick" class="text-indigo-400 underline">進入搖桿</a></p>
        </div>
        <div class="px-6 mb-8">
            <div class="bg-gray-900/50 rounded-xl p-4 border border-gray-700">
                <div class="flex justify-between items-center mb-4 text-left">
                    <div><p class="text-[10px] text-gray-500 uppercase">電池電壓</p><p class="text-lg font-mono font-bold text-green-400"><span id="val_v">0.00</span>V</p></div>
                    <div class="text-right"><p class="text-[10px] text-gray-500 uppercase">即時功率</p><p class="text-xs font-mono text-indigo-300">T:<span id="val_rt">0</span> S:<span id="val_rs">0</span></p></div>
                </div>
                <div class="grid grid-cols-2 gap-2 border-t border-gray-700/50 pt-4 text-[9px]">
                    <div><p class="text-gray-500 mb-1">T-CONFIG</p><div class="flex space-x-1"><span class="bg-indigo-500/5 px-1 rounded border border-white/5">L:<span id="cfg_lt"></span></span><span class="bg-indigo-500/5 px-1 rounded border border-white/5">S:<span id="cfg_st"></span></span></div></div>
                    <div><p class="text-gray-500 mb-1 text-right">S-CONFIG</p><div class="flex space-x-1 justify-end"><span class="bg-indigo-500/5 px-1 rounded border border-white/5">L:<span id="cfg_ls"></span></span><span class="bg-indigo-500/5 px-1 rounded border border-white/5">S:<span id="cfg_ss"></span></span></div></div>
                </div>
            </div>
        </div>
        <div class="dpad-grid mb-10">
            <div></div><div class="btn-dpad" onmousedown="mt(255,0)" ontouchstart="mt(255,0)"><svg viewBox="0 0 24 24" fill="currentColor"><path d="M12 4l-8 8h16z"/></svg></div><div></div>
            <div class="btn-dpad" onmousedown="mt(0,-255)" ontouchstart="mt(0,-255)"><svg viewBox="0 0 24 24" fill="currentColor"><path d="M4 12l8-8v16z"/></svg></div>
            <div class="btn-dpad bg-red-500/10 border-red-500/20" onmousedown="sp()" ontouchstart="sp()"><div class="w-3 h-3 bg-red-500 rounded-sm"></div></div>
            <div class="btn-dpad" onmousedown="mt(0,255)" ontouchstart="mt(0,255)"><svg viewBox="0 0 24 24" fill="currentColor"><path d="M20 12l-8-8v16z"/></svg></div>
            <div></div><div class="btn-dpad" onmousedown="mt(-255,0)" ontouchstart="mt(-255,0)"><svg viewBox="0 0 24 24" fill="currentColor"><path d="M12 20l-8-8h16z"/></svg></div><div></div>
        </div>
        <div class="p-4 bg-gray-900/40 text-center flex justify-between text-[10px] text-gray-600"><a href="/update_factory" class="hover:text-indigo-400 underline">維護中心</a><span>Ver 2026.04.21.04</span></div>
    </div>
    <script>
        let cT=0, cS=0, itv=null;
        function mt(t,s){ cT=t; cS=s; if(itv)clearInterval(itv); itv=setInterval(()=>sc(cT,cS),50); sc(cT,cS); }
        function sp(){ cT=0; cS=0; if(itv)clearInterval(itv); sc(0,0); }
        function sc(t,s){ fetch(`/control?t=${t}&s=${s}`).then(r=>r.json()).then(d=>{ document.getElementById('val_v').textContent=d.v.toFixed(2); document.getElementById('val_rt').textContent=d.rt; document.getElementById('val_rs').textContent=d.rs; }); }
        fetch('/config').then(r=>r.json()).then(d=>{ document.getElementById('cfg_lt').textContent=d.pwmEffectiveLimitT; document.getElementById('cfg_st').textContent=d.rampAccelStepT; document.getElementById('cfg_ls').textContent=d.pwmEffectiveLimitS; document.getElementById('cfg_ss').textContent=d.rampAccelStepS; });
        document.addEventListener('mouseup', sp); document.addEventListener('touchend', sp);
    </script>
</body>
</html>)rawliteral";

            const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            const magnitude = Math.min(1.0, distance / maxRadius);
            
            // 使用 Scaled-to-Square 映射，確保在圓形搖桿邊緣的任何角度（包括斜角）都能輸出 255
            let rawX = offsetX / maxRadius;
            let rawY = offsetY / maxRadius;
            
            let maxAxial = Math.max(Math.abs(rawX), Math.abs(rawY));
            if (maxAxial > 0) {
                let scale = magnitude / maxAxial;
                rawX *= scale;
                rawY *= scale;
            }

            let speedS = Math.round(rawX * 255);
            let speedT = Math.round(rawY * 255);

            if (Math.abs(speedT) < DEADZONE_PWM) speedT = 0;
            if (Math.abs(speedS) < DEADZONE_PWM) speedS = 0;

            valYEl.textContent = speedT;
            valXEl.textContent = speedS;
            
            let currentStatus = "靜止";
            let statusColor = "text-green-400";
            if (Math.abs(speedT) > 0 || Math.abs(speedS) > 0) {
                 statusColor = "text-yellow-400";
                 if (speedT > 50 && Math.abs(speedS) < 50) currentStatus = "前進中";
                 else if (speedT < -50 && Math.abs(speedS) < 50) currentStatus = "後退中";
                 else if (speedS > 50) currentStatus = "右轉中";
                 else if (speedS < -50) currentStatus = "左轉中";
                 else currentStatus = "移動中";
            }
            statusEl.textContent = currentStatus;
            statusEl.className = "text-sm font-bold " + statusColor;

            // 僅更新快取，實際發送由 setInterval 負責
            lastMotorT = speedT;
            lastMotorS = speedS;
        }

        // Config Elements Dashboard
        const cfgLT = document.getElementById('cfg_lt');
        const cfgST = document.getElementById('cfg_st');
        const cfgKT = document.getElementById('cfg_kt');
        const cfgLS = document.getElementById('cfg_ls');
        const cfgSS = document.getElementById('cfg_ss');
        const cfgKS = document.getElementById('cfg_ks');

        function fetchConfig() {
            fetch('/config')
                .then(r => r.json())
                .then(data => {
                    if(cfgLT) cfgLT.textContent = data.pwmEffectiveLimitT;
                    if(cfgST) cfgST.textContent = data.rampAccelStepT;
                    if(cfgKT) cfgKT.textContent = data.pwmStartKickT;
                    if(cfgLS) cfgLS.textContent = data.pwmEffectiveLimitS;
                    if(cfgSS) cfgSS.textContent = data.rampAccelStepS;
                    if(cfgKS) cfgKS.textContent = data.pwmStartKickS;
                })
                .catch(e => console.error('Config fetch failed', e));
        }
        fetchConfig();

        function sendControl(T, S) {
            fetch(`${baseIp}/control?t=${T}&s=${S}`)
                .then(response => response.json())
                .then(data => {
                    // 更新電壓與即時 Ramp 數值
                    if (data.v !== undefined) {
                        valVEl.textContent = data.v.toFixed(2);
                        // 低電壓警示 (假設 1S 鋰電池 3.4V)
                        if (data.v < 3.4) valVEl.className = "text-red-500";
                        else if (data.v < 3.6) valVEl.className = "text-yellow-400";
                        else valVEl.className = "text-green-400";
                    }
                    if (data.rt !== undefined) valRTEl.textContent = data.rt;
                    if (data.rs !== undefined) valRSELEl.textContent = data.rs;
                })
                .catch(err => console.error('Fetch error:', err));
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
            
            // 歸零快取並發送最後一次停止命令
            lastMotorT = 0;
            lastMotorS = 0;
            updateMotorValues(0, 0); 
            sendControl(0, 0);
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

            // 使用定時器發送控制命令 (20Hz / 每 50ms 一次)
            // 這樣可以平衡「即時性」與「網路負擔」，避免因請求過快導致 ESP32 當機
            if (controlInterval) clearInterval(controlInterval);
            controlInterval = setInterval(() => {
                sendControl(lastMotorT, lastMotorS);
            }, 50); 
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
</html>)rawliteral";

const char UPDATE_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Updating Vibe Racer...</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { background: #0f172a; color: #f9fafb; font-family: system-ui; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; }
        .glass { background: rgba(30, 41, 59, 0.7); backdrop-filter: blur(12px); border: 1px solid rgba(255,255,255,0.1); border-radius: 24px; }
        .spinner { border: 4px solid rgba(255, 255, 255, 0.1); border-left-color: #6366f1; border-radius: 50%; width: 40px; height: 40px; animation: spin 1s linear infinite; }
        @keyframes spin { to { transform: rotate(360deg); } }
    </style>
</head>
<body class="p-4 text-center">
    <div class="glass p-8 max-w-sm w-full shadow-2xl">
        <div class="flex justify-center mb-6">
            <div class="spinner"></div>
        </div>
        <h1 class="text-2xl font-bold text-indigo-400 mb-2">系統更新中...</h1>
        <p class="text-gray-400 text-sm mb-6">正在從 GitHub 獲取最新韌體，請勿關閉電源。</p>
        
        <div class="space-y-3 text-left text-xs text-gray-500 mb-8">
            <div class="flex items-center space-x-2"><span class="w-2 h-2 bg-green-500 rounded-full"></span> <span>連線至 GitHub...</span></div>
            <div class="flex items-center space-x-2"><span class="w-2 h-2 bg-indigo-500 rounded-full animate-pulse"></span> <span>下載並寫入 Flash...</span></div>
            <div class="flex items-center space-x-2"><span class="w-2 h-2 bg-gray-600 rounded-full"></span> <span>重新啟動裝置...</span></div>
        </div>

        <p id="countdown" class="text-indigo-300/60 text-xs">預計於 40 秒後自動跳轉</p>
    </div>

    <script>
        let timeLeft = 40;
        const countdownEl = document.getElementById('countdown');
        const timer = setInterval(() => {
            timeLeft--;
            countdownEl.textContent = `預計於 ${timeLeft} 秒後自動跳轉`;
            if (timeLeft <= 0) {
                clearInterval(timer);
                location.href = '/';
            }
        }, 1000);

        // 30 秒後開始嘗試連線小車，一旦成功就跳轉
        setTimeout(() => {
            setInterval(() => {
                fetch('/').then(r => { if(r.ok) location.href = '/'; }).catch(e => {});
            }, 3000);
        }, 30000);
    </script>
</body>
</html>)rawliteral";
const char MAINTENANCE_PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Vibe Racer Maintenance</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        body { background: #0f172a; color: #f9fafb; font-family: system-ui; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 1rem;}
        .glass { background: rgba(30, 41, 59, 0.7); backdrop-filter: blur(12px); border: 1px solid rgba(255,255,255,0.1); border-radius: 24px; width: 100%; max-width: 450px; overflow: hidden; }
        .section { border-bottom: 1px solid rgba(255,255,255,0.05); padding: 1.5rem; }
        .btn { transition: all 0.2s; cursor: pointer; border-radius: 12px; font-weight: bold; }
        .btn:hover { transform: translateY(-1px); }
        input[type="file"]::file-selector-button { background: #374151; color: white; padding: 0.5rem 1rem; border: none; border-radius: 8px; margin-right: 1rem; cursor: pointer; }
    </style>
</head>
<body>
    <div class="glass shadow-2xl">
        <div class="bg-indigo-600/20 p-6 border-b border-indigo-500/30">
            <h1 class="text-2xl font-bold text-indigo-300">維修與更新</h1>
            <p class="text-xs text-gray-400 mt-1">裝置 ID: %HOSTNAME% | 版本: <span class="text-indigo-400 font-mono">%VERSION%</span></p>
        </div>

        <!-- 系統設定區 -->
        <div class="section">
            <h2 class="text-sm font-bold text-gray-400 uppercase tracking-widest mb-4">系統設定</h2>
            <div class="flex items-center justify-between p-4 bg-white/5 rounded-2xl border border-white/5">
                <div>
                    <h3 class="font-bold text-gray-200">開機自動更新</h3>
                    <p class="text-xs text-gray-500">開機連網後自動檢查 GitHub 版本</p>
                </div>
                <button onclick="location.href='/toggle_auto_update'" class="relative inline-flex items-center h-6 rounded-full w-11 transition-colors focus:outline-none %TOGGLE_BG_COLOR%">
                    <span class="inline-block w-4 h-4 transform bg-white rounded-full transition-transform %TOGGLE_DOT_POS%"></span>
                </button>
            </div>
        </div>

        <!-- GitHub 更新區 -->
        <div class="section bg-indigo-500/5">
            <h2 class="text-sm font-bold text-gray-400 uppercase tracking-widest mb-4">雲端更新 (GitHub)</h2>
            <div class="p-4 rounded-2xl border border-indigo-500/20">
                <p class="text-xs text-gray-500 mb-4">自動抓取 <code>latest/firmware.bin</code> 並更新至 Factory 分區。</p>
                <button onclick="if(confirm('確認要從 GitHub 下載最新韌體？')) location.href='/update_github';" 
                    class="w-full btn bg-indigo-600 hover:bg-indigo-500 text-white p-3 shadow-lg flex items-center justify-center space-x-2">
                    <span>🚀 啟動 GitHub 雲端更新</span>
                </button>
            </div>
        </div>

        <!-- 本地上傳區 -->
        <div class="section">
            <h2 class="text-sm font-bold text-gray-400 uppercase tracking-widest mb-4">本地上傳 (Manual)</h2>
            <form method="POST" action="/update_factory" enctype="multipart/form-data" class="space-y-4">
                <div class="p-4 bg-gray-900/40 rounded-2xl border border-white/5">
                    <input type="file" name="update" class="text-xs text-gray-400 w-full">
                </div>
                <button type="submit" class="w-full btn bg-gray-700 hover:bg-gray-600 text-gray-300 p-3">
                    傳送並更新 Factory 分區
                </button>
            </form>
        </div>

        <div class="p-4 text-center">
            <a href="/" class="text-xs text-gray-500 hover:text-indigo-400 transition">← 返回控制主頁</a>
        </div>
    </div>
</body>
</html>)rawliteral";

// --- 自定義 Factory OTA 更新處理 ---
void handleFactoryUpdate() {
    String html = MAINTENANCE_PAGE_HTML;
    html.replace("%HOSTNAME%", globalHostname);
    html.replace("%VERSION%", CURRENT_VERSION);
    
    // 處理 Toggle 狀態
    if (motorConfig.autoUpdateEnabled == 1) {
        html.replace("%TOGGLE_BG_COLOR%", "bg-indigo-600");
        html.replace("%TOGGLE_DOT_POS%", "translate-x-6");
    } else {
        html.replace("%TOGGLE_BG_COLOR%", "bg-gray-700");
        html.replace("%TOGGLE_DOT_POS%", "translate-x-1");
    }

    server.send(200, "text/html", html);
}

void handleToggleAutoUpdate() {
    motorConfig.autoUpdateEnabled = (motorConfig.autoUpdateEnabled == 1) ? 0 : 1;
    saveMotorConfig();
    // 重新跳轉回維修頁面
    server.sendHeader("Location", "/update_factory");
    server.send(303);
}

void handleFactoryUpdateUpload() {
    HTTPUpload& upload = server.upload();
    static bool ota_started = false;
    static const esp_partition_t* factory_partition = NULL;
    static esp_ota_handle_t update_handle = 0;

    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Factory OTA Start: %s\n", upload.filename.c_str());
        factory_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
        if (factory_partition == NULL) {
            Serial.println("❌ 找不到 Factory 分區!");
            return;
        }
        esp_err_t err = esp_ota_begin(factory_partition, OTA_SIZE_UNKNOWN, &update_handle);
        if (err != ESP_OK) {
            Serial.printf("❌ esp_ota_begin 失敗: %s\n", esp_err_to_name(err));
            return;
        }
        ota_started = true;
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (ota_started) {
            esp_err_t err = esp_ota_write(update_handle, upload.buf, upload.currentSize);
            if (err != ESP_OK) {
                Serial.printf("❌ esp_ota_write 失敗: %s\n", esp_err_to_name(err));
                ota_started = false;
            }
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (ota_started) {
            esp_err_t err = esp_ota_end(update_handle);
            if (err != ESP_OK) {
                Serial.printf("❌ esp_ota_end 失敗: %s\n", esp_err_to_name(err));
            } else {
                err = esp_ota_set_boot_partition(factory_partition);
                if (err != ESP_OK) {
                    Serial.printf("❌ 設定啟動分區失敗: %s\n", esp_err_to_name(err));
                } else {
                    Serial.println("✅ Factory OTA 更新成功! 正在重啟...");
                    server.send(200, "text/plain", "SUCCESS. Rebooting...");
                    delay(500);
                    ESP.restart();
                }
            }
        }
        ota_started = false;
    }
}

// --- GitHub 雲端 OTA 更新邏輯 ---
void performGitHubCloudUpdate() {
    String url = "https://github.com/VibeCoding-tw/esp32c3-launcher/releases/latest/download/firmware.bin";
    Serial.println("--- 啟動 GitHub 雲端更新 ---");
    Serial.print("Target: "); Serial.println(url);

    WiFiClientSecure client;
    client.setInsecure(); // 在此場景下跳過證書驗證以提高相容性

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // 必須追蹤 GitHub 的 302 重定向
    
    if (http.begin(client, url)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            int contentLength = http.getSize();
            if (contentLength <= 0) {
                Serial.println("❌ 無法取得檔案大小");
                http.end();
                return;
            }

            // 取得 Factory 分區
            const esp_partition_t* factory_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
            if (factory_partition == NULL) {
                Serial.println("❌ 找不到 Factory 分區!");
                http.end();
                return;
            }

            esp_ota_handle_t update_handle = 0;
            esp_err_t err = esp_ota_begin(factory_partition, OTA_SIZE_UNKNOWN, &update_handle);
            if (err != ESP_OK) {
                Serial.printf("❌ esp_ota_begin 失敗: %s\n", esp_err_to_name(err));
                http.end();
                return;
            }

            WiFiClient* stream = http.getStreamPtr();
            size_t written = 0;
            uint8_t buff[1024];
            
            while (http.connected() && (written < contentLength)) {
                size_t size = stream->available();
                if (size) {
                    int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
                    esp_ota_write(update_handle, buff, c);
                    written += c;
                    if (written % 10240 == 0) Serial.printf("Progress: %u%%\r", (written * 100) / contentLength);
                }
                delay(1);
            }

            if (written == contentLength) {
                err = esp_ota_end(update_handle);
                if (err == ESP_OK) {
                    err = esp_ota_set_boot_partition(factory_partition);
                    if (err == ESP_OK) {
                        Serial.println("\n✅ GitHub 雲端更新成功! 正在重啟...");
                        delay(500);
                        ESP.restart();
                    }
                }
            } else {
                Serial.println("\n❌ 下載不完整");
            }
        } else {
            Serial.printf("❌ HTTP GET 失敗, code: %d\n", httpCode);
        }
        http.end();
    } else {
        Serial.println("❌ 無法連線至 GitHub");
    }
}

void handleGitHubUpdate() {
    server.send(200, "text/html", UPDATE_PAGE_HTML);
    // 延遲一下讓 Web Response 噴出去再開始
    delay(1000);
    performGitHubCloudUpdate();
}

// 檢查遠端版本並決定是否更新
void checkAndPerformAutoUpdate() {
    if (motorConfig.autoUpdateEnabled != 1) {
        Serial.println("[AutoUpdate] 功能已關閉，跳過檢查。");
        return;
    }

    Serial.println("[AutoUpdate] 正在檢查 GitHub 上的最新版本...");
    String versionUrl = "https://github.com/VibeCoding-tw/esp32c3-launcher/releases/latest/download/version.txt";
    
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    if (http.begin(client, versionUrl)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String remoteVersion = http.getString();
            remoteVersion.trim();
            Serial.printf("[AutoUpdate] 本地版本: %s, 遠端版本: %s\n", CURRENT_VERSION, remoteVersion.c_str());
            
            if (remoteVersion != CURRENT_VERSION && remoteVersion.length() > 0) {
                Serial.println("[AutoUpdate] 偵測到新版本! 準備開始自動更新...");
                performGitHubCloudUpdate();
            } else {
                Serial.println("[AutoUpdate] 目前已是最新版本。");
            }
        } else {
            Serial.printf("[AutoUpdate] 無法取得版本資訊, code: %d\n", httpCode);
        }
        http.end();
    }
}

// --- 產生 Hostname ---
void generateHostname() {    
    globalHostname = "esp32c3-" + WiFi.macAddress(); 
    globalHostname.replace(":", ""); 
    globalHostname.toLowerCase(); 
    Serial.printf("Generated Hostname: %s\n", globalHostname.c_str());
}

// 設定預設值與從 NVS 載入參數
void loadMotorConfig() {
    // 預設值 (如果 NVS 中沒有)
    MotorConfig_t defaultConfig = {
        .controlTimeoutT = 500, 
        .controlTimeoutS = 2000, 
        .pwmEffectiveLimitT = 200, 
        .rampAccelStepT = 3, 
        .pwmStartKickT = 60, 
        .pwmEffectiveLimitS = 255, 
        .rampAccelStepS = 20, // 提升轉向步長至 20
        .pwmStartKickS = 120,
        .autoUpdateEnabled = 0, // 預設關閉自動更新以策安全
        .padding = 0
    };

    preferences.begin("motor-config", true);
    size_t size = preferences.getBytes("config", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();

    if (size != sizeof(MotorConfig_t)) {
        motorConfig = defaultConfig; // 載入預設值
        Serial.println("❌ NVS 無馬達參數，使用預設值。");
    } else {
        Serial.println("✅ 從 NVS 載入馬達參數成功。");
    }
    
    // 輸出當前配置
    Serial.printf("  Timeout T: %u ms, S: %u ms\n", motorConfig.controlTimeoutT, motorConfig.controlTimeoutS);
    Serial.printf("  T Limit: %d, T Step: %d, T Kick: %d\n", motorConfig.pwmEffectiveLimitT, motorConfig.rampAccelStepT, motorConfig.pwmStartKickT);
    Serial.printf("  S Limit: %d, S Step: %d, S Kick: %d\n", motorConfig.pwmEffectiveLimitS, motorConfig.rampAccelStepS, motorConfig.pwmStartKickS);
    Serial.printf("  Auto Update: %s\n", motorConfig.autoUpdateEnabled ? "Enabled" : "Disabled");
}

// 將當前參數儲存到 NVS
void saveMotorConfig() {
    preferences.begin("motor-config", false);
    preferences.putBytes("config", &motorConfig, sizeof(MotorConfig_t));
    preferences.end();
    Serial.println("✅ 馬達參數已儲存到 NVS。");
}

// --- 電壓採樣與濾波 ---
void updateBatteryVoltage() {
    if (millis() - lastBatteryCheck < BATT_CHECK_INTERVAL) return;
    lastBatteryCheck = millis();

    int rawAdc = analogRead(BATT_ADC_PIN);
    // 將 ADC 轉換為電壓 ( C3 的 ADC 範圍約 0-3.3V, 12-bit )
    float sensedVolt = (rawAdc / 4095.0) * ADC_VOLT_REF * DIVIDER_RATIO;

    // 指數移動平均濾波 ( Expo Smoothing )
    if (batteryVoltage < 0.1) batteryVoltage = sensedVolt; // 第一次讀取初始化
    else batteryVoltage = (batteryVoltage * 0.9) + (sensedVolt * 0.1);
}

// --- 修正後的 PWM 寫入 (增加 H 橋安全) ---
void setMotorPwm(int speedT, int speedS) {
    // T 馬達防直通處理
    if (speedT > 0) { 
        ledcWrite(LEDC_CH_A2, 0); // 確保另一端先關閉
        ledcWrite(LEDC_CH_A1, speedT);
    } else if (speedT < 0) { 
        ledcWrite(LEDC_CH_A1, 0);
        ledcWrite(LEDC_CH_A2, -speedT); 
    } else { 
        ledcWrite(LEDC_CH_A1, 0); ledcWrite(LEDC_CH_A2, 0);
    }

    // S 馬達防直通處理
    if (speedS > 0) { 
        ledcWrite(LEDC_CH_B1, 0);
        ledcWrite(LEDC_CH_B2, speedS);
    } else if (speedS < 0) { 
        ledcWrite(LEDC_CH_B2, 0);
        ledcWrite(LEDC_CH_B1, -speedS); 
    } else { 
        ledcWrite(LEDC_CH_B1, 0); ledcWrite(LEDC_CH_B2, 0);
    }
}

// --- 修正後的馬達 Ramping 任務 ---
// 額外定義一個變數追蹤 S 馬達持續輸出的時間
unsigned long sMotorStartTime = 0;
const unsigned long S_MOTOR_MAX_ON_TIME = 3000; // 延長至 3 秒，讓使用者能持續維持大幅度轉向

void motorRampTask() {
    if (millis() - lastRampTime < RAMP_INTERVAL_MS) return;
    lastRampTime = millis();
    
    // --- 1. 速度馬達 (T Motor) 邏輯 ---
    if (targetSpeedT == 0) {
        // 速度馬達通常允許慣性滑行或較快減速
        currentSpeedT = 0; 
    } else {
        if (currentSpeedT == 0) {
            // 優化：Kickstart 最小也必須等於目標，或取 Kickstart 值但不得過衝
            int kick = (targetSpeedT > 0) ? motorConfig.pwmStartKickT : -motorConfig.pwmStartKickT;
            if (abs(kick) > abs(targetSpeedT)) currentSpeedT = targetSpeedT;
            else currentSpeedT = kick;
        }
        
        int diffT = targetSpeedT - currentSpeedT;
        if (abs(diffT) <= motorConfig.rampAccelStepT) {
            currentSpeedT = targetSpeedT;
        } else {
            currentSpeedT += (diffT > 0) ? motorConfig.rampAccelStepT : -motorConfig.rampAccelStepT;
        }
    }

    // --- 2. 轉向馬達 (S Motor) 邏輯優化 ---
    if (targetSpeedS == 0) {
        currentSpeedS = 0;
        sMotorStartTime = 0; // 重置計時器
    } else {
        // 紀錄開始轉向的時間點，用以實作「超時降壓」保護邏輯
        if (sMotorStartTime == 0) sMotorStartTime = millis();

        // 啟動 Kickstart
        if (currentSpeedS == 0) {
            // 優化：Kickstart 不應超過目標速度，減少小角度轉向時的抖動
            int kick = (targetSpeedS > 0) ? motorConfig.pwmStartKickS : -motorConfig.pwmStartKickS;
            if (abs(kick) > abs(targetSpeedS)) currentSpeedS = targetSpeedS;
            else currentSpeedS = kick;
        }

        // 轉向 Ramping: 對於兩線式馬達，轉向通常需要比速度更快的響應
        int diffS = targetSpeedS - currentSpeedS;
        if (abs(diffS) <= motorConfig.rampAccelStepS) {
            currentSpeedS = targetSpeedS;
        } else {
            currentSpeedS += (diffS > 0) ? motorConfig.rampAccelStepS : -motorConfig.rampAccelStepS;
        }

        // 【安全保護】如果轉向馬達輸出時間過長，可能是打死了，強制降低輸出以防燒毀
        if (millis() - sMotorStartTime > S_MOTOR_MAX_ON_TIME) {
            // 將輸出限制在 230 (維持強勁扭矩對抗彈簧，但避免長期滿載)
            currentSpeedS = constrain(currentSpeedS, -230, 230); 
        }
    }

    // 最終約束
    currentSpeedT = constrain(currentSpeedT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT);
    currentSpeedS = constrain(currentSpeedS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
    
    setMotorPwm(currentSpeedT, currentSpeedS); 

    // 僅在有動作時輸出 Serial
    static int pLastT, pLastS;
    if (currentSpeedT != pLastT || currentSpeedS != pLastS) {
        if (currentSpeedT != 0 || currentSpeedS != 0) {
            Serial.printf("[Ramp] T:%d | S:%d\n", currentSpeedT, currentSpeedS);
        }
        pLastT = currentSpeedT; pLastS = currentSpeedS;
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
        
        targetSpeedT = constrain(rawT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT); 
        targetSpeedS = constrain(rawS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
        
        lastControlTime = millis();

        // 構造 JSON 回應 (包含電壓與即時 Ramp 速度)
        String json = "{";
        json += "\"t\":" + String(targetSpeedT) + ",";
        json += "\"s\":" + String(targetSpeedS) + ",";
        json += "\"rt\":" + String(currentSpeedT) + ","; // Real-time Throttle
        json += "\"rs\":" + String(currentSpeedS) + ","; // Real-time Steering
        json += "\"v\":" + String(batteryVoltage, 2) + ",";
        bool isTimeoutT = (millis() - lastControlTime > motorConfig.controlTimeoutT);
        bool isTimeoutS = (millis() - lastControlTime > motorConfig.controlTimeoutS);
        json += "\"to\":" + String(isTimeoutT ? 1 : 0) + ",";
        json += "\"tos\":" + String(isTimeoutS ? 1 : 0);
        json += "}";

        if (pControlCharacteristic) {
            String response = "T:" + String(targetSpeedT) + ",S:" + String(targetSpeedS);    
            pControlCharacteristic->setValue(response.c_str());
            pControlCharacteristic->notify(); 
        }

        server.send(200, "application/json", json); 
    } else {
        server.send(400, "text/plain", "Invalid arguments");
    }
}

// 處理馬達配置參數 (GET/POST)
void handleMotorConfig() {
    if (server.method() == HTTP_GET) {
        // GET: 回傳當前配置 (JSON 格式)
        String json = "{\n";
        json += "  \"timeoutT\": " + String(motorConfig.controlTimeoutT) + ",\n";
        json += "  \"timeoutS\": " + String(motorConfig.controlTimeoutS) + ",\n";
        json += "  \"pwmEffectiveLimitT\": " + String(motorConfig.pwmEffectiveLimitT) + ",\n";
        json += "  \"rampAccelStepT\": " + String(motorConfig.rampAccelStepT) + ",\n";
        json += "  \"pwmStartKickT\": " + String(motorConfig.pwmStartKickT) + ",\n";
        json += "  \"pwmEffectiveLimitS\": " + String(motorConfig.pwmEffectiveLimitS) + ",\n";
        json += "  \"rampAccelStepS\": " + String(motorConfig.rampAccelStepS) + ",\n";
        json += "  \"pwmStartKickS\": " + String(motorConfig.pwmStartKickS) + "\n";
        json += "}";
        server.send(200, "application/json", json);
    } else if (server.method() == HTTP_POST) {
        // POST: 接收並更新配置
        if (server.hasArg("timeoutT")) motorConfig.controlTimeoutT = server.arg("timeoutT").toInt();
        if (server.hasArg("timeoutS")) motorConfig.controlTimeoutS = server.arg("timeoutS").toInt();
        if (server.hasArg("limitT")) motorConfig.pwmEffectiveLimitT = server.arg("limitT").toInt();
        if (server.hasArg("stepT")) motorConfig.rampAccelStepT = server.arg("stepT").toInt();
        if (server.hasArg("kickT")) motorConfig.pwmStartKickT = server.arg("kickT").toInt();
        if (server.hasArg("limitS")) motorConfig.pwmEffectiveLimitS = server.arg("limitS").toInt();
        if (server.hasArg("stepS")) motorConfig.rampAccelStepS = server.arg("stepS").toInt();
        if (server.hasArg("kickS")) motorConfig.pwmStartKickS = server.arg("kickS").toInt();
        if (server.hasArg("autoUpdate")) motorConfig.autoUpdateEnabled = server.arg("autoUpdate").toInt();

        saveMotorConfig(); // 儲存到 NVS
        
        // 通知 BLE 客戶端配置已更新
        if (pMotorConfigCharacteristic) {
            pMotorConfigCharacteristic->setValue((uint8_t*)&motorConfig, sizeof(MotorConfig_t));
            pMotorConfigCharacteristic->notify();
        }

        server.send(200, "text/plain", "Motor config updated and saved.");
    } else {
        server.send(405, "text/plain", "Method Not Allowed");
    }
}

void setupWebServer() {
    Serial.println("--- 啟動 Web Server (STA 模式) ---");
    server.on("/", HTTP_GET, handleRoot);
    server.on("/joystick", HTTP_GET, handleJoystick);
    server.on("/control", HTTP_GET, handleControl);
    server.on("/config", HTTP_ANY, handleMotorConfig); // 新增配置接口
    server.on("/update_factory", HTTP_GET, handleFactoryUpdate);
    server.on("/update_factory", HTTP_POST, [](){}, handleFactoryUpdateUpload);
    server.on("/update_github", HTTP_GET, handleGitHubUpdate);
    server.on("/toggle_auto_update", HTTP_GET, handleToggleAutoUpdate);
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
        // 連線時通知客戶端當前配置
        if (pMotorConfigCharacteristic) {
            pMotorConfigCharacteristic->setValue((uint8_t*)&motorConfig, sizeof(MotorConfig_t));
            pMotorConfigCharacteristic->notify();
        }
    }
    void onDisconnect(BLEServer* pServer) {
        Serial.println("❌ BLE Client Disconnected. Setting flag to restart...");
        should_restart_advertising = true; 
        vTaskDelay(pdMS_TO_TICKS(5));
    }
};

// --- BLE 特徵寫入回調 (控制) ---
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            String command = String(rxValue.c_str());
            Serial.printf("[BLE CMD] Received RAW: %s\n", command.c_str()); 
            
            int commaIndex = command.indexOf(',');
            if (commaIndex > 0) {
                int rawT = command.substring(0, commaIndex).toInt();
                int rawS = command.substring(commaIndex + 1).toInt();
                
                // 使用 Limit 參數進行約束
                targetSpeedT = constrain(rawT, -motorConfig.pwmEffectiveLimitT, motorConfig.pwmEffectiveLimitT); 
                targetSpeedS = constrain(rawS, -motorConfig.pwmEffectiveLimitS, motorConfig.pwmEffectiveLimitS);
                
                lastControlTime = millis(); 
                Serial.printf("  -> Target Set: T=%d, S=%d\n", targetSpeedT, targetSpeedS);
            }
        }
    }
};

// 處理馬達配置參數
class MotorConfigCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* pCharacteristic) {
        // 客戶端讀取時，將當前 motorConfig 結構體以二進位方式傳輸
        pCharacteristic->setValue((uint8_t*)&motorConfig, sizeof(MotorConfig_t));
        Serial.println("[BLE Config] Motor config read request served.");
    }

    void onWrite(BLECharacteristic* pCharacteristic) {
        // 客戶端寫入時，將二進位資料直接寫入 motorConfig 結構體
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() == sizeof(MotorConfig_t)) {
            memcpy((void*)&motorConfig, rxValue.data(), sizeof(MotorConfig_t));
            saveMotorConfig(); // 儲存到 NVS
            Serial.println("[BLE Config] Motor config updated and saved via BLE.");
        } else {
            Serial.printf("[BLE Config] Write failed: Invalid size (%zu, expected %zu)\n", rxValue.length(), sizeof(MotorConfig_t));
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

// --- setupBleServer_Bluedroid (新增馬達配置特徵) ---
void setupBleServer_Bluedroid() {
    BLEDevice::init(globalHostname.c_str());
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 1. 馬達控制服務
    BLEService *pControlService = pServer->createService(MOTOR_SERVICE_UUID);
    pControlCharacteristic = pControlService->createCharacteristic(
        MOTOR_CONTROL_CHAR_UUID, 
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pControlCharacteristic->addDescriptor(new BLE2902());
    pControlCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // 2. 馬達配置特徵 (在控制服務下)
    pMotorConfigCharacteristic = pControlService->createCharacteristic(
        MOTOR_CONFIG_CHAR_UUID, 
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pMotorConfigCharacteristic->addDescriptor(new BLE2902());
    pMotorConfigCharacteristic->setCallbacks(new MotorConfigCharacteristicCallbacks());


    // 3. Wi-Fi 配置服務
    BLEService *pConfigService = pServer->createService(CONFIG_SERVICE_UUID);
    
    // 從 NVS 讀取目前儲存的 Wi-Fi 資訊，以便客戶端讀取
    preferences.begin("wifi-config", true);
    String current_ssid = preferences.getString("ssid", "");
    String current_pass = preferences.getString("pass", "");
    preferences.end();

    pSsidCharacteristic = pConfigService->createCharacteristic(
        SSID_CHAR_UUID, 
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pSsidCharacteristic->setValue(current_ssid.c_str());
    pSsidCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());

    pPassCharacteristic = pConfigService->createCharacteristic(
        PASS_CHAR_UUID, 
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pPassCharacteristic->setValue(current_pass.c_str());
    pPassCharacteristic->setCallbacks(new ConfigCharacteristicCallbacks());
    
    // 啟動服務
    pConfigService->start();
    pControlService->start(); 

    // 4. 啟動廣告
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID); 
    pAdvertising->addServiceUUID(MOTOR_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    pAdvertising->start(); 

    Serial.println("✅ BLE Advertising Started (Bluedroid).");
}


// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    // 載入馬達配置 (必須在馬達邏輯和 BLE 啟動前完成)
    loadMotorConfig(); 

    // --- 初始化馬達控制腳位 (DRV8833) ---
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
    
    // 初始化 ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // 0 - 3.1V 範圍
    
    // 0. 產生唯一的 Hostname
    generateHostname();
    
    // 1. 啟動 BLE
    setupBleServer_Bluedroid();

    // 2. 嘗試連線 Wi-Fi
    connectToSavedWiFi();
}

// --- Loop ---
void loop() {    
    // 1. 超時安全停止檢查 (分別針對 T 與 S)
    if (targetSpeedT != 0 || targetSpeedS != 0) {
        unsigned long elapsed = millis() - lastControlTime;
        
        // 檢查加速馬達
        if (targetSpeedT != 0 && elapsed > motorConfig.controlTimeoutT) {
            Serial.printf("🚨 加速超時 (T)! 強制歸零 (%u ms)\n", motorConfig.controlTimeoutT);
            targetSpeedT = 0;
        }
        
        // 檢查轉向馬達
        if (targetSpeedS != 0 && elapsed > motorConfig.controlTimeoutS) {
            Serial.printf("🚨 轉向超時 (S)! 強制歸零 (%u ms)\n", motorConfig.controlTimeoutS);
            targetSpeedS = 0;
        }

        if (targetSpeedT == 0 && targetSpeedS == 0 && pControlCharacteristic) {
            pControlCharacteristic->setValue("TIMEOUT:0,0");
            pControlCharacteristic->notify(); 
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
        
        // --- 開機自動更新檢查 ---
        // 稍微延遲一下，確保網路完全穩定
        delay(1000); 
        checkAndPerformAutoUpdate();
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

    // 7. 電壓採樣
    updateBatteryVoltage();
    
    // 保持 loop() 有機會切換任務
    delay(1); 
}
