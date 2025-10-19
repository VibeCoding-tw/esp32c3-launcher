#include <WiFi.h>
#include <WiFiManager.h>      // 使用標準同步 (Blocking) 的 WiFiManager
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

// --- Configuration ---
const int JUMP_DELAY_SECONDS = 5;   // 跳轉到用戶應用程式前的倒數時間
bool isConfigurationMode = false;   // 標記是否處於 Wi-Fi 配置模式

// 使用標準同步的 WiFiManager
WiFiManager wm;

// --- Utility: Print partition info ---
void printPartitionInfo(const esp_partition_t *p, const char *tag) {
    if (p) {
        Serial.printf("[%s] Label: %s | Type: %d | Subtype: %d | Address: 0x%X | Size: %u\n",
                      tag, p->label, p->type, p->subtype, p->address, p->size);
    } else {
        Serial.printf("[%s] No partition found!\n", tag);
    }
}

// --- Find the latest valid OTA app (using esp_image_verify) ---
const esp_partition_t* findLatestUserApp() {
    const esp_partition_t *candidate = nullptr;
    
    // 迭代所有 APP 分區類型
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);

    while (it != NULL) {
        const esp_partition_t *p = esp_partition_get(it);
        
        // 只檢查 OTA 分區 (ota_0 或 ota_1)
        if (p->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 || p->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
            esp_partition_pos_t pos = {
                .offset = p->address,
                .size = p->size
            };
            esp_image_metadata_t data;
            
            // 核心優化點：驗證映像檔是否完整且有效
            if (esp_image_verify(ESP_IMAGE_VERIFY, &pos, &data) == ESP_OK) {
                Serial.printf("Valid OTA app found: %s (Type: %d, Size: %u)\n", p->label, p->subtype, p->size);
                // 找到有效的映像檔後，設為候選者。這裡簡單地取第一個找到的有效 App
                candidate = p;
                break; 
            } else {
                Serial.printf("Invalid OTA app: %s\n", p->label);
            }
        }
        it = esp_partition_next(it);
    }
    
    // 釋放迭代器，防止記憶體洩漏
    if (it != NULL) {
        esp_partition_iterator_release(it);
    }
    
    return candidate;
}

// --- Start latest OTA app if exists ---
void startLatestApp() {
    const esp_partition_t *user_partition = findLatestUserApp();
    if (user_partition) {
        Serial.println("-------------------------------------------------------");
        Serial.printf("Found valid User App in partition: %s\n", user_partition->label);
        Serial.printf("Jumping to User App in %d seconds...\n", JUMP_DELAY_SECONDS);
        Serial.println("-------------------------------------------------------");

        for (int i = 0; i < JUMP_DELAY_SECONDS; i++) {
            Serial.printf("Jumping in %d...\n", JUMP_DELAY_SECONDS - i);
            yield(); 
            delay(1000);
        }

        // 設置下一次啟動的分區
        esp_err_t err = esp_ota_set_boot_partition(user_partition);
        if (err == ESP_OK) {
            Serial.println("Boot partition set successfully. Restarting...");
            delay(500);
            esp_restart();
        } else {
            Serial.printf("Failed to set boot partition (err=%d). Staying in Launcher mode.\n", err);
        }
    } else {
        Serial.println("-------------------------------------------------------");
        Serial.println("No valid User App found. Staying in Launcher mode.");
        Serial.println("-------------------------------------------------------");
    }
}

// --- Connect or start WiFi config portal ---
void connectToWiFi() {
    // 設置配置入口網站的回調函式
    // 注意：標準 WiFiManager 的 autoConnect 是阻塞的。
    wm.setAPCallback([](WiFiManager *wm) {
        isConfigurationMode = true;
        Serial.println("Entering Configuration AP Mode (ESP32-Setup).");
        Serial.println("Connect to AP: " + WiFi.softAPSSID());
        Serial.println("IP Address: " + WiFi.softAPIP().toString());
    });
    
    Serial.println("Starting WiFiManager autoConnect...");

    // autoConnect 只會嘗試連接已儲存的 Wi-Fi 網路。
    // 如果失敗，它會啟動配置門戶，並在整個過程中阻塞 setup()
    if (!wm.autoConnect("ESP32-Setup")) {
        // 如果 autoConnect 返回 false，表示配置門戶已超時或發生其他錯誤
        // 由於我們在 setAPCallback 中設置了 isConfigurationMode，所以這裡主要用於記錄
        if (!isConfigurationMode) {
            Serial.println("AutoConnect failed, setting Configuration Mode flag.");
            isConfigurationMode = true;
        }
    } else {
        // 連線成功
        isConfigurationMode = false;
        Serial.println("WiFi connected successfully!");
    }

    if(WiFi.status() == WL_CONNECTED) {
        Serial.println("Local IP: " + WiFi.localIP().toString());
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();

    Serial.println("\n=======================================================");
    Serial.println("            ESP32 FACTORY LAUNCHER STARTED          ");
    Serial.println("=======================================================");
    printPartitionInfo(running, "Running");
    printPartitionInfo(boot, "Boot");

    // 1. 連線 Wi-Fi (會阻塞直到連線成功或進入配置模式)
    connectToWiFi();

    if (!isConfigurationMode) {
        // 只有在成功連網狀態下才啟動 OTA 和 mDNS
        
        // 2. Setup mDNS and OTA
        String hostname = "esp32c3-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        if (MDNS.begin(hostname.c_str())) {
            Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
        }

        ArduinoOTA.setHostname(hostname.c_str());
        
        // 密碼 'mysecurepassword' 必須與 PlatformIO 的 upload_flags 保持一致
        ArduinoOTA.setPassword("mysecurepassword"); 

        ArduinoOTA.onStart([]() { 
            Serial.println("OTA update started..."); 
        });
        ArduinoOTA.onEnd([]() { Serial.println("\nOTA update completed! Restarting..."); });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("OTA Error [%u]\n", error);
        });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress * 100) / total);
        });
        ArduinoOTA.begin();

        // 3. Attempt jump to User App
        startLatestApp();
    }
}

// --- Loop ---
void loop() {
    // 標準 WiFiManager 配置門戶完成後，程式碼會繼續執行，
    // loop 中只需要處理 OTA 任務即可。
    if (!isConfigurationMode) {
        ArduinoOTA.handle();
    }
    // 確保系統任務得以運行
    yield();
}