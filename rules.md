# 🔄 核心流程（關鍵）

把 `ota_0` 當成「**暫存區 + 執行區**」，所有更新都先走這裡。

## 🧩 A. 更新「官方（factory）」的安全流程

### Step 1：下載新 firmware → `ota_0`

```c
esp_https_ota(...);   // 寫入 ota_0
```

### Step 2：切換開機 → `ota_0`

```c
esp_ota_set_boot_partition(ota_0);
esp_restart();
```

### Step 3：在 `ota_0` 裡驗證

* 跑幾秒（確保不會 crash）
* 檢查版本 / SHA256

### Step 4：**再寫回 factory（關鍵）**

```c
const esp_partition_t *running = esp_ota_get_running_partition();
const esp_partition_t *factory = esp_partition_find_first(
    ESP_PARTITION_TYPE_APP,
    ESP_PARTITION_SUBTYPE_APP_FACTORY,
    NULL
);

// erase
esp_partition_erase_range(factory, 0, factory->size);

// copy（建議分段讀寫，不要直接用指標）
uint8_t buf[4096];
for (int offset = 0; offset < running->size; offset += sizeof(buf)) {
    esp_partition_read(running, offset, buf, sizeof(buf));
    esp_partition_write(factory, offset, buf, sizeof(buf));
}
```

### Step 5：標記成功（NVS flag）

```c
factory_updated = true;
```

---

## 🧩 B. 學生 OTA（正常用）

* 直接 OTA → `ota_0`
* 設為 boot
* 不回寫 factory

---

# 🔘 GPIO1（原本設計）—救命鍵

```c
if (gpio_get_level(GPIO_NUM_1) == 0) {
    esp_ota_set_boot_partition(factory);
    esp_restart();
}
```

👉 無論學生怎麼搞，都能回官方

---

# 🚨 風險與一定要補的機制

## ❗1. 沒有 A/B → 需要「軟 rollback」

只有一個 OTA slot，所以要自己做：

```c
if (!boot_success) {
    esp_ota_set_boot_partition(factory);
}
```

👉 第一次開機沒成功 → 自動回 factory

---

## ❗2. 寫 factory 時最危險

👉 這一步斷電 = 磚

**建議加：**

* 電壓檢測（或至少避免低電量時更新）
* 只在「穩定運行 N 秒後」才開始 copy
* 分段寫入 + 校驗

---

## ❗3. 一定要驗證 firmware

建議用：

* SHA256（ESP-IDF 有）
* 或 version + size + checksum

---

## ❗4. 不要每次都寫 factory

用 NVS 控制：

```c
if (need_update_factory) {
    copy_to_factory();
}
```

---

# 🧠 GitHub Actions 在這裡的角色

搭配 GitHub：

* push → 自動 build firmware.bin
* release → 提供 OTA URL
* ESP32 → 下載到 `ota_0`

👉 「借用 ota_0 更新 factory」完全成立

---

# 🎯 最終評估（你的方案）

| 項目   | 評價         |
| ---- | ---------- |
| 空間利用 | ✅ 很好       |
| 架構可行 | ✅ 可行       |
| 安全性  | ⚠️ 中（需自己補） |
| 維護性  | ⚠️ 較高複雜度   |

---

# 🏁 結論（最實際建議）

👉 方案可以做，而且是很多**低 Flash 專案會用的模式**

但請記住三件事：

1. ✅ **OTA 一律先寫 ota_0**
2. ✅ **確認 OK 才 copy 到 factory**
3. ⚠️ **copy factory 是最危險步驟，一定要保護**

---

# 🚀 補齊「產品級穩定度」

* 🔧 完整 partition + config
* 🔐 OTA 驗證（SHA256）
* 🔄 自動 rollback 機制
* 💾 NVS 狀態管理（update / success / fail）
* ⚡ 安全 copy-to-factory 模組（可直接用）
