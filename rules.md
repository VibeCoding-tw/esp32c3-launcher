# ESP32-C3 Firmware Management Rules

## Partition Strategy
- **Factory Partition (工廠出貨區)**: 
    - Purpose: Stable "Safe Zone" firmware.
    - Update Method: **Strictly USB only** (PlatformIO Upload).
    - Recovery: Fallback target when student code fails.
- **OTA_1 Partition (學生練習區)**:
    - Purpose: Flexible zone for student customization and experimentation.
    - Update Method: **OTA (Github Cloud Update)**.
    - Behavior: The firmware running in the Factory zone should download updates into this area.

## Hardware Recovery Mechanism
- **GPIO 1 Fallback**: If the firmware in the practice area fails to boot, triggering GPIO 1 should force the device back into the Factory partition.

## Development Workflow
1. Version bumps are tracked per partition.
2. Firmware in the Factory zone should **NEVER** attempt to overwrite the Factory partition via OTA to prevent "self-killing" logic locks.
3. All cloud updates from the Factory interface will be directed to the practice partition.
