/*
   BarButtons firmware - Custom AP Keymap Configuration version

   Modified from original BarButtons v1 (https://jaxeadv.com/barbuttons).
   Instead of OTA firmware updates over STA, this version:
     - Hosts a WiFi Access Point ("BarButtons-Config")
     - Serves a web page at 192.168.4.1 for configuring the keymap of 8 buttons
     - Persists the keymap to NVS flash via the NVS API
     - Accepts OTA firmware (.bin) uploads directly from the browser

   HOW TO CONFIGURE / UPDATE:
     1. Hold Button 4 for ~5 seconds until the LED flashes rapidly.
     2. Connect to WiFi: SSID "BarButtons-Config", password "barbuttons".
     3. Open http://192.168.4.1 in a browser.
     4. Keymap: set Short/Long Press actions per button, then "Save & Reboot".
     5. Firmware: choose a .bin file and press "Flash Firmware".
     6. To exit config mode without changes, tap Button 4 on the device.

   This work is licensed under the Creative Commons Attribution-NonCommercial 4.0
   International License. http://creativecommons.org/licenses/by-nc/4.0/
*/

#include <cstdio>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_pm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ---------------------------------------------------------------------------
// Build flags
// ---------------------------------------------------------------------------
const int DEBUG = 1;  // Set to 1 only when Serial monitor is attached

// ---------------------------------------------------------------------------
// Firmware version -- shown in the web config UI
// ---------------------------------------------------------------------------
const char FIRMWARE_VERSION[] = "1.1.0";

// ---------------------------------------------------------------------------
// Manager includes
// ---------------------------------------------------------------------------
#include "HardwareConfig.h"
#include "StatusLedManager.h"
#include "BLEManager.h"
#include "ConfigManager.h"
#include "ButtonManager.h"

// ---------------------------------------------------------------------------
// Global manager instances
// ---------------------------------------------------------------------------
StatusLedManager ledManager;
BLEManager       bleManager("JaxeADV", 100);
ConfigManager    configManager;
ButtonManager    buttonManager;

// Apply the currently active keymap to ButtonManager.
// Call once in setup() and again whenever the active keymap changes.
void applyKeymap() {
  for (int i = 0; i < 8; i++) {
    char btn = '1' + i;
    if (i == 3) {
      // Button 4: config trigger -- non-repeating, 5-second total hold threshold
      buttonManager.setButtonRepeating(btn, false);
      buttonManager.setButtonLongPressTime(btn, ButtonManager::LONG_PRESS_CONFIG_TIME);
    } else {
      // Repeat mode when no distinct long-press action is configured (getLongKey == 0)
      buttonManager.setButtonRepeating(btn, configManager.getLongKey(i) == 0);
    }
  }
}

// ---------------------------------------------------------------------------
// Config mode -- starts the AP, runs the client loop, then restores BLE
// ---------------------------------------------------------------------------
void start_config_mode() {
  if (DEBUG) printf("Entering config AP mode\n");

  // On ESP32-C3 the radio is shared; stop BLE before starting WiFi AP
  bleManager.end();
  vTaskDelay(pdMS_TO_TICKS(100));

  // Start AP and register web routes (also flashes the LED 5 times)
  // NOTE: ledManager status is intentionally NOT yet APP_CONFIG here.
  // Setting it before the button is released would immediately trigger
  // the on_short_press config-exit check. See drainButton below.
  configManager.beginConfigAP();

  // Drain the button-4 RELEASED event that triggered config mode.
  // While status != APP_CONFIG the on_short_press handler will NOT set the exit flag.
  buttonManager.drainButton(3000);
  if (DEBUG) printf("Button released, entering config loop\n");

  // Only NOW switch to config-mode status so the exit check becomes active
  ledManager.setStatus(APP_CONFIG);
  ledManager.resetLedState();

  // esp_http_server handles requests in its own task; this loop just drives
  // the LED blink and watches for a Button-4 tap to exit config mode.
  while (!configManager.isExitRequested()) {
    configManager.handleClient(); // no-op with esp_http_server
    buttonManager.update();       // tap of '4' calls on_short_press -> sets exit flag
    ledManager.update();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  configManager.endConfigAP();

  // Restart BLE with the (possibly updated) device name
  bleManager.begin(configManager.getBleName());
  ledManager.setStatus(APP_BT_DISCONNECTED);
  ledManager.resetLedState();

  if (DEBUG) printf("Config mode exited, BLE restarting\n");
}

// ---------------------------------------------------------------------------
// High-level button event callbacks -- registered with ButtonManager
// ---------------------------------------------------------------------------

// Short press: tap (non-repeating buttons), or repeated fire (repeating buttons)
void on_short_press(char btn) {
  AppStatus status = ledManager.getStatus();

  // In config mode a tap of button 4 exits AP mode without saving
  if (status == APP_CONFIG) {
    if (btn == '4') configManager.setExitRequested(true);
    return;
  }

  if (status == APP_CONNECTED || status == APP_CONNECTED_BLINK || status == APP_BT_DISCONNECTED) {
    int idx = ConfigManager::btnIndex(btn);
    if (idx < 0) return;

    uint8_t shortKey = configManager.getShortKey(idx);
    if (shortKey == 0) return;

    if (DEBUG) printf("Short press: %c -> key=0x%02X (%d)\n", btn, shortKey, shortKey);
    bleManager.write(shortKey);
    ledManager.flashLed(1, 150, 0);
  }
}

// Long press: button held beyond its long-press threshold
void on_long_press(char btn) {
  AppStatus status = ledManager.getStatus();
  if (status != APP_CONNECTED && status != APP_CONNECTED_BLINK && status != APP_BT_DISCONNECTED) return;

  // Button 4 long-press always enters config mode, regardless of keymap
  if (btn == '4') {
    start_config_mode();
    return;
  }

  int idx = ConfigManager::btnIndex(btn);
  if (idx < 0) return;

  uint8_t longKey = configManager.getLongKey(idx);
  if (DEBUG) printf("Long press: %c -> key=0x%02X (%d)\n", btn, longKey, longKey);
  if (longKey != 0) {
    bleManager.write(longKey);
    ledManager.flashLed(1, 150, 0);
  }
}

// Combo: 'pressed' was pressed while 'held' was already active
void on_combo(char held, char pressed) {
  if (held == '4') {
    int newKeymap = -1;
    if      (pressed == '1') newKeymap = 1;
    else if (pressed == '2') newKeymap = 2;
    else if (pressed == '3') newKeymap = 3;

    if (newKeymap > 0) {
      if (DEBUG) printf("Key combo: hold 4 + press %c -> keymap %d\n", pressed, newKeymap);
      configManager.setActiveKeymap(newKeymap);
      applyKeymap();
      ledManager.flashLed(newKeymap, 150, 100);
    }
  }
}

// ---------------------------------------------------------------------------
// ESP-IDF entry point
// ---------------------------------------------------------------------------
extern "C" void app_main() {
  // NVS must be initialised before any NVS or NimBLE calls.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // TCP/IP stack and default event loop are required for WiFi AP (config mode).
  esp_netif_init();
  esp_event_loop_create_default();

  ledManager.begin(LED_PIN);

  configManager.begin(&ledManager, FIRMWARE_VERSION);
  configManager.loadKeymap();
  configManager.loadActiveKeymap();
  configManager.loadBleName();

  bleManager.begin(configManager.getBleName());

  // If "Clear BLE Bonds" was requested from the web UI, delete stored bonds
  // now that NimBLE is initialised. The flag was written before the reboot.
  if (configManager.isClearBondsRequested()) {
    BLEManager::clearAllBonds();
    configManager.clearClearBondsFlag();
    if (DEBUG) printf("BLE bonds cleared on request.\n");
  }

  buttonManager.begin();
  applyKeymap();
  // Flash N times to indicate which keymap is active on boot
  ledManager.flashLed(configManager.getActiveKeymap(), 150, 100);

  buttonManager.setShortPressHandler(on_short_press);
  buttonManager.setLongPressHandler(on_long_press);
  buttonManager.setComboHandler(on_combo);

  // Enable automatic light sleep when the CPU is idle.
  // Requires CONFIG_PM_ENABLE=y and CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
  // in sdkconfig.defaults (already set).
  esp_pm_config_t pm_config = {
    .max_freq_mhz      = 160,
    .min_freq_mhz      = 40,   // lowest valid ESP32-C3 frequency that keeps all peripherals stable
    .light_sleep_enable = true,
  };
  esp_err_t pm_err = esp_pm_configure(&pm_config);
  if (DEBUG) {
    if (pm_err == ESP_OK) printf("Light sleep enabled.\n");
    else printf("esp_pm_configure failed: %s\n", esp_err_to_name(pm_err));
  }

  if (DEBUG) printf("Setup complete.\n");

  // Main loop
  while (true) {
    buttonManager.update();

    // Track BLE connection state changes
    AppStatus status = ledManager.getStatus();
    if (status == APP_BT_DISCONNECTED && bleManager.isConnected()) {
      ledManager.setStatus(APP_CONNECTED);
      ledManager.resetLedState();
    }
    if (status != APP_BT_DISCONNECTED && status != APP_CONFIG && !bleManager.isConnected()) {
      ledManager.setStatus(APP_BT_DISCONNECTED);
    }

    // Drive LED blink pattern and any pending flash animation
    ledManager.update();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
