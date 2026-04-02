/*
   BarButtons firmware - Custom AP Keymap Configuration version

   Modified from original BarButtons v1 (https://jaxeadv.com/barbuttons).
   Instead of OTA firmware updates over STA, this version:
     - Hosts a WiFi Access Point ("BarButtons-Config")
     - Serves a web page at 192.168.4.1 for configuring the keymap of 8 buttons
     - Persists the keymap to NVS flash via the Preferences library
     - Accepts OTA firmware (.bin) uploads directly from the browser

   HOW TO CONFIGURE / UPDATE:
     1. Hold Button 4 for ~5 seconds until the LED flashes rapidly.
     2. Connect to WiFi: SSID "BarButtons-Config", password "barbuttons".
     3. Open http://192.168.4.1 in a browser.
     4. Keymap: set Short/Long Press actions per button, then "Save & Reboot".
     5. Firmware: choose a .bin file and press "Flash Firmware".
     6. To exit config mode without changes, tap Button 4 on the device.

   IMPORTANT — PARTITION SCHEME:
     In Arduino IDE set Tools > Partition Scheme to:
     "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
     This is required because the sketch exceeds the default 1.28 MB app limit.

   This work is licensed under the Creative Commons Attribution-NonCommercial 4.0
   International License. http://creativecommons.org/licenses/by-nc/4.0/
*/

// ---------------------------------------------------------------------------
// Build flags
// ---------------------------------------------------------------------------
const int DEBUG = 1;  // Set to 1 only when Serial monitor is attached

// ---------------------------------------------------------------------------
// Firmware version — shown in the web config UI
// ---------------------------------------------------------------------------
const char FIRMWARE_VERSION[] = "1.0.1";

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
      // Button 4: config trigger — non-repeating, 5-second total hold threshold
      buttonManager.setButtonRepeating(btn, false);
      buttonManager.setButtonLongPressTime(btn, ButtonManager::LONG_PRESS_CONFIG_TIME);
    } else {
      // Repeat mode when no distinct long-press action is configured (getLongKey == 0)
      buttonManager.setButtonRepeating(btn, configManager.getLongKey(i) == 0);
    }
  }
}

// ---------------------------------------------------------------------------
// Config mode — starts the AP, runs the client loop, then restores BLE
// ---------------------------------------------------------------------------
void start_config_mode() {
  if (DEBUG) Serial.println("Entering config AP mode");

  // On ESP32-C3 the radio is shared; stop BLE before starting WiFi AP
  bleManager.end();
  delay(100);

  // Start AP and register web routes (also flashes the LED 5 times)
  // NOTE: ledManager status is intentionally NOT yet APP_CONFIG here.
  // Setting it before the button is released would immediately trigger
  // the on_short_press config-exit check. See drainButton below.
  configManager.beginConfigAP();

  // Drain the button-4 RELEASED event that triggered config mode.
  // While status != APP_CONFIG the on_short_press handler will NOT set the exit flag.
  buttonManager.drainButton(3000);
  if (DEBUG) Serial.println("Button released, entering config loop");

  // Only NOW switch to config-mode status so the exit check becomes active
  ledManager.setStatus(APP_CONFIG);
  ledManager.resetLedState();

  while (!configManager.isExitRequested()) {
    configManager.handleClient();
    buttonManager.update(); // tap of '4' calls on_short_press → sets exit flag
    ledManager.update();
    delay(5);
  }

  configManager.endConfigAP();

  // Restart BLE with the (possibly updated) device name
  bleManager.begin(configManager.getBleName());
  ledManager.setStatus(APP_BT_DISCONNECTED);
  ledManager.resetLedState();

  if (DEBUG) Serial.println("Config mode exited, BLE restarting");
}

// ---------------------------------------------------------------------------
// High-level button event callbacks — registered with ButtonManager
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

    if (DEBUG) { Serial.print("Short press: "); Serial.print(btn); Serial.printf(" -> key=0x%02X (%d)\n", shortKey, shortKey); }
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
  if (DEBUG) Serial.printf("Long press: %c -> key=0x%02X (%d)\n", btn, longKey, longKey);
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
      if (DEBUG) Serial.printf("Key combo: hold 4 + press %c -> keymap %d\n", pressed, newKeymap);
      configManager.setActiveKeymap(newKeymap);
      applyKeymap();
      // Flash N times (1/2/3) to acknowledge which keymap is now active
      ledManager.flashLed(newKeymap, 150, 100);
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
  if (DEBUG) {
    Serial.begin(9600);
    delay(2000); // Wait for USB-CDC to re-enumerate after reset
  }

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
    if (DEBUG) Serial.println("BLE bonds cleared on request.");
  }

  // Initialise ButtonManager and apply the active keymap's per-button configuration
  buttonManager.begin();
  applyKeymap();
  // Flash N times to indicate which keymap is active on boot (same pattern as keymap switching)
  ledManager.flashLed(configManager.getActiveKeymap(), 150, 100);

  buttonManager.setShortPressHandler(on_short_press);
  buttonManager.setLongPressHandler(on_long_press);
  buttonManager.setComboHandler(on_combo);

  if (DEBUG) Serial.println("Setup complete.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
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

  delay(10);
}
