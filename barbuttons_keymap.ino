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
const char FIRMWARE_VERSION[] = "1.0.0";

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

// ---------------------------------------------------------------------------
// Keypad state tracking
// ---------------------------------------------------------------------------
int  last_button_state = IDLE;
bool button4_is_held   = false; // true while button '4' is in HOLD (for combo detection)

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
  // the RELEASED handler's exit-config check. See drain below.
  configManager.beginConfigAP();

  // Drain the button-4 RELEASED event that triggered config mode.
  // While status != APP_CONFIG the RELEASED handler will NOT set exit flag.
  buttonManager.drainButton(3000);
  if (DEBUG) Serial.println("Button released, entering config loop");

  // Only NOW switch to config-mode status so the exit check becomes active
  ledManager.setStatus(APP_CONFIG);
  ledManager.resetLedState();

  while (!configManager.isExitRequested()) {
    configManager.handleClient();
    buttonManager.getButton(); // tap of '4' calls keypad_handler → sets exit flag
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
// Key send helpers
// ---------------------------------------------------------------------------
void send_repeating_key(uint8_t key) {
  digitalWrite(LED_PIN, HIGH);
  while (buttonManager.getState() == HOLD) {
    bleManager.write(key);
    delay(ButtonManager::LONG_PRESS_REPEAT_INTERVAL);
    buttonManager.getButton();
  }
  digitalWrite(LED_PIN, LOW);
}

void send_short_press(KeypadEvent key) {
  if (DEBUG) { Serial.print("Short press: "); Serial.println(key); }

  AppStatus status = ledManager.getStatus();
  if (status == APP_CONNECTED || status == APP_CONNECTED_BLINK || status == APP_BT_DISCONNECTED) {
    int idx = ConfigManager::btnIndex(key);
    if (idx >= 0 && configManager.getShortKey(idx) != 0) {
      bleManager.write(configManager.getShortKey(idx));
      ledManager.flashLed(1, 150, 0);
    }
  }
}

void send_long_press(KeypadEvent key) {
  if (DEBUG) { Serial.print("Long press: "); Serial.println(key); }

  AppStatus status = ledManager.getStatus();
  if (status == APP_CONNECTED || status == APP_CONNECTED_BLINK || status == APP_BT_DISCONNECTED) {

    // Button 4 long-press is always the config trigger, regardless of keymap
    if (key == '4') {
      if (buttonManager.waitForButtonHold(ButtonManager::LONG_PRESS_TIME_CONFIG)) {
        start_config_mode();
      }
      return;
    }

    int idx = ConfigManager::btnIndex(key);
    if (idx < 0) return;

    if (configManager.getLongKey(idx) == 0) {
      // Auto-repeat the short key
      send_repeating_key(configManager.getShortKey(idx));
    } else {
      // Send the distinct long-press key once
      bleManager.write(configManager.getLongKey(idx));
      ledManager.flashLed(1, 150, 0);
    }
  }
}

// ---------------------------------------------------------------------------
// Key combo handler — hold '4' then press '3'
// Add specific internal action inside this function.
// ---------------------------------------------------------------------------
void handle_key_combo_4_3() {
  if (DEBUG) Serial.println("Key combo: hold 4 + press 3");
  ledManager.flashLed(3, 100, 50);
}

// ---------------------------------------------------------------------------
// Keypad event handler — registered with ButtonManager; ties all managers
// ---------------------------------------------------------------------------
void keypad_handler(KeypadEvent key) {
  if (DEBUG) Serial.println("keypad_handler");

  AppStatus status = ledManager.getStatus();

  // Use the state of the specific button that triggered this event.
  // getState() always reflects key[0] (the oldest active key), so when button 3
  // is pressed while button 4 is in HOLD, getState() would return HOLD and the
  // PRESSED case — where combo detection lives — would never be reached.
  switch (buttonManager.getButtonState(key)) {

    case PRESSED:
      last_button_state = PRESSED;
      // Detect combo: '3' pressed while '4' is held.
      // The combo takes priority over normal key handling for this press.
      // As long as '4' remains held, pressing '3' again will retrigger the combo.
      if (key == '3' && button4_is_held) {
        handle_key_combo_4_3();
        last_button_state = IDLE; // Prevent normal short press on release
        break;
      }
      if (configManager.isKeyInstant(key) && status != APP_CONFIG) send_short_press(key);
      break;

    case HOLD:
      last_button_state = HOLD;
      if (key == '4') button4_is_held = true;
      send_long_press(key);
      break;

    case RELEASED:
      if (key == '4') button4_is_held = false;
      // Tap of '4' during config mode exits AP mode without saving
      if (status == APP_CONFIG && key == '4') {
        configManager.setExitRequested(true);
        last_button_state = RELEASED;
        break;
      }

      if (last_button_state == PRESSED) {
        if (!(configManager.isKeyInstant(key) && status != APP_CONFIG)) {
          send_short_press(key);
        }
      }
      last_button_state = RELEASED;

      if (status == APP_CONNECTED || status == APP_CONNECTED_BLINK) {
        ledManager.resetLedState();
      }
      bleManager.releaseAll();
      break;

    case IDLE:
      // Only reset button4_is_held when button 4 itself goes idle.
      if (key == '4') button4_is_held = false;

      // Only update global state / LED / BLE when the whole keypad is idle
      // (i.e. key[0] — the oldest tracked button — has also gone idle).
      // This prevents button 3 going idle during a combo from prematurely
      // resetting state while button 4 is still held.
      if (buttonManager.getState() == IDLE) {
        last_button_state = IDLE;

        if (status == APP_CONNECTED || status == APP_CONNECTED_BLINK) {
          ledManager.resetLedState();
        }
        bleManager.releaseAll();
      }
      break;
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
  configManager.loadBleName();

  bleManager.begin(configManager.getBleName());

  // If "Clear BLE Bonds" was requested from the web UI, delete stored bonds
  // now that NimBLE is initialised. The flag was written before the reboot.
  if (configManager.isClearBondsRequested()) {
    BLEManager::clearAllBonds();
    configManager.clearClearBondsFlag();
    if (DEBUG) Serial.println("BLE bonds cleared on request.");
  }

  buttonManager.begin(keypad_handler);

  if (DEBUG) Serial.println("Setup complete.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
  buttonManager.getButton();

  // Track BLE connection state changes
  AppStatus status = ledManager.getStatus();
  if (status == APP_BT_DISCONNECTED && bleManager.isConnected()) {
    ledManager.setStatus(APP_CONNECTED);
    ledManager.resetLedState();
  }
  if (status != APP_BT_DISCONNECTED && status != APP_CONFIG && !bleManager.isConnected()) {
    ledManager.setStatus(APP_BT_DISCONNECTED);
  }

  // Drive LED blink pattern (only when keypad is idle)
  if (buttonManager.getState() == IDLE) {
    ledManager.update();
  }

  delay(10);
}
