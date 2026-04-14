/*
   Remote Buttons firmware - Custom AP Keymap Configuration version

   Modified from original BarButtons v1 (https://jaxeadv.com/barbuttons).
   Instead of OTA firmware updates over STA, this version:
     - Hosts a WiFi Access Point ("RemoteButtons-Config")
     - Serves a web page at 192.168.4.1 for configuring the keymap of 8 buttons
     - Persists the keymap to NVS flash via the NVS API
     - Accepts OTA firmware (.bin) uploads directly from the browser

   HOW TO CONFIGURE / UPDATE:
     1. Hold Button 4 for ~5 seconds until the LED flashes rapidly.
     2. Connect to WiFi: SSID "RemoteButtons-Config", password "remotebuttons".
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
extern const int DEBUG = 1;       // Set to 1 only when Serial monitor is attached
extern const bool LEGACY = false; // Legacy has a different pin layout and no battery support

// ---------------------------------------------------------------------------
// Firmware version -- shown in the web config UI
// ---------------------------------------------------------------------------
const char FIRMWARE_VERSION[] = "1.4.0";

// ---------------------------------------------------------------------------
// Manager includes
// ---------------------------------------------------------------------------
#include "HardwareConfig.h"
#include "StatusLedManager.h"
#include "ble/BLEManager.h"
#include "config/ConfigManager.h"
#include "buttons/ButtonManager.h"
#include "BatteryManager.h"
#include "main.h"

// ---------------------------------------------------------------------------
// Global manager instances
// ---------------------------------------------------------------------------
StatusLedManager ledManager;
BLEManager bleManager("Drakarah", 100);
ConfigManager configManager;
ButtonManager buttonManager;
BatteryManager batteryManager;

std::string currentOutputTarget = "";

// Apply the currently active keymap to ButtonManager.
// Call once in setup() and again whenever the active keymap changes.
void applyKeymap()
{
  for (int i = 0; i < 8; i++)
  {
    char btn = '1' + i;
    if (i == 3)
    {
      // Button 4: config trigger -- non-repeating, 5-second total hold threshold
      buttonManager.setButtonRepeating(btn, false);
      buttonManager.setButtonLongPressTime(btn, ButtonManager::LONG_PRESS_CONFIG_TIME);
    }
    else
    {
      // BT Home targets don't use key-repeat (they fire a broadcast on every
      // press event; repeating doesn't make sense for those).
      bool hasBTHome = (configManager.getShortEntry(i).target == TARGET_BTHOME ||
                        configManager.getLongEntry(i).target == TARGET_BTHOME);
      // Repeat mode when no distinct long-press action is configured (key == 0)
      // and no BT Home target is involved.
      buttonManager.setButtonRepeating(btn, !hasBTHome && configManager.getLongEntry(i).key == 0);
    }
  }
}

// ---------------------------------------------------------------------------
// Config mode -- starts the AP, runs the client loop, then restores BLE
// ---------------------------------------------------------------------------
void start_config_mode()
{
  if (DEBUG)
    printf("[MAIN] Entering config AP mode\n");

  // Collect the bond list while NimBLE is still active so the web UI can
  // offer each known peer as an HID target option.
  const auto bondList = bleManager.getBondedAddresses();

  // On ESP32-C3 the radio is shared; stop BLE before starting WiFi AP
  bleManager.end();
  vTaskDelay(pdMS_TO_TICKS(100));

  // Start AP and register web routes (also flashes the LED 5 times)
  // NOTE: ledManager status is intentionally NOT yet APP_CONFIG here.
  // Setting it before the button is released would immediately trigger
  // the on_short_press config-exit check. See drainButton below.
  {
    const bool batAvail = (!LEGACY && configManager.isBatteryEnabled());
    const int batMv  = batAvail ? batteryManager.getLastVoltageMv() : -1;
    const int batPct = batAvail ? batteryManager.getLastPercent()   : -1;
    configManager.beginConfigAP(bondList, batMv, batPct);
  }

  // Drain the button-4 RELEASED event that triggered config mode.
  // While status != APP_CONFIG the on_short_press handler will NOT set the exit flag.
  buttonManager.drainButton(3000);
  if (DEBUG)
    printf("[MAIN] Button released, entering config loop\n");

  // Only NOW switch to config-mode status so the exit check becomes active
  ledManager.setStatus(APP_CONFIG);
  ledManager.resetLedState();

  // esp_http_server handles requests in its own task; this loop just drives
  // the LED blink and watches for a Button-4 tap to exit config mode.
  while (!configManager.isExitRequested())
  {
    configManager.handleClient(); // no-op with esp_http_server
    buttonManager.update();       // tap of '4' calls on_short_press -> sets exit flag
    ledManager.update();
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  configManager.endConfigAP();

  // Restart BLE with the (possibly updated) device name
  bleManager.begin(configManager.getBleName(), configManager.allowBLEPowerSaving(), configManager.getMaxBLEConnections());
  ledManager.setStatus(APP_BT_DISCONNECTED);
  ledManager.resetLedState();

  if (DEBUG)
    printf("[MAIN] Config mode exited, BLE restarting\n");
}

std::string getCurrentOutputTarget()
{
  const std::vector<std::string> connections = bleManager.getConnections();
  const auto it = std::find(connections.begin(), connections.end(), currentOutputTarget);

  // If current target no longer exists → go back to broadcast
  if (it == connections.end())
  {
    currentOutputTarget = "";
    ledManager.flashLed(1, 1000, 100);
  }
  return currentOutputTarget;
}

void toggleOutputTarget()
{
  const std::vector<std::string> connections = bleManager.getConnections();
  if (connections.empty())
  {
    currentOutputTarget = "";
    ledManager.flashLed(1, 1000, 100);
    return;
  }
  else if (connections.size() == 1)
  {
    if (DEBUG)
      printf("[MAIN] Only one connection, staying on BROADCAST\n");
    currentOutputTarget = "";
    ledManager.flashLed(1, 1000, 100);
    return;
  }

  // If currently broadcasting → go to first device
  if (currentOutputTarget.empty())
  {
    currentOutputTarget = connections[0];
    ledManager.flashLed(1, 150, 100);
  }
  else
  {
    auto it = std::find(connections.begin(), connections.end(), currentOutputTarget);

    // If current target no longer exists → go back to broadcast
    if (it == connections.end())
    {
      currentOutputTarget = "";
      ledManager.flashLed(1, 1000, 100);
    }
    else
    {
      size_t index = std::distance(connections.begin(), it);
      index++;

      if (index >= connections.size())
      {
        // Wrap back to broadcast (empty string)
        currentOutputTarget = "";
        ledManager.flashLed(1, 1000, 100);
      }
      else
      {
        currentOutputTarget = connections[index];
        ledManager.flashLed(index + 1, 150, 100);
      }
    }
  }
  if (DEBUG)
    printf("[MAIN] Output target set to: %s\n", currentOutputTarget == "" ? "BROADCAST" : currentOutputTarget.c_str());
}

// ---------------------------------------------------------------------------
// High-level button event callbacks -- registered with ButtonManager
// ---------------------------------------------------------------------------

// Short press: tap (non-repeating buttons), or repeated fire (repeating buttons)
void on_short_press(char btn)
{
  const AppStatus status = ledManager.getStatus();

  // In config mode a tap of button 4 exits AP mode without saving
  if (status == APP_CONFIG)
  {
    if (btn == '4')
      configManager.setExitRequested(true);
    return;
  }

  const int idx = Config::btnIndex(btn);
  if (idx < 0)
    return;

  const auto &entry = configManager.getShortEntry(idx);

  // BT Home broadcast target: send a BTHome advertisement and return.
  if (entry.target == TARGET_BTHOME)
  {
    if (DEBUG)
      printf("[MAIN] Short press: %c -> BTHome broadcast\n", btn);
    bleManager.getAdvertisingManager().broadcastBTHomeButtonPress(
        BLEAdvertisingManager::BTHOME_BUTTON_PRESS, idx + 1);
    ledManager.flashLed(1, 150, 0);
    return;
  }

  if (entry.key == 0)
    return;

  // Determine HID target: fixed MAC (TARGET_HID) or runtime selector (TARGET_SELECT).
  std::string target;
  if (entry.target == TARGET_HID)
    target = entry.mac; // empty string = broadcast to all
  else
    target = getCurrentOutputTarget(); // TARGET_SELECT

  if (DEBUG)
    printf("[MAIN] Short press: %c -> key=0x%02X (%d) to target %s\n",
           btn, entry.key, entry.key, target.empty() ? "BROADCAST" : target.c_str());

  if (bleManager.isConnected())
  {
    bleManager.write(target, entry.key);
    ledManager.flashLed(1, 150, 0);
  }
}

// Long press: button held beyond its long-press threshold
void on_long_press(char btn)
{
  // Button 4 long-press always enters config mode, regardless of keymap
  if (btn == '4')
  {
    start_config_mode();
    return;
  }

  const int idx = Config::btnIndex(btn);
  if (idx < 0)
    return;

  const auto &entry = configManager.getLongEntry(idx);

  // BT Home broadcast target: send a BTHome long-press advertisement and return.
  if (entry.target == TARGET_BTHOME)
  {
    if (DEBUG)
      printf("[MAIN] Long press: %c -> BTHome broadcast\n", btn);
    bleManager.getAdvertisingManager().broadcastBTHomeButtonPress(
        BLEAdvertisingManager::BTHOME_BUTTON_LONG_PRESS, idx + 1);
    ledManager.flashLed(1, 150, 0);
    return;
  }

  // Determine HID target: fixed MAC (TARGET_HID) or runtime selector (TARGET_SELECT).
  std::string target;
  if (entry.target == TARGET_HID)
    target = entry.mac; // empty string = broadcast to all
  else
    target = getCurrentOutputTarget(); // TARGET_SELECT

  if (DEBUG)
    printf("[MAIN] Long press: %c -> key=0x%02X (%d) to target %s\n",
           btn, entry.key, entry.key, target.empty() ? "BROADCAST" : target.c_str());

  if (bleManager.isConnected())
  {
    if (entry.key != 0)
    {
      bleManager.write(target, entry.key);
      ledManager.flashLed(1, 150, 0);
    }
  }
}

// Combo: 'pressed' was pressed while 'held' was already active
void on_combo(char held, char pressed)
{
  if (DEBUG)
    printf("[MAIN] Key combo: hold %c + press %c\n", held, pressed);

  if (held == '4')
  {
    if (pressed >= '1' && pressed <= '3')
      toggleKeymap(pressed);
    else if (pressed == '5')
      toggleOutputTarget();
    else if (pressed == '6')
      bleManager.getAdvertisingManager().startCycle();
    return;
  }

  // Look up configured combos for the active keymap
  const int keymapIdx = configManager.getActiveKeymap() - 1;
  const int count     = configManager.getComboCount(keymapIdx);
  for (int j = 0; j < count; j++)
  {
    const ComboEntry &combo = configManager.getComboEntry(keymapIdx, j);
    if (combo.held != held || combo.pressed != pressed)
      continue;

    // BT Home target
    if (combo.target == TARGET_BTHOME)
    {
      if (DEBUG)
        printf("[MAIN] Combo: hold %c + press %c -> BTHome\n", held, pressed);
      bleManager.getAdvertisingManager().broadcastBTHomeButtonPress(
          BLEAdvertisingManager::BTHOME_BUTTON_PRESS, Config::btnIndex(pressed) + 1);
      ledManager.flashLed(1, 150, 0);
      return;
    }

    if (combo.key == 0)
      return;

    std::string target;
    if (combo.target == TARGET_HID)
      target = combo.mac;
    else
      target = getCurrentOutputTarget();

    if (DEBUG)
      printf("[MAIN] Combo: hold %c + press %c -> key=0x%02X to %s\n",
             held, pressed, combo.key, target.empty() ? "BROADCAST" : target.c_str());

    if (bleManager.isConnected())
    {
      bleManager.write(target, combo.key);
      ledManager.flashLed(1, 150, 0);
    }
    return;
  }
}

void toggleKeymap(char pressed)
{
  int newKeymap = -1;
  if (pressed == '1')
    newKeymap = 1;
  else if (pressed == '2')
    newKeymap = 2;
  else if (pressed == '3')
    newKeymap = 3;

  if (newKeymap > 0)
  {
    if (DEBUG)
      printf("[MAIN] Key combo: hold 4 + press %c -> keymap %d\n", pressed, newKeymap);
    configManager.setActiveKeymap(newKeymap);
    applyKeymap();
    ledManager.flashLed(newKeymap, 150, 100);
  }
}

// Battery reading event — fired by BatteryManager only when the percentage changes.
void on_battery_updated(uint8_t percent)
{
  if (DEBUG)
    printf("[MAIN] Battery: %d%%\n", percent);
  bleManager.setBatteryLevel(percent);
}

// ---------------------------------------------------------------------------
// ESP-IDF entry point
// ---------------------------------------------------------------------------
extern "C" void app_main()
{
  if (DEBUG)
    vTaskDelay(pdMS_TO_TICKS(1000)); // time to open Serial monitor after reset

  // NVS must be initialised before any NVS or NimBLE calls.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    nvs_flash_erase();
    nvs_flash_init();
  }

  // TCP/IP stack and default event loop are required for WiFi AP (config mode).
  esp_netif_init();
  esp_event_loop_create_default();

  configManager.begin(FIRMWARE_VERSION);
  configManager.loadConfig();

  bleManager.begin(configManager.getBleName(), configManager.allowBLEPowerSaving(), configManager.getMaxBLEConnections());

  ledManager.begin(getLEDPin(LEGACY));

  // If "Clear BLE Bonds" was requested from the web UI, delete stored bonds
  // now that NimBLE is initialised. The flag was written before the reboot.
  if (configManager.isClearBondsRequested())
  {
    BLEManager::clearAllBonds();
    configManager.clearClearBondsFlag();

    if (DEBUG)
      printf("[MAIN] BLE bonds cleared on request.\n");
  }

  buttonManager.setPinConfiguration(getKeypadRowPins(LEGACY),
                                    getKeypadColPins(LEGACY));
  buttonManager.begin();

  applyKeymap();
  // Flash N times to indicate which keymap is active on boot
  ledManager.flashLed(configManager.getActiveKeymap(), 150, 100);

  buttonManager.setShortPressHandler(on_short_press);
  buttonManager.setLongPressHandler(on_long_press);
  buttonManager.setComboHandler(on_combo);

  if (!LEGACY && configManager.isBatteryEnabled())
  {
    batteryManager.begin(ADC_BATTERY_CHANNEL);
    batteryManager.setBatteryReadingHandler(on_battery_updated);
  }

  if (!DEBUG)
  {
    // Enable automatic light sleep when the CPU is idle.
    // Requires CONFIG_PM_ENABLE=y and CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
    // in sdkconfig.defaults (already set).
    const esp_pm_config_t pm_config = {
        .max_freq_mhz = 160, // reduce from 160 to save power; BLE and WiFi still work fine at 80 MHz
        .min_freq_mhz = 40,  // lowest valid ESP32-C3 frequency that keeps all peripherals stable
        .light_sleep_enable = true,
    };
    if (esp_pm_configure(&pm_config) != ESP_OK)
      printf("[MAIN] Warning: power management configuration failed\n");
  }
  else
  {
    printf("[MAIN] Light sleep not enabled in DEBUG mode. If you want to debug light sleep issues change this because serial is not reliable in light sleep.\n");
  }

  if (DEBUG)
    printf("[MAIN] Setup complete.\n");

  const bool batteryEnabled = !LEGACY && configManager.isBatteryEnabled();

  // Main loop
  while (true)
  {
    buttonManager.update();

    if (batteryEnabled)
      batteryManager.update();

    // Track BLE connection state changes
    const AppStatus status = ledManager.getStatus();
    if (status == APP_CONFIG)
    {
    }
    else
    {
      if (bleManager.isConnected())
      {
        if (bleManager.getAdvertisingManager().isAdvertising() && status != APP_BT_CONNECTED_ADVERTISING)
        {
          ledManager.setStatus(APP_BT_CONNECTED_ADVERTISING);
          ledManager.resetLedState();
        }
        else if (!bleManager.getAdvertisingManager().isAdvertising() && status != APP_CONNECTED)
        {
          ledManager.setStatus(APP_CONNECTED);
          ledManager.resetLedState();
        }
      }
      else
      {
        if (status != APP_BT_DISCONNECTED && status != APP_CONFIG)
        {
          ledManager.setStatus(APP_BT_DISCONNECTED);
          ledManager.resetLedState();
        }
      }
    }

    // Drive LED blink pattern and any pending flash animation
    ledManager.update();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
