# barbuttons-mod

Mod of [BarButtons](https://jaxeadv.com/barbuttons) targeting an **ESP32-C3 Zero** microcontroller.
Two types of changes are covered: firmware and physical casing.

---

## Firmware (`barbuttons_keymap.ino`)

### Overview

Based on the original BarButtons v1 firmware, this version replaces the STA-based OTA workflow with a **self-hosted WiFi Access Point** and a **browser-based configuration interface**.
It also swaps the default ESP32 BLE library for [NimBLE-Arduino (h2zero)](https://github.com/h2zero/NimBLE-Arduino), which persists CCCD notification subscriptions per bonded peer in NVS — meaning keystrokes keep working after a device reset without re-pairing.

### Features

| Feature | Description |
|---|---|
| **AP config mode** | Starts a WiFi AP (`BarButtons-Config` / `barbuttons`) on demand |
| **Web keymap editor** | Browser UI at `http://192.168.4.1` to configure short and long press actions for all 8 buttons |
| **NVS persistence** | Keymap stored in flash via the `Preferences` library; survives reboots and firmware updates |
| **OTA firmware update** | Upload a compiled `.bin` directly from the browser; device reboots automatically |
| **BLE bond management** | "Clear BLE Bonds" button in the web UI; safe to use when the device fails to auto-reconnect |
| **NimBLE BLE keyboard** | HID keyboard over BLE with secure bonding (Secure Connections, Just Works); CCCD state persisted per peer |
| **LED status indicator** | Blink pattern varies by state — see table below |

### LED blink patterns

| State | ON duration | OFF duration |
|---|---|---|
| BT disconnected | 500 ms | 500 ms |
| Config mode | 100 ms | 3000 ms |
| BT connected | LED off (flashes briefly on keypress) | — |

### Entering / exiting config mode

1. **Hold Button 4 for ~5 seconds** (LED starts flashing rapidly).
2. Connect to WiFi SSID **`BarButtons-Config`**, password **`barbuttons`**.
3. Open **`http://192.168.4.1`** in a browser.
4. **Keymap tab** — set Short Press / Long Press action per button, then click **Save & Reboot**.
5. **Firmware tab** — choose a `.bin` and click **Flash Firmware** for OTA update.
6. **BLE Bonds tab** — click **Clear BLE Bonds & Reboot** if the device no longer auto-connects.
7. To exit config mode **without** any changes, tap Button 4 on the device.

> **Note:** Button 4 long-press is reserved as the config trigger and cannot be remapped.

### Button keymap defaults

| Button | Short press | Long press |
|---|---|---|
| 1 | `+` | repeat `+` |
| 2 | `-` | repeat `-` |
| 3 | `n` | `d` |
| 4 | *(reserved — config trigger)* | enter config mode |
| 5 | Up Arrow | repeat Up Arrow |
| 6 | Left Arrow | repeat Left Arrow |
| 7 | Right Arrow | repeat Right Arrow |
| 8 | Down Arrow | repeat Down Arrow |

Long press set to **"Repeat short key"** (value `0`) means the short key auto-repeats while the button is held.
Any other key code sends that key exactly once on long press.

### Pin assignments (ESP32-C3 Zero)

| Signal | GPIO |
|---|---|
| LED | 6 |
| Row 0 | 2 |
| Row 1 | 1 |
| Row 2 | 0 |
| Col 0 | 3 |
| Col 1 | 4 |
| Col 2 | 5 |

### Required libraries

- [Keypad](https://github.com/Chris--A/Keypad)
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- `WiFi`, `WebServer`, `Update`, `Preferences` — bundled with the Arduino ESP32 core

### Build & flash

1. Install the **ESP32** board package in Arduino IDE (target board: `ESP32C3 Dev Module` or the equivalent Zero variant).
2. Set **Tools → Partition Scheme** to:
   > **Minimal SPIFFS (1.9 MB APP with OTA / 190 KB SPIFFS)**

   This is required because the sketch exceeds the default 1.28 MB app partition.
3. Set `const int DEBUG = 0;` before a production flash (saves flash and avoids waiting for USB-CDC on boot).
4. Compile and upload via USB.

---

## Hardware / Casing modifications

The 3-D printed casing was modified from the original BarButtons design as follows:

- **M5 bolts** used instead of the original M4 bolts for the main assembly.
- **Heat-set inserts for M3 bolts** replacing the plain holes for the smaller fasteners.
- **Wemos D1 Mini pocket removed**; the cavity is resized to fit an **ESP32-C3 Zero** (smaller footprint).
- **LED aperture changed to 5 mm × 5 mm** (square) to match the 5 mm LEDs that were available, replacing the original larger round hole.
