#pragma once

#include <cstdio>
#include <cstring>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include "HIDTypes.h"
#include "KeyCodes.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern const int DEBUG;

// ---------------------------------------------------------------------------
// Combined HID report descriptor:
//   Report ID 1 — standard boot-compatible keyboard (8-byte report)
//   Report ID 2 — Consumer Control (2-byte usage code for media keys)
// ---------------------------------------------------------------------------
static const uint8_t _hidReportDesc[] = {
  // --- Keyboard (Report ID 1) ---
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x06,        // Usage (Keyboard)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        //   Report ID (1)
  0x05, 0x07,        //   Usage Page (Key Codes)
  0x19, 0xE0,        //   Usage Minimum (Left Control)
  0x29, 0xE7,        //   Usage Maximum (Right GUI)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x75, 0x01,        //   Report Size (1)
  0x95, 0x08,        //   Report Count (8)
  0x81, 0x02,        //   Input (Data, Var, Abs)    — modifier byte
  0x95, 0x01,        //   Report Count (1)
  0x75, 0x08,        //   Report Size (8)
  0x81, 0x01,        //   Input (Const)             — reserved byte
  0x95, 0x06,        //   Report Count (6)
  0x75, 0x08,        //   Report Size (8)
  0x25, 0x65,        //   Logical Maximum (101)
  0x05, 0x07,        //   Usage Page (Key Codes)
  0x19, 0x00,        //   Usage Minimum (0)
  0x29, 0x65,        //   Usage Maximum (101)
  0x81, 0x00,        //   Input (Data, Array, Abs)  — key slots
  0xC0,              // End Collection
  // --- Consumer Control (Report ID 2) ---
  0x05, 0x0C,        // Usage Page (Consumer)
  0x09, 0x01,        // Usage (Consumer Control)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x02,        //   Report ID (2)
  0x15, 0x00,        //   Logical Minimum (0)
  0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
  0x19, 0x00,        //   Usage Minimum (0)
  0x2A, 0xFF, 0x03,  //   Usage Maximum (1023)
  0x75, 0x10,        //   Report Size (16)
  0x95, 0x01,        //   Report Count (1)
  0x81, 0x00,        //   Input (Data, Array, Abs)  — usage code
  0xC0               // End Collection
};

// ASCII 32..126 → HID scan code. Bit 7 set means LEFT_SHIFT is also needed.
// US QWERTY layout.
static const uint8_t _asciiToHid[95] = {
  0x2C,                                                       // ' '  32
  0x9E,0xB4,0xA0,0xA1,0xA2,0xA4,0x34,0xA6,0xA7,0xA5,        // !-*  33-42
  0xAE,0x36,0x2D,0x37,0x38,                                  // +-/  43-47
  0x27,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,        // 0-9  48-57
  0xB3,0x33,0xB6,0x2E,0xB7,0xB8,0x9F,                       // :-@  58-64
  0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,                  // A-H  65-72
  0x8C,0x8D,0x8E,0x8F,0x90,0x91,0x92,0x93,                  // I-P  73-80
  0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,        // Q-Z  81-90
  0x2F,0x31,0x30,0xA3,0xAD,0x35,                            // [-`  91-96
  0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,                  // a-h  97-104
  0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,                  // i-p  105-112
  0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,        // q-z  113-122
  0xAF,0xB1,0xB0,0xB5                                        // {-~  123-126
};

struct KbReport { uint8_t mod; uint8_t reserved; uint8_t keys[6]; };

// ---------------------------------------------------------------------------
// BLEManager — manages all BLE keyboard functionality
// NimBLE persists CCCD (notification subscription) in NVS per bonded peer,
// so keystrokes continue working after device resets without re-pairing.
// Security: bonding with Secure Connections, Just Works (no MITM, no pin).
// ---------------------------------------------------------------------------
class BLEManager : public NimBLEServerCallbacks {
public:
  BLEManager(const char* mfr, uint8_t bat) : _mfr(mfr), _bat(bat) {}

  void begin(const char* name) {
    memset(&_rep, 0, sizeof(_rep));
    NimBLEDevice::init(name);
    // init() must come first. setSecurityAuth() after init() sets bond=true,
    // MITM=true, SC=true. The IO capability is left at NimBLE's default —
    // overriding it with NO_INPUT_OUTPUT prevents Android from bonding.
    NimBLEDevice::setSecurityAuth(true, true, true);
    if (DEBUG) printf("Bonds in NVS: %d\n", NimBLEDevice::getNumBonds());

    _srv = NimBLEDevice::createServer();
    _srv->setCallbacks(this);
    _srv->advertiseOnDisconnect(true); // NimBLE restarts advertising automatically

    _hid   = new NimBLEHIDDevice(_srv);
    _input   = _hid->getInputReport(1);
    _inputCC = _hid->getInputReport(2);

    _hid->setManufacturer(_mfr);
    _hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
    _hid->setHidInfo(0x00, 0x02);
    _hid->setReportMap((uint8_t*)_hidReportDesc, sizeof(_hidReportDesc));
    _hid->startServices();
    _hid->setBatteryLevel(_bat);

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setAppearance(HID_KEYBOARD);
    adv->addServiceUUID(_hid->getHidService()->getUUID());

    // Include device name in scan response so Android/Windows show it in
    // the pairing list and recognise it as a keyboard.
    NimBLEAdvertisementData scanData;
    scanData.setName(name);
    adv->setScanResponseData(scanData);

    // If a bonded peer exists, use directed advertising so Android/Windows
    // auto-reconnects without user interaction after a device reset.
    // After the directed window expires NimBLE falls back to undirected advertising.
    if (NimBLEDevice::getNumBonds() > 0) {
      NimBLEAddress peer = NimBLEDevice::getBondedAddress(0);
      if (DEBUG) {
        printf("Directed adv to bonded peer: ");
        printf("%s\n", peer.toString().c_str());
      }
      adv->start(0, &peer);
    } else {
      adv->start();
    }
  }

  void end() {
    NimBLEDevice::deinit(false); // false = keep bond data in NVS
    _connected = false;
    _srv       = nullptr;
    _hid       = nullptr;
    _input     = nullptr;
    _inputCC   = nullptr;
  }

  bool isConnected() { return _connected; }

  // Send a single key tap (press + immediate release).
  // Accepts any KEY_* constant, including media keys (KEY_MEDIA_PLAY_PAUSE, etc.).
  void write(uint8_t key) {
    if (DEBUG) printf("[BLE] write: key=0x%02X (%d)\n", key, key);
    if (isMediaKey(key)) { pressMedia(key); vTaskDelay(pdMS_TO_TICKS(10)); releaseAllMedia(); return; }
    press(key); vTaskDelay(pdMS_TO_TICKS(10)); releaseAll();
  }

  // Hold a regular keyboard key down (accumulates modifiers / keys until releaseAll).
  void press(uint8_t key) {
    uint8_t scan = 0, modBit = 0;
    toHID(key, scan, modBit);
    if (DEBUG) printf("[BLE] press: key=0x%02X -> scan=0x%02X mod=0x%02X\n", key, scan, modBit);
    if (modBit)    _rep.mod |= modBit;
    if (scan)      for (int i = 0; i < 6; i++) if (!_rep.keys[i]) { _rep.keys[i] = scan; break; }
    send();
  }

  // Release all held keyboard keys.
  void releaseAll() { memset(&_rep, 0, sizeof(_rep)); send(); }

  // Hold a media key down until releaseAllMedia() is called.
  // Accepts KEY_MEDIA_* constants (play/pause, stop, next, prev, vol up/down, mute).
  void pressMedia(uint8_t key) {
    uint16_t usage = mediaKeyToUsage(key);
    if (DEBUG) printf("[BLE] pressMedia: key=0x%02X usage=0x%04X\n", key, usage);
    _repCC = usage;
    sendCC();
  }

  // Release all held media keys.
  void releaseAllMedia() { _repCC = 0; sendCC(); }

  // Update the BLE Battery Service level (0–100 %).
  // Notifies the connected host if a connection is active.
  void setBatteryLevel(uint8_t level) {
    _bat = level;
    if (_hid) _hid->setBatteryLevel(level, _connected);
  }

  // Delete all stored BLE bonds from NVS
  static void clearAllBonds() { NimBLEDevice::deleteAllBonds(); }

private:
  const char*           _mfr;
  uint8_t               _bat;
  bool                  _connected = false;
  NimBLEServer*         _srv       = nullptr;
  NimBLEHIDDevice*      _hid       = nullptr;
  NimBLECharacteristic* _input     = nullptr;  // HID Report ID 1 — keyboard input
  NimBLECharacteristic* _inputCC   = nullptr;  // HID Report ID 2 — Consumer Control (media keys)
  KbReport              _rep       = {};        // Current keyboard report state
  uint16_t              _repCC     = 0;         // Current Consumer Control report state (USB usage code)

  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    _connected = true;
    if (DEBUG) printf("BLE connected\n");
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    _connected = false;
    if (DEBUG) printf("BLE disconnected (reason %d), bonds in NVS: %d\n",
                             reason, NimBLEDevice::getNumBonds());
    // advertising restart handled automatically by advertiseOnDisconnect(true)
  }

  void onAuthenticationComplete(NimBLEConnInfo& conn_info) override {
    if (DEBUG) printf("Auth complete — bonded: %s, bonds stored: %d\n",
                             conn_info.isBonded() ? "yes" : "no",
                             NimBLEDevice::getNumBonds());
  }

  // Transmits the current keyboard report over BLE.
  void send() {
    if (DEBUG) {
      printf("[BLE] send: connected=%d input=%s | mod=0x%02X keys=[%02X %02X %02X %02X %02X %02X]\n",
        (int)_connected, _input ? "ok" : "NULL",
        _rep.mod,
        _rep.keys[0], _rep.keys[1], _rep.keys[2],
        _rep.keys[3], _rep.keys[4], _rep.keys[5]);
    }
    if (_connected && _input) {
      _input->setValue((uint8_t*)&_rep, sizeof(_rep));
      _input->notify();
    }
  }

  // Returns true if key is a media key (KEY_MEDIA_*).
  static bool isMediaKey(uint8_t k) { return mediaKeyToUsage(k) != 0; }

  // Maps a KEY_MEDIA_* constant to its USB Consumer Control HID usage code.
  // Returns 0 for non-media keys.
  static uint16_t mediaKeyToUsage(uint8_t k) {
    switch (k) {
      case KEY_MEDIA_PLAY_PAUSE: return 0x00CD;  // Play/Pause
      case KEY_MEDIA_STOP:       return 0x00B7;  // Stop
      case KEY_MEDIA_NEXT:       return 0x00B5;  // Next Track
      case KEY_MEDIA_PREV:       return 0x00B6;  // Previous Track
      case KEY_MEDIA_VOL_UP:     return 0x00E9;  // Volume Increment
      case KEY_MEDIA_VOL_DOWN:   return 0x00EA;  // Volume Decrement
      case KEY_MEDIA_MUTE:       return 0x00E2;  // Mute
      default:                   return 0;
    }
  }

  // Transmits the current Consumer Control (media key) report over BLE.
  void sendCC() {
    if (DEBUG) printf("[BLE] sendCC: connected=%d inputCC=%s | usage=0x%04X\n",
      (int)_connected, _inputCC ? "ok" : "NULL", _repCC);
    if (_connected && _inputCC) {
      uint8_t buf[2] = { (uint8_t)(_repCC & 0xFF), (uint8_t)(_repCC >> 8) };
      _inputCC->setValue(buf, 2);
      _inputCC->notify();
    }
  }

  // Convert Arduino-Keyboard / BleKeyboard code → HID scan code + modifier bit
  static void toHID(uint8_t k, uint8_t& scan, uint8_t& mod) {
    scan = 0; mod = 0;
    if (k >= 0x80 && k <= 0x87) { mod = 1 << (k - 0x80); return; } // modifier keys
    if (k >= 32  && k <= 126)   {
      uint8_t e = _asciiToHid[k - 32];
      if (e & 0x80) { mod = 0x02; scan = e & 0x7F; } else { scan = e; }
      return;
    }
    switch (k) {                    // special keys
      case 0xB0: scan=0x28; break;  // Return
      case 0xB1: scan=0x29; break;  // Esc
      case 0xB2: scan=0x2A; break;  // Backspace
      case 0xB3: scan=0x2B; break;  // Tab
      case 0xC1: scan=0x39; break;  // Caps Lock
      case 0xC2: scan=0x3A; break;  // F1
      case 0xC3: scan=0x3B; break;  // F2
      case 0xC4: scan=0x3C; break;  // F3
      case 0xC5: scan=0x3D; break;  // F4
      case 0xC6: scan=0x3E; break;  // F5
      case 0xC7: scan=0x3F; break;  // F6
      case 0xC8: scan=0x40; break;  // F7
      case 0xC9: scan=0x41; break;  // F8
      case 0xCA: scan=0x42; break;  // F9
      case 0xCB: scan=0x43; break;  // F10
      case 0xCC: scan=0x44; break;  // F11
      case 0xCD: scan=0x45; break;  // F12
      case 0xD1: scan=0x49; break;  // Insert
      case 0xD2: scan=0x4A; break;  // Home
      case 0xD3: scan=0x4B; break;  // Page Up
      case 0xD4: scan=0x4C; break;  // Delete
      case 0xD5: scan=0x4D; break;  // End
      case 0xD6: scan=0x4E; break;  // Page Down
      case 0xD7: scan=0x4F; break;  // Right Arrow
      case 0xD8: scan=0x50; break;  // Left Arrow
      case 0xD9: scan=0x51; break;  // Down Arrow
      case 0xDA: scan=0x52; break;  // Up Arrow
    }
  }
};
