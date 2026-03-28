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
// Includes
// ---------------------------------------------------------------------------
#include <Keypad.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
// BLE — NimBLE-Arduino (h2zero). Unlike the default ESP32 BLE library, NimBLE
// stores CCCD subscriptions in NVS for each bonded peer, so notifications
// survive device resets without requiring re-pairing.
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include "HIDTypes.h"
// NimBLE manages CCCD (BLE2902) internally — no separate include needed.

// ---------------------------------------------------------------------------
// Key code constants
// Same numeric values as BleKeyboard.h / Arduino Keyboard.h so NVS keymap
// data stored by earlier firmware remains valid.
// ---------------------------------------------------------------------------
#define KEY_LEFT_CTRL    0x80
#define KEY_LEFT_SHIFT   0x81
#define KEY_LEFT_ALT     0x82
#define KEY_LEFT_GUI     0x83
#define KEY_RIGHT_CTRL   0x84
#define KEY_RIGHT_SHIFT  0x85
#define KEY_RIGHT_ALT    0x86
#define KEY_RIGHT_GUI    0x87
#define KEY_RETURN       0xB0
#define KEY_ESC          0xB1
#define KEY_BACKSPACE    0xB2
#define KEY_TAB          0xB3
#define KEY_INSERT       0xD1
#define KEY_HOME         0xD2
#define KEY_PAGE_UP      0xD3
#define KEY_DELETE       0xD4
#define KEY_END          0xD5
#define KEY_PAGE_DOWN    0xD6
#define KEY_RIGHT_ARROW  0xD7
#define KEY_LEFT_ARROW   0xD8
#define KEY_DOWN_ARROW   0xD9
#define KEY_UP_ARROW     0xDA
#define KEY_CAPS_LOCK    0xC1
#define KEY_F1   0xC2
#define KEY_F2   0xC3
#define KEY_F3   0xC4
#define KEY_F4   0xC5
#define KEY_F5   0xC6
#define KEY_F6   0xC7
#define KEY_F7   0xC8
#define KEY_F8   0xC9
#define KEY_F9   0xCA
#define KEY_F10  0xCB
#define KEY_F11  0xCC
#define KEY_F12  0xCD

// ---------------------------------------------------------------------------
// Custom BLE Keyboard — NimBLE-Arduino edition
// NimBLE persists CCCD (notification subscription) in NVS per bonded peer,
// so keystrokes continue working after device resets without re-pairing.
// Security: bonding with Secure Connections, Just Works (no MITM, no pin).
// ---------------------------------------------------------------------------

// Standard boot-compatible HID keyboard descriptor (8-byte report, Report ID 1)
static const uint8_t _hidReportDesc[] = {
  0x05, 0x01,   // Usage Page (Generic Desktop)
  0x09, 0x06,   // Usage (Keyboard)
  0xA1, 0x01,   // Collection (Application)
  0x85, 0x01,   //   Report ID (1)
  0x05, 0x07,   //   Usage Page (Key Codes)
  0x19, 0xE0,   //   Usage Minimum (Left Control)
  0x29, 0xE7,   //   Usage Maximum (Right GUI)
  0x15, 0x00,   //   Logical Minimum (0)
  0x25, 0x01,   //   Logical Maximum (1)
  0x75, 0x01,   //   Report Size (1)
  0x95, 0x08,   //   Report Count (8)
  0x81, 0x02,   //   Input (Data, Var, Abs)    — modifier byte
  0x95, 0x01,   //   Report Count (1)
  0x75, 0x08,   //   Report Size (8)
  0x81, 0x01,   //   Input (Const)             — reserved byte
  0x95, 0x06,   //   Report Count (6)
  0x75, 0x08,   //   Report Size (8)
  0x25, 0x65,   //   Logical Maximum (101)
  0x05, 0x07,   //   Usage Page (Key Codes)
  0x19, 0x00,   //   Usage Minimum (0)
  0x29, 0x65,   //   Usage Maximum (101)
  0x81, 0x00,   //   Input (Data, Array, Abs)  — key slots
  0xC0          // End Collection
};

// ASCII 32..126 → HID scan code. Bit 7 set means LEFT_SHIFT is also needed.
// US QWERTY layout.
static const uint8_t _asciiToHid[95] PROGMEM = {
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

class CustomBLEKeyboard : public NimBLEServerCallbacks {
public:
  CustomBLEKeyboard(const char* name, const char* mfr, uint8_t bat)
    : _name(name), _mfr(mfr), _bat(bat) {}

  void begin() {
    memset(&_rep, 0, sizeof(_rep));
    NimBLEDevice::init(_name);
    // init() must come first. setSecurityAuth() after init() sets bond=true,
    // MITM=true, SC=true. The IO capability is left at NimBLE's default —
    // overriding it with NO_INPUT_OUTPUT prevents Android from bonding.
    NimBLEDevice::setSecurityAuth(true, true, true);
    if (DEBUG) Serial.printf("Bonds in NVS: %d\n", NimBLEDevice::getNumBonds());

    _srv = NimBLEDevice::createServer();
    _srv->setCallbacks(this);
    _srv->advertiseOnDisconnect(true); // NimBLE restarts advertising automatically

    _hid   = new NimBLEHIDDevice(_srv);
    _input = _hid->getInputReport(1);

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
    scanData.setName(_name);
    adv->setScanResponseData(scanData);

    // If a bonded peer exists, use directed advertising so Android/Windows
    // auto-reconnects without user interaction after a device reset.
    // After the directed window expires NimBLE falls back to undirected advertising.
    if (NimBLEDevice::getNumBonds() > 0) {
      NimBLEAddress peer = NimBLEDevice::getBondedAddress(0);
      if (DEBUG) {
        Serial.print("Directed adv to bonded peer: ");
        Serial.println(peer.toString().c_str());
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
  }

  bool isConnected() { return _connected; }

  // Tap: press then immediately release
  void write(uint8_t key) { press(key); delay(10); releaseAll(); }

  // Hold key (accumulates modifiers / keys until releaseAll)
  void press(uint8_t key) {
    uint8_t scan = 0, modBit = 0;
    toHID(key, scan, modBit);
    if (modBit)       _rep.mod |= modBit;
    else if (scan)    for (int i = 0; i < 6; i++) if (!_rep.keys[i]) { _rep.keys[i] = scan; break; }
    send();
  }

  void releaseAll() { memset(&_rep, 0, sizeof(_rep)); send(); }

private:
  const char*           _name;
  const char*           _mfr;
  uint8_t               _bat;
  bool                  _connected = false;
  NimBLEServer*         _srv   = nullptr;
  NimBLEHIDDevice*      _hid   = nullptr;
  NimBLECharacteristic* _input = nullptr;
  KbReport              _rep;

  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    _connected = true;
    if (DEBUG) Serial.println("BLE connected");
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    _connected = false;
    if (DEBUG) Serial.printf("BLE disconnected (reason %d), bonds in NVS: %d\n",
                             reason, NimBLEDevice::getNumBonds());
    // advertising restart handled automatically by advertiseOnDisconnect(true)
  }

  void onAuthenticationComplete(NimBLEConnInfo& conn_info) override {
    if (DEBUG) Serial.printf("Auth complete — bonded: %s, bonds stored: %d\n",
                             conn_info.isBonded() ? "yes" : "no",
                             NimBLEDevice::getNumBonds());
  }

  void send() {
    if (_connected && _input) {
      _input->setValue((uint8_t*)&_rep, sizeof(_rep));
      _input->notify();
    }
  }

  // Convert Arduino-Keyboard / BleKeyboard code → HID scan code + modifier bit
  static void toHID(uint8_t k, uint8_t& scan, uint8_t& mod) {
    scan = 0; mod = 0;
    if (k >= 0x80 && k <= 0x87) { mod = 1 << (k - 0x80); return; } // modifier keys
    if (k >= 32  && k <= 126)   {
      uint8_t e = pgm_read_byte(&_asciiToHid[k - 32]);
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

// ---------------------------------------------------------------------------
// BLE keyboard instance
// ---------------------------------------------------------------------------
// BLE device name persisted in NVS; loaded before bleKeyboard.begin() in setup().
#define BLE_NAME_MAX_LEN 32
const char DEFAULT_BLE_NAME[] = "BarButtons";
char ble_name[BLE_NAME_MAX_LEN + 1] = "BarButtons"; // initialised to DEFAULT_BLE_NAME; overwritten by load_ble_name()

CustomBLEKeyboard bleKeyboard(ble_name, "JaxeADV", 100);

// ---------------------------------------------------------------------------
// Keypad layout  (unchanged from original)
// ---------------------------------------------------------------------------
const byte ROWS = 3;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1', '5', '4'},
  {'2', '6', '7'},
  {'3', '8', '9'}
};

const int LED_PIN        = 6;
byte      rowPins[COLS]  = {2, 1, 0};
byte      colPins[ROWS]  = {3, 4, 5};

// ---------------------------------------------------------------------------
// Keypad instance + state
// ---------------------------------------------------------------------------
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

int last_keypad_state = IDLE;

const int long_press_time            = 500;
const int long_press_repeat_interval = 100;
const int long_press_time_config     = 4500; // additional hold after first 500 ms = 5 s total

// ---------------------------------------------------------------------------
// App state + LED
// ---------------------------------------------------------------------------
enum AppStatus {
  APP_BT_DISCONNECTED = 0,  // Not connected to BT
  APP_CONFIG          = 1,  // Config mode (AP active, rapid blink)
  APP_CONNECTED       = 2,  // BT connected, main menu
  APP_CONNECTED_BLINK = 3   // BT connected, keymap indicator flash
};
AppStatus app_status = APP_BT_DISCONNECTED;
int led_delays[4][2] = {
  {  500,  500 },  // APP_BT_DISCONNECTED
  {  100, 3000 },  // APP_CONFIG
  { 3000,  100 },  // APP_CONNECTED
  { 3000,  100 }   // APP_CONNECTED_BLINK
};
int keymap_indicator_led_delays[2] = {100, 50};
int keymap_indicator_countdown     = 0;
int led_state      = 0;
int led_state_time = 0;

// ---------------------------------------------------------------------------
// Configurable keymap  (8 buttons, indices 0-7 = button labels '1'-'8')
// ---------------------------------------------------------------------------
// short_keys[i] : key code sent on short press
// long_keys[i]  : key code sent on long press; 0 = auto-repeat short key while held
uint8_t short_keys[8];
uint8_t long_keys[8];

// Factory defaults — mirror the original hard-coded behaviour
const uint8_t DEFAULT_SHORT[8] = {
  '+', '-', 'n', 'c',
  KEY_UP_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_DOWN_ARROW
};
const uint8_t DEFAULT_LONG[8] = {
  0,   // btn1: repeat '+'
  0,   // btn2: repeat '-'
  'd', // btn3: long = 'd', short = 'n'
  0,   // btn4: reserved for config trigger (not configurable)
  0,   // btn5: repeat UP
  0,   // btn6: repeat LEFT
  0,   // btn7: repeat RIGHT
  0    // btn8: repeat DOWN
};

// ---------------------------------------------------------------------------
// NVS — load / save keymap
// ---------------------------------------------------------------------------
Preferences prefs;

void load_keymap() {
  prefs.begin("keymap", /*readOnly=*/true);
  for (int i = 0; i < 8; i++) {
    short_keys[i] = prefs.getUChar(("s" + String(i)).c_str(), DEFAULT_SHORT[i]);
    long_keys[i]  = prefs.getUChar(("l" + String(i)).c_str(), DEFAULT_LONG[i]);
  }
  prefs.end();
  if (DEBUG) {
    Serial.println("Keymap loaded from NVS:");
    for (int i = 0; i < 8; i++) {
      Serial.printf("  btn%d  short=%d  long=%d\n", i + 1, short_keys[i], long_keys[i]);
    }
  }
}

void save_keymap() {
  prefs.begin("keymap", /*readOnly=*/false);
  for (int i = 0; i < 8; i++) {
    prefs.putUChar(("s" + String(i)).c_str(), short_keys[i]);
    prefs.putUChar(("l" + String(i)).c_str(), long_keys[i]);
  }
  prefs.end();
  if (DEBUG) Serial.println("Keymap saved to NVS.");
}

// ---------------------------------------------------------------------------
// NVS — load / save BLE name
// ---------------------------------------------------------------------------
void load_ble_name() {
  prefs.begin("config", /*readOnly=*/true);
  String saved = prefs.getString("blename", DEFAULT_BLE_NAME);
  saved.toCharArray(ble_name, sizeof(ble_name));
  prefs.end();
  if (DEBUG) Serial.printf("BLE name loaded: %s\n", ble_name);
}

void save_ble_name() {
  prefs.begin("config", /*readOnly=*/false);
  prefs.putString("blename", ble_name);
  prefs.end();
  if (DEBUG) Serial.printf("BLE name saved: %s\n", ble_name);
}

// ---------------------------------------------------------------------------
// AP credentials
// ---------------------------------------------------------------------------
const char* AP_SSID = "BarButtons-Config";
const char* AP_PSWD = "barbuttons";

// ---------------------------------------------------------------------------
// Config web page  (stored in program flash, not RAM)
// ---------------------------------------------------------------------------
// The server replaces the placeholders SHORTVALS, LONGVALS, and BLENAME with
// the current keymap values and BLE device name before sending.
// ---------------------------------------------------------------------------
const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BarButtons Keymap</title>
<style>
body{font-family:sans-serif;max-width:740px;margin:20px auto;padding:0 12px}
h2{color:#2c2c2c;margin-bottom:4px}
p.sub{color:#666;font-size:.9em;margin-top:0}
table{width:100%;border-collapse:collapse;margin-top:12px}
th,td{padding:7px 10px;text-align:left;border:1px solid #ddd}
th{background:#f5f5f5;font-weight:600}
select{width:100%;box-sizing:border-box;padding:3px}
.save{margin-top:16px;background:#2a7cdf;color:#fff;border:none;
      padding:10px 28px;font-size:1em;border-radius:4px;cursor:pointer}
.save:hover{background:#1a5cb0}
.hint{color:#888;font-size:.82em;margin-top:6px}
.danger{background:#c0392b}.danger:hover{background:#962d22}
.field{margin-top:14px}
.field label{font-weight:600;display:block;margin-bottom:4px}
.field input[type=text]{width:100%;max-width:320px;box-sizing:border-box;
  padding:5px 8px;font-size:1em;border:1px solid #ccc;border-radius:3px}
</style>
</head>
<body>
<h2>BarButtons Keymap Config</h2>
<p class="sub">Firmware v2 &mdash; AP configuration mode</p>
<form method="POST" action="/save">
<table>
<thead>
  <tr><th>Button</th><th>Short Press</th><th>Long Press</th></tr>
</thead>
<tbody id="rows"></tbody>
</table>
<p class="hint">Button 4 long-press = always enters this config mode (not remappable).<br>
Tap Button 4 on the device to exit without saving.</p>
<div class="field">
<label for="blename">BLE Device Name:</label>
<input type="text" id="blename" name="blename" value="BLENAME"
       maxlength="32" pattern="[ -~]+" required
       placeholder="BarButtons">
<p class="hint">1&#8211;32 printable ASCII characters &mdash; shown in the Bluetooth pairing dialog.</p>
</div>
<input type="submit" class="save" value="Save &amp; Reboot">
</form>
<hr style="margin:24px 0">
<h3>Firmware Update</h3>
<form method="POST" action="/update" enctype="multipart/form-data" id="otaFrm">
<p><input type="file" name="firmware" accept=".bin" required></p>
<input type="submit" class="save" value="Flash Firmware" id="otaBtn">
<p class="hint" id="otaSt">Upload a .bin compiled for this board. Device reboots after a successful flash.</p>
</form>
<hr style="margin:24px 0">
<h3>BLE Bonds</h3>
<p class="sub">Clear stored pairing data if the device fails to reconnect after a reset. You will need to re-pair your phone afterwards.</p>
<form method="POST" action="/clearbonds" id="bondFrm">
<input type="submit" class="save danger" value="Clear BLE Bonds &amp; Reboot" id="bondBtn">
</form>
<hr style="margin:24px 0">
<h3>Reset to Defaults</h3>
<p class="sub">Restore the default keymap and BLE device name. All custom settings will be lost.</p>
<form method="POST" action="/resetdefaults" id="resetFrm">
<input type="submit" class="save danger" value="Reset to Defaults &amp; Reboot" id="resetBtn">
</form>
<script>
document.getElementById('otaFrm').onsubmit=function(){
  var b=document.getElementById('otaBtn');
  b.value='Uploading\u2026';b.disabled=true;
  document.getElementById('otaSt').textContent='Uploading \u2014 do not close this page\u2026';
};
document.getElementById('bondFrm').onsubmit=function(){
  if(!confirm('Clear all BLE bonds?\nThe device will reboot. You will need to re-pair your phone.')) return false;
  var b=document.getElementById('bondBtn');
  b.value='Clearing\u2026';b.disabled=true;
};
document.getElementById('resetFrm').onsubmit=function(){
  if(!confirm('Reset to defaults?\nThis will restore the default keymap and BLE device name.\nThe device will reboot.')) return false;
  var b=document.getElementById('resetBtn');
  b.value='Resetting\u2026';b.disabled=true;
};
// Key options: [code, label]
var K=[
  [43,'+'],[45,'-'],[42,'*'],[47,'/'],[61,'='],
  [32,'Space'],[46,'.'],[44,','],[59,';'],[58,':'],
  [33,'!'],[63,'?'],[64,'@'],[35,'#'],[37,'%'],
  [176,'Enter'],[177,'Esc'],[178,'Backspace'],[179,'Tab'],
  [209,'Insert'],[210,'Home'],[211,'Page Up'],
  [212,'Delete'],[213,'End'],[214,'Page Down'],
  [218,'Up Arrow'],[217,'Down Arrow'],[216,'Left Arrow'],[215,'Right Arrow'],
  [193,'Caps Lock'],
  [194,'F1'],[195,'F2'],[196,'F3'],[197,'F4'],[198,'F5'],[199,'F6'],
  [200,'F7'],[201,'F8'],[202,'F9'],[203,'F10'],[204,'F11'],[205,'F12']
];
for(var i=97;i<=122;i++) K.push([i, String.fromCharCode(i)]); // a-z
for(var i=48;i<=57;i++)  K.push([i, String.fromCharCode(i)]); // 0-9

var S=[SHORTVALS];
var L=[LONGVALS];

function buildOpts(cur, withRepeat) {
  var h = '';
  if (withRepeat)
    h += '<option value="0"' + (cur === 0 ? ' selected' : '') + '>\u21BA Repeat short key</option>';
  K.forEach(function(k) {
    var sel = (k[0] === cur && !(withRepeat && cur === 0)) ? ' selected' : '';
    h += '<option value="' + k[0] + '"' + sel + '>' + k[1] + '</option>';
  });
  return h;
}

var tbody = document.getElementById('rows');
for (var i = 0; i < 8; i++) {
  var tr = document.createElement('tr');
  // Button 4 (index 3) long press is reserved — show as disabled info cell
  var longCell = (i === 3)
    ? '<td><em style="color:#999">reserved (config trigger)</em><input type="hidden" name="l3" value="' + L[3] + '"></td>'
    : '<td><select name="l' + i + '">' + buildOpts(L[i], true) + '</select></td>';
  tr.innerHTML =
    '<td><strong>Button ' + (i + 1) + '</strong></td>' +
    '<td><select name="s' + i + '">' + buildOpts(S[i], false) + '</select></td>' +
    longCell;
  tbody.appendChild(tr);
}
</script>
</body>
</html>
)rawliteral";


// Forward declaration (body defined below with other keypad helpers)
void flash_led(int times, int length, int delay_time);

// ---------------------------------------------------------------------------
// AP / WebServer config mode
// ---------------------------------------------------------------------------
WebServer server(80);
bool      exit_config_mode = false;

void handle_root() {
  String html = FPSTR(CONFIG_HTML);
  String sv, lv;
  for (int i = 0; i < 8; i++) {
    if (i) { sv += ','; lv += ','; }
    sv += String(short_keys[i]);
    lv += String(long_keys[i]);
  }
  html.replace("SHORTVALS", sv);
  html.replace("LONGVALS",  lv);
  // Escape BLE name for safe use in an HTML attribute value
  String bn;
  for (int i = 0; ble_name[i]; i++) {
    char c = ble_name[i];
    if      (c == '&') bn += "&amp;";
    else if (c == '"') bn += "&quot;";
    else if (c == '<') bn += "&lt;";
    else if (c == '>') bn += "&gt;";
    else               bn += c;
  }
  html.replace("BLENAME", bn);
  server.send(200, "text/html", html);
}

void handle_save() {
  for (int i = 0; i < 8; i++) {
    String si = String(i);
    if (server.hasArg("s" + si)) short_keys[i] = (uint8_t)server.arg("s" + si).toInt();
    if (server.hasArg("l" + si)) long_keys[i]  = (uint8_t)server.arg("l" + si).toInt();
  }
  save_keymap();

  if (server.hasArg("blename")) {
    String newName = server.arg("blename");
    newName.trim();
    // Validate: 1–32 printable ASCII characters
    bool valid = newName.length() > 0 && newName.length() <= BLE_NAME_MAX_LEN;
    if (valid) {
      for (unsigned int j = 0; j < newName.length(); j++) {
        if (newName[j] < 0x20 || newName[j] > 0x7E) { valid = false; break; }
      }
    }
    if (valid) {
      newName.toCharArray(ble_name, sizeof(ble_name));
      save_ble_name();
    }
  }

  server.send(200, "text/html",
    "<!DOCTYPE html><html>"
    "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
    "<h2>&#10003; Saved!</h2><p>Rebooting&hellip;</p>"
    "</body></html>");

  flash_led(3, 80, 80);
  delay(800);
  ESP.restart();
}

void handle_clear_bonds() {
  // BLE is stopped while the WiFi AP is running (shared radio on ESP32-C3),
  // so we can't call deleteAllBonds() here. Instead write a flag to NVS and
  // reboot — setup() will delete the bonds once NimBLE is initialised again.
  prefs.begin("sys", false);
  prefs.putBool("clrbond", true);
  prefs.end();

  server.send(200, "text/html",
    "<!DOCTYPE html><html>"
    "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
    "<h2>&#10003; Bonds cleared!</h2><p>Rebooting&hellip; Re-pair your phone when the device is discoverable.</p>"
    "</body></html>");

  flash_led(3, 80, 80);
  delay(800);
  ESP.restart();
}

void handle_reset_defaults() {
  // Restore default keymap
  for (int i = 0; i < 8; i++) {
    short_keys[i] = DEFAULT_SHORT[i];
    long_keys[i]  = DEFAULT_LONG[i];
  }
  save_keymap();

  // Restore default BLE name
  strncpy(ble_name, DEFAULT_BLE_NAME, sizeof(ble_name));
  ble_name[sizeof(ble_name) - 1] = '\0';
  save_ble_name();

  server.send(200, "text/html",
    "<!DOCTYPE html><html>"
    "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
    "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
    "<h2>&#10003; Reset to defaults!</h2><p>Rebooting&hellip;</p>"
    "</body></html>");

  flash_led(3, 80, 80);
  delay(800);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// OTA upload handlers
// ---------------------------------------------------------------------------
// Called repeatedly during the POST body stream — writes each chunk to flash.
void handle_ota_upload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    if (DEBUG) Serial.printf("OTA start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      if (DEBUG) Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      if (DEBUG) Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(/*setMD5=*/true)) {
      if (DEBUG) Serial.printf("OTA written: %u bytes\n", upload.totalSize);
    } else {
      if (DEBUG) Update.printError(Serial);
    }
  }
}

// Called once after the full upload body is received — sends the result page.
void handle_ota_finish() {
  bool ok = !Update.hasError();
  server.send(200, "text/html",
    ok
      ? "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
        "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
        "<h2>&#10003; Update successful!</h2><p>Rebooting&hellip;</p></body></html>"
      : "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
        "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
        "<h2>&#10007; Update failed.</h2><p><a href='/'>Try again</a></p></body></html>");
  if (ok) {
    flash_led(5, 50, 50);
    delay(500);
    ESP.restart();
  }
}


void start_config_ap() {
  if (DEBUG) Serial.println("Entering config AP mode");

  exit_config_mode = false;
  // NOTE: app_status intentionally left at its current value here.
  // Setting it to 1 before the button is released would immediately trigger
  // the RELEASED handler's exit-config check. See drain loop below.

  // On ESP32-C3 the radio is shared; stop BLE before starting WiFi AP
  bleKeyboard.end();
  delay(100);

  WiFi.softAP(AP_SSID, AP_PSWD);
  if (DEBUG) {
    Serial.print("AP started: ");
    Serial.println(WiFi.softAPIP());
  }

  server.on("/",            HTTP_GET,  handle_root);
  server.on("/save",        HTTP_POST, handle_save);
  server.on("/clearbonds",  HTTP_POST, handle_clear_bonds);
  server.on("/resetdefaults", HTTP_POST, handle_reset_defaults);
  server.on("/update",      HTTP_POST, handle_ota_finish, handle_ota_upload);
  server.begin();

  // 5 quick flashes to signal we're in config mode
  flash_led(5, 100, 100);

  // Drain the button-4 RELEASED event that triggered config mode.
  // While app_status != APP_CONFIG the RELEASED handler will NOT set exit_config_mode,
  // so it is safe to poll here until the keypad goes idle.
  unsigned long drain_start = millis();
  while (keypad.getState() != IDLE && millis() - drain_start < 3000) {
    keypad.getKey();
    delay(10);
  }
  if (DEBUG) Serial.println("Button released, entering config loop");

  // Only NOW switch to config-mode status so the exit check becomes active
  app_status     = APP_CONFIG;
  led_state      = 0;
  led_state_time = millis();

  while (!exit_config_mode) {
    server.handleClient();
    keypad.getKey(); // keeps the keypad ISR happy; tap of '4' sets exit_config_mode

    // Drive the LED blink pattern for APP_CONFIG
    if ((millis() - (unsigned long)led_state_time) > (unsigned long)led_delays[APP_CONFIG][led_state]) {
      led_state = 1 - led_state;
      digitalWrite(LED_PIN, led_state);
      led_state_time = millis();
    }
    delay(5);
  }

  // --- Clean up ---
  server.stop();
  server.close();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);

  // Restart BLE
  bleKeyboard.begin();
  app_status     = APP_BT_DISCONNECTED; // will advance to APP_CONNECTED when BLE connects
  led_state      = 0;
  led_state_time = millis();

  if (DEBUG) Serial.println("Config mode exited, BLE restarting");
}

// ---------------------------------------------------------------------------
// Keymap helpers
// ---------------------------------------------------------------------------

// Map button label '1'-'8' to keymap array index 0-7. '9' returns -1 (unused).
int btn_index(char key) {
  if (key >= '1' && key <= '8') return key - '1';
  return -1;
}

// A button is "instant" (fires on key-down rather than key-up) when its long
// press mode is auto-repeat — mirroring the original firmware's instant_keys logic.
bool is_key_instant(char key) {
  int idx = btn_index(key);
  if (idx < 0) return false;
  return long_keys[idx] == 0; // 0 = repeat → instant
}

// ---------------------------------------------------------------------------
// Key send routines
// ---------------------------------------------------------------------------
void send_repeating_key(uint8_t key) {
  digitalWrite(LED_PIN, HIGH);
  while (keypad.getState() == HOLD) {
    bleKeyboard.write(key);
    delay(long_press_repeat_interval);
    keypad.getKey();
  }
  digitalWrite(LED_PIN, LOW);
}

void send_short_press(KeypadEvent key) {
  if (DEBUG) { Serial.print("Short press: "); Serial.println(key); }

  if (app_status == APP_CONNECTED || app_status == APP_CONNECTED_BLINK || app_status == APP_BT_DISCONNECTED) {
    int idx = btn_index(key);
    if (idx >= 0 && short_keys[idx] != 0) {
      bleKeyboard.write(short_keys[idx]);
      flash_led(1, 150, 0);
    }
  }
}

void send_long_press(KeypadEvent key) {
  if (DEBUG) { Serial.print("Long press: "); Serial.println(key); }

  if (app_status == APP_CONNECTED || app_status == APP_CONNECTED_BLINK || app_status == APP_BT_DISCONNECTED) {

    // Button 4 long-press is always the config trigger, regardless of keymap
    if (key == '4') {
      if (wait_for_key_hold(long_press_time_config)) {
        start_config_ap();
      }
      return;
    }

    int idx = btn_index(key);
    if (idx < 0) return;

    if (long_keys[idx] == 0) {
      // Auto-repeat the short key
      send_repeating_key(short_keys[idx]);
    } else {
      // Send the distinct long-press key once
      bleKeyboard.write(long_keys[idx]);
      flash_led(1, 150, 0);
    }
  }
}

void flash_led(int times, int length, int delay_time) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(length);
    digitalWrite(LED_PIN, LOW);
    if (i < (times - 1)) delay(delay_time);
  }
}

bool wait_for_key_hold(int key_hold_time) {
  int start = millis();
  while (keypad.getState() == HOLD && millis() < (unsigned long)(start + key_hold_time)) {
    delay(20);
    keypad.getKey();
  }
  return (keypad.getState() == HOLD);
}

// ---------------------------------------------------------------------------
// Keypad event handler
// ---------------------------------------------------------------------------
void keypad_handler(KeypadEvent key) {
  if (DEBUG) Serial.println("keypad_handler");

  switch (keypad.getState()) {

    case PRESSED:
      last_keypad_state = keypad.getState();
      if (is_key_instant(key) && app_status != APP_CONFIG) send_short_press(key);
      break;

    case HOLD:
      last_keypad_state = keypad.getState();
      send_long_press(key);
      break;

    case RELEASED:
      // Tap of '4' during config mode exits AP mode without saving
      if (app_status == APP_CONFIG && key == '4') {
        exit_config_mode = true;
        last_keypad_state = keypad.getState();
        break;
      }

      if (last_keypad_state == PRESSED) {
        if (!(is_key_instant(key) && app_status != APP_CONFIG)) {
          send_short_press(key);
        }
      }
      last_keypad_state = keypad.getState();

      if (app_status == APP_CONNECTED || app_status == APP_CONNECTED_BLINK) {
        digitalWrite(LED_PIN, LOW);
        led_state      = 0;
        led_state_time = millis();
      }
      bleKeyboard.releaseAll();
      break;

    case IDLE:
      last_keypad_state = keypad.getState();

      if (app_status == APP_CONNECTED || app_status == APP_CONNECTED_BLINK) {
        digitalWrite(LED_PIN, LOW);
        led_state      = 0;
        led_state_time = millis();
      }
      bleKeyboard.releaseAll();
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

  pinMode(LED_PIN, OUTPUT);

  load_keymap(); // Load from NVS (or fall back to defaults)
  load_ble_name(); // Load BLE device name from NVS (or fall back to "BarButtons")

  bleKeyboard.begin();

  // If "Clear BLE Bonds" was requested from the web UI, delete stored bonds
  // now that NimBLE is initialised. The flag was written before the reboot.
  prefs.begin("sys", false);
  if (prefs.getBool("clrbond", false)) {
    NimBLEDevice::deleteAllBonds();
    prefs.remove("clrbond");
    if (DEBUG) Serial.println("BLE bonds cleared on request.");
  }
  prefs.end();

  keypad.addEventListener(keypad_handler);
  keypad.setHoldTime(long_press_time);

  if (DEBUG) Serial.println("Setup complete.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
  keypad.getKey();

  // Track BLE connection state
  if (app_status == APP_BT_DISCONNECTED && bleKeyboard.isConnected()) {
    app_status = APP_CONNECTED;
    digitalWrite(LED_PIN, LOW);   // LED off as soon as connected
    led_state      = 0;
    led_state_time = millis();
  }
  if (app_status != APP_BT_DISCONNECTED && app_status != APP_CONFIG && !bleKeyboard.isConnected()) app_status = APP_BT_DISCONNECTED;

  // LED blink pattern (only when keypad is idle)
  if (keypad.getState() == IDLE) {
    if (app_status == APP_CONNECTED_BLINK) {
      if ((millis() - (unsigned long)led_state_time) > (unsigned long)keymap_indicator_led_delays[led_state]) {
        led_state = 1 - led_state;
        digitalWrite(LED_PIN, led_state);
        led_state_time = millis();
        if (led_state == 0) {
          if (--keymap_indicator_countdown == 0) app_status = APP_CONNECTED;
        }
      }
    } else if (app_status != APP_CONNECTED) {
      // APP_BT_DISCONNECTED / APP_CONFIG: run the blink timer
      if ((millis() - (unsigned long)led_state_time) > (unsigned long)led_delays[app_status][led_state]) {
        led_state = 1 - led_state;
        digitalWrite(LED_PIN, led_state);
        led_state_time = millis();
      }
    }
    // APP_CONNECTED: LED stays off; button presses use flash_led() directly
  }

  delay(10);
}
