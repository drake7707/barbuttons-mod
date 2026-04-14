#include "ble/BLEManager.h"

// ---------------------------------------------------------------------------
// Combined HID report descriptor:
//   Report ID 1 — standard boot-compatible keyboard (8-byte report)
//   Report ID 2 — Consumer Control (2-byte usage code for media keys)
// ---------------------------------------------------------------------------
static const uint8_t _hidReportDesc[] = {
    // --- Keyboard (Report ID 1) ---
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x01, //   Report ID (1)
    0x05, 0x07, //   Usage Page (Key Codes)
    0x19, 0xE0, //   Usage Minimum (Left Control)
    0x29, 0xE7, //   Usage Maximum (Right GUI)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input (Data, Var, Abs)    — modifier byte
    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x01, //   Input (Const)             — reserved byte
    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x25, 0x65, //   Logical Maximum (101)
    0x05, 0x07, //   Usage Page (Key Codes)
    0x19, 0x00, //   Usage Minimum (0)
    0x29, 0x65, //   Usage Maximum (101)
    0x81, 0x00, //   Input (Data, Array, Abs)  — key slots
    0xC0,       // End Collection
    // --- Consumer Control (Report ID 2) ---
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       //   Report ID (2)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x03, //   Logical Maximum (1023)
    0x19, 0x00,       //   Usage Minimum (0)
    0x2A, 0xFF, 0x03, //   Usage Maximum (1023)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x00,       //   Input (Data, Array, Abs)  — usage code
    0xC0              // End Collection
};

// ASCII 32..126 → HID scan code. Bit 7 set means LEFT_SHIFT is also needed.
// US QWERTY layout.
static const uint8_t _asciiToHid[95] = {
    0x2C,                                                       // ' '  32
    0x9E, 0xB4, 0xA0, 0xA1, 0xA2, 0xA4, 0x34, 0xA6, 0xA7, 0xA5, // !-*  33-42
    0xAE, 0x36, 0x2D, 0x37, 0x38,                               // +-/  43-47
    0x27, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, // 0-9  48-57
    0xB3, 0x33, 0xB6, 0x2E, 0xB7, 0xB8, 0x9F,                   // :-@  58-64
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B,             // A-H  65-72
    0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93,             // I-P  73-80
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, // Q-Z  81-90
    0x2F, 0x31, 0x30, 0xA3, 0xAD, 0x35,                         // [-`  91-96
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,             // a-h  97-104
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,             // i-p  105-112
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, // q-z  113-122
    0xAF, 0xB1, 0xB0, 0xB5                                      // {-~  123-126
};

BLEManager::BLEManager(const char *manufacturer, uint8_t batteryLevel)
    : _manufacturer(manufacturer), _battery(batteryLevel), _advManager(_connections) {}

void BLEManager::begin(const char *name, bool negotiatePowerSavingConnectionParameters, uint8_t maxConnections)
{
  _maxConnections = maxConnections;
  _negotiatePowerSavingConnectionParameters = negotiatePowerSavingConnectionParameters;

  memset(&_report, 0, sizeof(_report));
  NimBLEDevice::init(name);
  // init() must come first. setSecurityAuth() after init() sets bond=true,
  // MITM=true, SC=true. The IO capability is left at NimBLE's default —
  // overriding it with NO_INPUT_OUTPUT prevents Android from bonding.
  NimBLEDevice::setSecurityAuth(true, true, true);
  if (DEBUG)
    printf("[BLE] Bonds in NVS: %d\n", NimBLEDevice::getNumBonds());

  _server = NimBLEDevice::createServer();
  _server->setCallbacks(this);

  _server->advertiseOnDisconnect(false); // we manage advertising ourselves to control directed vs undirected

  _hid = new NimBLEHIDDevice(_server);
  _input = _hid->getInputReport(1);
  _inputCC = _hid->getInputReport(2);

  _hid->setManufacturer(_manufacturer);
  _hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  _hid->setHidInfo(0x00, 0x02);
  _hid->setReportMap((uint8_t *)_hidReportDesc, sizeof(_hidReportDesc));  
  _hid->setBatteryLevel(_battery);

  _advManager.begin(_hid, name, maxConnections);

  // If a bonded peer exists, use directed advertising so Android/Windows
  // auto-reconnects without user interaction after a device reset.
  // Sequential bond advertising: tries each unconnected bond in turn before
  // falling back to undirected advertising.
  _advManager.startCycle();
}

void BLEManager::end()
{
  NimBLEDevice::deinit(false); // false = keep bond data in NVS
  _connections.clear();
  _server = nullptr;
  _hid = nullptr;
  _input = nullptr;
  _inputCC = nullptr;
}

bool BLEManager::isConnected() const { return !_connections.empty(); }

BLEAdvertisingManager& BLEManager::getAdvertisingManager()
{
  return _advManager;
}

std::vector<std::string> BLEManager::getConnections() const
{
  std::vector<std::string> peers;
  for (const auto &pair : _connections)
  {
    peers.push_back(pair.first);
  }
  return peers;
}

std::vector<std::string> BLEManager::getBondedAddresses()
{
  std::vector<std::string> result;
  int bondCount = NimBLEDevice::getNumBonds();
  for (int i = 0; i < bondCount; i++)
  {
    NimBLEAddress addr = NimBLEDevice::getBondedAddress(i);
    result.push_back(addr.toString());
  }
  if (DEBUG)
    printf("[BLE] getBondedAddresses: %d bonds\n", bondCount);
  return result;
}

void BLEManager::write(std::string &target, uint8_t key)
{
  if (DEBUG)
    printf("[BLE] write: key=0x%02X (%d)\n", key, key);
  if (isMediaKey(key))
  {
    pressMedia(target, key);
    vTaskDelay(pdMS_TO_TICKS(10));
    releaseAllMedia(target);
    return;
  }
  press(target, key);
  vTaskDelay(pdMS_TO_TICKS(10));
  releaseAll(target);
}

void BLEManager::press(std::string &target, uint8_t key)
{
  uint8_t scanCode = 0, modifierBit = 0;
  toHID(key, scanCode, modifierBit);
  if (DEBUG)
    printf("[BLE] press: key=0x%02X -> scan=0x%02X mod=0x%02X\n", key, scanCode, modifierBit);
  if (modifierBit)
    _report.modifier |= modifierBit;
  if (scanCode)
    for (int i = 0; i < 6; i++)
      if (!_report.keys[i])
      {
        _report.keys[i] = scanCode;
        break;
      }
  send(target);
}

void BLEManager::releaseAll(std::string &target)
{
  memset(&_report, 0, sizeof(_report));
  send(target);
}

void BLEManager::pressMedia(std::string &target, uint8_t key)
{
  uint16_t usage = mediaKeyToUsage(key);
  if (DEBUG)
    printf("[BLE] pressMedia: key=0x%02X usage=0x%04X\n", key, usage);
  _reportCC = usage;
  sendCC(target);
}

void BLEManager::releaseAllMedia(std::string &target)
{
  _reportCC = 0;
  sendCC(target);
}

void BLEManager::setBatteryLevel(uint8_t level)
{
  _battery = level;
  if (_hid)
    _hid->setBatteryLevel(level, !_connections.empty());
}

void BLEManager::clearAllBonds() { NimBLEDevice::deleteAllBonds(); }

void BLEManager::onConnect(NimBLEServer *server, NimBLEConnInfo &connectionInfo)
{
  _connections[connectionInfo.getIdAddress().toString()] = connectionInfo.getConnHandle();

  if (DEBUG)
    printf("[BLE] Connected to %s\n", connectionInfo.getIdAddress().toString().c_str());

  _advManager.startCycle();

  if (_negotiatePowerSavingConnectionParameters)
  {
    _server->updateConnParams(connectionInfo.getConnHandle(), 40, 80, 4, 400);
    printf("[BLE] Requested power-saving connection parameters: interval 50–100 ms, latency 4, timeout 4 s\n");
  }
  else
  {
    printf("[BLE] Power-saving connection parameters negotiation disabled; using default BLE connection parameters\n");
  }
}

void BLEManager::onDisconnect(NimBLEServer *, NimBLEConnInfo &connectionInfo, int reason)
{
  _connections.erase(connectionInfo.getIdAddress().toString());
  if (DEBUG)
    printf("[BLE] Disconnected (reason %d), bonds in NVS: %d\n",
           reason, NimBLEDevice::getNumBonds());

  _advManager.startCycle();
}

void BLEManager::onAuthenticationComplete(NimBLEConnInfo &connectionInfo)
{
  if (DEBUG)
    printf("[BLE] Auth complete for %s — bonded: %s, bonds stored: %d connections=%d \n",
           connectionInfo.getIdAddress().toString().c_str(),
           connectionInfo.isBonded() ? "yes" : "no",
           NimBLEDevice::getNumBonds(),
           (int)_connections.size());
}

void BLEManager::send(std::string &target)
{
  if (DEBUG)
  {
    printf("[BLE] send: connections=%d input=%s | mod=0x%02X keys=[%02X %02X %02X %02X %02X %02X], target=%s\n",
           (int)_connections.size(), _input ? "ok" : "NULL",
           _report.mod,
           _report.keys[0], _report.keys[1], _report.keys[2],
           _report.keys[3], _report.keys[4], _report.keys[5], target.c_str());
  }
  if (!_connections.empty() && _input)
  {
    _input->setValue((uint8_t *)&_report, sizeof(_report));

    if (target == "")
      _input->notify(); // broadcast to all connected peers
    else if (_connections.find(target) != _connections.end())
    {
      auto handle = _connections[target];
      _input->notify(handle);
    }
    else
    {
      if (DEBUG)
        printf("[BLE] Warning: target %s not found among connected peers\n", target.c_str());
    }
  }
}

void BLEManager::sendCC(std::string &target)
{
  if (DEBUG)
    printf("[BLE] sendCC: connections=%d inputCC=%s | usage=0x%04X, target=%s\n",
           (int)_connections.size(), _inputCC ? "ok" : "NULL", _reportCC, target.c_str());
  if (!_connections.empty() && _inputCC)
  {
    uint8_t buf[2] = {(uint8_t)(_reportCC & 0xFF), (uint8_t)(_reportCC >> 8)};
    _inputCC->setValue(buf, 2);

    if (target == "")
      _inputCC->notify(); // broadcast to all connected peers
    else if (_connections.find(target) != _connections.end())
    {
      auto handle = _connections[target];
      _inputCC->notify(handle);
    }
    else
    {
      if (DEBUG)
        printf("[BLE] Warning: target %s not found among connected peers\n", target.c_str());
    }
  }
}

bool BLEManager::isMediaKey(uint8_t keyCode) { return mediaKeyToUsage(keyCode) != 0; }

// Maps a KEY_MEDIA_* constant to its USB Consumer Control HID usage code.
// Returns 0 for non-media keys.
uint16_t BLEManager::mediaKeyToUsage(uint8_t keyCode)
{
  switch (keyCode)
  {
  case KEY_MEDIA_PLAY_PAUSE:
    return 0x00CD; // Play/Pause
  case KEY_MEDIA_STOP:
    return 0x00B7; // Stop
  case KEY_MEDIA_NEXT:
    return 0x00B5; // Next Track
  case KEY_MEDIA_PREV:
    return 0x00B6; // Previous Track
  case KEY_MEDIA_VOL_UP:
    return 0x00E9; // Volume Increment
  case KEY_MEDIA_VOL_DOWN:
    return 0x00EA; // Volume Decrement
  case KEY_MEDIA_MUTE:
    return 0x00E2; // Mute
  default:
    return 0;
  }
}

// Convert Arduino-Keyboard / BleKeyboard code → HID scan code + modifier bit
void BLEManager::toHID(uint8_t keyCode, uint8_t &scanCode, uint8_t &modifier)
{
  scanCode = 0;
  modifier = 0;
  if (keyCode >= 0x80 && keyCode <= 0x87)
  {
    modifier = 1 << (keyCode - 0x80);
    return;
  } // modifier keys
  if (keyCode >= 32 && keyCode <= 126)
  {
    uint8_t entry = _asciiToHid[keyCode - 32];
    if (entry & 0x80)
    {
      modifier = 0x02;
      scanCode = entry & 0x7F;
    }
    else
    {
      scanCode = entry;
    }
    return;
  }
  switch (keyCode)
  { // special keys
  case 0xB0:
    scanCode = 0x28;
    break; // Return
  case 0xB1:
    scanCode = 0x29;
    break; // Esc
  case 0xB2:
    scanCode = 0x2A;
    break; // Backspace
  case 0xB3:
    scanCode = 0x2B;
    break; // Tab
  case 0xC1:
    scanCode = 0x39;
    break; // Caps Lock
  case 0xC2:
    scanCode = 0x3A;
    break; // F1
  case 0xC3:
    scanCode = 0x3B;
    break; // F2
  case 0xC4:
    scanCode = 0x3C;
    break; // F3
  case 0xC5:
    scanCode = 0x3D;
    break; // F4
  case 0xC6:
    scanCode = 0x3E;
    break; // F5
  case 0xC7:
    scanCode = 0x3F;
    break; // F6
  case 0xC8:
    scanCode = 0x40;
    break; // F7
  case 0xC9:
    scanCode = 0x41;
    break; // F8
  case 0xCA:
    scanCode = 0x42;
    break; // F9
  case 0xCB:
    scanCode = 0x43;
    break; // F10
  case 0xCC:
    scanCode = 0x44;
    break; // F11
  case 0xCD:
    scanCode = 0x45;
    break; // F12
  case 0xD1:
    scanCode = 0x49;
    break; // Insert
  case 0xD2:
    scanCode = 0x4A;
    break; // Home
  case 0xD3:
    scanCode = 0x4B;
    break; // Page Up
  case 0xD4:
    scanCode = 0x4C;
    break; // Delete
  case 0xD5:
    scanCode = 0x4D;
    break; // End
  case 0xD6:
    scanCode = 0x4E;
    break; // Page Down
  case 0xD7:
    scanCode = 0x4F;
    break; // Right Arrow
  case 0xD8:
    scanCode = 0x50;
    break; // Left Arrow
  case 0xD9:
    scanCode = 0x51;
    break; // Down Arrow
  case 0xDA:
    scanCode = 0x52;
    break; // Up Arrow
  }
}
