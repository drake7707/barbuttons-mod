#include "BLEAdvertisingManager.h"

BLEAdvertisingManager::BLEAdvertisingManager(const std::map<std::string, uint16_t> &connections)
    : _connections(connections) {}

void BLEAdvertisingManager::begin(NimBLEHIDDevice *hid, const char *deviceName, uint8_t maxConnections)
{
  _maxConnections = maxConnections;
  _hid = hid;
  _deviceName = deviceName;

  configureHIDAdvertising();

  // When advertising ends (directed timeout, connection, or explicit stop)
  // NimBLE fires this callback — we use it to step to the next bond or fall
  // back to undirected advertising.
  NimBLEDevice::getAdvertising()->setAdvertisingCompleteCallback(
      [this](NimBLEAdvertising *) { advance(); });
}

void BLEAdvertisingManager::configureHIDAdvertising()
{
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  // Build the primary advertisement payload explicitly so we can switch
  // back to it cleanly after a BTHome broadcast (NimBLE has no public API
  // to clear custom advertisement data once set).
  NimBLEAdvertisementData advData;
  // 0x06 = LE General Discoverable Mode | BR/EDR Not Supported
  advData.setFlags(0x06);
  // Appearance: HID Keyboard (0x03C1).  Full AD structure layout:
  //   [0x03]  length  = 3 bytes follow (type + 2-byte value)
  //   [0x19]  type    = Appearance
  //   [0xC1]  value LSB of 0x03C1 (HID Keyboard appearance)
  //   [0x03]  value MSB of 0x03C1
  uint8_t appearance[] = {0x03, 0x19, 0xC1, 0x03};
  advData.addData(appearance, sizeof(appearance));
  advData.addServiceUUID(_hid->getHidService()->getUUID());
  adv->setAdvertisementData(advData);

  // Include device name in scan response so Android/Windows show it in
  // the pairing list and recognise it as a keyboard.
  NimBLEAdvertisementData scanData;
  scanData.setName(_deviceName);
  adv->setScanResponseData(scanData);
}

void BLEAdvertisingManager::startCycle()
{
  if (_connections.size() >= _maxConnections)
  {
    if (DEBUG)
      printf("[BLE ADV] Connection limit reached (%d), not advertising\n", _maxConnections);
    return;
  }

  _nextBondIdx = 0;
  _advertisingCycleStartMs = pdTICKS_TO_MS(xTaskGetTickCount());

  if (DEBUG)
    printf("[BLE ADV] Starting advertising cycle with %d connection%s\n",
           (int)_connections.size(), _connections.size() == 1 ? "" : "s");

  advance();
}

void BLEAdvertisingManager::advance()
{
  // If we just finished a BTHome broadcast, restore HID advertising data
  // and restart the normal advertising cycle only if it was running before
  // the broadcast was triggered.
  if (_btHomeBroadcastActive)
  {
    _btHomeBroadcastActive = false;
    configureHIDAdvertising();
    if (_restoreHIDAfterBTHome)
    {
      if (DEBUG)
        printf("[BLE ADV] BTHome broadcast complete, restarting HID advertising cycle\n");
      startCycle();
    }
    else
    {
      if (DEBUG)
        printf("[BLE ADV] BTHome broadcast complete, HID advertising was not running — not restarting\n");
    }
    return;
  }

  if (_connections.size() >= _maxConnections)
  {
    if (DEBUG)
      printf("[BLE ADV] Connection limit reached (%d), stopping advertising cycle\n", _maxConnections);
    return;
  }

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  // Guard: onConnect() calls startCycle() which may already have started a
  // new advertising step before this callback fires for the previous directed
  // advertisement that just connected.
  if (adv->isAdvertising())
  {
    if (DEBUG)
      printf("[BLE ADV] Advertising already active, skipping advance\n");
    return;
  }

  // Calculate remaining time budget.
  // remainingBudgetMs == 0 means "infinite" (no connection exists yet).
  uint32_t remainingBudgetMs = 0;
  if (!_connections.empty())
  {
    // Unsigned subtraction is intentional: correct even if xTaskGetTickCount()
    // wraps (every ~49.7 days), since the advertising budget is only 60 s.
    uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount()) - _advertisingCycleStartMs;
    if (elapsed >= MAX_ADVERTISING_DURATION_AFTER_ALREADY_CONNECTED_MS)
    {
      if (DEBUG)
        printf("[BLE ADV] Advertising budget exhausted after %lu ms, stopping\n", elapsed);
      return;
    }
    remainingBudgetMs = MAX_ADVERTISING_DURATION_AFTER_ALREADY_CONNECTED_MS - elapsed;
  }

  // Try each unconnected bonded peer in order, starting from _nextBondIdx.
  int count = NimBLEDevice::getNumBonds();
  for (int i = _nextBondIdx; i < count; i++)
  {
    NimBLEAddress addr = NimBLEDevice::getBondedAddress(i);
    std::string mac = addr.toString();

    if (_connections.find(mac) != _connections.end())
      continue; // already connected, skip to next bond

    _nextBondIdx = i + 1;
    uint32_t dirDuration = (remainingBudgetMs == 0)
                               ? DIRECTED_ADV_STEP_DURATION_MS
                               : std::min(remainingBudgetMs, DIRECTED_ADV_STEP_DURATION_MS);

    if (DEBUG)
      printf("[BLE ADV] Start directed advertising to bonded peer %s for %lu ms\n", mac.c_str(), dirDuration);

    adv->start(dirDuration, &addr);
    return;
  }

  // All bonds tried (or no bonds at all) — fall back to undirected advertising.
  if (DEBUG)
    printf("[BLE ADV] All bonds tried, starting undirected advertising for %lu ms\n", remainingBudgetMs);

  adv->start(remainingBudgetMs);
}

bool BLEAdvertisingManager::isAdvertising()
{
  return NimBLEDevice::getAdvertising()->isAdvertising();
}

void BLEAdvertisingManager::broadcastBTHomeButtonPress(uint8_t eventType, uint8_t button)
{
  if (button < 1 || button > 8)
  {
    if (DEBUG)
      printf("[BLE ADV] BTHome: invalid button %d (must be 1–8)\n", button);
    return;
  }

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  bool wasAdvertising = adv->isAdvertising();
  if (wasAdvertising)
    adv->stop();

  // Record whether HID advertising was running so we can restore it after the
  // broadcast.  If a BTHome broadcast is already in flight (rapid button press)
  // we keep the flag from the first press so the original intent is preserved:
  // if HID was running when the first press fired, it should still be restarted
  // once all BTHome broadcasts are done.
  if (!_btHomeBroadcastActive)
    _restoreHIDAfterBTHome = wasAdvertising;

  // ---------------------------------------------------------------------------
  // BTHome v2 service data payload
  //
  // Byte 0   — device info: version=2, trigger-based=1, no encryption → 0x44
  //              bit 0   : encryption (0 = none)
  //              bit 1–1 : reserved
  //              bit 2   : trigger-based device (1 = event/trigger)
  //              bit 3–4 : reserved
  //              bit 5–7 : BTHome version (010 = v2)
  //
  // Bytes 1–16 — 8 × Button object (object ID 0x3A, 1-byte value)
  //              Only the slot for the pressed button carries a non-zero event;
  //              all other slots carry 0x00 (None).  Advertising all 8 slots
  //              from the first packet lets Home Assistant create all 8 button
  //              entities immediately on discovery.
  // ---------------------------------------------------------------------------
  static const uint8_t DEVICE_INFO   = 0x44; // v2, trigger-based, no encryption
  static const uint8_t BUTTON_OBJ_ID = 0x3A; // BTHome Button object ID

  uint8_t payload[1 + 8 * 2];
  payload[0] = DEVICE_INFO;
  for (int i = 0; i < 8; i++)
  {
    payload[1 + i * 2]     = BUTTON_OBJ_ID;
    payload[1 + i * 2 + 1] = (i == (button - 1)) ? eventType : BTHOME_BUTTON_NONE;
  }

  // BTHome uses 16-bit service data UUID 0xFCD2.
  NimBLEAdvertisementData btHomeData;
  btHomeData.setServiceData(NimBLEUUID((uint16_t)0xFCD2),
                            std::string(reinterpret_cast<char *>(payload), sizeof(payload)));
  adv->setAdvertisementData(btHomeData);

  // Clear scan response for the BTHome broadcast — HA needs only the service data.
  adv->setScanResponseData(NimBLEAdvertisementData());

  _btHomeBroadcastActive = true;

  if (DEBUG)
    printf("[BLE ADV] BTHome: broadcasting button %d event 0x%02X for %lu ms\n",
           button, eventType, BTHOME_BROADCAST_DURATION_MS);

  adv->start(BTHOME_BROADCAST_DURATION_MS);
}