#include "BLEAdvertisingManager.h"

BLEAdvertisingManager::BLEAdvertisingManager(const std::map<std::string, uint16_t> &connections)
    : _connections(connections) {}

void BLEAdvertisingManager::begin(NimBLEHIDDevice *hid, const char *deviceName, uint8_t maxConnections)
{
  _maxConnections = maxConnections;

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(HID_KEYBOARD);
  adv->addServiceUUID(hid->getHidService()->getUUID());

  // Include device name in scan response so Android/Windows show it in
  // the pairing list and recognise it as a keyboard.
  NimBLEAdvertisementData scanData;
  scanData.setName(deviceName);
  adv->setScanResponseData(scanData);

  // When advertising ends (directed timeout, connection, or explicit stop)
  // NimBLE fires this callback — we use it to step to the next bond or fall
  // back to undirected advertising.
  adv->setAdvertisingCompleteCallback([this](NimBLEAdvertising *)
                                      { advance(); });
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