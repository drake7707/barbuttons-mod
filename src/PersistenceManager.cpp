#include "PersistenceManager.h"

const uint8_t PersistenceManager::DEFAULT_SHORT[8] = {
    '+', '-', 'n', 'c',
    KEY_UP_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_DOWN_ARROW};
const uint8_t PersistenceManager::DEFAULT_LONG[8] = {
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
// loadAll -- load all persistent settings from NVS in one call
// ---------------------------------------------------------------------------
void PersistenceManager::loadAll()
{
  _loadKeymap();
  _loadActiveKeymap();
  _loadBleName();
  _loadBatteryEnabled();
  _loadBLEPowerSaving();
  _loadMaxBLEConnections();
}

// ---------------------------------------------------------------------------
// NVS -- keymaps (3 slots)
// ---------------------------------------------------------------------------
void PersistenceManager::_loadKeymap()
{
  const char *namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int km = 0; km < 3; km++)
  {
    nvs_handle_t h;
    bool opened = (nvs_open(namespaces[km], NVS_READONLY, &h) == ESP_OK);
    for (int i = 0; i < 8; i++)
    {
      char key[8]; // prefix + up to 2 digits + null

      // Short key
      snprintf(key, sizeof(key), "s%d", i);
      uint8_t v = DEFAULT_SHORT[i];
      if (opened)
        nvs_get_u8(h, key, &v);
      _shortEntries[km][i].key = v;

      // Long key
      snprintf(key, sizeof(key), "l%d", i);
      v = DEFAULT_LONG[i];
      if (opened)
        nvs_get_u8(h, key, &v);
      _longEntries[km][i].key = v;

      // Short target type (new field; default TARGET_SELECT for backward compatibility)
      snprintf(key, sizeof(key), "st%d", i);
      uint8_t tgt = TARGET_SELECT;
      if (opened)
        nvs_get_u8(h, key, &tgt);
      if (tgt > TARGET_BTHOME) tgt = TARGET_SELECT;
      _shortEntries[km][i].target = (KeyTarget)tgt;

      // Long target type
      snprintf(key, sizeof(key), "lt%d", i);
      tgt = TARGET_SELECT;
      if (opened)
        nvs_get_u8(h, key, &tgt);
      if (tgt > TARGET_BTHOME) tgt = TARGET_SELECT;
      _longEntries[km][i].target = (KeyTarget)tgt;

      // Short HID peer MAC (new field; default empty = broadcast all)
      snprintf(key, sizeof(key), "sm%d", i);
      _shortEntries[km][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(_shortEntries[km][i].mac);
        nvs_get_str(h, key, _shortEntries[km][i].mac, &macLen);
      }

      // Long HID peer MAC
      snprintf(key, sizeof(key), "lm%d", i);
      _longEntries[km][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(_longEntries[km][i].mac);
        nvs_get_str(h, key, _longEntries[km][i].mac, &macLen);
      }
    }
    if (opened)
      nvs_close(h);
  }
  if (DEBUG)
  {
    printf("[CONFIG] Keymaps loaded from NVS:\n");
    for (int km = 0; km < 3; km++)
    {
      printf("[CONFIG]   Keymap %d:\n", km + 1);
      for (int i = 0; i < 8; i++)
        printf("[CONFIG]     btn%d  short=%d(tgt=%d mac=%s)  long=%d(tgt=%d mac=%s)\n",
               i + 1,
               _shortEntries[km][i].key, (int)_shortEntries[km][i].target, _shortEntries[km][i].mac,
               _longEntries[km][i].key, (int)_longEntries[km][i].target, _longEntries[km][i].mac);
    }
  }
}

void PersistenceManager::saveKeymap()
{
  const char *namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int km = 0; km < 3; km++)
  {
    nvs_handle_t h;
    if (nvs_open(namespaces[km], NVS_READWRITE, &h) != ESP_OK)
      continue;
    for (int i = 0; i < 8; i++)
    {
      char key[8]; // prefix + up to 2 digits + null
      snprintf(key, sizeof(key), "s%d", i);
      nvs_set_u8(h, key, _shortEntries[km][i].key);
      snprintf(key, sizeof(key), "l%d", i);
      nvs_set_u8(h, key, _longEntries[km][i].key);
      snprintf(key, sizeof(key), "st%d", i);
      nvs_set_u8(h, key, (uint8_t)_shortEntries[km][i].target);
      snprintf(key, sizeof(key), "lt%d", i);
      nvs_set_u8(h, key, (uint8_t)_longEntries[km][i].target);
      snprintf(key, sizeof(key), "sm%d", i);
      nvs_set_str(h, key, _shortEntries[km][i].mac);
      snprintf(key, sizeof(key), "lm%d", i);
      nvs_set_str(h, key, _longEntries[km][i].mac);
    }
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] All keymaps saved to NVS.\n");
}

// ---------------------------------------------------------------------------
// NVS -- active keymap index (1, 2, or 3)
// ---------------------------------------------------------------------------
void PersistenceManager::_loadActiveKeymap()
{
  nvs_handle_t h;
  uint8_t saved = 1;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "activekm", &saved);
    nvs_close(h);
  }
  _activeKeymap = (saved >= 1 && saved <= 3) ? (int)saved : 1;
  if (DEBUG)
    printf("[CONFIG] Active keymap loaded: %d\n", _activeKeymap);
}

void PersistenceManager::setActiveKeymap(int slot)
{
  if (slot < 1 || slot > 3)
    return;
  _activeKeymap = slot;
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "activekm", (uint8_t)_activeKeymap);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Active keymap set to: %d\n", _activeKeymap);
}

// ---------------------------------------------------------------------------
// NVS -- BLE device name
// ---------------------------------------------------------------------------
void PersistenceManager::_loadBleName()
{
  nvs_handle_t h;
  if (nvs_open("config", NVS_READONLY, &h) == ESP_OK)
  {
    size_t len = sizeof(_bleName);
    if (nvs_get_str(h, "blename", _bleName, &len) != ESP_OK)
      strncpy(_bleName, DEFAULT_BLE_NAME, sizeof(_bleName) - 1);
    nvs_close(h);
  }
  else
  {
    strncpy(_bleName, DEFAULT_BLE_NAME, sizeof(_bleName) - 1);
  }
  _bleName[sizeof(_bleName) - 1] = '\0';
  if (DEBUG)
    printf("[CONFIG] BLE name loaded: %s\n", _bleName);
}

void PersistenceManager::saveBleName()
{
  nvs_handle_t h;
  if (nvs_open("config", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_str(h, "blename", _bleName);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] BLE name saved: %s\n", _bleName);
}

void PersistenceManager::setBleName(const char* name)
{
  strncpy(_bleName, name, sizeof(_bleName) - 1);
  _bleName[sizeof(_bleName) - 1] = '\0';
}

// ---------------------------------------------------------------------------
// NVS -- battery enable flag (default: disabled)
// ---------------------------------------------------------------------------
void PersistenceManager::_loadBatteryEnabled()
{
  nvs_handle_t h;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "baten", &flag);
    nvs_close(h);
  }
  _batteryEnabled = (flag != 0);
  if (DEBUG)
    printf("[CONFIG] Battery enabled: %s\n", _batteryEnabled ? "yes" : "no");
}

void PersistenceManager::saveBatteryEnabled()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "baten", _batteryEnabled ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Battery enabled saved: %s\n", _batteryEnabled ? "yes" : "no");
}

void PersistenceManager::_loadBLEPowerSaving()
{
  nvs_handle_t h;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "blepsen", &flag);
    nvs_close(h);
  }
  _blePowerSaving = (flag != 0);
  if (DEBUG)
    printf("[CONFIG] BLE power saving: %s\n", _blePowerSaving ? "yes" : "no");
}

void PersistenceManager::saveBLEPowerSaving()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "blepsen", _blePowerSaving ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] BLE power saving: %s\n", _blePowerSaving ? "yes" : "no");
}

void PersistenceManager::_loadMaxBLEConnections()
{
  nvs_handle_t h;
  uint8_t val = 1;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "maxbleconn", &val);
    nvs_close(h);
  }
  _maxBLEConnections = (val >= 1 && val <= 3) ? val : 1;
  if (DEBUG)
    printf("[CONFIG] Max BLE connections: %d\n", _maxBLEConnections);
}

void PersistenceManager::saveMaxBLEConnections()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "maxbleconn", _maxBLEConnections);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Max BLE connections saved: %d\n", _maxBLEConnections);
}

// ---------------------------------------------------------------------------
// NVS -- "clear bonds" flag
// ---------------------------------------------------------------------------
void PersistenceManager::requestClearBonds()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "clrbond", 1);
    nvs_commit(h);
    nvs_close(h);
  }
}

bool PersistenceManager::isClearBondsRequested()
{
  nvs_handle_t h;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "clrbond", &flag);
    nvs_close(h);
  }
  return flag != 0;
}

void PersistenceManager::clearClearBondsFlag()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_erase_key(h, "clrbond");
    nvs_commit(h);
    nvs_close(h);
  }
}

// ---------------------------------------------------------------------------
// Keymap accessors
// ---------------------------------------------------------------------------
const PersistenceManager::KeyEntry& PersistenceManager::getShortEntry(int idx) const
{
  static const KeyEntry empty{};
  int km = (_activeKeymap >= 1 && _activeKeymap <= 3) ? _activeKeymap - 1 : 0;
  return (idx >= 0 && idx < 8) ? _shortEntries[km][idx] : empty;
}

const PersistenceManager::KeyEntry& PersistenceManager::getLongEntry(int idx) const
{
  static const KeyEntry empty{};
  int km = (_activeKeymap >= 1 && _activeKeymap <= 3) ? _activeKeymap - 1 : 0;
  return (idx >= 0 && idx < 8) ? _longEntries[km][idx] : empty;
}

PersistenceManager::KeyEntry& PersistenceManager::rawShortEntry(int km, int idx)
{
  if (km < 0 || km >= 3) km = 0;
  if (idx < 0 || idx >= 8) idx = 0;
  return _shortEntries[km][idx];
}

PersistenceManager::KeyEntry& PersistenceManager::rawLongEntry(int km, int idx)
{
  if (km < 0 || km >= 3) km = 0;
  if (idx < 0 || idx >= 8) idx = 0;
  return _longEntries[km][idx];
}

int PersistenceManager::btnIndex(char key)
{
  if (key >= '1' && key <= '8')
    return key - '1';
  return -1;
}
