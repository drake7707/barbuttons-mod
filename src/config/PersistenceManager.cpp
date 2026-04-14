#include "config/PersistenceManager.h"

// ---------------------------------------------------------------------------
// loadConfig -- read all settings from NVS into 'config' in one pass.
// ---------------------------------------------------------------------------
void PersistenceManager::loadConfig(Config &c)
{
  // --- Keymaps (3 slots x 8 buttons) ---
  const char *namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int km = 0; km < 3; km++)
  {
    nvs_handle_t h;
    bool opened = (nvs_open(namespaces[km], NVS_READONLY, &h) == ESP_OK);
    for (int i = 0; i < 8; i++)
    {
      char key[8];

      snprintf(key, sizeof(key), "s%d", i);
      uint8_t v = Config::DEFAULT_SHORT[i];
      if (opened)
        nvs_get_u8(h, key, &v);
      c.shortEntries[km][i].key = v;

      snprintf(key, sizeof(key), "l%d", i);
      v = Config::DEFAULT_LONG[i];
      if (opened)
        nvs_get_u8(h, key, &v);
      c.longEntries[km][i].key = v;

      snprintf(key, sizeof(key), "st%d", i);
      uint8_t tgt = TARGET_SELECT;
      if (opened)
        nvs_get_u8(h, key, &tgt);
      if (tgt > TARGET_BTHOME)
        tgt = TARGET_SELECT;
      c.shortEntries[km][i].target = (KeyTarget)tgt;

      snprintf(key, sizeof(key), "lt%d", i);
      tgt = TARGET_SELECT;
      if (opened)
        nvs_get_u8(h, key, &tgt);
      if (tgt > TARGET_BTHOME)
        tgt = TARGET_SELECT;
      c.longEntries[km][i].target = (KeyTarget)tgt;

      snprintf(key, sizeof(key), "sm%d", i);
      c.shortEntries[km][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(c.shortEntries[km][i].mac);
        nvs_get_str(h, key, c.shortEntries[km][i].mac, &macLen);
      }

      snprintf(key, sizeof(key), "lm%d", i);
      c.longEntries[km][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(c.longEntries[km][i].mac);
        nvs_get_str(h, key, c.longEntries[km][i].mac, &macLen);
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
               c.shortEntries[km][i].key, (int)c.shortEntries[km][i].target, c.shortEntries[km][i].mac,
               c.longEntries[km][i].key, (int)c.longEntries[km][i].target, c.longEntries[km][i].mac);
    }
  }

  // --- Active keymap index ---
  {
    nvs_handle_t h;
    uint8_t saved = 1;
    if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
    {
      nvs_get_u8(h, "activekm", &saved);
      nvs_close(h);
    }
    c.activeKeymap = (saved >= 1 && saved <= 3) ? (int)saved : 1;
  }
  if (DEBUG)
    printf("[CONFIG] Active keymap loaded: %d\n", c.activeKeymap);

  // --- BLE name ---
  strncpy(c.bleName, DEFAULT_BLE_NAME, sizeof(c.bleName) - 1);
  c.bleName[sizeof(c.bleName) - 1] = '\0';
  {
    nvs_handle_t h;
    if (nvs_open("config", NVS_READONLY, &h) == ESP_OK)
    {
      size_t len = sizeof(c.bleName);
      nvs_get_str(h, "blename", c.bleName, &len);
      nvs_close(h);
    }
  }
  if (DEBUG)
    printf("[CONFIG] BLE name loaded: %s\n", c.bleName);

  // --- Flags from "sys" namespace ---
  {
    nvs_handle_t h;
    if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
    {
      uint8_t flag = 0;
      nvs_get_u8(h, "baten", &flag);
      c.batteryEnabled = (flag != 0);

      flag = 0;
      nvs_get_u8(h, "blepsen", &flag);
      c.blePowerSaving = (flag != 0);

      uint8_t val = 1;
      nvs_get_u8(h, "maxbleconn", &val);
      c.maxBLEConnections = (val >= 1 && val <= 3) ? val : 1;

      nvs_close(h);
    }
  }
  if (DEBUG)
  {
    printf("[CONFIG] Battery enabled: %s\n", c.batteryEnabled ? "yes" : "no");
    printf("[CONFIG] BLE power saving: %s\n", c.blePowerSaving ? "yes" : "no");
    printf("[CONFIG] Max BLE connections: %d\n", c.maxBLEConnections);
  }
}

// ---------------------------------------------------------------------------
// saveConfig -- write all settings from 'config' to NVS in one pass.
// ---------------------------------------------------------------------------
void PersistenceManager::saveConfig(const Config &c)
{
  // --- Keymaps ---
  const char *namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int km = 0; km < 3; km++)
  {
    nvs_handle_t h;
    if (nvs_open(namespaces[km], NVS_READWRITE, &h) != ESP_OK)
      continue;
    for (int i = 0; i < 8; i++)
    {
      char key[8];
      snprintf(key, sizeof(key), "s%d", i);
      nvs_set_u8(h, key, c.shortEntries[km][i].key);
      snprintf(key, sizeof(key), "l%d", i);
      nvs_set_u8(h, key, c.longEntries[km][i].key);
      snprintf(key, sizeof(key), "st%d", i);
      nvs_set_u8(h, key, (uint8_t)c.shortEntries[km][i].target);
      snprintf(key, sizeof(key), "lt%d", i);
      nvs_set_u8(h, key, (uint8_t)c.longEntries[km][i].target);
      snprintf(key, sizeof(key), "sm%d", i);
      nvs_set_str(h, key, c.shortEntries[km][i].mac);
      snprintf(key, sizeof(key), "lm%d", i);
      nvs_set_str(h, key, c.longEntries[km][i].mac);
    }
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Keymaps saved to NVS.\n");

  // --- BLE name ---
  {
    nvs_handle_t h;
    if (nvs_open("config", NVS_READWRITE, &h) == ESP_OK)
    {
      nvs_set_str(h, "blename", c.bleName);
      nvs_commit(h);
      nvs_close(h);
    }
  }
  if (DEBUG)
    printf("[CONFIG] BLE name saved: %s\n", c.bleName);

  // --- Active keymap + flags ---
  {
    nvs_handle_t h;
    if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
    {
      nvs_set_u8(h, "activekm", (uint8_t)c.activeKeymap);
      nvs_set_u8(h, "baten", c.batteryEnabled ? 1 : 0);
      nvs_set_u8(h, "blepsen", c.blePowerSaving ? 1 : 0);
      nvs_set_u8(h, "maxbleconn", c.maxBLEConnections);
      nvs_commit(h);
      nvs_close(h);
    }
  }
  if (DEBUG)
  {
    printf("[CONFIG] Active keymap saved: %d\n", c.activeKeymap);
    printf("[CONFIG] Battery enabled saved: %s\n", c.batteryEnabled ? "yes" : "no");
    printf("[CONFIG] BLE power saving saved: %s\n", c.blePowerSaving ? "yes" : "no");
    printf("[CONFIG] Max BLE connections saved: %d\n", c.maxBLEConnections);
  }
}

// ---------------------------------------------------------------------------
// saveActiveKeymap -- lightweight runtime save (button-combo keymap switch).
// ---------------------------------------------------------------------------
void PersistenceManager::saveActiveKeymap(int slot)
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "activekm", (uint8_t)slot);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Active keymap saved: %d\n", slot);
}

// ---------------------------------------------------------------------------
// Clear-bonds flag
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
