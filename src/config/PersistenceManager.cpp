#include "config/PersistenceManager.h"
#include <esp_err.h>
#include <cstdio>

// ---------------------------------------------------------------------------
// loadConfig -- read all settings from NVS into 'config' in one pass.
// ---------------------------------------------------------------------------
void PersistenceManager::loadConfig(Config &config)
{
  // --- Keymaps (3 slots x 8 buttons) ---
  const char *const namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int keymap = 0; keymap < 3; keymap++)
  {
    nvs_handle_t nvsHandle;
    bool opened = (nvs_open(namespaces[keymap], NVS_READONLY, &nvsHandle) == ESP_OK);
    for (int i = 0; i < 8; i++)
    {
      char nvsKey[8];

      snprintf(nvsKey, sizeof(nvsKey), "s%d", i);
      uint8_t value = Config::DEFAULT_SHORT[i];
      if (opened)
        nvs_get_u8(nvsHandle, nvsKey, &value);
      config.shortEntries[keymap][i].key = value;

      snprintf(nvsKey, sizeof(nvsKey), "l%d", i);
      value = Config::DEFAULT_LONG[i];
      if (opened)
        nvs_get_u8(nvsHandle, nvsKey, &value);
      config.longEntries[keymap][i].key = value;

      snprintf(nvsKey, sizeof(nvsKey), "st%d", i);
      uint8_t rawTarget = TARGET_SELECT;
      if (opened)
        nvs_get_u8(nvsHandle, nvsKey, &rawTarget);
      if (rawTarget > TARGET_IR_NEC)
        rawTarget = TARGET_SELECT;
      config.shortEntries[keymap][i].target = (KeyTarget)rawTarget;

      snprintf(nvsKey, sizeof(nvsKey), "lt%d", i);
      rawTarget = TARGET_SELECT;
      if (opened)
        nvs_get_u8(nvsHandle, nvsKey, &rawTarget);
      if (rawTarget > TARGET_IR_NEC)
        rawTarget = TARGET_SELECT;
      config.longEntries[keymap][i].target = (KeyTarget)rawTarget;

      snprintf(nvsKey, sizeof(nvsKey), "sm%d", i);
      config.shortEntries[keymap][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(config.shortEntries[keymap][i].mac);
        nvs_get_str(nvsHandle, nvsKey, config.shortEntries[keymap][i].mac, &macLen);
      }

      snprintf(nvsKey, sizeof(nvsKey), "lm%d", i);
      config.longEntries[keymap][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(config.longEntries[keymap][i].mac);
        nvs_get_str(nvsHandle, nvsKey, config.longEntries[keymap][i].mac, &macLen);
      }

      // --- IR NEC fields (short press) ---
      config.shortEntries[keymap][i].irAddress = 0;
      config.shortEntries[keymap][i].irCommand = 0;
      config.shortEntries[keymap][i].irRepeats = 1;
      if (opened)
      {
        snprintf(nvsKey, sizeof(nvsKey), "sia%d", i);
        nvs_get_u16(nvsHandle, nvsKey, &config.shortEntries[keymap][i].irAddress);
        snprintf(nvsKey, sizeof(nvsKey), "sic%d", i);
        nvs_get_u16(nvsHandle, nvsKey, &config.shortEntries[keymap][i].irCommand);
        snprintf(nvsKey, sizeof(nvsKey), "sir%d", i);
        nvs_get_u8(nvsHandle, nvsKey, &config.shortEntries[keymap][i].irRepeats);
      }
      if (config.shortEntries[keymap][i].irRepeats < 1)
        config.shortEntries[keymap][i].irRepeats = 1;
      if (config.shortEntries[keymap][i].irRepeats > IR_MAX_REPEATS)
        config.shortEntries[keymap][i].irRepeats = IR_MAX_REPEATS;

      // --- IR NEC fields (long press) ---
      config.longEntries[keymap][i].irAddress = 0;
      config.longEntries[keymap][i].irCommand = 0;
      config.longEntries[keymap][i].irRepeats = 1;
      if (opened)
      {
        snprintf(nvsKey, sizeof(nvsKey), "lia%d", i);
        nvs_get_u16(nvsHandle, nvsKey, &config.longEntries[keymap][i].irAddress);
        snprintf(nvsKey, sizeof(nvsKey), "lic%d", i);
        nvs_get_u16(nvsHandle, nvsKey, &config.longEntries[keymap][i].irCommand);
        snprintf(nvsKey, sizeof(nvsKey), "lir%d", i);
        nvs_get_u8(nvsHandle, nvsKey, &config.longEntries[keymap][i].irRepeats);
      }
      if (config.longEntries[keymap][i].irRepeats < 1)
        config.longEntries[keymap][i].irRepeats = 1;
      if (config.longEntries[keymap][i].irRepeats > IR_MAX_REPEATS)
        config.longEntries[keymap][i].irRepeats = IR_MAX_REPEATS;
    }
    if (opened)
      nvs_close(nvsHandle);
  }
  if (DEBUG)
  {
    printf("[CONFIG] Keymaps loaded from NVS:\n");
    for (int keymap = 0; keymap < 3; keymap++)
    {
      printf("[CONFIG]   Keymap %d:\n", keymap + 1);
      for (int i = 0; i < 8; i++)
        printf("[CONFIG]     btn%d  short=%d(tgt=%d mac=%s)  long=%d(tgt=%d mac=%s)\n",
               i + 1,
               config.shortEntries[keymap][i].key, (int)config.shortEntries[keymap][i].target, config.shortEntries[keymap][i].mac,
               config.longEntries[keymap][i].key, (int)config.longEntries[keymap][i].target, config.longEntries[keymap][i].mac);
    }
  }

  // --- Combos (3 slots x up to MAX_COMBOS entries) ---
  {
    const char *const comboNs[3] = {"combos", "combos2", "combos3"};
    for (int keymap = 0; keymap < 3; keymap++)
    {
      nvs_handle_t nvsHandle;
      bool opened = (nvs_open(comboNs[keymap], NVS_READONLY, &nvsHandle) == ESP_OK);
      uint8_t count = 0;
      if (opened) nvs_get_u8(nvsHandle, "cc", &count);
      if (count > Config::MAX_COMBOS) count = Config::MAX_COMBOS;
      config.comboCounts[keymap] = count;
      for (int j = 0; j < count; j++)
      {
        char nvsKey[8];
        ComboEntry &c = config.comboEntries[keymap][j];

        uint8_t heldVal = 0;
        snprintf(nvsKey, sizeof(nvsKey), "ch%d", j);
        if (opened) nvs_get_u8(nvsHandle, nvsKey, &heldVal);
        c.held = (char)heldVal;

        uint8_t pressedVal = 0;
        snprintf(nvsKey, sizeof(nvsKey), "cp%d", j);
        if (opened) nvs_get_u8(nvsHandle, nvsKey, &pressedVal);
        c.pressed = (char)pressedVal;

        c.key = 0;
        snprintf(nvsKey, sizeof(nvsKey), "ck%d", j);
        if (opened) nvs_get_u8(nvsHandle, nvsKey, &c.key);

        uint8_t rawTarget = TARGET_SELECT;
        snprintf(nvsKey, sizeof(nvsKey), "ct%d", j);
        if (opened) nvs_get_u8(nvsHandle, nvsKey, &rawTarget);
        if (rawTarget > TARGET_IR_NEC) rawTarget = TARGET_SELECT;
        c.target = (KeyTarget)rawTarget;

        c.mac[0] = '\0';
        snprintf(nvsKey, sizeof(nvsKey), "cm%d", j);
        if (opened)
        {
          size_t macLen = sizeof(c.mac);
          nvs_get_str(nvsHandle, nvsKey, c.mac, &macLen);
        }

        // --- IR NEC fields (combo) ---
        c.irAddress = 0;
        c.irCommand = 0;
        c.irRepeats = 1;
        if (opened)
        {
          snprintf(nvsKey, sizeof(nvsKey), "cia%d", j);
          nvs_get_u16(nvsHandle, nvsKey, &c.irAddress);
          snprintf(nvsKey, sizeof(nvsKey), "cic%d", j);
          nvs_get_u16(nvsHandle, nvsKey, &c.irCommand);
          snprintf(nvsKey, sizeof(nvsKey), "cir%d", j);
          nvs_get_u8(nvsHandle, nvsKey, &c.irRepeats);
        }
        if (c.irRepeats < 1)
          c.irRepeats = 1;
        if (c.irRepeats > IR_MAX_REPEATS)
          c.irRepeats = IR_MAX_REPEATS;
      }
      if (opened) nvs_close(nvsHandle);
    }
  }
  if (DEBUG)
  {
    for (int keymap = 0; keymap < 3; keymap++)
      printf("[CONFIG] Combos keymap %d: count=%d\n", keymap + 1, config.comboCounts[keymap]);
  }

  // --- Active keymap index ---
  {
    nvs_handle_t nvsHandle;
    uint8_t savedSlot = 1;
    if (nvs_open("sys", NVS_READONLY, &nvsHandle) == ESP_OK)
    {
      nvs_get_u8(nvsHandle, "activekm", &savedSlot);
      nvs_close(nvsHandle);
    }
    config.activeKeymap = (savedSlot >= 1 && savedSlot <= 3) ? (int)savedSlot : 1;
  }
  if (DEBUG)
    printf("[CONFIG] Active keymap loaded: %d\n", config.activeKeymap);

  // --- BLE name ---
  strncpy(config.bleName, DEFAULT_BLE_NAME, sizeof(config.bleName) - 1);
  config.bleName[sizeof(config.bleName) - 1] = '\0';
  {
    nvs_handle_t nvsHandle;
    if (nvs_open("config", NVS_READONLY, &nvsHandle) == ESP_OK)
    {
      size_t len = sizeof(config.bleName);
      nvs_get_str(nvsHandle, "blename", config.bleName, &len);
      nvs_close(nvsHandle);
    }
  }
  if (DEBUG)
    printf("[CONFIG] BLE name loaded: %s\n", config.bleName);

  // --- Flags from "sys" namespace ---
  {
    nvs_handle_t nvsHandle;
    if (nvs_open("sys", NVS_READONLY, &nvsHandle) == ESP_OK)
    {
      uint8_t flag = 0;
      nvs_get_u8(nvsHandle, "baten", &flag);
      config.batteryEnabled = (flag != 0);

      flag = 0;
      nvs_get_u8(nvsHandle, "blepsen", &flag);
      config.blePowerSaving = (flag != 0);

      uint8_t value = 1;
      nvs_get_u8(nvsHandle, "maxbleconn", &value);
      config.maxBLEConnections = (value >= 1 && value <= 3) ? value : 1;

      nvs_close(nvsHandle);
    }
  }
  if (DEBUG)
  {
    printf("[CONFIG] Battery enabled: %s\n", config.batteryEnabled ? "yes" : "no");
    printf("[CONFIG] BLE power saving: %s\n", config.blePowerSaving ? "yes" : "no");
    printf("[CONFIG] Max BLE connections: %d\n", config.maxBLEConnections);
  }
}

// ---------------------------------------------------------------------------
// saveConfig -- write all settings from 'config' to NVS in one pass.
// ---------------------------------------------------------------------------
void PersistenceManager::saveConfig(const Config &config)
{
  printf("[CONFIG] saveConfig: starting\n");

  // --- Keymaps ---
  const char *const namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int keymap = 0; keymap < 3; keymap++)
  {
    nvs_handle_t nvsHandle;
    esp_err_t kmErr = nvs_open(namespaces[keymap], NVS_READWRITE, &nvsHandle);
    if (kmErr != ESP_OK)
    {
      printf("[CONFIG] saveConfig: nvs_open('%s') failed: %s\n", namespaces[keymap], esp_err_to_name(kmErr));
      continue;
    }
    auto nvsSet8  = [&](const char *k, uint8_t v)  { esp_err_t e = nvs_set_u8( nvsHandle, k, v); if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_u8  '%s' in '%s' failed: %s\n", k, namespaces[keymap], esp_err_to_name(e)); };
    auto nvsSet16 = [&](const char *k, uint16_t v) { esp_err_t e = nvs_set_u16(nvsHandle, k, v); if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_u16 '%s' in '%s' failed: %s\n", k, namespaces[keymap], esp_err_to_name(e)); };
    auto nvsSetStr= [&](const char *k, const char *v) { esp_err_t e = nvs_set_str(nvsHandle, k, v); if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_str '%s' in '%s' failed: %s\n", k, namespaces[keymap], esp_err_to_name(e)); };
    for (int i = 0; i < 8; i++)
    {
      char nvsKey[8];
      snprintf(nvsKey, sizeof(nvsKey), "s%d",  i); nvsSet8 (nvsKey, config.shortEntries[keymap][i].key);
      snprintf(nvsKey, sizeof(nvsKey), "l%d",  i); nvsSet8 (nvsKey, config.longEntries[keymap][i].key);
      snprintf(nvsKey, sizeof(nvsKey), "st%d", i); nvsSet8 (nvsKey, (uint8_t)config.shortEntries[keymap][i].target);
      snprintf(nvsKey, sizeof(nvsKey), "lt%d", i); nvsSet8 (nvsKey, (uint8_t)config.longEntries[keymap][i].target);
      snprintf(nvsKey, sizeof(nvsKey), "sm%d", i); nvsSetStr(nvsKey, config.shortEntries[keymap][i].mac);
      snprintf(nvsKey, sizeof(nvsKey), "lm%d", i); nvsSetStr(nvsKey, config.longEntries[keymap][i].mac);
      // IR NEC fields
      snprintf(nvsKey, sizeof(nvsKey), "sia%d", i); nvsSet16(nvsKey, config.shortEntries[keymap][i].irAddress);
      snprintf(nvsKey, sizeof(nvsKey), "sic%d", i); nvsSet16(nvsKey, config.shortEntries[keymap][i].irCommand);
      snprintf(nvsKey, sizeof(nvsKey), "sir%d", i); nvsSet8 (nvsKey, config.shortEntries[keymap][i].irRepeats);
      snprintf(nvsKey, sizeof(nvsKey), "lia%d", i); nvsSet16(nvsKey, config.longEntries[keymap][i].irAddress);
      snprintf(nvsKey, sizeof(nvsKey), "lic%d", i); nvsSet16(nvsKey, config.longEntries[keymap][i].irCommand);
      snprintf(nvsKey, sizeof(nvsKey), "lir%d", i); nvsSet8 (nvsKey, config.longEntries[keymap][i].irRepeats);
    }
    esp_err_t kmCommitErr = nvs_commit(nvsHandle);
    if (kmCommitErr != ESP_OK)
      printf("[CONFIG] saveConfig: nvs_commit('%s') failed: %s\n", namespaces[keymap], esp_err_to_name(kmCommitErr));
    nvs_close(nvsHandle);
  }
  if (DEBUG)
    printf("[CONFIG] Keymaps saved to NVS.\n");

  // --- Combos ---
  {
    const char *const comboNs[3] = {"combos", "combos2", "combos3"};
    for (int keymap = 0; keymap < 3; keymap++)
    {
      nvs_handle_t nvsHandle;
      esp_err_t cErr = nvs_open(comboNs[keymap], NVS_READWRITE, &nvsHandle);
      if (cErr != ESP_OK)
      {
        printf("[CONFIG] saveConfig: nvs_open('%s') failed: %s\n", comboNs[keymap], esp_err_to_name(cErr));
        continue;
      }
      auto nvsSet8  = [&](const char *k, uint8_t v)     { esp_err_t e = nvs_set_u8( nvsHandle, k, v); if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_u8  '%s' in '%s' failed: %s\n", k, comboNs[keymap], esp_err_to_name(e)); };
      auto nvsSet16 = [&](const char *k, uint16_t v)    { esp_err_t e = nvs_set_u16(nvsHandle, k, v); if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_u16 '%s' in '%s' failed: %s\n", k, comboNs[keymap], esp_err_to_name(e)); };
      auto nvsSetStr= [&](const char *k, const char *v) { esp_err_t e = nvs_set_str(nvsHandle, k, v); if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_str '%s' in '%s' failed: %s\n", k, comboNs[keymap], esp_err_to_name(e)); };
      nvsSet8("cc", config.comboCounts[keymap]);
      for (int j = 0; j < config.comboCounts[keymap]; j++)
      {
        char nvsKey[8];
        const ComboEntry &c = config.comboEntries[keymap][j];
        snprintf(nvsKey, sizeof(nvsKey), "ch%d", j); nvsSet8 (nvsKey, (uint8_t)c.held);
        snprintf(nvsKey, sizeof(nvsKey), "cp%d", j); nvsSet8 (nvsKey, (uint8_t)c.pressed);
        snprintf(nvsKey, sizeof(nvsKey), "ck%d", j); nvsSet8 (nvsKey, c.key);
        snprintf(nvsKey, sizeof(nvsKey), "ct%d", j); nvsSet8 (nvsKey, (uint8_t)c.target);
        snprintf(nvsKey, sizeof(nvsKey), "cm%d", j); nvsSetStr(nvsKey, c.mac);
        // IR NEC fields
        snprintf(nvsKey, sizeof(nvsKey), "cia%d", j); nvsSet16(nvsKey, c.irAddress);
        snprintf(nvsKey, sizeof(nvsKey), "cic%d", j); nvsSet16(nvsKey, c.irCommand);
        snprintf(nvsKey, sizeof(nvsKey), "cir%d", j); nvsSet8 (nvsKey, c.irRepeats);
      }
      esp_err_t cCommitErr = nvs_commit(nvsHandle);
      if (cCommitErr != ESP_OK)
        printf("[CONFIG] saveConfig: nvs_commit('%s') failed: %s\n", comboNs[keymap], esp_err_to_name(cCommitErr));
      nvs_close(nvsHandle);
    }
  }
  if (DEBUG)
    printf("[CONFIG] Combos saved to NVS.\n");

  // --- BLE name ---
  {
    nvs_handle_t nvsHandle;
    esp_err_t cfgErr = nvs_open("config", NVS_READWRITE, &nvsHandle);
    if (cfgErr == ESP_OK)
    {
      esp_err_t e = nvs_set_str(nvsHandle, "blename", config.bleName);
      if (e != ESP_OK)
        printf("[CONFIG] saveConfig: nvs_set_str 'blename' in 'config' failed: %s\n", esp_err_to_name(e));
      esp_err_t cfgCommitErr = nvs_commit(nvsHandle);
      if (cfgCommitErr != ESP_OK)
        printf("[CONFIG] saveConfig: nvs_commit('config') failed: %s\n", esp_err_to_name(cfgCommitErr));
      nvs_close(nvsHandle);
    }
    else
    {
      printf("[CONFIG] saveConfig: nvs_open('config') failed: %s\n", esp_err_to_name(cfgErr));
    }
  }
  if (DEBUG)
    printf("[CONFIG] BLE name saved: %s\n", config.bleName);

  // --- Active keymap + flags ---
  {
    nvs_handle_t nvsHandle;
    esp_err_t sysErr = nvs_open("sys", NVS_READWRITE, &nvsHandle);
    if (sysErr == ESP_OK)
    {
      auto logErr = [](const char *k, esp_err_t e) {
        if (e != ESP_OK) printf("[CONFIG] saveConfig: nvs_set_u8 '%s' in 'sys' failed: %s\n", k, esp_err_to_name(e));
      };
      logErr("activekm",   nvs_set_u8(nvsHandle, "activekm",   (uint8_t)config.activeKeymap));
      logErr("baten",      nvs_set_u8(nvsHandle, "baten",      config.batteryEnabled  ? 1 : 0));
      logErr("blepsen",    nvs_set_u8(nvsHandle, "blepsen",    config.blePowerSaving  ? 1 : 0));
      logErr("maxbleconn", nvs_set_u8(nvsHandle, "maxbleconn", config.maxBLEConnections));
      esp_err_t sysCommitErr = nvs_commit(nvsHandle);
      if (sysCommitErr != ESP_OK)
        printf("[CONFIG] saveConfig: nvs_commit('sys') failed: %s\n", esp_err_to_name(sysCommitErr));
      nvs_close(nvsHandle);
    }
    else
    {
      printf("[CONFIG] saveConfig: nvs_open('sys') failed: %s\n", esp_err_to_name(sysErr));
    }
  }
  if (DEBUG)
  {
    printf("[CONFIG] Active keymap saved: %d\n",    config.activeKeymap);
    printf("[CONFIG] Battery enabled saved: %s\n",  config.batteryEnabled  ? "yes" : "no");
    printf("[CONFIG] BLE power saving saved: %s\n", config.blePowerSaving  ? "yes" : "no");
    printf("[CONFIG] Max BLE connections saved: %d\n", config.maxBLEConnections);
  }
}

// ---------------------------------------------------------------------------
// saveActiveKeymap -- lightweight runtime save (button-combo keymap switch).
// ---------------------------------------------------------------------------
void PersistenceManager::saveActiveKeymap(int slot)
{
  nvs_handle_t nvsHandle;
  if (nvs_open("sys", NVS_READWRITE, &nvsHandle) == ESP_OK)
  {
    nvs_set_u8(nvsHandle, "activekm", (uint8_t)slot);
    nvs_commit(nvsHandle);
    nvs_close(nvsHandle);
  }
  if (DEBUG)
    printf("[CONFIG] Active keymap saved: %d\n", slot);
}

// ---------------------------------------------------------------------------
// Clear-bonds flag
// ---------------------------------------------------------------------------
void PersistenceManager::requestClearBonds()
{
  nvs_handle_t nvsHandle;
  if (nvs_open("sys", NVS_READWRITE, &nvsHandle) == ESP_OK)
  {
    nvs_set_u8(nvsHandle, "clrbond", 1);
    nvs_commit(nvsHandle);
    nvs_close(nvsHandle);
  }
}

bool PersistenceManager::isClearBondsRequested() const
{
  nvs_handle_t nvsHandle;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &nvsHandle) == ESP_OK)
  {
    nvs_get_u8(nvsHandle, "clrbond", &flag);
    nvs_close(nvsHandle);
  }
  return flag != 0;
}

void PersistenceManager::clearClearBondsFlag()
{
  nvs_handle_t nvsHandle;
  if (nvs_open("sys", NVS_READWRITE, &nvsHandle) == ESP_OK)
  {
    nvs_erase_key(nvsHandle, "clrbond");
    nvs_commit(nvsHandle);
    nvs_close(nvsHandle);
  }
}
