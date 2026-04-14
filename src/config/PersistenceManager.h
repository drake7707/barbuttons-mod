#pragma once

#include <nvs_flash.h>
#include <nvs.h>
#include "config/Config.h"

extern const int DEBUG;
extern const bool LEGACY;

// ---------------------------------------------------------------------------
// PersistenceManager -- stateless NVS I/O layer.
// Reads/writes a Config struct to/from NVS.  No in-memory state is kept here.
// ---------------------------------------------------------------------------

class PersistenceManager
{
public:
  // Load all settings from NVS into 'config'.
  void loadConfig(Config &config);

  // Persist the full config to NVS (used after the web UI save handler).
  void saveConfig(const Config &config);

  // Lightweight save of only the active keymap index (used at runtime when
  // the user switches keymaps via button combo, without a full config save).
  void saveActiveKeymap(int slot);

  // ---------------------------------------------------------------------------
  // NVS -- "clear bonds" flag (no associated Config value; inherently stateless)
  // ---------------------------------------------------------------------------
  void requestClearBonds();
  bool isClearBondsRequested();
  void clearClearBondsFlag();
};
