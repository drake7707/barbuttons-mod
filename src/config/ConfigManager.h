#pragma once

#include <vector>
#include <string>
#include "config/Config.h"
#include "config/PersistenceManager.h"
#include "config/WebUIConfigManager.h"

// ---------------------------------------------------------------------------
// ConfigManager -- owns the live Config state and ties PersistenceManager
// (NVS I/O) and WebUIConfigManager (AP + web server) together.
// ---------------------------------------------------------------------------

class ConfigManager
{
public:
  
  void begin(const char *firmwareVersion);

  // Load all settings from NVS into the internal config state.
  void loadConfig();

  // Persist the full config to NVS (called by the web UI save handler).
  void saveConfig();

  // ---------------------------------------------------------------------------
  // Active keymap
  // ---------------------------------------------------------------------------
  void setActiveKeymap(int slot); // updates config + lightweight NVS write
  int  getActiveKeymap() const { return _config.activeKeymap; }

  // ---------------------------------------------------------------------------
  // Keymap read accessors (use the active keymap slot)
  // ---------------------------------------------------------------------------
  const KeyEntry &getShortEntry(int buttonIndex) const;
  const KeyEntry &getLongEntry(int buttonIndex) const;

  // Keymap access across all three slots for the web save handler.
  const KeyEntry &rawShortEntry(int keymap, int buttonIndex) const;
  const KeyEntry &rawLongEntry(int keymap, int buttonIndex) const;
  KeyEntry       &rawShortEntry(int keymap, int buttonIndex);
  KeyEntry       &rawLongEntry(int keymap, int buttonIndex);

  // ---------------------------------------------------------------------------
  // Getters for the rest of the config
  // ---------------------------------------------------------------------------
  const char *getBleName()           const { return _config.bleName; }
  bool        isBatteryEnabled()     const { return _config.batteryEnabled; }
  bool        allowBLEPowerSaving()  const { return _config.blePowerSaving; }
  uint8_t     getMaxBLEConnections() const { return _config.maxBLEConnections; }

  // ---------------------------------------------------------------------------
  // Setters (used by the web save handler to update in-memory config)
  // ---------------------------------------------------------------------------
  void setBleName(const char *name);
  void setBatteryEnabled(bool enabled);
  void setBlePowerSaving(bool enabled);
  void setMaxBLEConnections(uint8_t maxConnections);

  // ---------------------------------------------------------------------------
  // Clear-bonds flag (NVS only, no config value)
  // ---------------------------------------------------------------------------
  void requestClearBonds()     { _persistence.requestClearBonds(); }
  bool isClearBondsRequested() const { return _persistence.isClearBondsRequested(); }
  void clearClearBondsFlag()   { _persistence.clearClearBondsFlag(); }

  // ---------------------------------------------------------------------------
  // Web UI / AP
  // ---------------------------------------------------------------------------
  void beginConfigAP(const std::vector<std::string> &bondList,
                     int batVoltageMv = -1, int batPercent = -1);
  void handleClient()           { _webUI.handleClient(); }
  void endConfigAP()            { _webUI.endConfigAP(); }
  bool isExitRequested() const  { return _webUI.isExitRequested(); }
  void setExitRequested(bool requested) { _webUI.setExitRequested(requested); }

private:
  Config             _config;
  PersistenceManager _persistence;
  WebUIConfigManager _webUI;
};
