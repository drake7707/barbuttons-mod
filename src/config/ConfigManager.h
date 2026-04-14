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
  // The full runtime configuration state.
  Config config;

  // Sub-managers.
  PersistenceManager persistence;
  WebUIConfigManager webUI;

  // Inject the StatusLedManager and firmware version string (passed to webUI).
  void begin(StatusLedManager *led, const char *firmwareVersion);

  // Load all settings from NVS into 'config'.
  void loadConfig();

  // Persist the full config to NVS (called by the web UI save handler).
  void saveConfig();

  // ---------------------------------------------------------------------------
  // Active keymap
  // ---------------------------------------------------------------------------
  void setActiveKeymap(int slot); // updates config + lightweight NVS write
  int getActiveKeymap() const { return config.activeKeymap; }

  // ---------------------------------------------------------------------------
  // Keymap read accessors (use the active keymap slot)
  // ---------------------------------------------------------------------------
  const KeyEntry &getShortEntry(int buttonIndex) const;
  const KeyEntry &getLongEntry(int buttonIndex) const;

  // Mutable access across all three keymap slots (used by web save handler).
  KeyEntry &rawShortEntry(int keymap, int buttonIndex);
  KeyEntry &rawLongEntry(int keymap, int buttonIndex);

  // ---------------------------------------------------------------------------
  // Convenience getters for the rest of the config
  // ---------------------------------------------------------------------------
  const char *getBleName() const { return config.bleName; }
  bool isBatteryEnabled() const { return config.batteryEnabled; }
  bool allowBLEPowerSaving() const { return config.blePowerSaving; }
  uint8_t getMaxBLEConnections() const { return config.maxBLEConnections; }

  // ---------------------------------------------------------------------------
  // Clear-bonds flag (NVS only, no config value)
  // ---------------------------------------------------------------------------
  void requestClearBonds() { persistence.requestClearBonds(); }
  bool isClearBondsRequested() { return persistence.isClearBondsRequested(); }
  void clearClearBondsFlag() { persistence.clearClearBondsFlag(); }

  // ---------------------------------------------------------------------------
  // Web UI / AP
  // ---------------------------------------------------------------------------
  void beginConfigAP(const std::vector<std::string> &bondList,
                     int batVoltageMv = -1, int batPercent = -1);
  void handleClient() { webUI.handleClient(); }
  void endConfigAP() { webUI.endConfigAP(); }
  bool isExitRequested() const { return webUI.isExitRequested(); }
  void setExitRequested(bool v) { webUI.setExitRequested(v); }
};
