#include "config/ConfigManager.h"

void ConfigManager::begin(const char *firmwareVersion)
{
  _webUI.begin(firmwareVersion);
}

void ConfigManager::loadConfig()
{
  _persistence.loadConfig(_config);
}

void ConfigManager::saveConfig()
{
  _persistence.saveConfig(_config);
}

void ConfigManager::setActiveKeymap(int slot)
{
  if (slot < 1 || slot > 3)
    return;
  _config.activeKeymap = slot;
  _persistence.saveActiveKeymap(slot);
}

const KeyEntry &ConfigManager::getShortEntry(int buttonIndex) const
{
  static const KeyEntry empty{};
  int keymap = (_config.activeKeymap >= 1 && _config.activeKeymap <= 3) ? _config.activeKeymap - 1 : 0;
  return (buttonIndex >= 0 && buttonIndex < 8) ? _config.shortEntries[keymap][buttonIndex] : empty;
}

const KeyEntry &ConfigManager::getLongEntry(int buttonIndex) const
{
  static const KeyEntry empty{};
  int keymap = (_config.activeKeymap >= 1 && _config.activeKeymap <= 3) ? _config.activeKeymap - 1 : 0;
  return (buttonIndex >= 0 && buttonIndex < 8) ? _config.longEntries[keymap][buttonIndex] : empty;
}

const KeyEntry &ConfigManager::rawShortEntry(int keymap, int buttonIndex) const
{
  static const KeyEntry empty{};
  if (keymap < 0 || keymap >= 3 || buttonIndex < 0 || buttonIndex >= 8)
    return empty;
  return _config.shortEntries[keymap][buttonIndex];
}

const KeyEntry &ConfigManager::rawLongEntry(int keymap, int buttonIndex) const
{
  static const KeyEntry empty{};
  if (keymap < 0 || keymap >= 3 || buttonIndex < 0 || buttonIndex >= 8)
    return empty;
  return _config.longEntries[keymap][buttonIndex];
}

KeyEntry &ConfigManager::rawShortEntry(int keymap, int buttonIndex)
{
  if (keymap < 0 || keymap >= 3)     keymap      = 0;
  if (buttonIndex < 0 || buttonIndex >= 8) buttonIndex = 0;
  return _config.shortEntries[keymap][buttonIndex];
}

KeyEntry &ConfigManager::rawLongEntry(int keymap, int buttonIndex)
{
  if (keymap < 0 || keymap >= 3)     keymap      = 0;
  if (buttonIndex < 0 || buttonIndex >= 8) buttonIndex = 0;
  return _config.longEntries[keymap][buttonIndex];
}

void ConfigManager::setBleName(const char *name)
{
  strncpy(_config.bleName, name, sizeof(_config.bleName) - 1);
  _config.bleName[sizeof(_config.bleName) - 1] = '\0';
}

void ConfigManager::setBatteryEnabled(bool enabled)
{
  _config.batteryEnabled = enabled;
}

void ConfigManager::setBlePowerSaving(bool enabled)
{
  _config.blePowerSaving = enabled;
}

void ConfigManager::setMaxBLEConnections(uint8_t maxConnections)
{
  _config.maxBLEConnections = maxConnections;
}

void ConfigManager::beginConfigAP(const std::vector<std::string> &bondList,
                                   int batVoltageMv, int batPercent)
{
  _webUI.beginConfigAP(this, bondList, batVoltageMv, batPercent);
}
