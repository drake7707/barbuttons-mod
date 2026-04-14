#include "config/ConfigManager.h"

void ConfigManager::begin(StatusLedManager *led, const char *firmwareVersion)
{
  webUI.begin(led, firmwareVersion);
}

void ConfigManager::loadConfig()
{
  persistence.loadConfig(config);
}

void ConfigManager::saveConfig()
{
  persistence.saveConfig(config);
}

void ConfigManager::setActiveKeymap(int slot)
{
  if (slot < 1 || slot > 3)
    return;
  config.activeKeymap = slot;
  persistence.saveActiveKeymap(slot);
}

const KeyEntry &ConfigManager::getShortEntry(int buttonIndex) const
{
  static const KeyEntry empty{};
  int km = (config.activeKeymap >= 1 && config.activeKeymap <= 3) ? config.activeKeymap - 1 : 0;
  return (buttonIndex >= 0 && buttonIndex < 8) ? config.shortEntries[km][buttonIndex] : empty;
}

const KeyEntry &ConfigManager::getLongEntry(int buttonIndex) const
{
  static const KeyEntry empty{};
  int km = (config.activeKeymap >= 1 && config.activeKeymap <= 3) ? config.activeKeymap - 1 : 0;
  return (buttonIndex >= 0 && buttonIndex < 8) ? config.longEntries[km][buttonIndex] : empty;
}

KeyEntry &ConfigManager::rawShortEntry(int keymap, int buttonIndex)
{
  if (keymap < 0 || keymap >= 3)
    keymap = 0;
  if (buttonIndex < 0 || buttonIndex >= 8)
    buttonIndex = 0;
  return config.shortEntries[keymap][buttonIndex];
}

KeyEntry &ConfigManager::rawLongEntry(int keymap, int buttonIndex)
{
  if (keymap < 0 || keymap >= 3)
    keymap = 0;
  if (buttonIndex < 0 || buttonIndex >= 8)
    buttonIndex = 0;
  return config.longEntries[keymap][buttonIndex];
}

void ConfigManager::beginConfigAP(const std::vector<std::string> &bondList,
                                  int batVoltageMv, int batPercent)
{
  webUI.beginConfigAP(this, bondList, batVoltageMv, batPercent);
}
