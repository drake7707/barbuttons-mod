#pragma once

#include <cstdint>
#include <cstring>
#include "buttons/KeyCodes.h"

#define BLE_NAME_MAX_LEN 32
extern const char DEFAULT_BLE_NAME[];

// ---------------------------------------------------------------------------
// Per-key target type
// ---------------------------------------------------------------------------
enum KeyTarget : uint8_t {
  TARGET_SELECT = 0, // use the runtime target selector
  TARGET_HID    = 1, // send to a specific HID peer (or broadcast)
  TARGET_BTHOME = 2, // broadcast a BTHome advertisement
  TARGET_IR_NEC = 3  // transmit an IR NEC command
};

// Safety limit: maximum full NEC frames emitted per button press.
// The IR LED is driven through a BJT at up to 200 mA; long bursts risk overheating.
static constexpr uint8_t IR_MAX_REPEATS = 3;

// ---------------------------------------------------------------------------
// One button action (short or long press)
// ---------------------------------------------------------------------------
struct KeyEntry {
  uint8_t   key       = 0;
  KeyTarget target    = TARGET_SELECT;
  char      mac[18]   = {}; // HID peer MAC ("" = broadcast all)
  uint16_t  irAddress = 0;  // NEC extended 16-bit address
  uint16_t  irCommand = 0;  // NEC extended 16-bit command
  uint8_t   irRepeats = 1;  // number of full NEC frames to emit (1..IR_MAX_REPEATS)
};

// ---------------------------------------------------------------------------
// One button combo action (hold one button, press another)
// ---------------------------------------------------------------------------
struct ComboEntry {
  char      held      = 0;   // ASCII of held button ('1'..'8'), 0 = unused
  char      pressed   = 0;   // ASCII of pressed button ('1'..'8'), 0 = unused
  uint8_t   key       = 0;
  KeyTarget target    = TARGET_SELECT;
  char      mac[18]   = {}; // HID peer MAC ("" = broadcast all)
  uint16_t  irAddress = 0;
  uint16_t  irCommand = 0;
  uint8_t   irRepeats = 1;
};

// ---------------------------------------------------------------------------
// Config -- the full runtime configuration state of the device
// ---------------------------------------------------------------------------
struct Config {
  // Factory defaults (definitions in Config.cpp)
  static const uint8_t DEFAULT_SHORT[8];
  static const uint8_t DEFAULT_LONG[8];

  static const int MAX_COMBOS = 16; // per keymap slot

  KeyEntry  shortEntries[3][8]           = {};
  KeyEntry  longEntries[3][8]            = {};
  ComboEntry comboEntries[3][MAX_COMBOS] = {};
  uint8_t   comboCounts[3]               = {};
  int       activeKeymap                 = 1;
  bool      batteryEnabled               = false;
  bool      blePowerSaving               = false;
  uint8_t   maxBLEConnections            = 1;
  char      bleName[BLE_NAME_MAX_LEN + 1] = {};

  // Map a button character ('1'..'8') to a 0-based index; returns -1 if invalid.
  static int btnIndex(char key);
};
