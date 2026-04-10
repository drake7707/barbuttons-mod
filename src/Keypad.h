#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <esp_timer.h>

// ---------------------------------------------------------------------------
// Keypad — matrix keypad driver (replaces Chris--A/Keypad library).
// Identical state-machine logic; GPIO calls use the ESP-IDF driver directly.
// ---------------------------------------------------------------------------

typedef enum { IDLE, PRESSED, HOLD, RELEASED } KeyState;
const char NO_KEY = '\0';

#define LIST_MAX 10
#define makeKeymap(x) ((char*)(x))

struct Key {
  char     kchar        = NO_KEY;
  int      kcode        = -1;
  KeyState kstate       = IDLE;
  bool     stateChanged = false;
};

class Keypad {
public:
  uint32_t bitMap[10] = {};
  Key      key[LIST_MAX];
  uint32_t holdTimer  = 0;

  Keypad(char* userKeymap, uint8_t* row, uint8_t* col, uint8_t numRows, uint8_t numCols);

  void setHoldTime(uint32_t ms);

  // Scan the matrix and update key[] state; returns true if any state changed.
  bool getKeys();

private:
  char*    _keymap;
  uint8_t* _rowPins;
  uint8_t* _colPins;
  uint8_t  _rows, _cols;
  uint32_t _debounceTime, _holdTime, _startTime;

  static uint32_t _ms();
  static void _cfgInputPullup(uint8_t pin);
  static void _cfgOutput(uint8_t pin);
  static void _cfgInput(uint8_t pin);

  void _scanKeys();
  bool _updateList();
  int  _findByCode(int code) const;
  void _nextKeyState(uint8_t idx, bool button);
  void _transitionTo(uint8_t idx, KeyState next);
};
