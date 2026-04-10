#pragma once

#include <cstdio>
#include <cstring>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Keypad.h"

extern const int DEBUG;

// ---------------------------------------------------------------------------
// ButtonManager -- manages the physical keypad hardware and all button timing.
//
// Design overview:
//   * update() must be called every loop iteration.  It scans the matrix and
//     fires high-level callbacks; no raw Keypad state leaks to the caller.
//   * Per-button behaviour is configured via setButtonRepeating() and
//     setButtonLongPressTime() after begin() but before the first update().
//
// Callback semantics:
//   onShortPress(btn)       -- tap (release before SHORT_PRESS_MAX), OR one call
//                             on press-down + repeated calls at REPEAT_MS for
//                             repeating buttons.
//   onLongPress(btn)        -- fired while a non-repeating button is still held,
//                             once its longPressTime threshold is crossed.
//   onCombo(held, pressed)  -- fired when 'pressed' is pressed while 'held' is
//                             already active (even if both appear in the same scan).
// ---------------------------------------------------------------------------

class ButtonManager {
public:
  // Timing constants (milliseconds)
  static const uint32_t SHORT_PRESS_MAX        = 500;
  static const uint32_t REPEAT_MS              = 100;
  static const uint32_t LONG_PRESS_CONFIG_TIME = 5000;

  ButtonManager();

  // Apply the row and column pin layout before begin().
  // Use getKeypadRowPins() / getKeypadColPins() from HardwareConfig.h to
  // select the correct configuration for the battery / no-battery mode.
  void setPinConfiguration(const uint8_t* rowPins, const uint8_t* colPins);

  void begin();

  void setShortPressHandler(void (*h)(char btn));
  void setLongPressHandler(void (*h)(char btn));
  void setComboHandler(void (*h)(char held, char pressed));

  // repeating=true  : onShortPress fires immediately on press-down, then repeats
  //                   at REPEAT_MS intervals after SHORT_PRESS_MAX ms of hold.
  // repeating=false : onShortPress fires on release if held < SHORT_PRESS_MAX;
  //                   onLongPress fires while held once longPressTime is reached.
  void setButtonRepeating(char btn, bool repeating);

  void setButtonLongPressTime(char btn, uint32_t ms);


  void print_keypad_state();

  void update();

  bool isIdle() const;

  // Poll until all buttons are idle without firing any callbacks.
  // Call after entering config mode to consume the triggering button-4 release.
  void drainButton(int timeoutMs);

private:
  static const uint8_t ROWS     = 3;
  static const uint8_t COLS     = 3;
  static const int     MAX_BTNS = 9;

  char _oldPrint[64] = {};

  char    _buttons[ROWS][COLS] = {
    {'1', '5', '4'},
    {'2', '6', '7'},
    {'3', '8', '9'}
  };

  uint8_t _rowPins[ROWS] = {};
  uint8_t _colPins[COLS] = {};

  Keypad _keypad;

  void (*_shortPressHandler)(char)  = nullptr;
  void (*_longPressHandler)(char)   = nullptr;
  void (*_comboHandler)(char, char) = nullptr;

  bool     _repeating[MAX_BTNS]     = {};
  uint32_t _longPressTime[MAX_BTNS] = {};
  bool     _active[MAX_BTNS]        = {};
  uint32_t _pressStart[MAX_BTNS]    = {};
  uint32_t _lastRepeat[MAX_BTNS]    = {};
  bool     _longFired[MAX_BTNS]     = {};
  bool     _comboFired[MAX_BTNS]    = {};

  static int  _btnIdx(char btn);
  static char _btnChar(int idx);
};
