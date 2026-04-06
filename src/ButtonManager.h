#pragma once

#include <cstdio>
#include <cstring>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Keypad.h"
#include "HardwareConfig.h"

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

  ButtonManager()
    : _keypad(makeKeymap(_buttons), _rowPins, _colPins, ROWS, COLS)
  {
    memcpy(_rowPins, KEYPAD_ROW_PINS, sizeof(_rowPins));
    memcpy(_colPins, KEYPAD_COL_PINS, sizeof(_colPins));
  }

  void begin() {
    for (int i = 0; i < MAX_BTNS; i++) {
      _repeating[i]     = false;
      _longPressTime[i] = SHORT_PRESS_MAX;
      _active[i]        = false;
      _longFired[i]     = false;
      _comboFired[i]    = false;
      _pressStart[i]    = 0;
      _lastRepeat[i]    = 0;
    }
    _keypad.setHoldTime(SHORT_PRESS_MAX);
  }

  void setShortPressHandler(void (*h)(char btn))           { _shortPressHandler = h; }
  void setLongPressHandler(void (*h)(char btn))            { _longPressHandler  = h; }
  void setComboHandler(void (*h)(char held, char pressed)) { _comboHandler      = h; }

  // repeating=true  : onShortPress fires immediately on press-down, then repeats
  //                   at REPEAT_MS intervals after SHORT_PRESS_MAX ms of hold.
  // repeating=false : onShortPress fires on release if held < SHORT_PRESS_MAX;
  //                   onLongPress fires while held once longPressTime is reached.
  void setButtonRepeating(char btn, bool repeating) {
    int i = _btnIdx(btn);
    if (i >= 0) _repeating[i] = repeating;
  }

  void setButtonLongPressTime(char btn, uint32_t ms) {
    int i = _btnIdx(btn);
    if (i >= 0) _longPressTime[i] = ms;
  }

  char oldPrint[64] = {};
  void print_keypad_state() {
    char str[64] = {};
    size_t pos = 0;
    for (int i = 0; i < LIST_MAX; i++) {
      char     btn = _keypad.key[i].kchar;
      KeyState ks  = _keypad.key[i].kstate;
      if (btn == NO_KEY) continue;
      int bi = _btnIdx(btn);
      if (bi < 0) continue;
      if (pos < sizeof(str) - 5) {
        char st = ks == PRESSED ? 'P' : ks == HOLD ? 'H' : ks == RELEASED ? 'R' : 'I';
        pos += snprintf(str + pos, sizeof(str) - pos, "%c=%c ", btn, st);
      }
    }
    if (strcmp(str, oldPrint) != 0) {
      printf("BUTTON STATE: %s\n", str);
      strncpy(oldPrint, str, sizeof(oldPrint) - 1);
    }
  }

  void update() {
    _keypad.getKeys();
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000LL);

    if (DEBUG) print_keypad_state();

    for (int i = 0; i < LIST_MAX; i++) {
      char     btn = _keypad.key[i].kchar;
      KeyState ks  = _keypad.key[i].kstate;

      if (btn == NO_KEY) continue;

      int bi = _btnIdx(btn);
      if (bi < 0) continue;

      switch (ks) {

        case PRESSED:
          if (!_active[bi]) {
            if (DEBUG) printf("[BUTTON] %c PRESSED\n", btn);
            _active[bi]     = true;
            _pressStart[bi] = now;
            _lastRepeat[bi] = now;
            _longFired[bi]  = false;
            _comboFired[bi] = false;

            // Combo detection: another button currently active?
            // No timing guard -- both buttons can appear as PRESSED in the same
            // scan cycle (now - _pressStart[j] == 0), so a threshold would
            // silently discard the combo.  Whichever button was registered first
            // in the key[] array is treated as the "held" button.
            // Both buttons are marked combo-involved to suppress their individual
            // short-press events on release.
            for (int j = 0; j < MAX_BTNS; j++) {
              if (j == bi || !_active[j]) continue;
              if (_comboHandler) _comboHandler(_btnChar(j), btn);
              _comboFired[bi] = true;
              _comboFired[j]  = true;
              break;
            }

            if (!_comboFired[bi] && _repeating[bi]) {
              if (_shortPressHandler) _shortPressHandler(btn);
            }
          }
          break;

        case HOLD:
          if (_active[bi]) {
            uint32_t held = now - _pressStart[bi];
            if (_repeating[bi]) {
              if (held >= SHORT_PRESS_MAX && (now - _lastRepeat[bi]) >= REPEAT_MS) {
                if (_shortPressHandler) _shortPressHandler(btn);
                _lastRepeat[bi] = now;
              }
            } else {
              if (!_longFired[bi] && !_comboFired[bi] && held >= _longPressTime[bi]) {
                if (_longPressHandler) _longPressHandler(btn);
                _longFired[bi] = true;
              }
            }
          }
          break;

        case RELEASED:
          if (_active[bi]) {
            if (DEBUG) printf("[BUTTON] %c RELEASED\n", btn);
            uint32_t held = now - _pressStart[bi];
            _active[bi] = false;
            // Short press fires on release for a genuine tap (< SHORT_PRESS_MAX,
            // not consumed by a long-press or combo)
            if (!_longFired[bi] && !_comboFired[bi] && !_repeating[bi] &&
                held < SHORT_PRESS_MAX) {
              if (_shortPressHandler) _shortPressHandler(btn);
            }
          }
          break;

        case IDLE:
          if (_active[bi]) {
            // RELEASED state was missed -- treat as release now
            uint32_t held = now - _pressStart[bi];
            _active[bi] = false;
            if (!_longFired[bi] && !_comboFired[bi] && !_repeating[bi] &&
                held < SHORT_PRESS_MAX) {
              if (_shortPressHandler) _shortPressHandler(btn);
            }
          }
          break;
      }
    }
  }

  bool isIdle() const {
    for (int i = 0; i < MAX_BTNS; i++) if (_active[i]) return false;
    return true;
  }

  // Poll until all buttons are idle without firing any callbacks.
  // Call after entering config mode to consume the triggering button-4 release.
  void drainButton(int timeoutMs) {
    uint32_t start = (uint32_t)(esp_timer_get_time() / 1000LL);
    while ((uint32_t)(esp_timer_get_time() / 1000LL) - start < (uint32_t)timeoutMs) {
      _keypad.getKeys();
      bool anyActive = false;
      for (int i = 0; i < LIST_MAX; i++) {
        KeyState ks = _keypad.key[i].kstate;
        if (ks == PRESSED || ks == HOLD || ks == RELEASED) { anyActive = true; break; }
      }
      if (!anyActive) break;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    for (int i = 0; i < MAX_BTNS; i++) _active[i] = false;
  }

private:
  static const uint8_t ROWS     = 3;
  static const uint8_t COLS     = 3;
  static const int     MAX_BTNS = 9;

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

  static int  _btnIdx(char btn) { return (btn >= '1' && btn <= '9') ? btn - '1' : -1; }
  static char _btnChar(int idx) { return '1' + idx; }
};
