#pragma once

#include <Arduino.h>
#include <Keypad.h>
#include "HardwareConfig.h"

extern const int DEBUG;

// ---------------------------------------------------------------------------
// ButtonManager — manages the physical keypad hardware and all button timing.
//
// Design overview:
//   • update() must be called every loop iteration.  It scans the matrix and
//     fires high-level callbacks; no raw Keypad state leaks to the caller.
//   • Per-button behaviour is configured via setButtonRepeating() and
//     setButtonLongPressTime() after begin() but before the first update().
//
// Callback semantics:
//   onShortPress(btn)       — tap (release before SHORT_PRESS_MAX), OR one call
//                             on press-down + repeated calls at REPEAT_MS for
//                             repeating buttons.
//   onLongPress(btn)        — fired while a non-repeating button is still held,
//                             once its longPressTime threshold is crossed.
//   onCombo(held, pressed)  — fired when 'pressed' is pressed while 'held' has
//                             been held for ≥ SHORT_PRESS_MAX ms.
// ---------------------------------------------------------------------------

class ButtonManager {
public:
  // Timing constants (milliseconds)
  static const unsigned long SHORT_PRESS_MAX        = 500;   // tap threshold; also combo-detection threshold
  static const unsigned long REPEAT_MS              = 100;   // interval between repeated short-press calls
  static const unsigned long LONG_PRESS_CONFIG_TIME = 5000;  // total hold time to trigger config mode (button 4)

  ButtonManager()
    : _keypad(makeKeymap(_buttons), _rowPins, _colPins, ROWS, COLS)
  {
    memcpy(_rowPins, KEYPAD_ROW_PINS, sizeof(_rowPins));
    memcpy(_colPins, KEYPAD_COL_PINS, sizeof(_colPins));
  }

  // Initialise hardware.  Call once in setup() before any update() calls.
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

  // ---- Callback registration -----------------------------------------------

  void setShortPressHandler(void (*h)(char btn))             { _shortPressHandler = h; }
  void setLongPressHandler(void (*h)(char btn))              { _longPressHandler  = h; }
  void setComboHandler(void (*h)(char held, char pressed))   { _comboHandler      = h; }

  // ---- Per-button configuration --------------------------------------------
  // repeating=true  : onShortPress fires immediately on press-down, then repeats
  //                   at REPEAT_MS intervals after SHORT_PRESS_MAX ms of hold.
  // repeating=false : onShortPress fires on release if held < SHORT_PRESS_MAX;
  //                   onLongPress fires while held once longPressTime is reached.
  void setButtonRepeating(char btn, bool repeating) {
    int i = _btnIdx(btn);
    if (i >= 0) _repeating[i] = repeating;
  }

  // Override the long-press threshold for a specific button (default: SHORT_PRESS_MAX).
  void setButtonLongPressTime(char btn, unsigned long ms) {
    int i = _btnIdx(btn);
    if (i >= 0) _longPressTime[i] = ms;
  }

  // ---- Main update loop ----------------------------------------------------
  // Call every loop() iteration.  Scans the matrix and fires callbacks.
  void update() {
    _keypad.getKey();               // scan the matrix; updates _keypad.key[]
    unsigned long now = millis();

    for (int i = 0; i < LIST_MAX; i++) {
      char     btn = _keypad.key[i].kchar;
      KeyState ks  = _keypad.key[i].kstate;

      if (btn == NO_KEY) continue;

      int bi = _btnIdx(btn);
      if (bi < 0) continue;

      switch (ks) {

        case PRESSED:
          if (!_active[bi]) {
            // New press — initialise per-button tracking
            _active[bi]     = true;
            _pressStart[bi] = now;
            _lastRepeat[bi] = now;
            _longFired[bi]  = false;
            _comboFired[bi] = false;

            // Combo detection: another button held for ≥ SHORT_PRESS_MAX?
            for (int j = 0; j < MAX_BTNS; j++) {
              if (j == bi || !_active[j]) continue;
              if ((now - _pressStart[j]) >= SHORT_PRESS_MAX) {
                if (_comboHandler) _comboHandler(_btnChar(j), btn);
                _comboFired[bi] = true;
                break; // one combo at a time
              }
            }

            // Repeating buttons fire immediately on press-down
            if (!_comboFired[bi] && _repeating[bi]) {
              if (_shortPressHandler) _shortPressHandler(btn);
            }
          }
          break;

        case HOLD:
          if (_active[bi]) {
            unsigned long held = now - _pressStart[bi];
            if (_repeating[bi]) {
              // Repeat short-press at REPEAT_MS after the initial SHORT_PRESS_MAX hold
              if (held >= SHORT_PRESS_MAX &&
                  (now - _lastRepeat[bi]) >= REPEAT_MS) {
                if (_shortPressHandler) _shortPressHandler(btn);
                _lastRepeat[bi] = now;
              }
            } else {
              // Fire long-press once when threshold is crossed
              if (!_longFired[bi] && !_comboFired[bi] &&
                  held >= _longPressTime[bi]) {
                if (_longPressHandler) _longPressHandler(btn);
                _longFired[bi] = true;
              }
            }
          }
          break;

        case RELEASED:
          if (_active[bi]) {
            unsigned long held = now - _pressStart[bi];
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
            // RELEASED state was missed — treat as release now
            unsigned long held = now - _pressStart[bi];
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

  // True when no buttons are currently being tracked as active
  bool isIdle() const {
    for (int i = 0; i < MAX_BTNS; i++) {
      if (_active[i]) return false;
    }
    return true;
  }

  // Poll until all buttons are idle without firing any callbacks.
  // Call after entering config mode to consume the triggering button-4 release.
  void drainButton(int timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)timeoutMs) {
      _keypad.getKey();
      bool anyActive = false;
      for (int i = 0; i < LIST_MAX; i++) {
        KeyState ks = _keypad.key[i].kstate;
        if (ks == PRESSED || ks == HOLD || ks == RELEASED) { anyActive = true; break; }
      }
      if (!anyActive) break;
      delay(10);
    }
    // Clear all internal tracking state to avoid stale data after the drain
    for (int i = 0; i < MAX_BTNS; i++) _active[i] = false;
  }

private:
  static const byte ROWS     = 3;
  static const byte COLS     = 3;
  static const int  MAX_BTNS = 9; // buttons '1'–'9'

  char _buttons[ROWS][COLS] = {
    {'1', '5', '4'},
    {'2', '6', '7'},
    {'3', '8', '9'}
  };

  byte _rowPins[ROWS] = {};
  byte _colPins[COLS] = {};

  Keypad _keypad;

  // Callbacks
  void (*_shortPressHandler)(char)      = nullptr;
  void (*_longPressHandler)(char)       = nullptr;
  void (*_comboHandler)(char, char)     = nullptr;

  // Per-button configuration (index = btn - '1')
  bool          _repeating[MAX_BTNS]    = {};
  unsigned long _longPressTime[MAX_BTNS]= {};

  // Per-button runtime state
  bool          _active[MAX_BTNS]       = {};
  unsigned long _pressStart[MAX_BTNS]   = {};
  unsigned long _lastRepeat[MAX_BTNS]   = {};
  bool          _longFired[MAX_BTNS]    = {};
  bool          _comboFired[MAX_BTNS]   = {};

  // Map '1'–'9' → 0–8; returns -1 for unrecognised chars
  static int  _btnIdx(char btn)  { return (btn >= '1' && btn <= '9') ? btn - '1' : -1; }
  static char _btnChar(int idx)  { return '1' + idx; }
};
