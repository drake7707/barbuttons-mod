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
//   onCombo(held, pressed)  — fired when 'pressed' is pressed while 'held' is
//                             already active (even if both appear in the same scan).
// ---------------------------------------------------------------------------

class ButtonManager {
public:
  // Timing constants (milliseconds)
  static const unsigned long SHORT_PRESS_MAX        = 500;   // tap threshold (ms)
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
 
  String oldPrint;
  void print_keypad_state() {
    String str;
    
    for (int i = 0; i < LIST_MAX; i++) {
      char     btn = _keypad.key[i].kchar;
      KeyState ks  = _keypad.key[i].kstate;
    
      if (btn == NO_KEY) continue;

      int bi = _btnIdx(btn);
      if (bi < 0) continue;

      str += btn;
      str += "=";
      switch(ks) {
        case PRESSED:
          str += "P";
          break;
        case HOLD:
          str += "H";
          break;
        case RELEASED:
          str += "R";
          break;
        case IDLE:
          str += "I";
          break;
      }
      str += " ";
    }
    if(str != oldPrint) {
      Serial.print("BUTTON STATE: ");
      Serial.println(str);
    }
    oldPrint = str;
  
  }

  // ---- Main update loop ----------------------------------------------------
  // Call every loop() iteration.  Scans the matrix and fires callbacks.
  void update() {
    _keypad.getKey();               // scan the matrix; updates _keypad.key[]
    unsigned long now = millis();

    if(DEBUG) print_keypad_state();

    for (int i = 0; i < LIST_MAX; i++) {
      char     btn = _keypad.key[i].kchar;
      KeyState ks  = _keypad.key[i].kstate;

      if (btn == NO_KEY) continue;

      int bi = _btnIdx(btn);
      if (bi < 0) continue;

      switch (ks) {

        case PRESSED:
          if (!_active[bi]) {
            if (DEBUG) { Serial.print("[BUTTON] "); Serial.print(btn); Serial.println(" PRESSED");}
            // New press — initialise per-button tracking
            _active[bi]     = true;
            _pressStart[bi] = now;
            _lastRepeat[bi] = now;
            _longFired[bi]  = false;
            _comboFired[bi] = false;

            // Combo detection: another button currently active?
            // No timing guard — both buttons can appear as PRESSED in the same
            // scan cycle (now - _pressStart[j] == 0), so a threshold would
            // silently discard the combo.  Whichever button was registered first
            // in the key[] array is treated as the "held" button.
            // Both buttons are marked combo-involved to suppress their individual
            // short-press events on release.
            for (int j = 0; j < MAX_BTNS; j++) {
              if (j == bi || !_active[j]) continue;
              if (_comboHandler) _comboHandler(_btnChar(j), btn);
              _comboFired[bi] = true;  // suppress short press for the new button
              _comboFired[j]  = true;  // suppress short press for the held button too
              break; // one combo at a time
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
            if (DEBUG) { Serial.print("[BUTTON] "); Serial.print(btn); Serial.println(" RELEASED");}
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

  // Physical layout of the 3×3 key matrix.
  // Each character is the logical button label used throughout the firmware.
  // The order matches the wiring: _buttons[row][col] = label.
  char _buttons[ROWS][COLS] = {
    {'1', '5', '4'},
    {'2', '6', '7'},
    {'3', '8', '9'}
  };

  // GPIO pin arrays for the keypad matrix rows and columns.
  // Populated in the constructor from HardwareConfig constants so that
  // ButtonManager does not depend directly on the hardware-config header.
  byte _rowPins[ROWS] = {};
  byte _colPins[COLS] = {};

  // Underlying Keypad library instance — handles low-level matrix scanning,
  // debouncing, and PRESSED/HOLD/RELEASED/IDLE state tracking.
  Keypad _keypad;

  // ---- Registered event callbacks (set via setXxxHandler) -----------------
  // Called by update() when the corresponding event is detected.
  // All are nullptr until explicitly set; update() checks before calling.
  void (*_shortPressHandler)(char)      = nullptr;  // tap or auto-repeat fire
  void (*_longPressHandler)(char)       = nullptr;  // sustained hold beyond longPressTime
  void (*_comboHandler)(char, char)     = nullptr;  // second button pressed while first is held

  // ---- Per-button configuration (index = btn - '1') -----------------------
  // Set once after begin() via setButtonRepeating() / setButtonLongPressTime().

  // true  → fire shortPressHandler immediately on press-down, then auto-repeat
  //         at REPEAT_MS intervals after SHORT_PRESS_MAX hold duration.
  // false → fire shortPressHandler on release (if held < SHORT_PRESS_MAX),
  //         or fire longPressHandler once the longPressTime threshold is crossed.
  bool          _repeating[MAX_BTNS]    = {};

  // Per-button hold threshold (ms) before longPressHandler fires.
  // Defaults to SHORT_PRESS_MAX; overridden for button 4 with LONG_PRESS_CONFIG_TIME.
  // Ignored for repeating buttons (they never fire longPressHandler).
  unsigned long _longPressTime[MAX_BTNS]= {};

  // ---- Per-button runtime state (reset on each press) ---------------------

  // true while the button is physically held down (PRESSED or HOLD state).
  // Cleared on RELEASED or IDLE (missed-release recovery).
  bool          _active[MAX_BTNS]       = {};

  // millis() timestamp recorded when the button first transitioned to PRESSED.
  // Used to calculate how long the button has been held: (now - _pressStart[i]).
  unsigned long _pressStart[MAX_BTNS]   = {};

  // millis() timestamp of the most recent auto-repeat fire for this button.
  // Used to enforce the REPEAT_MS spacing between successive repeat callbacks.
  // Only meaningful when _repeating[i] is true.
  unsigned long _lastRepeat[MAX_BTNS]   = {};

  // Guard flag: set to true the first time longPressHandler is called for this
  // press.  Prevents the callback from firing again on subsequent HOLD scans and
  // also suppresses the shortPressHandler on release (one action per press).
  bool          _longFired[MAX_BTNS]    = {};

  // Guard flag: set to true when this button is involved in a combo press
  // (either as the "held" button or the newly "pressed" button).
  // Suppresses both the shortPressHandler and longPressHandler for that button
  // so the combo action is the only event fired.
  bool          _comboFired[MAX_BTNS]   = {};

  // Map '1'–'9' → 0–8; returns -1 for unrecognised chars
  static int  _btnIdx(char btn)  { return (btn >= '1' && btn <= '9') ? btn - '1' : -1; }
  static char _btnChar(int idx)  { return '1' + idx; }
};
