#pragma once

#include <Arduino.h>
#include <Keypad.h>
#include "HardwareConfig.h"

extern const int DEBUG;

// ---------------------------------------------------------------------------
// ButtonManager — manages the physical keypad hardware and button timing
// ---------------------------------------------------------------------------

class ButtonManager {
public:
  // Timing constants (milliseconds)
  static const int LONG_PRESS_TIME            = 500;
  static const int LONG_PRESS_REPEAT_INTERVAL = 100;
  static const int LONG_PRESS_TIME_CONFIG     = 4500; // additional hold after first 500 ms = 5 s total

  ButtonManager()
    : _keypad(makeKeymap(_buttons), _rowPins, _colPins, ROWS, COLS)
  {
    memcpy(_rowPins, KEYPAD_ROW_PINS, sizeof(_rowPins));
    memcpy(_colPins, KEYPAD_COL_PINS, sizeof(_colPins));
  }

  // Register the keypad event handler and configure hold time
  void begin(void (*handler)(KeypadEvent)) {
    _keypad.addEventListener(handler);
    _keypad.setHoldTime(LONG_PRESS_TIME);
  }

  // Poll the keypad — must be called every loop iteration
  char getButton() { return _keypad.getKey(); }

  // Current keypad state (IDLE, PRESSED, HOLD, RELEASED) — state of key[0]
  KeyState getState() { return _keypad.getState(); }

  // Return the state of a specific button character by searching all active key
  // slots.  This is needed for multi-key scenarios (e.g. hold '4' + press '3')
  // where getState() always reflects key[0] (the first-pressed button) rather
  // than the button that just triggered the event listener.
  // Returns IDLE if the button is not currently tracked.
  KeyState getButtonState(char btn) {
    for (int i = 0; i < LIST_MAX; i++) {
      if (_keypad.key[i].kchar == btn) {
        return _keypad.key[i].kstate;
      }
    }
    return IDLE;
  }

  // Wait while the button remains HOLD for up to hold_time ms.
  // Returns true if the button is still held at the end of the wait.
  bool waitForButtonHold(int hold_time) {
    unsigned long start = millis();
    while (_keypad.getState() == HOLD &&
           millis() < (unsigned long)(start + hold_time)) {
      delay(20);
      _keypad.getKey();
    }
    return (_keypad.getState() == HOLD);
  }

  // Drain a button-release event by polling until the keypad goes IDLE.
  // Typically called after entering config mode to consume the triggering
  // RELEASED event before the config loop begins.
  void drainButton(int timeoutMs) {
    unsigned long start = millis();
    while (_keypad.getState() != IDLE &&
           millis() - start < (unsigned long)timeoutMs) {
      _keypad.getKey();
      delay(10);
    }
  }

private:
  static const byte ROWS = 3;
  static const byte COLS = 3;

  char _buttons[ROWS][COLS] = {
    {'1', '5', '4'},
    {'2', '6', '7'},
    {'3', '8', '9'}
  };

  byte _rowPins[ROWS] = {};
  byte _colPins[COLS] = {};

  Keypad _keypad;
};
