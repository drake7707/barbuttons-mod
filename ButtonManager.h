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
    : _keypad(makeKeymap(_keys), _rowPins, _colPins, ROWS, COLS)
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
  char getKey() { return _keypad.getKey(); }

  // Current keypad state (IDLE, PRESSED, HOLD, RELEASED)
  KeyState getState() { return _keypad.getState(); }

  // Wait while the key remains HOLD for up to key_hold_time ms.
  // Returns true if the key is still held at the end of the wait.
  bool waitForKeyHold(int key_hold_time) {
    unsigned long start = millis();
    while (_keypad.getState() == HOLD &&
           millis() < (unsigned long)(start + key_hold_time)) {
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

  char _keys[ROWS][COLS] = {
    {'1', '5', '4'},
    {'2', '6', '7'},
    {'3', '8', '9'}
  };

  byte _rowPins[ROWS] = {};
  byte _colPins[COLS] = {};

  Keypad _keypad;
};
