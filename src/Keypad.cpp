#include "Keypad.h"

Keypad::Keypad(char* userKeymap, uint8_t* row, uint8_t* col, uint8_t numRows, uint8_t numCols)
  : _keymap(userKeymap), _rowPins(row), _colPins(col),
    _rows(numRows), _cols(numCols),
    _debounceTime(10), _holdTime(500), _startTime(0) {}

void Keypad::setHoldTime(uint32_t ms) { _holdTime = ms; }

bool Keypad::getKeys() {
  uint32_t now = _ms();
  if (now - _startTime <= _debounceTime) return false;
  _scanKeys();
  bool activity = _updateList();
  _startTime = _ms();
  return activity;
}

uint32_t Keypad::_ms() { return (uint32_t)(esp_timer_get_time() / 1000LL); }

void Keypad::_cfgInputPullup(uint8_t pin) {
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << pin;
  io.mode         = GPIO_MODE_INPUT;
  io.pull_up_en   = GPIO_PULLUP_ENABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&io);
}

void Keypad::_cfgOutput(uint8_t pin) {
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << pin;
  io.mode         = GPIO_MODE_OUTPUT;
  io.pull_up_en   = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&io);
}

void Keypad::_cfgInput(uint8_t pin) {
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << pin;
  io.mode         = GPIO_MODE_INPUT;
  io.pull_up_en   = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&io);
}

// Drive-low column, read rows; active-low => bit set = key pressed.
void Keypad::_scanKeys() {
  for (uint8_t r = 0; r < _rows; r++) _cfgInputPullup(_rowPins[r]);
  for (uint8_t c = 0; c < _cols; c++) {
    _cfgOutput(_colPins[c]);
    gpio_set_level((gpio_num_t)_colPins[c], 0);
    for (uint8_t r = 0; r < _rows; r++) {
      uint32_t pressed = !gpio_get_level((gpio_num_t)_rowPins[r]);
      bitMap[r] = (bitMap[r] & ~(1u << c)) | (pressed << c);
    }
    gpio_set_level((gpio_num_t)_colPins[c], 1);
    _cfgInput(_colPins[c]);
  }
}

bool Keypad::_updateList() {
  bool anyActivity = false;
  // Clear IDLE slots
  for (uint8_t i = 0; i < LIST_MAX; i++) {
    if (key[i].kstate == IDLE) {
      key[i].kchar = NO_KEY; key[i].kcode = -1; key[i].stateChanged = false;
    }
  }
  // Update existing keys and add newly pressed ones
  for (uint8_t r = 0; r < _rows; r++) {
    for (uint8_t c = 0; c < _cols; c++) {
      bool button  = (bitMap[r] >> c) & 1u;
      char keyChar = _keymap[r * _cols + c];
      int  keyCode = r * _cols + c;
      int  idx     = _findByCode(keyCode);
      if (idx >= 0) {
        _nextKeyState(idx, button);
      } else if (button) {
        for (uint8_t i = 0; i < LIST_MAX; i++) {
          if (key[i].kchar == NO_KEY) {
            key[i].kchar  = keyChar;
            key[i].kcode  = keyCode;
            key[i].kstate = IDLE;
            _nextKeyState(i, button);
            break;
          }
        }
      }
    }
  }
  for (uint8_t i = 0; i < LIST_MAX; i++) if (key[i].stateChanged) anyActivity = true;
  return anyActivity;
}

int Keypad::_findByCode(int code) const {
  for (uint8_t i = 0; i < LIST_MAX; i++) if (key[i].kcode == code) return i;
  return -1;
}

void Keypad::_nextKeyState(uint8_t idx, bool button) {
  key[idx].stateChanged = false;
  uint32_t now = _ms();
  switch (key[idx].kstate) {
    case IDLE:
      if (button) { _transitionTo(idx, PRESSED); holdTimer = now; }
      break;
    case PRESSED:
      if (now - holdTimer > _holdTime) _transitionTo(idx, HOLD);
      else if (!button)                _transitionTo(idx, RELEASED);
      break;
    case HOLD:
      if (!button) _transitionTo(idx, RELEASED);
      break;
    case RELEASED:
      _transitionTo(idx, IDLE);
      break;
  }
}

void Keypad::_transitionTo(uint8_t idx, KeyState next) {
  key[idx].kstate       = next;
  key[idx].stateChanged = true;
}
