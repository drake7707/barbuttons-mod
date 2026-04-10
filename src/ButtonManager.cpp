#include "ButtonManager.h"

ButtonManager::ButtonManager()
  : _keypad(makeKeymap(_buttons), _rowPins, _colPins, ROWS, COLS)
{}

void ButtonManager::setPinConfiguration(const uint8_t* rowPins, const uint8_t* colPins) {
  memcpy(_rowPins, rowPins, sizeof(_rowPins));
  memcpy(_colPins, colPins, sizeof(_colPins));
}

void ButtonManager::begin() {
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

void ButtonManager::setShortPressHandler(void (*h)(char btn))           { _shortPressHandler = h; }
void ButtonManager::setLongPressHandler(void (*h)(char btn))            { _longPressHandler  = h; }
void ButtonManager::setComboHandler(void (*h)(char held, char pressed)) { _comboHandler      = h; }

void ButtonManager::setButtonRepeating(char btn, bool repeating) {
  int i = _btnIdx(btn);
  if (i >= 0) _repeating[i] = repeating;
}

void ButtonManager::setButtonLongPressTime(char btn, uint32_t ms) {
  int i = _btnIdx(btn);
  if (i >= 0) _longPressTime[i] = ms;
}

void ButtonManager::print_keypad_state() {
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

void ButtonManager::update() {
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

bool ButtonManager::isIdle() const {
  for (int i = 0; i < MAX_BTNS; i++) if (_active[i]) return false;
  return true;
}

void ButtonManager::drainButton(int timeoutMs) {
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

int  ButtonManager::_btnIdx(char btn) { return (btn >= '1' && btn <= '9') ? btn - '1' : -1; }
char ButtonManager::_btnChar(int idx) { return '1' + idx; }
