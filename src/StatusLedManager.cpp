#include "StatusLedManager.h"

void StatusLedManager::begin(int pin) {
  _pin          = pin;
  _ledState     = 0;
  _ledStateTime = millis_now();
  _status       = APP_BT_DISCONNECTED;

  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << pin;
  io.mode         = GPIO_MODE_OUTPUT;
  io.pull_up_en   = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&io);
  gpio_set_level((gpio_num_t)pin, 0);
}

void StatusLedManager::flashLed(int times, unsigned long length, unsigned long delayTime) {
  _flashActive    = true;
  _flashRemaining = times;
  _flashOnTime    = length;
  _flashOffTime   = delayTime;
  _flashLedOn     = true;
  gpio_set_level((gpio_num_t)_pin, 1);
  _flashStateTime = millis_now();
}

void StatusLedManager::resetLedState() {
  _ledState     = 0;
  _ledStateTime = millis_now();
  gpio_set_level((gpio_num_t)_pin, 0);
}

void StatusLedManager::update() {
  uint32_t now = millis_now();

  // A pending flash animation takes priority over the status blink pattern
  if (_flashActive) {
    if (_flashLedOn) {
      if (now - _flashStateTime >= _flashOnTime) {
        _flashRemaining--;
        if (_flashRemaining == 0) {
          // Last flash complete — turn off and return to status blink
          gpio_set_level((gpio_num_t)_pin, 0);
          _flashActive  = false;
          _ledStateTime = now;  // reset status blink timer
        } else {
          // More flashes to go — turn off and wait for off delay
          gpio_set_level((gpio_num_t)_pin, 0);
          _flashLedOn     = false;
          _flashStateTime = now;
        }
      }
    } else {
      if (now - _flashStateTime >= _flashOffTime) {
        // Start next flash
        gpio_set_level((gpio_num_t)_pin, 1);
        _flashLedOn     = true;
        _flashStateTime = now;
      }
    }
    return;
  }

  if (_status == APP_CONNECTED_BLINK) {
    if ((now - _ledStateTime) > (uint32_t)_keymapIndicatorLedDelays[_ledState]) {
      _ledState = 1 - _ledState;
      gpio_set_level((gpio_num_t)_pin, _ledState);
      _ledStateTime = now;
      if (_ledState == 0) {
        if (--_keymapIndicatorCountdown == 0) _status = APP_CONNECTED;
      }
    }
  } else if (_status != APP_CONNECTED) {
    // APP_BT_DISCONNECTED / APP_CONFIG: run the blink timer
    if ((now - _ledStateTime) > (uint32_t)_ledDelays[_status][_ledState]) {
      _ledState = 1 - _ledState;
      gpio_set_level((gpio_num_t)_pin, _ledState);
      _ledStateTime = now;
    }
  }
  // APP_CONNECTED: LED stays off; key-press events use flashLed() directly
}
