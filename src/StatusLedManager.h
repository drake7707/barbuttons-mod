#pragma once

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Returns milliseconds since boot (wraps after ~49 days; safe for interval arithmetic).
static inline uint32_t millis_now() { return (uint32_t)(esp_timer_get_time() / 1000LL); }

// ---------------------------------------------------------------------------
// AppStatus — shared application-state enum used by multiple managers
// ---------------------------------------------------------------------------
enum AppStatus {
  APP_BT_DISCONNECTED = 0,  // Not connected to BT
  APP_CONFIG          = 1,  // Config mode (AP active, rapid blink)
  APP_CONNECTED       = 2,  // BT connected, main menu
  APP_CONNECTED_BLINK = 3   // BT connected, keymap indicator flash
};

// ---------------------------------------------------------------------------
// StatusLedManager — manages the status LED blink patterns and app state
// ---------------------------------------------------------------------------
class StatusLedManager {
public:
  void begin(int pin) {
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

  // Schedule a non-blocking LED flash animation.
  // The LED will blink 'times' times, each ON for 'length' ms with 'delayTime' ms
  // between flashes.  The animation is driven by update() calls in the main loop.
  void flashLed(int times, unsigned long length, unsigned long delayTime) {
    _flashActive    = true;
    _flashRemaining = times;
    _flashOnTime    = length;
    _flashOffTime   = delayTime;
    _flashLedOn     = true;
    gpio_set_level((gpio_num_t)_pin, 1);
    _flashStateTime = millis_now();
  }

  void setStatus(AppStatus s) { _status = s; }
  AppStatus getStatus()       { return _status; }

  // Reset LED to off and restart the blink timer
  void resetLedState() {
    _ledState     = 0;
    _ledStateTime = millis_now();
    gpio_set_level((gpio_num_t)_pin, 0);
  }

  // Drive blink pattern and flash animation — call every loop iteration
  void update() {
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

private:
  int       _pin                      = 0;
  AppStatus _status                   = APP_BT_DISCONNECTED;
  int       _ledState                 = 0;
  uint32_t  _ledStateTime             = 0;
  int       _keymapIndicatorCountdown = 0;

  // Non-blocking flash animation state
  bool     _flashActive    = false;
  int      _flashRemaining = 0;
  bool     _flashLedOn     = false;
  uint32_t _flashOnTime    = 0;
  uint32_t _flashOffTime   = 0;
  uint32_t _flashStateTime = 0;

  int _ledDelays[4][2] = {
    {  500,  500 },  // APP_BT_DISCONNECTED
    {  100, 3000 },  // APP_CONFIG
    { 3000,  100 },  // APP_CONNECTED
    { 3000,  100 }   // APP_CONNECTED_BLINK
  };
  int _keymapIndicatorLedDelays[2] = {100, 50};
};
