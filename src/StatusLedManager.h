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
enum AppStatus
{
  APP_BT_DISCONNECTED = 0,         // Not connected to BT, advertising if not in config mode
  APP_CONFIG = 1,                  // Config mode (AP active, rapid blink)
  APP_CONNECTED = 2,               // BT connected, main menu
  APP_BT_CONNECTED_ADVERTISING = 3 // BT connected, advertising while already connected
};

// ---------------------------------------------------------------------------
// StatusLedManager — manages the status LED blink patterns and app state
// ---------------------------------------------------------------------------
class StatusLedManager
{
public:
  void begin(int pin);

  // Schedule a non-blocking LED flash animation.
  // The LED will blink 'times' times, each ON for 'length' ms with 'delayTime' ms
  // between flashes.  The animation is driven by update() calls in the main loop.
  void flashLed(int times, unsigned long length, unsigned long delayTime);

  void setStatus(AppStatus s) { _status = s; }
  AppStatus getStatus() { return _status; }

  // Reset LED to off and restart the blink timer
  void resetLedState();

  // Drive blink pattern and flash animation — call every loop iteration
  void update();

private:
  int _pin = 0;
  AppStatus _status = APP_BT_DISCONNECTED;
  int _ledState = 0;
  uint32_t _ledStateTime = 0;

  // Non-blocking flash animation state
  bool _flashActive = false;
  int _flashRemaining = 0;
  bool _flashLedOn = false;
  uint32_t _flashOnTime = 0;
  uint32_t _flashOffTime = 0;
  uint32_t _flashStateTime = 0;

  int _ledDelays[4][2] = {
      {500, 500},  // APP_BT_DISCONNECTED
      {100, 3000}, // APP_CONFIG
      {0, 0},      // APP_CONNECTED
      {2000, 100}  // APP_BT_CONNECTED_ADVERTISING
  };
};
