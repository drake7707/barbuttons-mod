#pragma once

#include <cstdio>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern const int DEBUG;

// Voltage divider resistor values (kΩ).  R1 is between battery+ and the ADC
// pin; R2 is between the ADC pin and GND.  V_bat = V_adc × (R1+R2) / R2.
static constexpr int VDIV_R1_KOHM = 680;
static constexpr int VDIV_R2_KOHM = 220;

// Li-ion cell voltage range mapped to 0–100 % charge (millivolts).
static constexpr int BAT_MIN_MV  = 3000;
static constexpr int BAT_MAX_MV  = 4200;

// ---------------------------------------------------------------------------
// BatteryManager — periodically reads battery voltage via ADC and fires a
// callback with the estimated charge percentage.
//
// Hardware: VDIV_R1_KOHM kΩ (battery+ → ADC pin) and VDIV_R2_KOHM kΩ
//   (ADC pin → GND) voltage divider.
//
// Li-ion charge mapping: BAT_MIN_MV → 0 %, BAT_MAX_MV → 100 %.
//
// ADC: ADC1 channel 0 (GPIO0) with 12 dB attenuation (0–3.1 V range).
// ---------------------------------------------------------------------------
class BatteryManager {
public:
  using BatteryReadingHandler = void (*)(uint8_t percent);

  // Initialise ADC1 on the given channel (default CH0 = GPIO0).
  // A reading is triggered on the very first update() call; subsequent reads
  // fire every readIntervalMs milliseconds.
  void begin(adc_channel_t channel = ADC_CHANNEL_0,
             uint32_t readIntervalMs = 60000);

  void setBatteryReadingHandler(BatteryReadingHandler handler);

  // Call every loop iteration. Fires the handler when the read interval elapses.
  void update();

private:
  adc_channel_t             _channel    = ADC_CHANNEL_0;
  uint32_t                  _intervalMs = 60000;
  uint32_t                  _lastReadMs = 0;
  uint8_t                   _lastPct    = 0xFF; // sentinel: no reading yet
  adc_oneshot_unit_handle_t _adcHandle  = nullptr;
  adc_cali_handle_t         _calHandle  = nullptr;
  bool                      _calEnabled = false;
  BatteryReadingHandler     _handler    = nullptr;
  int                       _lastMv     = -1;   // last battery voltage in mV; -1 until first ADC read

  void _initCalibration();

public:
  // Returns the last measured battery percentage (0-100), or -1 if no reading yet.
  int getLastPercent()   const { return _lastPct == 0xFF ? -1 : (int)_lastPct; }
  // Returns the last measured battery voltage in millivolts, or -1 if no reading yet.
  int getLastVoltageMv() const { return _lastMv; }
};
