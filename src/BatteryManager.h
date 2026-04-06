#pragma once

#include <cstdio>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
             uint32_t readIntervalMs = 60000) {
    _channel    = channel;
    _intervalMs = readIntervalMs;

    adc_oneshot_unit_init_cfg_t unitCfg = {
      .unit_id  = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitCfg, &_adcHandle));

    adc_oneshot_chan_cfg_t chanCfg = {
      .atten    = ADC_ATTEN_DB_12,      // 0–3.1 V full-scale input range
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adcHandle, _channel, &chanCfg));

    _initCalibration();

    if (DEBUG) printf("[Battery] ADC init, calibration %s\n",
                      _calEnabled ? "enabled" : "disabled (raw estimate)");

    // Trigger a read on the very first update() call by back-dating the
    // last-read timestamp by one full interval.  Unsigned wrap-around is
    // intentional and handled correctly by the subtraction in update().
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    _lastReadMs  = now - _intervalMs;
  }

  void setBatteryReadingHandler(BatteryReadingHandler handler) {
    _handler = handler;
  }

  // Call every loop iteration. Fires the handler when the read interval elapses.
  void update() {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - _lastReadMs < _intervalMs) return;
    _lastReadMs = now;

    int raw = 0;
    if (adc_oneshot_read(_adcHandle, _channel, &raw) != ESP_OK) {
      if (DEBUG) printf("[Battery] ADC read error\n");
      return;
    }

    // Convert raw ADC value to millivolts.
    int adc_mv = 0;
    if (_calEnabled && _calHandle) {
      adc_cali_raw_to_voltage(_calHandle, raw, &adc_mv);
    } else {
      // Fallback: linear approximation, 3 100 mV full scale, 12-bit resolution.
      adc_mv = (int)((int64_t)raw * 3100 / 4095);
    }

    // Scale up through the voltage divider to get the actual battery voltage.
    int bat_mv = (int)((int64_t)adc_mv * (VDIV_R1_KOHM + VDIV_R2_KOHM) / VDIV_R2_KOHM);

    // Map [BAT_MIN_MV, BAT_MAX_MV] → [0, 100] % and clamp.
    int pct = (bat_mv - BAT_MIN_MV) * 100 / (BAT_MAX_MV - BAT_MIN_MV);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    if (DEBUG) printf("[Battery] raw=%d adc=%d mV bat=%d mV -> %d%%\n",
                      raw, adc_mv, bat_mv, pct);

    if (_handler) _handler((uint8_t)pct);
  }

private:
  adc_channel_t             _channel    = ADC_CHANNEL_0;
  uint32_t                  _intervalMs = 60000;
  uint32_t                  _lastReadMs = 0;
  adc_oneshot_unit_handle_t _adcHandle  = nullptr;
  adc_cali_handle_t         _calHandle  = nullptr;
  bool                      _calEnabled = false;
  BatteryReadingHandler     _handler    = nullptr;

  void _initCalibration() {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
      .unit_id  = ADC_UNIT_1,
      .chan     = _channel,
      .atten    = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    _calEnabled = (adc_cali_create_scheme_curve_fitting(&cfg, &_calHandle) == ESP_OK);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
      .unit_id  = ADC_UNIT_1,
      .atten    = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    _calEnabled = (adc_cali_create_scheme_line_fitting(&cfg, &_calHandle) == ESP_OK);
#endif
  }
};
