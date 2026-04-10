#include "BatteryManager.h"

// ADC fallback (no calibration): full-scale input range (mV) and max raw value
// for 12-bit resolution (2^12 - 1 = 4095).
static constexpr int ADC_FULL_SCALE_MV = 3100;
static constexpr int ADC_MAX_RAW       = 4095;

// Sentinel value for _lastPct meaning "no reading taken yet".
// Must be outside the valid 0–100 % range.
static constexpr uint8_t BAT_PCT_NO_READING = 0xFF;

void BatteryManager::begin(adc_channel_t channel, uint32_t readIntervalMs) {
  _channel    = channel;
  _intervalMs = readIntervalMs;
  _lastPct    = BAT_PCT_NO_READING;

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

void BatteryManager::setBatteryReadingHandler(BatteryReadingHandler handler) {
  _handler = handler;
}

void BatteryManager::update() {
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
    // Fallback: linear approximation, 3100 mV full scale, 12-bit resolution.
    adc_mv = (int)((int64_t)raw * ADC_FULL_SCALE_MV / ADC_MAX_RAW);
  }

  // Scale up through the voltage divider to get the actual battery voltage.
  int bat_mv = (int)((int64_t)adc_mv * (VDIV_R1_KOHM + VDIV_R2_KOHM) / VDIV_R2_KOHM);

  // Map [BAT_MIN_MV, BAT_MAX_MV] → [0, 100] % and clamp.
  int pct = (bat_mv - BAT_MIN_MV) * 100 / (BAT_MAX_MV - BAT_MIN_MV);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;

  if (DEBUG) printf("[Battery] raw=%d adc=%d mV bat=%d mV -> %d%%\n",
                    raw, adc_mv, bat_mv, pct);

  if (_handler && (uint8_t)pct != _lastPct) {
    _lastPct = (uint8_t)pct;
    _handler(_lastPct);
  }
}

void BatteryManager::_initCalibration() {
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
