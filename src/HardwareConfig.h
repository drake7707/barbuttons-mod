#pragma once

#include <esp_adc/adc_oneshot.h>  // adc_channel_t

// ---------------------------------------------------------------------------
// HardwareConfig — pin assignments for the BarButtons board
// Edit this file when porting to a different board or PCB revision.
// ---------------------------------------------------------------------------

// Status LED
const int LED_PIN = 6;

// Keypad matrix — column drive lines (same regardless of battery mode).
const uint8_t KEYPAD_COL_PINS[] = {3, 4, 5};  // colPins[3]

// Keypad row pins come in two flavours depending on whether the battery ADC
// is enabled at runtime.
//   Default (battery disabled): GPIO0 is available as keypad row 2.
//   Battery enabled:            GPIO0 is reserved for ADC; row 2 moves to GPIO7.
static const uint8_t KEYPAD_ROW_PINS_DEFAULT[3] = {2, 1, 0};  // battery disabled
static const uint8_t KEYPAD_ROW_PINS_BATTERY[3] = {2, 1, 7};  // battery enabled

// Returns the correct row-pin array for the given battery mode.
inline const uint8_t* getKeypadRowPins(bool withBattery) {
  return withBattery ? KEYPAD_ROW_PINS_BATTERY : KEYPAD_ROW_PINS_DEFAULT;
}

// Battery voltage sense — GPIO0 = ADC1 channel 0 (only active when battery is enabled).
// Voltage is read through a 680 kΩ / 220 kΩ voltage divider so that the
// li-ion cell voltage (up to 4.2 V) is scaled down to the ADC input range.
const adc_channel_t ADC_BATTERY_CHANNEL = ADC_CHANNEL_0; // GPIO0 → ADC1 CH0 on ESP32-C3
