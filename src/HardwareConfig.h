#pragma once

#include <esp_adc/adc_oneshot.h>  // adc_channel_t

// ---------------------------------------------------------------------------
// HardwareConfig — pin assignments for the BarButtons board
// Edit this file when porting to a different board or PCB revision.
// ---------------------------------------------------------------------------

// Status LED
const int LED_PIN_LEGACY = 6;
const int LED_PIN = 2;

// Keypad matrix — column drive lines.
static const uint8_t KEYPAD_COL_PINS_LEGACY[3] = {3, 4, 5};  // battery disabled
static const uint8_t KEYPAD_COL_PINS[3]  = {3, 4, 5};  // battery enabled

inline const uint8_t getLEDPin(bool isLegacy) {
  return isLegacy ? LED_PIN_LEGACY : LED_PIN;
}

// Returns the correct column-pin array for the given battery mode.
inline const uint8_t* getKeypadColPins(bool isLegacy) {
  return isLegacy ? KEYPAD_COL_PINS_LEGACY : KEYPAD_COL_PINS;
}

// Keypad matrix — row drive lines. I moved the pins for the rows because ADC1 only works on GPIO0-5, and I wanted to use GPIO0 for battery voltage sensing. 
// The legacy pinout is preserved for older hardware builds that don't have battery support.
static const uint8_t KEYPAD_ROW_PINS_LEGACY[3] = {2, 1, 0};
static const uint8_t KEYPAD_ROW_PINS[3] = {8, 7, 6};  

// Returns the correct row-pin array for the given battery mode.
inline const uint8_t* getKeypadRowPins(bool isLegacy) {
  return isLegacy ? KEYPAD_ROW_PINS_LEGACY : KEYPAD_ROW_PINS;
}


// Battery voltage sense — GPIO0 = ADC1 channel 0 (only active when battery is enabled).
// Voltage is read through a 680 kΩ / 220 kΩ voltage divider so that the
// li-ion cell voltage (up to 4.2 V) is scaled down to the ADC input range.
const adc_channel_t ADC_BATTERY_CHANNEL = ADC_CHANNEL_0; // GPIO0 → ADC1 CH0 on ESP32-C3
