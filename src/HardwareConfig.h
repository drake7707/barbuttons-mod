#pragma once

// ---------------------------------------------------------------------------
// HardwareConfig — pin assignments for the BarButtons board
// Edit this file when porting to a different board or PCB revision.
// ---------------------------------------------------------------------------

// Status LED
const int LED_PIN = 6;

// Keypad matrix — row and column GPIO pins
// Rows are the sense lines, columns are the drive lines.
// Row 2 is on GPIO7 (GPIO0 is reserved for the battery ADC input).
const uint8_t KEYPAD_ROW_PINS[] = {2, 1, 7};  // rowPins[3]
const uint8_t KEYPAD_COL_PINS[] = {3, 4, 5};  // colPins[3]

// Battery voltage sense — GPIO0 = ADC1 channel 0.
// Voltage is read through a 680 kΩ / 220 kΩ voltage divider so that the
// li-ion cell voltage (up to 4.2 V) is scaled down to the ADC input range.
const uint8_t ADC_BATTERY_PIN = 0;
