#pragma once

// ---------------------------------------------------------------------------
// HardwareConfig — pin assignments for the BarButtons board
// Edit this file when porting to a different board or PCB revision.
// ---------------------------------------------------------------------------

// Status LED
const int LED_PIN = 6;

// Keypad matrix — row and column GPIO pins
// Rows are the sense lines, columns are the drive lines.
const byte KEYPAD_ROW_PINS[] = {2, 1, 0};  // rowPins[3]
const byte KEYPAD_COL_PINS[] = {3, 4, 5};  // colPins[3]
