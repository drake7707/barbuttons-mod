#pragma once

#include <cstdint>
#include <driver/rmt_tx.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern const int DEBUG;

// ---------------------------------------------------------------------------
// IRManager — owns the RMT TX channel used to drive the IR LED.
// Implements NEC extended protocol (16-bit address + 16-bit command, LSB first).
//
// Safety: the IR LED is driven through a BJT at up to 200 mA; transmissions
// are capped at IR_MAX_REPEATS full NEC frames per button press to prevent
// overheating from sustained current draw.
// ---------------------------------------------------------------------------
class IRManager
{
public:
  // Initialise the RMT TX channel on the given GPIO.
  // Safe to call even if IR is never used; other managers are unaffected.
  void begin(gpio_num_t pin);

  // Transmit `count` full NEC extended frames (clamped to IR_MAX_REPEATS).
  // Blocks until all frames have been sent.
  // address and command are 16-bit NEC extended values; each is sent LSB first.
  void sendNEC(uint16_t address, uint16_t command, uint8_t count);

  bool isInitialized() const { return _initialized; }

private:
  rmt_channel_handle_t _txChannel   = nullptr;
  rmt_encoder_handle_t _copyEncoder = nullptr;
  bool                 _initialized = false;

  // Encode one full NEC extended frame (1 header + 32 data bits + 1 stop)
  // into `out`.  Returns the number of rmt_symbol_word_t entries written (34).
  static int _buildFrame(uint16_t address, uint16_t command,
                         rmt_symbol_word_t *out, int maxOut);
};
