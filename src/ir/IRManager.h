#pragma once

#include <cstdint>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

extern const int DEBUG;

// ---------------------------------------------------------------------------
// IRManager — owns the RMT TX and (optionally) RX channels for the IR LED
// and receiver.  Implements NEC extended protocol (16-bit address + 16-bit
// command, LSB first).
//
// TX safety: the IR LED is driven through a BJT at up to 200 mA; transmissions
// are capped at IR_MAX_REPEATS full NEC frames per button press to prevent
// overheating from sustained current draw.
//
// RX usage: call beginRx() once at startup.  Call learnNEC() from the web
// /ir-learn handler — it arms the receiver, waits up to timeoutMs for a valid
// NEC frame, then disarms and returns the decoded address and command.
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

  // Initialise the RMT RX channel on the given GPIO (TSOP38238 or compatible).
  // The demodulator output is active-LOW; the RMT peripheral sees mark/space
  // pulses directly.  Safe to call even when no receiver is wired — learnNEC
  // will simply return false.
  void beginRx(gpio_num_t pin);

  // Arm the receiver and block until a valid NEC extended frame is received or
  // timeoutMs elapses.  Fills address and command on success.  Returns true on
  // a successful decode, false on timeout or framing error.
  // Automatically resets the RX channel state before returning so the next
  // call is safe.
  bool learnNEC(uint16_t &address, uint16_t &command, uint32_t timeoutMs);

  bool isRxInitialized() const { return _rxInitialized; }

private:
  // TX
  rmt_channel_handle_t _txChannel   = nullptr;
  rmt_encoder_handle_t _copyEncoder = nullptr;
  bool                 _initialized = false;

  // RX
  rmt_channel_handle_t _rxChannel     = nullptr;
  QueueHandle_t        _rxQueue       = nullptr;
  bool                 _rxInitialized = false;
  rmt_symbol_word_t    _rxSymbols[64] = {};

  // Reset the RX channel so the next learnNEC call starts cleanly.
  // Called after a timeout or a successful decode.
  void _resetRx();

  // ISR-context callback registered with rmt_rx_register_event_callbacks.
  static bool IRAM_ATTR _rxDoneCallback(rmt_channel_handle_t channel,
                                        const rmt_rx_done_event_data_t *edata,
                                        void *user_data);

  // Decode a raw RMT symbol sequence into a NEC extended frame.
  // Returns true and fills address/command on success.
  static bool _decodeNEC(const rmt_symbol_word_t *symbols, size_t count,
                         uint16_t &address, uint16_t &command);

  // Encode one full NEC extended frame (1 header + 32 data bits + 1 stop)
  // into `out`.  Returns the number of rmt_symbol_word_t entries written (34).
  static int _buildFrame(uint16_t address, uint16_t command,
                         rmt_symbol_word_t *out, int maxOut);
};
