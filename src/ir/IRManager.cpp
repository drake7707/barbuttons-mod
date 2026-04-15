#include "ir/IRManager.h"
#include "config/Config.h" // for IR_MAX_REPEATS

// ---------------------------------------------------------------------------
// NEC protocol timing (microseconds at 1 MHz RMT resolution = 1 µs / tick)
// ---------------------------------------------------------------------------
static constexpr uint32_t NEC_RMT_RESOLUTION_HZ = 1000000; // 1 MHz → 1 µs / tick
static constexpr uint16_t NEC_LEADER_MARK        = 9000;    // 9 ms burst
static constexpr uint16_t NEC_LEADER_SPACE       = 4500;    // 4.5 ms silence
static constexpr uint16_t NEC_BIT_MARK           = 560;     // ~562.5 µs burst
static constexpr uint16_t NEC_BIT_ZERO_SPACE     = 560;     // ~562.5 µs silence
static constexpr uint16_t NEC_BIT_ONE_SPACE      = 1690;    // ~1687.5 µs silence
static constexpr uint16_t NEC_STOP_MARK          = 560;     // final burst

// Symbols per frame: 1 header + 32 data bits + 1 stop = 34
static constexpr int NEC_FRAME_SYMBOLS = 34;

// Gap between repeated frames (ms)
static constexpr int NEC_INTER_FRAME_GAP_MS = 40;

// RX timing — maximum signal duration before the RMT peripheral treats the
// line as idle and fires the done callback.  Must be > 9 ms (leader mark).
static constexpr uint32_t NEC_RX_SIGNAL_RANGE_MAX_NS = 12000000; // 12 ms

// Duration tolerance: ±20 % is well within NEC spec variation.
static inline bool _matchDur(uint16_t actual, uint16_t expected)
{
  return actual > (expected * 4u / 5u) && actual < (expected * 6u / 5u);
}

// ---------------------------------------------------------------------------
void IRManager::begin(gpio_num_t pin)
{
  rmt_tx_channel_config_t txChanCfg = {};
  txChanCfg.gpio_num          = pin;
  txChanCfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  txChanCfg.resolution_hz     = NEC_RMT_RESOLUTION_HZ;
  txChanCfg.mem_block_symbols = 64; // must be ≥ NEC_FRAME_SYMBOLS (34)
  txChanCfg.trans_queue_depth = 4;

  if (rmt_new_tx_channel(&txChanCfg, &_txChannel) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to create RMT TX channel on GPIO %d\n", (int)pin);
    return;
  }

  // Modulate output at 38 kHz with ~33 % duty cycle (standard NEC carrier)
  rmt_carrier_config_t carrierCfg = {};
  carrierCfg.frequency_hz = 38000;
  carrierCfg.duty_cycle   = 0.33f;
  if (rmt_apply_carrier(_txChannel, &carrierCfg) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to apply carrier config\n");
    rmt_del_channel(_txChannel);
    _txChannel = nullptr;
    return;
  }

  rmt_copy_encoder_config_t copyEncCfg = {};
  if (rmt_new_copy_encoder(&copyEncCfg, &_copyEncoder) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to create copy encoder\n");
    rmt_del_channel(_txChannel);
    _txChannel = nullptr;
    return;
  }

  if (rmt_enable(_txChannel) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to enable RMT channel\n");
    rmt_del_encoder(_copyEncoder);
    rmt_del_channel(_txChannel);
    _copyEncoder = nullptr;
    _txChannel   = nullptr;
    return;
  }

  _initialized = true;
  if (DEBUG)
    printf("[IR] RMT TX initialised on GPIO %d\n", (int)pin);
}

// ---------------------------------------------------------------------------
void IRManager::sendNEC(uint16_t address, uint16_t command, uint8_t count)
{
  if (!_initialized)
    return;

  if (count == 0)
    count = 1;
  if (count > IR_MAX_REPEATS)
    count = IR_MAX_REPEATS;

  rmt_symbol_word_t symbols[NEC_FRAME_SYMBOLS];
  if (_buildFrame(address, command, symbols, NEC_FRAME_SYMBOLS) != NEC_FRAME_SYMBOLS)
    return;

  rmt_transmit_config_t txCfg = {};
  txCfg.loop_count = 0; // no hardware loop — we manage repetition manually

  for (uint8_t r = 0; r < count; r++)
  {
    esp_err_t err = rmt_transmit(_txChannel, _copyEncoder,
                                 symbols, sizeof(symbols), &txCfg);
    if (err == ESP_OK)
      rmt_tx_wait_all_done(_txChannel, pdMS_TO_TICKS(200));
    else if (DEBUG)
      printf("[IR] rmt_transmit error 0x%x\n", (int)err);

    if (r < (uint8_t)(count - 1u))
      vTaskDelay(pdMS_TO_TICKS(NEC_INTER_FRAME_GAP_MS));
  }

  if (DEBUG)
    printf("[IR] NEC addr=0x%04X cmd=0x%04X x%d\n", address, command, count);
}

// ---------------------------------------------------------------------------
// Build one full NEC extended frame into `out`.
// Returns NEC_FRAME_SYMBOLS on success, 0 if the buffer is too small.
// ---------------------------------------------------------------------------
int IRManager::_buildFrame(uint16_t address, uint16_t command,
                            rmt_symbol_word_t *out, int maxOut)
{
  if (maxOut < NEC_FRAME_SYMBOLS)
    return 0;

  int idx = 0;

  // Header
  out[idx].level0    = 1; out[idx].duration0 = NEC_LEADER_MARK;
  out[idx].level1    = 0; out[idx].duration1 = NEC_LEADER_SPACE;
  idx++;

  // 32 data bits: 16-bit address LSB-first, then 16-bit command LSB-first
  uint32_t data = ((uint32_t)command << 16) | address;
  for (int bit = 0; bit < 32; bit++)
  {
    bool one = (data >> bit) & 1u;
    out[idx].level0    = 1; out[idx].duration0 = NEC_BIT_MARK;
    out[idx].level1    = 0; out[idx].duration1 = one ? NEC_BIT_ONE_SPACE : NEC_BIT_ZERO_SPACE;
    idx++;
  }

  // Stop mark — level1/duration1 = 0 signals end-of-frame to the RMT peripheral
  out[idx].level0    = 1; out[idx].duration0 = NEC_STOP_MARK;
  out[idx].level1    = 0; out[idx].duration1 = 0;
  idx++;

  return idx; // == NEC_FRAME_SYMBOLS
}

// ---------------------------------------------------------------------------
// RX — initialise the RMT RX channel (TSOP38238 or compatible demodulator).
// ---------------------------------------------------------------------------
void IRManager::beginRx(gpio_num_t pin)
{
  rmt_rx_channel_config_t rxChanCfg = {};
  rxChanCfg.gpio_num          = pin;
  rxChanCfg.clk_src           = RMT_CLK_SRC_DEFAULT;
  rxChanCfg.resolution_hz     = NEC_RMT_RESOLUTION_HZ;
  rxChanCfg.mem_block_symbols = 64;

  if (rmt_new_rx_channel(&rxChanCfg, &_rxChannel) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to create RMT RX channel on GPIO %d\n", (int)pin);
    return;
  }

  _rxQueue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  if (!_rxQueue)
  {
    if (DEBUG)
      printf("[IR] Failed to create RX queue\n");
    rmt_del_channel(_rxChannel);
    _rxChannel = nullptr;
    return;
  }

  rmt_rx_event_callbacks_t cbs = {};
  cbs.on_recv_done = _rxDoneCallback;
  if (rmt_rx_register_event_callbacks(_rxChannel, &cbs, _rxQueue) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to register RX callback\n");
    vQueueDelete(_rxQueue);
    rmt_del_channel(_rxChannel);
    _rxQueue   = nullptr;
    _rxChannel = nullptr;
    return;
  }

  if (rmt_enable(_rxChannel) != ESP_OK)
  {
    if (DEBUG)
      printf("[IR] Failed to enable RMT RX channel\n");
    vQueueDelete(_rxQueue);
    rmt_del_channel(_rxChannel);
    _rxQueue   = nullptr;
    _rxChannel = nullptr;
    return;
  }

  _rxInitialized = true;
  if (DEBUG)
    printf("[IR] RMT RX initialised on GPIO %d\n", (int)pin);
}

// ---------------------------------------------------------------------------
// RX — ISR callback: forward received symbol data to the task queue.
// ---------------------------------------------------------------------------
bool IRAM_ATTR IRManager::_rxDoneCallback(rmt_channel_handle_t /*channel*/,
                                           const rmt_rx_done_event_data_t *edata,
                                           void *user_data)
{
  BaseType_t wakeup = pdFALSE;
  QueueHandle_t q   = static_cast<QueueHandle_t>(user_data);
  xQueueSendFromISR(q, edata, &wakeup);
  return wakeup == pdTRUE;
}

// ---------------------------------------------------------------------------
// RX — reset channel state so the next learnNEC call starts from a clean slate.
// ---------------------------------------------------------------------------
void IRManager::_resetRx()
{
  if (!_rxInitialized)
    return;
  rmt_disable(_rxChannel);
  rmt_enable(_rxChannel);
  xQueueReset(_rxQueue);
}

// ---------------------------------------------------------------------------
// RX — arm the receiver and wait up to timeoutMs for a valid NEC frame.
// ---------------------------------------------------------------------------
bool IRManager::learnNEC(uint16_t &address, uint16_t &command, uint32_t timeoutMs)
{
  if (!_rxInitialized)
    return false;

  xQueueReset(_rxQueue);

  rmt_receive_config_t rxCfg       = {};
  rxCfg.signal_range_min_ns        = 1250;                    // ~1.25 µs glitch filter
  rxCfg.signal_range_max_ns        = NEC_RX_SIGNAL_RANGE_MAX_NS; // 12 ms idle = end of frame

  esp_err_t err = rmt_receive(_rxChannel, _rxSymbols, sizeof(_rxSymbols), &rxCfg);
  if (err != ESP_OK)
  {
    // Channel may be stuck from a previous timeout — reset and retry once.
    _resetRx();
    err = rmt_receive(_rxChannel, _rxSymbols, sizeof(_rxSymbols), &rxCfg);
    if (err != ESP_OK)
    {
      if (DEBUG)
        printf("[IR] rmt_receive failed: 0x%x\n", (int)err);
      return false;
    }
  }

  rmt_rx_done_event_data_t rxData = {};
  if (xQueueReceive(_rxQueue, &rxData, pdMS_TO_TICKS(timeoutMs)) != pdTRUE)
  {
    // Timeout — reset so the channel is not left in a receiving state.
    _resetRx();
    if (DEBUG)
      printf("[IR] learnNEC timeout\n");
    return false;
  }

  bool ok = _decodeNEC(rxData.received_symbols, rxData.num_symbols, address, command);
  _resetRx(); // always reset after a capture so the next call is clean
  if (DEBUG)
  {
    if (ok)
      printf("[IR] learnNEC decoded addr=0x%04X cmd=0x%04X\n", address, command);
    else
      printf("[IR] learnNEC decode failed (%zu symbols)\n", rxData.num_symbols);
  }
  return ok;
}

// ---------------------------------------------------------------------------
// RX — decode raw RMT symbols into a NEC extended (32-bit) frame.
//
// The TSOP demodulator output is active-LOW when carrier is detected, so:
//   symbol[n].level0 == 0  →  carrier burst (mark)
//   symbol[n].level1 == 1  →  silence (space)
// ---------------------------------------------------------------------------
bool IRManager::_decodeNEC(const rmt_symbol_word_t *symbols, size_t count,
                            uint16_t &address, uint16_t &command)
{
  // Need at least a full NEC frame (34 symbols).
  if (count < (size_t)NEC_FRAME_SYMBOLS)
    return false;

  // Header: 9 ms mark + 4.5 ms space.
  if (!_matchDur(symbols[0].duration0, NEC_LEADER_MARK))
    return false;
  if (!_matchDur(symbols[0].duration1, NEC_LEADER_SPACE))
    return false;

  // 32 data bits, LSB first.
  uint32_t data = 0;
  for (int bit = 0; bit < 32; bit++)
  {
    const rmt_symbol_word_t &sym = symbols[1 + bit];
    if (!_matchDur(sym.duration0, NEC_BIT_MARK))
      return false;
    if (_matchDur(sym.duration1, NEC_BIT_ONE_SPACE))
      data |= (1u << bit);
    else if (!_matchDur(sym.duration1, NEC_BIT_ZERO_SPACE))
      return false; // invalid space duration
  }

  // Stop mark.
  if (!_matchDur(symbols[33].duration0, NEC_STOP_MARK))
    return false;

  // Bits 0..15 = 16-bit address; bits 16..31 = 16-bit command (matches _buildFrame).
  address = static_cast<uint16_t>(data & 0xFFFFu);
  command = static_cast<uint16_t>((data >> 16) & 0xFFFFu);
  return true;
}
