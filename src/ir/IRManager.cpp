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
