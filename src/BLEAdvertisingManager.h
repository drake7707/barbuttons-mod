#pragma once

#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <map>
#include <string>
#include <algorithm>

extern const int DEBUG;

// ---------------------------------------------------------------------------
// BLEAdvertisingManager — sequential directed + undirected advertising cycle
//
// On each startCycle() call the sequence is:
//   1. Directed advertising to every unconnected bonded peer in NVS order,
//      DIRECTED_ADV_STEP_DURATION_MS per peer.
//   2. Undirected advertising for the remainder of the time budget
//      (infinite when no connection exists yet).
//
// When a connection already exists the total cycle is capped at
// MAX_ADVERTISING_DURATION_AFTER_ALREADY_CONNECTED_MS to save battery.
// ---------------------------------------------------------------------------
class BLEAdvertisingManager
{
public:
  // connections — live reference to BLEManager's connection map
  BLEAdvertisingManager(const std::map<std::string, uint16_t> &connections);

  // Configure advertising data and register the advertising-complete callback.
  // maxConnections is taken here because it is set via the web UI before begin()
  // is called, not at construction time.
  // Must be called once after the HID device is fully set up.
  void begin(NimBLEHIDDevice *hid, const char *deviceName, uint8_t maxConnections);

  // Start a fresh advertising cycle (directed bonds first, then undirected).
  // Resets internal state; safe to call at any time.
  void startCycle();

  bool isAdvertising();

  // BTHome v2 button event types (object ID 0x3A).
  // See https://bthome.io/format/ for the full list.
  static const uint8_t BTHOME_BUTTON_NONE         = 0x00;
  static const uint8_t BTHOME_BUTTON_PRESS        = 0x01;
  static const uint8_t BTHOME_BUTTON_DOUBLE_PRESS = 0x02;
  static const uint8_t BTHOME_BUTTON_TRIPLE_PRESS = 0x03;
  static const uint8_t BTHOME_BUTTON_LONG_PRESS   = 0x04;
  static const uint8_t BTHOME_BUTTON_LONG_DOUBLE  = 0x05;
  static const uint8_t BTHOME_BUTTON_LONG_TRIPLE  = 0x06;

  // Send a compliant BTHome v2 advertisement for the given button (1–8) and
  // event type (one of the BTHOME_BUTTON_* constants above).
  //
  // The advertisement is trigger-based (device info 0x44) and contains one
  // Button object (0x3A) per button slot; only the active button slot carries
  // a non-zero event value.  If HID advertising was active at the time of the
  // call it is paused for the broadcast duration (~2 s) and then restarted;
  // if HID advertising was not running (budget exhausted, connection limit, …)
  // it is NOT restarted so the idle state is preserved.
  void broadcastBTHomeButtonPress(uint8_t eventType, uint8_t button);

private:
  const std::map<std::string, uint16_t> &_connections;
  uint8_t _maxConnections = 0;
  int _nextBondIdx = 0;
  uint32_t _advertisingCycleStartMs = 0;

  NimBLEHIDDevice *_hid = nullptr;
  std::string _deviceName;
  bool _btHomeBroadcastActive = false;
  // True if HID advertising was running when a BTHome broadcast was started;
  // used to decide whether to restart the HID cycle after the broadcast ends.
  bool _restoreHIDAfterBTHome = false;

  // Advance to the next advertising step.
  void advance();

  // (Re-)configure the NimBLEAdvertising object with HID keyboard advertisement
  // data.  Called once from begin() and again after every BTHome broadcast to
  // restore the normal HID advertising payload.
  void configureHIDAdvertising();

  // Total advertising window when at least one connection is already active.
  // Once this budget is spent the cycle stops to save battery.
  static const uint32_t MAX_ADVERTISING_DURATION_AFTER_ALREADY_CONNECTED_MS = 60000;

  // Per-bond directed advertising window.  Directed high-duty advertising has a
  // hardware-enforced ~1.28 s controller timeout; 1500 ms sits safely above it
  // so NimBLE always fires the advertising-complete callback before we attempt
  // to start the next step.
  static const uint32_t DIRECTED_ADV_STEP_DURATION_MS = 1500;

  // Duration of a single BTHome broadcast in milliseconds.
  // 2000 ms gives Home Assistant several scan windows to catch the advertisement.
  static const uint32_t BTHOME_BROADCAST_DURATION_MS = 2000;
};
