#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include "StatusLedManager.h"
#include "KeyCodes.h"

extern const int DEBUG;

// ---------------------------------------------------------------------------
// ConfigManager — manages keymap / BLE-name NVS storage, the config AP and
// the web server routes.  All persistent settings live here.
// ---------------------------------------------------------------------------

#define BLE_NAME_MAX_LEN 32
const char DEFAULT_BLE_NAME[] = "BarButtonsMod";

// ---------------------------------------------------------------------------
// Config web page  (stored in program flash, not RAM)
// The server replaces the placeholders SHORTVALS, LONGVALS, and BLENAME with
// the current keymap values and BLE device name before sending.
// ---------------------------------------------------------------------------
const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BarButtons Mod Keymap</title>
<style>
body{font-family:sans-serif;max-width:740px;margin:20px auto;padding:0 12px}
h2{color:#2c2c2c;margin-bottom:4px}
p.sub{color:#666;font-size:.9em;margin-top:0}
footer{margin-top:32px;padding-top:12px;border-top:1px solid #ddd;color:#999;font-size:.82em;text-align:center}
footer a{color:#888}
table{width:100%;border-collapse:collapse;margin-top:12px}
th,td{padding:7px 10px;text-align:left;border:1px solid #ddd}
th{background:#f5f5f5;font-weight:600}
select{width:100%;box-sizing:border-box;padding:3px}
.save{margin-top:16px;background:#2a7cdf;color:#fff;border:none;
      padding:10px 28px;font-size:1em;border-radius:4px;cursor:pointer}
.save:hover{background:#1a5cb0}
.hint{color:#888;font-size:.82em;margin-top:6px}
.danger{background:#c0392b}.danger:hover{background:#962d22}
.field{margin-top:14px}
.field label{font-weight:600;display:block;margin-bottom:4px}
.field input[type=text]{width:100%;max-width:320px;box-sizing:border-box;
  padding:5px 8px;font-size:1em;border:1px solid #ccc;border-radius:3px}
</style>
</head>
<body>
<h2>BarButtons Mod Keymap Config</h2>
<p class="sub">AP configuration mode</p>
<form method="POST" action="/save">
<table>
<thead>
  <tr><th>Button</th><th>Short Press</th><th>Long Press</th></tr>
</thead>
<tbody id="rows"></tbody>
</table>
<p class="hint">Button 4 long-press = always enters this config mode (not remappable).<br>
Tap Button 4 on the device to exit without saving.</p>
<div class="field">
<label for="blename">BLE Device Name:</label>
<input type="text" id="blename" name="blename" value="BLENAME"
       maxlength="32" pattern="[ -~]+" required
       placeholder="BarButtonsMod">
<p class="hint">1&#8211;32 printable ASCII characters &mdash; shown in the Bluetooth pairing dialog.</p>
</div>
<input type="submit" class="save" value="Save &amp; Reboot">
</form>
<hr style="margin:24px 0">
<h3>Firmware Update</h3>
<form method="POST" action="/update" enctype="multipart/form-data" id="otaFrm">
<p><input type="file" name="firmware" accept=".bin" required></p>
<input type="submit" class="save" value="Flash Firmware" id="otaBtn">
<p class="hint" id="otaSt">Upload a .bin compiled for this board. Device reboots after a successful flash.</p>
</form>
<hr style="margin:24px 0">
<h3>BLE Bonds</h3>
<p class="sub">Clear stored pairing data if the device fails to reconnect after a reset. You will need to re-pair your phone afterwards.</p>
<form method="POST" action="/clearbonds" id="bondFrm">
<input type="submit" class="save danger" value="Clear BLE Bonds &amp; Reboot" id="bondBtn">
</form>
<hr style="margin:24px 0">
<h3>Reset to Defaults</h3>
<p class="sub">Fill in the default keymap and BLE device name. Save &amp; Reboot to apply.</p>
<button type="button" class="save danger" onclick="resetToDefaults()">Reset to Defaults</button>
<script>
document.getElementById('otaFrm').onsubmit=function(){
  var b=document.getElementById('otaBtn');
  b.value='Uploading\u2026';b.disabled=true;
  document.getElementById('otaSt').textContent='Uploading \u2014 do not close this page\u2026';
};
document.getElementById('bondFrm').onsubmit=function(){
  if(!confirm('Clear all BLE bonds?\nThe device will reboot. You will need to re-pair your phone.')) return false;
  var b=document.getElementById('bondBtn');
  b.value='Clearing\u2026';b.disabled=true;
};
var DS=[DEFAULTSHORT];
var DL=[DEFAULTLONG];
function resetToDefaults(){
  var rows=document.getElementById('rows').rows;
  for(var i=0;i<8;i++){
    var sels=rows[i].querySelectorAll('select');
    if(sels[0]) sels[0].value=DS[i];
    if(sels[1]) sels[1].value=DL[i];
  }
  document.getElementById('blename').value=DEFAULTBLENAME;
}
// Key options: [code, label]
var K=[
  [43,'+'],[45,'-'],[42,'*'],[47,'/'],[61,'='],
  [32,'Space'],[46,'.'],[44,','],[59,';'],[58,':'],
  [33,'!'],[63,'?'],[64,'@'],[35,'#'],[37,'%'],
  [176,'Enter'],[177,'Esc'],[178,'Backspace'],[179,'Tab'],
  [209,'Insert'],[210,'Home'],[211,'Page Up'],
  [212,'Delete'],[213,'End'],[214,'Page Down'],
  [218,'Up Arrow'],[217,'Down Arrow'],[216,'Left Arrow'],[215,'Right Arrow'],
  [193,'Caps Lock'],
  [194,'F1'],[195,'F2'],[196,'F3'],[197,'F4'],[198,'F5'],[199,'F6'],
  [200,'F7'],[201,'F8'],[202,'F9'],[203,'F10'],[204,'F11'],[205,'F12']
];
for(var i=97;i<=122;i++) K.push([i, String.fromCharCode(i)]); // a-z
for(var i=48;i<=57;i++)  K.push([i, String.fromCharCode(i)]); // 0-9

var S=[SHORTVALS];
var L=[LONGVALS];

function buildOpts(cur, withRepeat) {
  var h = '';
  if (withRepeat)
    h += '<option value="0"' + (cur === 0 ? ' selected' : '') + '>\u21BA Repeat short key</option>';
  K.forEach(function(k) {
    var sel = (k[0] === cur && !(withRepeat && cur === 0)) ? ' selected' : '';
    h += '<option value="' + k[0] + '"' + sel + '>' + k[1] + '</option>';
  });
  return h;
}

var tbody = document.getElementById('rows');
for (var i = 0; i < 8; i++) {
  var tr = document.createElement('tr');
  // Button 4 (index 3) long press is reserved — show as disabled info cell
  var longCell = (i === 3)
    ? '<td><em style="color:#999">reserved (config trigger)</em><input type="hidden" name="l3" value="' + L[3] + '"></td>'
    : '<td><select name="l' + i + '">' + buildOpts(L[i], true) + '</select></td>';
  tr.innerHTML =
    '<td><strong>Button ' + (i + 1) + '</strong></td>' +
    '<td><select name="s' + i + '">' + buildOpts(S[i], false) + '</select></td>' +
    longCell;
  tbody.appendChild(tr);
}
</script>
<footer>BarButtonsMod vFWVER &mdash; by <a href="https://github.com/drake7707/" target="_blank" rel="noopener">Drakarah</a> &mdash; <a href="https://github.com/drake7707/" target="_blank" rel="noopener">github.com/drake7707</a></footer>
</body>
</html>
)rawliteral";

class ConfigManager {
public:
  // Factory defaults — mirror the original hard-coded behaviour
  static const uint8_t DEFAULT_SHORT[8];
  static const uint8_t DEFAULT_LONG[8];

  // Inject the StatusLedManager so web handlers can signal progress via LED,
  // and the firmware version string to display in the web config UI.
  void begin(StatusLedManager* led, const char* firmwareVersion) {
    _led = led;
    strncpy(_firmwareVersion, firmwareVersion, sizeof(_firmwareVersion) - 1);
    _firmwareVersion[sizeof(_firmwareVersion) - 1] = '\0';
  }

  // ---------------------------------------------------------------------------
  // NVS — keymap
  // ---------------------------------------------------------------------------
  void loadKeymap() {
    _prefs.begin("keymap", /*readOnly=*/true);
    for (int i = 0; i < 8; i++) {
      _short[i] = _prefs.getUChar(("s" + String(i)).c_str(), DEFAULT_SHORT[i]);
      _long[i]  = _prefs.getUChar(("l" + String(i)).c_str(), DEFAULT_LONG[i]);
    }
    _prefs.end();
    if (DEBUG) {
      Serial.println("Keymap loaded from NVS:");
      for (int i = 0; i < 8; i++) {
        Serial.printf("  btn%d  short=%d  long=%d\n", i + 1, _short[i], _long[i]);
      }
    }
  }

  void saveKeymap() {
    _prefs.begin("keymap", /*readOnly=*/false);
    for (int i = 0; i < 8; i++) {
      _prefs.putUChar(("s" + String(i)).c_str(), _short[i]);
      _prefs.putUChar(("l" + String(i)).c_str(), _long[i]);
    }
    _prefs.end();
    if (DEBUG) Serial.println("Keymap saved to NVS.");
  }

  // ---------------------------------------------------------------------------
  // NVS — BLE device name
  // ---------------------------------------------------------------------------
  void loadBleName() {
    _prefs.begin("config", /*readOnly=*/true);
    String saved = _prefs.getString("blename", DEFAULT_BLE_NAME);
    saved.toCharArray(_bleName, sizeof(_bleName));
    _prefs.end();
    if (DEBUG) Serial.printf("BLE name loaded: %s\n", _bleName);
  }

  void saveBleName() {
    _prefs.begin("config", /*readOnly=*/false);
    _prefs.putString("blename", _bleName);
    _prefs.end();
    if (DEBUG) Serial.printf("BLE name saved: %s\n", _bleName);
  }

  // ---------------------------------------------------------------------------
  // NVS — "clear bonds" flag (written before reboot; consumed after next init)
  // ---------------------------------------------------------------------------

  // Write a flag requesting bond deletion; caller should reboot afterwards.
  void requestClearBonds() {
    _prefs.begin("sys", false);
    _prefs.putBool("clrbond", true);
    _prefs.end();
  }

  // Returns true if the flag is set (called after NimBLE is initialised).
  bool isClearBondsRequested() {
    _prefs.begin("sys", /*readOnly=*/true);
    bool flag = _prefs.getBool("clrbond", false);
    _prefs.end();
    return flag;
  }

  // Clears the flag once bonds have been deleted.
  void clearClearBondsFlag() {
    _prefs.begin("sys", false);
    _prefs.remove("clrbond");
    _prefs.end();
  }

  // ---------------------------------------------------------------------------
  // Keymap / BLE name accessors
  // ---------------------------------------------------------------------------
  uint8_t getShortKey(int idx) const { return (idx >= 0 && idx < 8) ? _short[idx] : 0; }
  uint8_t getLongKey(int idx)  const { return (idx >= 0 && idx < 8) ? _long[idx]  : 0; }
  const char* getBleName()     const { return _bleName; }

  // Map button label '1'-'8' → keymap index 0-7. Returns -1 for unmapped keys.
  static int btnIndex(char key) {
    if (key >= '1' && key <= '8') return key - '1';
    return -1;
  }

  // A button fires on key-down (instant) when its long-press mode is auto-repeat.
  // Button 4 (idx 3) is excluded: its long-press slot is reserved for config mode,
  // so it must never be instant-fired on key-down regardless of the stored value.
  bool isKeyInstant(char key) const {
    int idx = btnIndex(key);
    if (idx < 0) return false;
    if (idx == 3) return false; // button 4 long-press is reserved; always non-instant
    return _long[idx] == 0; // 0 = repeat short key → instant fire
  }

  // ---------------------------------------------------------------------------
  // AP / WebServer — config mode
  // ---------------------------------------------------------------------------

  // Set up the WiFi AP and register web server routes.
  // Call drainButton() on ButtonManager after this returns, then setStatus(APP_CONFIG).
  void beginConfigAP() {
    _exitConfig = false;

    WiFi.softAP(_apSsid, _apPswd);
    if (DEBUG) {
      Serial.print("AP started: ");
      Serial.println(WiFi.softAPIP());
    }

    _server.on("/",           HTTP_GET,  [this]() { handleRoot();      });
    _server.on("/save",       HTTP_POST, [this]() { handleSave();      });
    _server.on("/clearbonds", HTTP_POST, [this]() { handleClearBonds(); });
    _server.on("/update",     HTTP_POST,
               [this]() { handleOtaFinish(); },
               [this]() { handleOtaUpload(); });
    _server.begin();

    // 5 quick flashes to signal we're in config mode
    if (_led) _led->flashLed(5, 100, 100);
  }

  // Process pending web server requests — call from the config-mode loop.
  void handleClient() { _server.handleClient(); }

  // Stop the AP and tear down the web server.
  void endConfigAP() {
    _server.stop();
    _server.close();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200);
  }

  bool isExitRequested() const  { return _exitConfig; }
  void setExitRequested(bool v) { _exitConfig = v; }

private:
  StatusLedManager* _led             = nullptr;
  bool              _exitConfig       = false;
  char              _firmwareVersion[32] = {};

  uint8_t _short[8] = {};
  uint8_t _long[8]  = {};
  char    _bleName[BLE_NAME_MAX_LEN + 1] = "BarButtonsMod";

  Preferences _prefs;
  WebServer   _server{80};

  const char* _apSsid = "BarButtons-Config";
  const char* _apPswd = "barbuttons";

  // ---------------------------------------------------------------------------
  // Web handler implementations
  // ---------------------------------------------------------------------------
  void handleRoot() {
    String html = FPSTR(CONFIG_HTML);
    String sv, lv;
    for (int i = 0; i < 8; i++) {
      if (i) { sv += ','; lv += ','; }
      sv += String(_short[i]);
      lv += String(_long[i]);
    }
    html.replace("SHORTVALS", sv);
    html.replace("LONGVALS",  lv);

    // Build default keymap lists for the JS reset function
    String dsv, dlv;
    for (int i = 0; i < 8; i++) {
      if (i) { dsv += ','; dlv += ','; }
      dsv += String(DEFAULT_SHORT[i]);
      dlv += String(DEFAULT_LONG[i]);
    }
    html.replace("DEFAULTSHORT", dsv);
    html.replace("DEFAULTLONG",  dlv);

    // Escape BLE name for safe use in an HTML attribute value
    String bn;
    for (int i = 0; _bleName[i]; i++) {
      char c = _bleName[i];
      if      (c == '&') bn += "&amp;";
      else if (c == '"') bn += "&quot;";
      else if (c == '<') bn += "&lt;";
      else if (c == '>') bn += "&gt;";
      else               bn += c;
    }
    html.replace("BLENAME", bn);

    // Escape default BLE name for safe embedding as a JS string literal
    String dbn;
    for (int i = 0; DEFAULT_BLE_NAME[i]; i++) {
      char c = DEFAULT_BLE_NAME[i];
      if      (c == '\\') dbn += "\\\\";
      else if (c == '\'') dbn += "\\'";
      else                dbn += c;
    }
    html.replace("DEFAULTBLENAME", "'" + dbn + "'");

    html.replace("FWVER", String(_firmwareVersion));

    _server.send(200, "text/html", html);
  }

  void handleSave() {
    for (int i = 0; i < 8; i++) {
      String si = String(i);
      if (_server.hasArg("s" + si)) _short[i] = (uint8_t)_server.arg("s" + si).toInt();
      if (_server.hasArg("l" + si)) _long[i]  = (uint8_t)_server.arg("l" + si).toInt();
    }
    saveKeymap();

    if (_server.hasArg("blename")) {
      String newName = _server.arg("blename");
      newName.trim();
      // Validate: 1–32 printable ASCII characters
      bool valid = newName.length() > 0 && newName.length() <= BLE_NAME_MAX_LEN;
      if (valid) {
        for (unsigned int j = 0; j < newName.length(); j++) {
          if (newName[j] < 0x20 || newName[j] > 0x7E) { valid = false; break; }
        }
      }
      if (valid) {
        newName.toCharArray(_bleName, sizeof(_bleName));
        saveBleName();
      }
    }

    _server.send(200, "text/html",
      "<!DOCTYPE html><html>"
      "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Saved!</h2><p>Rebooting&hellip;</p>"
      "</body></html>");

    if (_led) _led->flashLed(3, 80, 80);
    delay(800);
    ESP.restart();
  }

  void handleClearBonds() {
    // BLE is stopped while the WiFi AP is running (shared radio on ESP32-C3),
    // so we can't delete bonds here. Write a flag to NVS and reboot — setup()
    // will delete the bonds once NimBLE is initialised again.
    requestClearBonds();

    _server.send(200, "text/html",
      "<!DOCTYPE html><html>"
      "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Bonds cleared!</h2><p>Rebooting&hellip; Re-pair your phone when the device is discoverable.</p>"
      "</body></html>");

    if (_led) _led->flashLed(3, 80, 80);
    delay(800);
    ESP.restart();
  }

  // Called repeatedly during the POST body stream — writes each chunk to flash.
  void handleOtaUpload() {
    HTTPUpload& upload = _server.upload();

    if (upload.status == UPLOAD_FILE_START) {
      if (DEBUG) Serial.printf("OTA start: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        if (DEBUG) Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        if (DEBUG) Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(/*setMD5=*/true)) {
        if (DEBUG) Serial.printf("OTA written: %u bytes\n", upload.totalSize);
      } else {
        if (DEBUG) Update.printError(Serial);
      }
    }
  }

  // Called once after the full upload body is received — sends the result page.
  void handleOtaFinish() {
    bool ok = !Update.hasError();
    _server.send(200, "text/html",
      ok
        ? "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
          "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
          "<h2>&#10003; Update successful!</h2><p>Rebooting&hellip;</p></body></html>"
        : "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
          "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
          "<h2>&#10007; Update failed.</h2><p><a href='/'>Try again</a></p></body></html>");
    if (ok) {
      if (_led) _led->flashLed(5, 50, 50);
      delay(500);
      ESP.restart();
    }
  }
};

// Out-of-class definitions for static const members
const uint8_t ConfigManager::DEFAULT_SHORT[8] = {
  '+', '-', 'n', 'c',
  KEY_UP_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_DOWN_ARROW
};
const uint8_t ConfigManager::DEFAULT_LONG[8] = {
  0,   // btn1: repeat '+'
  0,   // btn2: repeat '-'
  'd', // btn3: long = 'd', short = 'n'
  0,   // btn4: reserved for config trigger (not configurable)
  0,   // btn5: repeat UP
  0,   // btn6: repeat LEFT
  0,   // btn7: repeat RIGHT
  0    // btn8: repeat DOWN
};
