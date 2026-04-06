#pragma once

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "StatusLedManager.h"
#include "KeyCodes.h"

extern const int DEBUG;

// ---------------------------------------------------------------------------
// ConfigManager -- manages keymap / BLE-name NVS storage, the config AP and
// the web server routes.  All persistent settings live here.
// ---------------------------------------------------------------------------

#define BLE_NAME_MAX_LEN 32
const char DEFAULT_BLE_NAME[] = "BarButtonsMod";

// ---------------------------------------------------------------------------
// Config web page  (stored in .rodata flash section)
// The server replaces the placeholders SHORTVALSn, LONGVALSn, ACTIVEKEYMAP,
// BLENAME, etc. with current values before sending.
// ---------------------------------------------------------------------------
const char CONFIG_HTML[] = R"rawliteral(
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
.tab{cursor:pointer;padding:7px 18px;border:1px solid #ccc;background:#f5f5f5;
     font-size:.95em;border-radius:3px 3px 0 0;margin-right:4px}
.tab.active{background:#2a7cdf;color:#fff;border-color:#2a7cdf}
.kmtab{border:1px solid #ddd;border-radius:0 4px 4px 4px;padding:10px;margin-bottom:12px}
</style>
</head>
<body>
<h2>BarButtons Mod Keymap Config</h2>
<p class="sub">AP configuration mode</p>
<form method="POST" action="/save">
<div style="margin-bottom:0">
  <button type="button" id="tab1" class="tab" onclick="showKm(1)">Keymap 1</button>
  <button type="button" id="tab2" class="tab" onclick="showKm(2)">Keymap 2</button>
  <button type="button" id="tab3" class="tab" onclick="showKm(3)">Keymap 3</button>
</div>
<div id="km1" class="kmtab" style="display:none">
  <table><thead><tr><th>Button</th><th>Short Press</th><th>Long Press</th></tr></thead>
  <tbody id="rows1"></tbody></table>
</div>
<div id="km2" class="kmtab" style="display:none">
  <table><thead><tr><th>Button</th><th>Short Press</th><th>Long Press</th></tr></thead>
  <tbody id="rows2"></tbody></table>
</div>
<div id="km3" class="kmtab" style="display:none">
  <table><thead><tr><th>Button</th><th>Short Press</th><th>Long Press</th></tr></thead>
  <tbody id="rows3"></tbody></table>
</div>
<p class="hint">Active keymap: <strong>ACTIVEKEYMAP</strong> &mdash; switch on device with Button&nbsp;4+1, 4+2, or 4+3 (LED flashes 1/2/3 times to confirm).<br>
Button 4 long-press = always enters this config mode (not remappable).<br>
Tap Button 4 on the device to exit without saving.</p>
<div class="field">
<label for="blename">BLE Device Name:</label>
<input type="text" id="blename" name="blename" value="BLENAME"
       maxlength="32" pattern="[ -~]+" required
       placeholder="BarButtonsMod">
<p class="hint">1&#8211;32 printable ASCII characters &mdash; shown in the Bluetooth pairing dialog.</p>
</div>
<div class="field">
<label><input type="checkbox" name="battery_enabled" value="1" BATTERYENABLED> Enable battery level reading</label>
<p class="hint">Reads the cell voltage via GPIO0 (ADC) and reports charge percentage over BLE.<br>
<strong>Requires</strong> the 680&thinsp;k&#8486; / 220&thinsp;k&#8486; voltage divider and remaps the third keypad row (GPIO0&thinsp;&rarr;&thinsp;GPIO7). Takes effect after Save&nbsp;&amp;&nbsp;Reboot.</p>
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
<p class="sub">Fill in the default keymap for the currently visible tab and the BLE device name. Save &amp; Reboot to apply.</p>
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
var KM=[
  {S:[SHORTVALS1],L:[LONGVALS1]},
  {S:[SHORTVALS2],L:[LONGVALS2]},
  {S:[SHORTVALS3],L:[LONGVALS3]}
];
var curKm=ACTIVEKEYMAP;
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
  [200,'F7'],[201,'F8'],[202,'F9'],[203,'F10'],[204,'F11'],[205,'F12'],
  [224,'Play/Pause'],[225,'Stop'],[226,'Next Track'],[227,'Prev Track'],
  [228,'Volume Up'],[229,'Volume Down'],[230,'Mute']
];
for(var i=97;i<=122;i++) K.push([i, String.fromCharCode(i)]); // a-z
for(var i=48;i<=57;i++)  K.push([i, String.fromCharCode(i)]); // 0-9

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

for (var km = 1; km <= 3; km++) {
  var tbody = document.getElementById('rows' + km);
  var S = KM[km - 1].S, L = KM[km - 1].L;
  for (var i = 0; i < 8; i++) {
    var tr = document.createElement('tr');
    // Button 4 (index 3) long press is reserved -- show as disabled info cell
    var longCell = (i === 3)
      ? '<td><em style="color:#999">reserved (config trigger)</em><input type="hidden" name="l' + km + '_3" value="' + L[3] + '"></td>'
      : '<td><select name="l' + km + '_' + i + '">' + buildOpts(L[i], true) + '</select></td>';
    tr.innerHTML =
      '<td><strong>Button ' + (i + 1) + '</strong></td>' +
      '<td><select name="s' + km + '_' + i + '">' + buildOpts(S[i], false) + '</select></td>' +
      longCell;
    tbody.appendChild(tr);
  }
}

function showKm(n) {
  document.getElementById('km' + curKm).style.display = 'none';
  document.getElementById('tab' + curKm).className = 'tab';
  curKm = n;
  document.getElementById('km' + curKm).style.display = '';
  document.getElementById('tab' + curKm).className = 'tab active';
}
showKm(curKm);

function resetToDefaults() {
  var rows = document.getElementById('rows' + curKm).rows;
  for (var i = 0; i < 8; i++) {
    var sels = rows[i].querySelectorAll('select');
    if (sels[0]) sels[0].value = DS[i];
    if (sels[1]) sels[1].value = DL[i];
  }
  document.getElementById('blename').value = DEFAULTBLENAME;
}
</script>
<footer>BarButtonsMod vFWVER &mdash; by <a href="https://github.com/drake7707/" target="_blank" rel="noopener">Drakarah</a> &mdash; <a href="https://github.com/drake7707/" target="_blank" rel="noopener">github.com/drake7707</a>. Derived from <a href='https://jaxeadv.com/barbuttons/'>BarButtons by JADXAdv</a></footer>
</body>
</html>
)rawliteral";

class ConfigManager {
public:
  // Factory defaults -- mirror the original hard-coded behaviour
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
  // NVS -- keymaps (3 slots)
  // ---------------------------------------------------------------------------
  void loadKeymap() {
    const char* namespaces[3] = {"keymap", "keymap2", "keymap3"};
    for (int km = 0; km < 3; km++) {
      nvs_handle_t h;
      bool opened = (nvs_open(namespaces[km], NVS_READONLY, &h) == ESP_OK);
      for (int i = 0; i < 8; i++) {
        char key[8]; // "s"/"l" + up to 3 digits + null
        snprintf(key, sizeof(key), "s%d", i);
        uint8_t v = DEFAULT_SHORT[i];
        if (opened) nvs_get_u8(h, key, &v);
        _short[km][i] = v;

        snprintf(key, sizeof(key), "l%d", i);
        v = DEFAULT_LONG[i];
        if (opened) nvs_get_u8(h, key, &v);
        _long[km][i] = v;
      }
      if (opened) nvs_close(h);
    }
    if (DEBUG) {
      printf("Keymaps loaded from NVS:\n");
      for (int km = 0; km < 3; km++) {
        printf("  Keymap %d:\n", km + 1);
        for (int i = 0; i < 8; i++)
          printf("    btn%d  short=%d  long=%d\n", i + 1, _short[km][i], _long[km][i]);
      }
    }
  }

  void saveKeymap() {
    const char* namespaces[3] = {"keymap", "keymap2", "keymap3"};
    for (int km = 0; km < 3; km++) {
      nvs_handle_t h;
      if (nvs_open(namespaces[km], NVS_READWRITE, &h) != ESP_OK) continue;
      for (int i = 0; i < 8; i++) {
        char key[8]; // "s"/"l" + up to 3 digits + null
        snprintf(key, sizeof(key), "s%d", i);
        nvs_set_u8(h, key, _short[km][i]);
        snprintf(key, sizeof(key), "l%d", i);
        nvs_set_u8(h, key, _long[km][i]);
      }
      nvs_commit(h);
      nvs_close(h);
    }
    if (DEBUG) printf("All keymaps saved to NVS.\n");
  }

  // ---------------------------------------------------------------------------
  // NVS -- active keymap index (1, 2, or 3)
  // ---------------------------------------------------------------------------
  void loadActiveKeymap() {
    nvs_handle_t h;
    uint8_t saved = 1;
    if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK) {
      nvs_get_u8(h, "activekm", &saved);
      nvs_close(h);
    }
    _activeKeymap = (saved >= 1 && saved <= 3) ? (int)saved : 1;
    if (DEBUG) printf("Active keymap loaded: %d\n", _activeKeymap);
  }

  void setActiveKeymap(int slot) {
    if (slot < 1 || slot > 3) return;
    _activeKeymap = slot;
    nvs_handle_t h;
    if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK) {
      nvs_set_u8(h, "activekm", (uint8_t)_activeKeymap);
      nvs_commit(h);
      nvs_close(h);
    }
    if (DEBUG) printf("Active keymap set to: %d\n", _activeKeymap);
  }

  int getActiveKeymap() const { return _activeKeymap; }

  // ---------------------------------------------------------------------------
  // NVS -- BLE device name
  // ---------------------------------------------------------------------------
  void loadBleName() {
    nvs_handle_t h;
    if (nvs_open("config", NVS_READONLY, &h) == ESP_OK) {
      size_t len = sizeof(_bleName);
      if (nvs_get_str(h, "blename", _bleName, &len) != ESP_OK)
        strncpy(_bleName, DEFAULT_BLE_NAME, sizeof(_bleName) - 1);
      nvs_close(h);
    } else {
      strncpy(_bleName, DEFAULT_BLE_NAME, sizeof(_bleName) - 1);
    }
    _bleName[sizeof(_bleName) - 1] = '\0';
    if (DEBUG) printf("BLE name loaded: %s\n", _bleName);
  }

  void saveBleName() {
    nvs_handle_t h;
    if (nvs_open("config", NVS_READWRITE, &h) == ESP_OK) {
      nvs_set_str(h, "blename", _bleName);
      nvs_commit(h);
      nvs_close(h);
    }
    if (DEBUG) printf("BLE name saved: %s\n", _bleName);
  }

  // ---------------------------------------------------------------------------
  // NVS -- battery enable flag (default: disabled)
  // ---------------------------------------------------------------------------
  void loadBatteryEnabled() {
    nvs_handle_t h;
    uint8_t flag = 0;
    if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK) {
      nvs_get_u8(h, "baten", &flag);
      nvs_close(h);
    }
    _batteryEnabled = (flag != 0);
    if (DEBUG) printf("Battery enabled: %s\n", _batteryEnabled ? "yes" : "no");
  }

  void saveBatteryEnabled() {
    nvs_handle_t h;
    if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK) {
      nvs_set_u8(h, "baten", _batteryEnabled ? 1 : 0);
      nvs_commit(h);
      nvs_close(h);
    }
    if (DEBUG) printf("Battery enabled saved: %s\n", _batteryEnabled ? "yes" : "no");
  }

  bool isBatteryEnabled() const { return _batteryEnabled; }

  // ---------------------------------------------------------------------------
  // NVS -- "clear bonds" flag
  // ---------------------------------------------------------------------------
  void requestClearBonds() {
    nvs_handle_t h;
    if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK) {
      nvs_set_u8(h, "clrbond", 1);
      nvs_commit(h);
      nvs_close(h);
    }
  }

  bool isClearBondsRequested() {
    nvs_handle_t h;
    uint8_t flag = 0;
    if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK) {
      nvs_get_u8(h, "clrbond", &flag);
      nvs_close(h);
    }
    return flag != 0;
  }

  void clearClearBondsFlag() {
    nvs_handle_t h;
    if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK) {
      nvs_erase_key(h, "clrbond");
      nvs_commit(h);
      nvs_close(h);
    }
  }

  // ---------------------------------------------------------------------------
  // Keymap / BLE name accessors
  // ---------------------------------------------------------------------------
  uint8_t getShortKey(int idx) const {
    int km = (_activeKeymap >= 1 && _activeKeymap <= 3) ? _activeKeymap - 1 : 0;
    return (idx >= 0 && idx < 8) ? _short[km][idx] : 0;
  }

  uint8_t getLongKey(int idx) const {
    int km = (_activeKeymap >= 1 && _activeKeymap <= 3) ? _activeKeymap - 1 : 0;
    return (idx >= 0 && idx < 8) ? _long[km][idx] : 0;
  }

  const char* getBleName() const { return _bleName; }

  static int btnIndex(char key) {
    if (key >= '1' && key <= '8') return key - '1';
    return -1;
  }

  // ---------------------------------------------------------------------------
  // AP / HTTP server -- config mode
  // ---------------------------------------------------------------------------
  void beginConfigAP() {
    _exitConfig = false;

    // WiFi AP setup
    _apNetif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_cfg = {};
    strncpy((char*)ap_cfg.ap.ssid,     _apSsid, sizeof(ap_cfg.ap.ssid));
    strncpy((char*)ap_cfg.ap.password, _apPswd, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len      = (uint8_t)strlen(_apSsid);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode      = WIFI_AUTH_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();
    if (DEBUG) printf("AP started: 192.168.4.1\n");

    // HTTP server
    httpd_config_t http_cfg  = HTTPD_DEFAULT_CONFIG();
    http_cfg.stack_size      = 8192;
    http_cfg.recv_wait_timeout = 60;
    http_cfg.send_wait_timeout = 60;
    httpd_start(&_server, &http_cfg);

    httpd_uri_t u_root   = { "/",           HTTP_GET,  _s_root,       this };
    httpd_uri_t u_save   = { "/save",       HTTP_POST, _s_save,       this };
    httpd_uri_t u_bonds  = { "/clearbonds", HTTP_POST, _s_clearbonds, this };
    httpd_uri_t u_update = { "/update",     HTTP_POST, _s_update,     this };
    httpd_register_uri_handler(_server, &u_root);
    httpd_register_uri_handler(_server, &u_save);
    httpd_register_uri_handler(_server, &u_bonds);
    httpd_register_uri_handler(_server, &u_update);

    // 5 quick flashes to signal we're in config mode
    if (_led) _led->flashLed(5, 100, 100);
  }

  // No-op: esp_http_server handles requests in its own task.
  void handleClient() {}

  void endConfigAP() {
    if (_server) { httpd_stop(_server); _server = nullptr; }
    esp_wifi_stop();
    esp_wifi_deinit();
    if (_apNetif) { esp_netif_destroy(_apNetif); _apNetif = nullptr; }
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  bool isExitRequested() const  { return _exitConfig; }
  void setExitRequested(bool v) { _exitConfig = v; }

private:
  StatusLedManager* _led          = nullptr;
  bool              _exitConfig   = false;
  char              _firmwareVersion[32] = {};
  uint8_t           _short[3][8]  = {};
  uint8_t           _long[3][8]   = {};
  int               _activeKeymap = 1;
  bool              _batteryEnabled = false;
  char              _bleName[BLE_NAME_MAX_LEN + 1] = "BarButtonsMod";
  httpd_handle_t    _server    = nullptr;
  esp_netif_t*      _apNetif   = nullptr;
  const char*       _apSsid    = "BarButtons-Config";
  const char*       _apPswd    = "barbuttons";

  // ---------------------------------------------------------------------------
  // String helpers
  // ---------------------------------------------------------------------------

  // Replace first occurrence of 'from' in 's' with 'to'.
  static void _strReplace(std::string& s, const std::string& from, const std::string& to) {
    size_t pos = s.find(from);
    if (pos != std::string::npos) s.replace(pos, from.size(), to);
  }

  // Trim leading/trailing whitespace in-place.
  static void _strTrim(std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) { s.clear(); return; }
    s.erase(0, b);
    size_t e = s.find_last_not_of(" \t\r\n");
    if (e != std::string::npos) s.erase(e + 1);
  }

  // URL-decode a percent-encoded buffer.
  static std::string _urlDecode(const char* src, size_t len) {
    std::string result;
    result.reserve(len);
    for (size_t i = 0; i < len; i++) {
      if (src[i] == '%' && i + 2 < len) {
        char h[3] = { src[i+1], src[i+2], '\0' };
        result += (char)strtol(h, nullptr, 16);
        i += 2;
      } else if (src[i] == '+') {
        result += ' ';
      } else {
        result += src[i];
      }
    }
    return result;
  }

  // Extract and URL-decode a named parameter from a URL-encoded body.
  static std::string _formParam(const char* body, const char* name) {
    size_t nlen = strlen(name);
    const char* p = body;
    while (p && *p) {
      if (strncmp(p, name, nlen) == 0 && p[nlen] == '=') {
        const char* val = p + nlen + 1;
        const char* end = strchr(val, '&');
        size_t vlen = end ? (size_t)(end - val) : strlen(val);
        return _urlDecode(val, vlen);
      }
      p = strchr(p, '&');
      if (p) p++;
    }
    return std::string();
  }

  // Read the full POST body into a std::string.
  static bool _readBody(httpd_req_t* req, std::string& out) {
    int total = req->content_len;
    if (total <= 0) { out.clear(); return true; }
    out.resize(total);
    int received = 0;
    while (received < total) {
      int r = httpd_req_recv(req, &out[received], total - received);
      if (r <= 0) return false;
      received += r;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  // Static trampoline handlers (httpd requires plain function pointers)
  // ---------------------------------------------------------------------------
  static esp_err_t _s_root(httpd_req_t* req) {
    ((ConfigManager*)req->user_ctx)->_handleRoot(req); return ESP_OK;
  }
  static esp_err_t _s_save(httpd_req_t* req) {
    ((ConfigManager*)req->user_ctx)->_handleSave(req); return ESP_OK;
  }
  static esp_err_t _s_clearbonds(httpd_req_t* req) {
    ((ConfigManager*)req->user_ctx)->_handleClearBonds(req); return ESP_OK;
  }
  static esp_err_t _s_update(httpd_req_t* req) {
    ((ConfigManager*)req->user_ctx)->_handleUpdate(req); return ESP_OK;
  }

  // ---------------------------------------------------------------------------
  // Web handler implementations
  // ---------------------------------------------------------------------------
  void _handleRoot(httpd_req_t* req) {
    std::string html(CONFIG_HTML);

    // Inject keymap values for all 3 keymap slots
    for (int km = 0; km < 3; km++) {
      std::string sv, lv;
      for (int i = 0; i < 8; i++) {
        if (i) { sv += ','; lv += ','; }
        sv += std::to_string(_short[km][i]);
        lv += std::to_string(_long[km][i]);
      }
      _strReplace(html, "SHORTVALS" + std::to_string(km + 1), sv);
      _strReplace(html, "LONGVALS"  + std::to_string(km + 1), lv);
    }

    _strReplace(html, "ACTIVEKEYMAP", std::to_string(_activeKeymap));

    std::string dsv, dlv;
    for (int i = 0; i < 8; i++) {
      if (i) { dsv += ','; dlv += ','; }
      dsv += std::to_string(DEFAULT_SHORT[i]);
      dlv += std::to_string(DEFAULT_LONG[i]);
    }
    _strReplace(html, "DEFAULTSHORT", dsv);
    _strReplace(html, "DEFAULTLONG",  dlv);

    // Escape default BLE name for safe embedding as a JS string literal
    std::string dbn;
    for (int i = 0; DEFAULT_BLE_NAME[i]; i++) {
      char c = DEFAULT_BLE_NAME[i];
      if      (c == '\\') dbn += "\\\\";
      else if (c == '\'') dbn += "\\'";
      else                dbn += c;
    }
    _strReplace(html, "DEFAULTBLENAME", "'" + dbn + "'");

    // Escape BLE name for safe use in an HTML attribute value
    std::string bn;
    for (int i = 0; _bleName[i]; i++) {
      char c = _bleName[i];
      if      (c == '&') bn += "&amp;";
      else if (c == '"') bn += "&quot;";
      else if (c == '<') bn += "&lt;";
      else if (c == '>') bn += "&gt;";
      else               bn += c;
    }
    _strReplace(html, "BLENAME", bn);
    _strReplace(html, "BATTERYENABLED", _batteryEnabled ? "checked" : "");
    _strReplace(html, "FWVER", std::string(_firmwareVersion));

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), (ssize_t)html.size());
  }

  void _handleSave(httpd_req_t* req) {
    std::string body;
    if (!_readBody(req, body)) { httpd_resp_send_500(req); return; }

    for (int km = 0; km < 3; km++) {
      for (int i = 0; i < 8; i++) {
        char si[8]; // km digit + "_" + btn digit + null
        snprintf(si, sizeof(si), "%d_%d", km + 1, i);
        std::string sv = _formParam(body.c_str(), (std::string("s") + si).c_str());
        std::string lv = _formParam(body.c_str(), (std::string("l") + si).c_str());
        if (!sv.empty()) _short[km][i] = (uint8_t)atoi(sv.c_str());
        if (!lv.empty()) _long[km][i]  = (uint8_t)atoi(lv.c_str());
      }
    }
    saveKeymap();

    std::string newName = _formParam(body.c_str(), "blename");
    _strTrim(newName);
    bool valid = newName.size() > 0 && newName.size() <= BLE_NAME_MAX_LEN;
    if (valid) {
      for (char c : newName) if (c < 0x20 || c > 0x7E) { valid = false; break; }
    }
    if (valid) {
      strncpy(_bleName, newName.c_str(), sizeof(_bleName) - 1);
      _bleName[sizeof(_bleName) - 1] = '\0';
      saveBleName();
    }

    std::string batParam = _formParam(body.c_str(), "battery_enabled");
    _batteryEnabled = (batParam == "1");
    saveBatteryEnabled();

    static const char resp[] =
      "<!DOCTYPE html><html>"
      "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Saved!</h2><p>Rebooting&hellip;</p></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    if (_led) _led->flashLed(3, 80, 80);
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
  }

  void _handleClearBonds(httpd_req_t* req) {
    // BLE is stopped while the WiFi AP is running (shared radio on ESP32-C3),
    // so we can't delete bonds here. Write a flag to NVS and reboot -- setup()
    // will delete the bonds once NimBLE is initialised again.
    requestClearBonds();

    static const char resp[] =
      "<!DOCTYPE html><html>"
      "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Bonds cleared!</h2><p>Rebooting&hellip; Re-pair your phone when the device is discoverable.</p>"
      "</body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    if (_led) _led->flashLed(3, 80, 80);
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
  }

  // Stream a multipart/form-data firmware upload directly to OTA flash.
  void _handleUpdate(httpd_req_t* req) {
    // Extract boundary from Content-Type header
    char ct[256] = {};
    httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct));
    const char* bnd_prefix = "boundary=";
    const char* bnd_start  = strstr(ct, bnd_prefix);
    if (!bnd_start) { httpd_resp_send_500(req); return; }
    bnd_start += strlen(bnd_prefix);

    std::string boundary("--");
    boundary  += bnd_start;
    std::string endBoundary(boundary + "--");
    size_t eblen = endBoundary.size();

    const esp_partition_t* ota_part = esp_ota_get_next_update_partition(nullptr);
    if (!ota_part) { httpd_resp_send_500(req); return; }

    esp_ota_handle_t ota_handle;
    if (esp_ota_begin(ota_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle) != ESP_OK) {
      httpd_resp_send_500(req); return;
    }

    const size_t BUF = 1024;
    char buf[BUF];
    int  remaining = req->content_len;
    bool ota_ok    = true;
    bool in_data   = false;

    // header_buf accumulates bytes until we find \r\n\r\n (part header end).
    std::string header_buf;
    header_buf.reserve(512);

    // tail holds the last (eblen+4) bytes so we can strip the trailing boundary.
    std::string tail;
    tail.reserve(eblen + 6);

    while (remaining > 0 && ota_ok) {
      size_t to_read = std::min((size_t)(remaining), BUF);
      int got = httpd_req_recv(req, buf, to_read);
      if (got <= 0) { ota_ok = false; break; }
      remaining -= got;

      if (!in_data) {
        header_buf.append(buf, got);
        if (header_buf.size() > 2048) { ota_ok = false; break; } // malformed header
        size_t pos = header_buf.find("\r\n\r\n");
        if (pos != std::string::npos) {
          in_data = true;
          const char* data   = header_buf.c_str() + pos + 4;
          size_t      dlen   = header_buf.size() - (pos + 4);
          if (dlen > 0) {
            tail.append(data, dlen);
            if (tail.size() > eblen + 4) {
              size_t wlen = tail.size() - (eblen + 4);
              if (esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK) ota_ok = false;
              tail.erase(0, wlen);
            }
          }
          header_buf.clear();
        }
      } else {
        tail.append(buf, got);
        if (tail.size() > eblen + 4) {
          size_t wlen = tail.size() - (eblen + 4);
          if (esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK) ota_ok = false;
          tail.erase(0, wlen);
        }
      }
    }

    // Write remaining tail, stripping the trailing boundary marker.
    if (ota_ok && !tail.empty()) {
      size_t pos = tail.rfind(endBoundary);
      size_t wlen = (pos != std::string::npos) ? pos : tail.size();
      // Strip the preceding \r\n that separates firmware data from boundary
      if (wlen >= 2) wlen -= 2;
      if (wlen > 0 && esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK) ota_ok = false;
    }

    if (ota_ok) {
      ota_ok = (esp_ota_end(ota_handle) == ESP_OK);
    } else {
      esp_ota_abort(ota_handle);
    }
    if (ota_ok) esp_ota_set_boot_partition(ota_part);

    static const char resp_ok[] =
      "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Update successful!</h2><p>Rebooting&hellip;</p></body></html>";
    static const char resp_fail[] =
      "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10007; Update failed.</h2><p><a href='/'>Try again</a></p></body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, ota_ok ? resp_ok : resp_fail, HTTPD_RESP_USE_STRLEN);

    if (ota_ok) {
      if (_led) _led->flashLed(5, 50, 50);
      vTaskDelay(pdMS_TO_TICKS(500));
      esp_restart();
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
