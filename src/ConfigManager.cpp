#include "ConfigManager.h"
#include "cJSON.h"

const char DEFAULT_BLE_NAME[] = "RemoteKeyboard";

// Out-of-class definitions for static const members
const uint8_t ConfigManager::DEFAULT_SHORT[8] = {
    '+', '-', 'n', 'c',
    KEY_UP_ARROW, KEY_LEFT_ARROW, KEY_RIGHT_ARROW, KEY_DOWN_ARROW};
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

void ConfigManager::begin(StatusLedManager *led, const char *firmwareVersion)
{
  _led = led;
  strncpy(_firmwareVersion, firmwareVersion, sizeof(_firmwareVersion) - 1);
  _firmwareVersion[sizeof(_firmwareVersion) - 1] = '\0';
}

// ---------------------------------------------------------------------------
// loadAll -- load all persistent settings from NVS in one call
// ---------------------------------------------------------------------------
void ConfigManager::loadAll()
{
  _loadKeymap();
  _loadActiveKeymap();
  _loadBleName();
  _loadBatteryEnabled();
  _loadBLEPowerSaving();
  _loadMaxBLEConnections();
}

// ---------------------------------------------------------------------------
// NVS -- keymaps (3 slots)
// ---------------------------------------------------------------------------
void ConfigManager::_loadKeymap()
{
  const char *namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int km = 0; km < 3; km++)
  {
    nvs_handle_t h;
    bool opened = (nvs_open(namespaces[km], NVS_READONLY, &h) == ESP_OK);
    for (int i = 0; i < 8; i++)
    {
      char key[8]; // prefix + up to 2 digits + null

      // Short key
      snprintf(key, sizeof(key), "s%d", i);
      uint8_t v = DEFAULT_SHORT[i];
      if (opened)
        nvs_get_u8(h, key, &v);
      _shortEntries[km][i].key = v;

      // Long key
      snprintf(key, sizeof(key), "l%d", i);
      v = DEFAULT_LONG[i];
      if (opened)
        nvs_get_u8(h, key, &v);
      _longEntries[km][i].key = v;

      // Short target type (new field; default TARGET_SELECT for backward compatibility)
      snprintf(key, sizeof(key), "st%d", i);
      uint8_t tgt = TARGET_SELECT;
      if (opened)
        nvs_get_u8(h, key, &tgt);
      if (tgt > TARGET_BTHOME) tgt = TARGET_SELECT;
      _shortEntries[km][i].target = (KeyTarget)tgt;

      // Long target type
      snprintf(key, sizeof(key), "lt%d", i);
      tgt = TARGET_SELECT;
      if (opened)
        nvs_get_u8(h, key, &tgt);
      if (tgt > TARGET_BTHOME) tgt = TARGET_SELECT;
      _longEntries[km][i].target = (KeyTarget)tgt;

      // Short HID peer MAC (new field; default empty = broadcast all)
      snprintf(key, sizeof(key), "sm%d", i);
      _shortEntries[km][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(_shortEntries[km][i].mac);
        nvs_get_str(h, key, _shortEntries[km][i].mac, &macLen);
      }

      // Long HID peer MAC
      snprintf(key, sizeof(key), "lm%d", i);
      _longEntries[km][i].mac[0] = '\0';
      if (opened)
      {
        size_t macLen = sizeof(_longEntries[km][i].mac);
        nvs_get_str(h, key, _longEntries[km][i].mac, &macLen);
      }
    }
    if (opened)
      nvs_close(h);
  }
  if (DEBUG)
  {
    printf("[CONFIG] Keymaps loaded from NVS:\n");
    for (int km = 0; km < 3; km++)
    {
      printf("[CONFIG]   Keymap %d:\n", km + 1);
      for (int i = 0; i < 8; i++)
        printf("[CONFIG]     btn%d  short=%d(tgt=%d mac=%s)  long=%d(tgt=%d mac=%s)\n",
               i + 1,
               _shortEntries[km][i].key, (int)_shortEntries[km][i].target, _shortEntries[km][i].mac,
               _longEntries[km][i].key, (int)_longEntries[km][i].target, _longEntries[km][i].mac);
    }
  }
}

void ConfigManager::saveKeymap()
{
  const char *namespaces[3] = {"keymap", "keymap2", "keymap3"};
  for (int km = 0; km < 3; km++)
  {
    nvs_handle_t h;
    if (nvs_open(namespaces[km], NVS_READWRITE, &h) != ESP_OK)
      continue;
    for (int i = 0; i < 8; i++)
    {
      char key[8]; // prefix + up to 2 digits + null
      snprintf(key, sizeof(key), "s%d", i);
      nvs_set_u8(h, key, _shortEntries[km][i].key);
      snprintf(key, sizeof(key), "l%d", i);
      nvs_set_u8(h, key, _longEntries[km][i].key);
      snprintf(key, sizeof(key), "st%d", i);
      nvs_set_u8(h, key, (uint8_t)_shortEntries[km][i].target);
      snprintf(key, sizeof(key), "lt%d", i);
      nvs_set_u8(h, key, (uint8_t)_longEntries[km][i].target);
      snprintf(key, sizeof(key), "sm%d", i);
      nvs_set_str(h, key, _shortEntries[km][i].mac);
      snprintf(key, sizeof(key), "lm%d", i);
      nvs_set_str(h, key, _longEntries[km][i].mac);
    }
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] All keymaps saved to NVS.\n");
}

// ---------------------------------------------------------------------------
// NVS -- active keymap index (1, 2, or 3)
// ---------------------------------------------------------------------------
void ConfigManager::_loadActiveKeymap()
{
  nvs_handle_t h;
  uint8_t saved = 1;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "activekm", &saved);
    nvs_close(h);
  }
  _activeKeymap = (saved >= 1 && saved <= 3) ? (int)saved : 1;
  if (DEBUG)
    printf("[CONFIG] Active keymap loaded: %d\n", _activeKeymap);
}

void ConfigManager::setActiveKeymap(int slot)
{
  if (slot < 1 || slot > 3)
    return;
  _activeKeymap = slot;
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "activekm", (uint8_t)_activeKeymap);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Active keymap set to: %d\n", _activeKeymap);
}

// ---------------------------------------------------------------------------
// NVS -- BLE device name
// ---------------------------------------------------------------------------
void ConfigManager::_loadBleName()
{
  nvs_handle_t h;
  if (nvs_open("config", NVS_READONLY, &h) == ESP_OK)
  {
    size_t len = sizeof(_bleName);
    if (nvs_get_str(h, "blename", _bleName, &len) != ESP_OK)
      strncpy(_bleName, DEFAULT_BLE_NAME, sizeof(_bleName) - 1);
    nvs_close(h);
  }
  else
  {
    strncpy(_bleName, DEFAULT_BLE_NAME, sizeof(_bleName) - 1);
  }
  _bleName[sizeof(_bleName) - 1] = '\0';
  if (DEBUG)
    printf("[CONFIG] BLE name loaded: %s\n", _bleName);
}

void ConfigManager::saveBleName()
{
  nvs_handle_t h;
  if (nvs_open("config", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_str(h, "blename", _bleName);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] BLE name saved: %s\n", _bleName);
}

// ---------------------------------------------------------------------------
// NVS -- battery enable flag (default: disabled)
// ---------------------------------------------------------------------------
void ConfigManager::_loadBatteryEnabled()
{
  nvs_handle_t h;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "baten", &flag);
    nvs_close(h);
  }
  _batteryEnabled = (flag != 0);
  if (DEBUG)
    printf("[CONFIG] Battery enabled: %s\n", _batteryEnabled ? "yes" : "no");
}

void ConfigManager::saveBatteryEnabled()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "baten", _batteryEnabled ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Battery enabled saved: %s\n", _batteryEnabled ? "yes" : "no");
}

void ConfigManager::_loadBLEPowerSaving()
{
  nvs_handle_t h;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "blepsen", &flag);
    nvs_close(h);
  }
  _blePowerSaving = (flag != 0);
  if (DEBUG)
    printf("[CONFIG] BLE power saving: %s\n", _blePowerSaving ? "yes" : "no");
}

void ConfigManager::saveBLEPowerSaving()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "blepsen", _blePowerSaving ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] BLE power saving: %s\n", _blePowerSaving ? "yes" : "no");
}

void ConfigManager::_loadMaxBLEConnections()
{
  nvs_handle_t h;
  uint8_t val = 1;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "maxbleconn", &val);
    nvs_close(h);
  }
  _maxBLEConnections = (val >= 1 && val <= 3) ? val : 1;
  if (DEBUG)
    printf("[CONFIG] Max BLE connections: %d\n", _maxBLEConnections);
}

void ConfigManager::saveMaxBLEConnections()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "maxbleconn", _maxBLEConnections);
    nvs_commit(h);
    nvs_close(h);
  }
  if (DEBUG)
    printf("[CONFIG] Max BLE connections saved: %d\n", _maxBLEConnections);
}

// ---------------------------------------------------------------------------
// NVS -- "clear bonds" flag
// ---------------------------------------------------------------------------
void ConfigManager::requestClearBonds()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_set_u8(h, "clrbond", 1);
    nvs_commit(h);
    nvs_close(h);
  }
}

bool ConfigManager::isClearBondsRequested()
{
  nvs_handle_t h;
  uint8_t flag = 0;
  if (nvs_open("sys", NVS_READONLY, &h) == ESP_OK)
  {
    nvs_get_u8(h, "clrbond", &flag);
    nvs_close(h);
  }
  return flag != 0;
}

void ConfigManager::clearClearBondsFlag()
{
  nvs_handle_t h;
  if (nvs_open("sys", NVS_READWRITE, &h) == ESP_OK)
  {
    nvs_erase_key(h, "clrbond");
    nvs_commit(h);
    nvs_close(h);
  }
}

// ---------------------------------------------------------------------------
// Keymap accessors
// ---------------------------------------------------------------------------
const ConfigManager::KeyEntry& ConfigManager::getShortEntry(int idx) const
{
  static const KeyEntry empty{};
  int km = (_activeKeymap >= 1 && _activeKeymap <= 3) ? _activeKeymap - 1 : 0;
  return (idx >= 0 && idx < 8) ? _shortEntries[km][idx] : empty;
}

const ConfigManager::KeyEntry& ConfigManager::getLongEntry(int idx) const
{
  static const KeyEntry empty{};
  int km = (_activeKeymap >= 1 && _activeKeymap <= 3) ? _activeKeymap - 1 : 0;
  return (idx >= 0 && idx < 8) ? _longEntries[km][idx] : empty;
}

int ConfigManager::btnIndex(char key)
{
  if (key >= '1' && key <= '8')
    return key - '1';
  return -1;
}

// ---------------------------------------------------------------------------
// AP / HTTP server -- config mode
// ---------------------------------------------------------------------------
void ConfigManager::beginConfigAP(const std::vector<std::string>& bondList,
                                   int batVoltageMv, int batPercent)
{
  _bondList    = bondList;
  _batVoltageMv = batVoltageMv;
  _batPercent   = batPercent;
  _exitConfig  = false;

  // WiFi AP setup
  _apNetif = esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  wifi_config_t ap_cfg = {};
  strncpy((char *)ap_cfg.ap.ssid, _apSsid, sizeof(ap_cfg.ap.ssid));
  strncpy((char *)ap_cfg.ap.password, _apPswd, sizeof(ap_cfg.ap.password));
  ap_cfg.ap.ssid_len = (uint8_t)strlen(_apSsid);
  ap_cfg.ap.max_connection = 4;
  ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
  esp_wifi_start();

  if (DEBUG)
    printf("[CONFIG] AP started: 192.168.4.1\n");

  // HTTP server
  httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
  http_cfg.stack_size = 8192;
  http_cfg.recv_wait_timeout = 60;
  http_cfg.send_wait_timeout = 60;
  httpd_start(&_server, &http_cfg);

  httpd_uri_t u_root = {"/", HTTP_GET, _s_root, this};
  httpd_uri_t u_save = {"/save", HTTP_POST, _s_save, this};
  httpd_uri_t u_bonds = {"/clearbonds", HTTP_POST, _s_clearbonds, this};
  httpd_uri_t u_update = {"/update", HTTP_POST, _s_update, this};
  httpd_register_uri_handler(_server, &u_root);
  httpd_register_uri_handler(_server, &u_save);
  httpd_register_uri_handler(_server, &u_bonds);
  httpd_register_uri_handler(_server, &u_update);

  // 5 quick flashes to signal we're in config mode
  if (_led)
    _led->flashLed(5, 100, 100);
}

void ConfigManager::endConfigAP()
{
  if (_server)
  {
    httpd_stop(_server);
    _server = nullptr;
  }
  esp_wifi_stop();
  esp_wifi_deinit();
  if (_apNetif)
  {
    esp_netif_destroy(_apNetif);
    _apNetif = nullptr;
  }
  vTaskDelay(pdMS_TO_TICKS(200));
}

// ---------------------------------------------------------------------------
// String helpers
// ---------------------------------------------------------------------------

// Replace all occurrences of 'from' in 's' with 'to'.
void ConfigManager::_strReplace(std::string &s, const std::string &from, const std::string &to)
{
  if (from.empty())
    return;
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos)
  {
    s.replace(pos, from.length(), to);
    pos += to.length();
  }
}

// Trim leading/trailing whitespace in-place.
void ConfigManager::_strTrim(std::string &s)
{
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
  {
    s.clear();
    return;
  }
  s.erase(0, b);
  size_t e = s.find_last_not_of(" \t\r\n");
  if (e != std::string::npos)
    s.erase(e + 1);
}

// URL-decode a percent-encoded buffer.
std::string ConfigManager::_urlDecode(const char *src, size_t len)
{
  std::string result;
  result.reserve(len);
  for (size_t i = 0; i < len; i++)
  {
    if (src[i] == '%' && i + 2 < len)
    {
      char h[3] = {src[i + 1], src[i + 2], '\0'};
      result += (char)strtol(h, nullptr, 16);
      i += 2;
    }
    else if (src[i] == '+')
    {
      result += ' ';
    }
    else
    {
      result += src[i];
    }
  }
  return result;
}

// Extract and URL-decode a named parameter from a URL-encoded body.
std::string ConfigManager::_formParam(const char *body, const char *name)
{
  size_t nlen = strlen(name);
  const char *p = body;
  while (p && *p)
  {
    if (strncmp(p, name, nlen) == 0 && p[nlen] == '=')
    {
      const char *val = p + nlen + 1;
      const char *end = strchr(val, '&');
      size_t vlen = end ? (size_t)(end - val) : strlen(val);
      return _urlDecode(val, vlen);
    }
    p = strchr(p, '&');
    if (p)
      p++;
  }
  return std::string();
}

// Read the full POST body into a std::string.
bool ConfigManager::_readBody(httpd_req_t *req, std::string &out)
{
  int total = req->content_len;
  if (total <= 0)
  {
    out.clear();
    return true;
  }
  out.resize(total);
  int received = 0;
  while (received < total)
  {
    int r = httpd_req_recv(req, &out[received], total - received);
    if (r <= 0)
      return false;
    received += r;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Static trampoline handlers (httpd requires plain function pointers)
// ---------------------------------------------------------------------------
esp_err_t ConfigManager::_s_root(httpd_req_t *req)
{
  ((ConfigManager *)req->user_ctx)->_handleRoot(req);
  return ESP_OK;
}
esp_err_t ConfigManager::_s_save(httpd_req_t *req)
{
  ((ConfigManager *)req->user_ctx)->_handleSave(req);
  return ESP_OK;
}
esp_err_t ConfigManager::_s_clearbonds(httpd_req_t *req)
{
  ((ConfigManager *)req->user_ctx)->_handleClearBonds(req);
  return ESP_OK;
}
esp_err_t ConfigManager::_s_update(httpd_req_t *req)
{
  ((ConfigManager *)req->user_ctx)->_handleUpdate(req);
  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Web handler implementations
// ---------------------------------------------------------------------------
void ConfigManager::_handleRoot(httpd_req_t *req)
{
  // Load the embedded HTML.  board_build.embed_txtfiles (ESP-IDF
  // COMPONENT_EMBED_TXTFILES) appends one null byte, so the span between
  // _start and _end includes that byte; subtract 1 for the string length.
  std::string html(reinterpret_cast<const char *>(config_html_start),
                   config_html_end - config_html_start - 1 /* strip null */);

  // Build a single settings JSON object using cJSON.
  // The web page reads all values from this object and populates the UI,
  // so the firmware does no direct HTML string manipulation beyond
  // replacing the SETTINGSJSON placeholder.
  cJSON *root = cJSON_CreateObject();

  cJSON_AddStringToObject(root, "fwVer", _firmwareVersion);
  cJSON_AddStringToObject(root, "bleName", _bleName);
  cJSON_AddStringToObject(root, "defaultBleName", DEFAULT_BLE_NAME);
  cJSON_AddBoolToObject(root, "batteryEnabled", _batteryEnabled);
  cJSON_AddBoolToObject(root, "blePowerSaving", _blePowerSaving);
  cJSON_AddBoolToObject(root, "batterySection", !LEGACY);
  cJSON_AddNumberToObject(root, "maxBleConnections", _maxBLEConnections);
  cJSON_AddNumberToObject(root, "activeKeymap", _activeKeymap);
  // Battery reading snapshot (-1 means not available / battery not enabled)
  cJSON_AddNumberToObject(root, "batVoltageMv", _batVoltageMv);
  cJSON_AddNumberToObject(root, "batPercent",   _batPercent);

  // Expose target-type constants so JS always stays in sync with the firmware
  cJSON_AddNumberToObject(root, "TARGET_SELECT", TARGET_SELECT);
  cJSON_AddNumberToObject(root, "TARGET_HID",    TARGET_HID);
  cJSON_AddNumberToObject(root, "TARGET_BTHOME", TARGET_BTHOME);

  // Default keymap values (used by Reset to Defaults)
  cJSON *dsArr = cJSON_CreateArray();
  cJSON *dlArr = cJSON_CreateArray();
  for (int i = 0; i < 8; i++)
  {
    cJSON_AddItemToArray(dsArr, cJSON_CreateNumber(DEFAULT_SHORT[i]));
    cJSON_AddItemToArray(dlArr, cJSON_CreateNumber(DEFAULT_LONG[i]));
  }
  cJSON_AddItemToObject(root, "defaultShort", dsArr);
  cJSON_AddItemToObject(root, "defaultLong",  dlArr);

  // Bond list -- known HID peers for the target selector dropdown
  cJSON *bondsArr = cJSON_CreateArray();
  for (const auto &b : _bondList)
    cJSON_AddItemToArray(bondsArr, cJSON_CreateString(b.c_str()));
  cJSON_AddItemToObject(root, "bonds", bondsArr);

  // Per-keymap button data (keys + target types + MACs)
  cJSON *kmArr = cJSON_CreateArray();
  for (int km = 0; km < 3; km++)
  {
    cJSON *kmObj = cJSON_CreateObject();
    cJSON *sArr  = cJSON_CreateArray(), *lArr  = cJSON_CreateArray();
    cJSON *stArr = cJSON_CreateArray(), *ltArr = cJSON_CreateArray();
    cJSON *smArr = cJSON_CreateArray(), *lmArr = cJSON_CreateArray();
    for (int i = 0; i < 8; i++)
    {
      cJSON_AddItemToArray(sArr,  cJSON_CreateNumber(_shortEntries[km][i].key));
      cJSON_AddItemToArray(lArr,  cJSON_CreateNumber(_longEntries[km][i].key));
      cJSON_AddItemToArray(stArr, cJSON_CreateNumber((uint8_t)_shortEntries[km][i].target));
      cJSON_AddItemToArray(ltArr, cJSON_CreateNumber((uint8_t)_longEntries[km][i].target));
      cJSON_AddItemToArray(smArr, cJSON_CreateString(_shortEntries[km][i].mac));
      cJSON_AddItemToArray(lmArr, cJSON_CreateString(_longEntries[km][i].mac));
    }
    cJSON_AddItemToObject(kmObj, "S",  sArr);
    cJSON_AddItemToObject(kmObj, "L",  lArr);
    cJSON_AddItemToObject(kmObj, "ST", stArr);
    cJSON_AddItemToObject(kmObj, "LT", ltArr);
    cJSON_AddItemToObject(kmObj, "SM", smArr);
    cJSON_AddItemToObject(kmObj, "LM", lmArr);
    cJSON_AddItemToArray(kmArr, kmObj);
  }
  cJSON_AddItemToObject(root, "km", kmArr);

  char *json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);

  _strReplace(html, "SETTINGSJSON", json ? std::string(json) : std::string("{}"));
  if (json)
    cJSON_free(json);

  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_send(req, html.c_str(), (ssize_t)html.size());
}

void ConfigManager::_handleSave(httpd_req_t *req)
{
  std::string body;
  if (!_readBody(req, body))
  {
    httpd_resp_send_500(req);
    return;
  }

  for (int km = 0; km < 3; km++)
  {
    for (int i = 0; i < 8; i++)
    {
      char si[8]; // km digit + "_" + btn digit + null
      snprintf(si, sizeof(si), "%d_%d", km + 1, i);
      std::string sv = _formParam(body.c_str(), (std::string("s") + si).c_str());
      std::string lv = _formParam(body.c_str(), (std::string("l") + si).c_str());
      if (!sv.empty())
        _shortEntries[km][i].key = (uint8_t)atoi(sv.c_str());
      if (!lv.empty())
        _longEntries[km][i].key = (uint8_t)atoi(lv.c_str());

      // Parse target for short press: "0"=select, "1:"=HID broadcast,
      // "1:AA:BB:CC:DD:EE:FF"=HID specific, "2"=BTHome
      std::string tsv = _formParam(body.c_str(), (std::string("ts") + si).c_str());
      if (!tsv.empty())
      {
        if (tsv[0] == '1')
        {
          _shortEntries[km][i].target = TARGET_HID;
          // Format: "1:<mac>" -- skip the "1:" prefix (2 chars) to get the MAC
          std::string m = (tsv.size() > 2) ? tsv.substr(2) : "";
          strncpy(_shortEntries[km][i].mac, m.c_str(), sizeof(_shortEntries[km][i].mac) - 1);
          _shortEntries[km][i].mac[sizeof(_shortEntries[km][i].mac) - 1] = '\0';
        }
        else if (tsv[0] == '2')
        {
          _shortEntries[km][i].target = TARGET_BTHOME;
          _shortEntries[km][i].mac[0] = '\0';
        }
        else
        {
          _shortEntries[km][i].target = TARGET_SELECT;
          _shortEntries[km][i].mac[0] = '\0';
        }
      }

      // Parse target for long press
      std::string tlv = _formParam(body.c_str(), (std::string("tl") + si).c_str());
      if (!tlv.empty())
      {
        if (tlv[0] == '1')
        {
          _longEntries[km][i].target = TARGET_HID;
          // Format: "1:<mac>" -- skip the "1:" prefix (2 chars) to get the MAC
          std::string m = (tlv.size() > 2) ? tlv.substr(2) : "";
          strncpy(_longEntries[km][i].mac, m.c_str(), sizeof(_longEntries[km][i].mac) - 1);
          _longEntries[km][i].mac[sizeof(_longEntries[km][i].mac) - 1] = '\0';
        }
        else if (tlv[0] == '2')
        {
          _longEntries[km][i].target = TARGET_BTHOME;
          _longEntries[km][i].mac[0] = '\0';
        }
        else
        {
          _longEntries[km][i].target = TARGET_SELECT;
          _longEntries[km][i].mac[0] = '\0';
        }
      }
    }
  }
  saveKeymap();

  std::string newName = _formParam(body.c_str(), "blename");
  _strTrim(newName);
  bool valid = newName.size() > 0 && newName.size() <= BLE_NAME_MAX_LEN;
  if (valid)
  {
    for (char c : newName)
      if (c < 0x20 || c > 0x7E)
      {
        valid = false;
        break;
      }
  }
  if (valid)
  {
    strncpy(_bleName, newName.c_str(), sizeof(_bleName) - 1);
    _bleName[sizeof(_bleName) - 1] = '\0';
    saveBleName();
  }

  std::string batParam = _formParam(body.c_str(), "battery_enabled");
  _batteryEnabled = (batParam == "1");
  saveBatteryEnabled();

  std::string blePowerSaveParam = _formParam(body.c_str(), "ble_power_saving");
  _blePowerSaving = (blePowerSaveParam == "1");
  saveBLEPowerSaving();

  std::string maxBleConnParam = _formParam(body.c_str(), "max_ble_connections");
  int maxBleConn = atoi(maxBleConnParam.c_str());
  if (maxBleConn >= 1 && maxBleConn <= 3)
  {
    _maxBLEConnections = (uint8_t)maxBleConn;
    saveMaxBLEConnections();
  }

  static const char resp[] =
      "<!DOCTYPE html><html>"
      "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Saved!</h2><p>Rebooting&hellip;</p></body></html>";
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

  if (_led)
    _led->flashLed(3, 80, 80);
  vTaskDelay(pdMS_TO_TICKS(800));
  esp_restart();
}

void ConfigManager::_handleClearBonds(httpd_req_t *req)
{
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

  if (_led)
    _led->flashLed(3, 80, 80);
  vTaskDelay(pdMS_TO_TICKS(800));
  esp_restart();
}

// Stream a multipart/form-data firmware upload directly to OTA flash.
void ConfigManager::_handleUpdate(httpd_req_t *req)
{
  // Extract boundary from Content-Type header
  char ct[256] = {};
  httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct));
  const char *bnd_prefix = "boundary=";
  const char *bnd_start = strstr(ct, bnd_prefix);
  if (!bnd_start)
  {
    httpd_resp_send_500(req);
    return;
  }
  bnd_start += strlen(bnd_prefix);

  std::string boundary("--");
  boundary += bnd_start;
  std::string endBoundary(boundary + "--");
  size_t eblen = endBoundary.size();

  const esp_partition_t *ota_part = esp_ota_get_next_update_partition(nullptr);
  if (!ota_part)
  {
    httpd_resp_send_500(req);
    return;
  }

  esp_ota_handle_t ota_handle;
  if (esp_ota_begin(ota_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle) != ESP_OK)
  {
    httpd_resp_send_500(req);
    return;
  }

  const size_t BUF = 1024;
  char buf[BUF];
  int remaining = req->content_len;
  bool ota_ok = true;
  bool in_data = false;

  // header_buf accumulates bytes until we find \r\n\r\n (part header end).
  std::string header_buf;
  header_buf.reserve(512);

  // tail holds the last (eblen+4) bytes so we can strip the trailing boundary.
  std::string tail;
  tail.reserve(eblen + 6);

  while (remaining > 0 && ota_ok)
  {
    size_t to_read = std::min((size_t)(remaining), BUF);
    int got = httpd_req_recv(req, buf, to_read);
    if (got <= 0)
    {
      ota_ok = false;
      break;
    }
    remaining -= got;

    if (!in_data)
    {
      header_buf.append(buf, got);
      if (header_buf.size() > 2048)
      {
        ota_ok = false;
        break;
      } // malformed header
      size_t pos = header_buf.find("\r\n\r\n");
      if (pos != std::string::npos)
      {
        in_data = true;
        const char *data = header_buf.c_str() + pos + 4;
        size_t dlen = header_buf.size() - (pos + 4);
        if (dlen > 0)
        {
          tail.append(data, dlen);
          if (tail.size() > eblen + 4)
          {
            size_t wlen = tail.size() - (eblen + 4);
            if (esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK)
              ota_ok = false;
            tail.erase(0, wlen);
          }
        }
        header_buf.clear();
      }
    }
    else
    {
      tail.append(buf, got);
      if (tail.size() > eblen + 4)
      {
        size_t wlen = tail.size() - (eblen + 4);
        if (esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK)
          ota_ok = false;
        tail.erase(0, wlen);
      }
    }
  }

  // Write remaining tail, stripping the trailing boundary marker.
  if (ota_ok && !tail.empty())
  {
    size_t pos = tail.rfind(endBoundary);
    size_t wlen = (pos != std::string::npos) ? pos : tail.size();
    // Strip the preceding \r\n that separates firmware data from boundary
    if (wlen >= 2)
      wlen -= 2;
    if (wlen > 0 && esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK)
      ota_ok = false;
  }

  if (ota_ok)
  {
    ota_ok = (esp_ota_end(ota_handle) == ESP_OK);
  }
  else
  {
    esp_ota_abort(ota_handle);
  }
  if (ota_ok)
    esp_ota_set_boot_partition(ota_part);

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

  if (ota_ok)
  {
    if (_led)
      _led->flashLed(5, 50, 50);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
  }
}
