#include "config/WebUIConfigManager.h"
#include "config/ConfigManager.h"
#include "cJSON.h"

void WebUIConfigManager::begin(StatusLedManager *led, const char *firmwareVersion)
{
  _led = led;
  strncpy(_firmwareVersion, firmwareVersion, sizeof(_firmwareVersion) - 1);
  _firmwareVersion[sizeof(_firmwareVersion) - 1] = '\0';
}

// ---------------------------------------------------------------------------
// AP / HTTP server -- config mode
// ---------------------------------------------------------------------------
void WebUIConfigManager::beginConfigAP(ConfigManager *cm,
                                       const std::vector<std::string> &bondList,
                                       int batVoltageMv, int batPercent)
{
  _configManager = cm;
  _bondList = bondList;
  _batVoltageMv = batVoltageMv;
  _batPercent = batPercent;
  _exitConfig = false;

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

  if (_led)
    _led->flashLed(5, 100, 100);
}

void WebUIConfigManager::endConfigAP()
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
void WebUIConfigManager::_strReplace(std::string &s, const std::string &from, const std::string &to)
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

void WebUIConfigManager::_strTrim(std::string &s)
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

std::string WebUIConfigManager::_urlDecode(const char *src, size_t len)
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
      result += ' ';
    else
      result += src[i];
  }
  return result;
}

std::string WebUIConfigManager::_formParam(const char *body, const char *name)
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

bool WebUIConfigManager::_readBody(httpd_req_t *req, std::string &out)
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
// Static trampolines
// ---------------------------------------------------------------------------
esp_err_t WebUIConfigManager::_s_root(httpd_req_t *req)
{
  ((WebUIConfigManager *)req->user_ctx)->_handleRoot(req);
  return ESP_OK;
}
esp_err_t WebUIConfigManager::_s_save(httpd_req_t *req)
{
  ((WebUIConfigManager *)req->user_ctx)->_handleSave(req);
  return ESP_OK;
}
esp_err_t WebUIConfigManager::_s_clearbonds(httpd_req_t *req)
{
  ((WebUIConfigManager *)req->user_ctx)->_handleClearBonds(req);
  return ESP_OK;
}
esp_err_t WebUIConfigManager::_s_update(httpd_req_t *req)
{
  ((WebUIConfigManager *)req->user_ctx)->_handleUpdate(req);
  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Web handlers
// ---------------------------------------------------------------------------
void WebUIConfigManager::_handleRoot(httpd_req_t *req)
{
  std::string html(reinterpret_cast<const char *>(config_html_start),
                   config_html_end - config_html_start - 1);

  const Config &cfg = _configManager->config;

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "fwVer", _firmwareVersion);
  cJSON_AddStringToObject(root, "bleName", cfg.bleName);
  cJSON_AddStringToObject(root, "defaultBleName", DEFAULT_BLE_NAME);
  cJSON_AddBoolToObject(root, "batteryEnabled", cfg.batteryEnabled);
  cJSON_AddBoolToObject(root, "blePowerSaving", cfg.blePowerSaving);
  cJSON_AddBoolToObject(root, "batterySection", !LEGACY);
  cJSON_AddNumberToObject(root, "maxBleConnections", cfg.maxBLEConnections);
  cJSON_AddNumberToObject(root, "activeKeymap", cfg.activeKeymap);
  cJSON_AddNumberToObject(root, "batVoltageMv", _batVoltageMv);
  cJSON_AddNumberToObject(root, "batPercent", _batPercent);

  cJSON_AddNumberToObject(root, "TARGET_SELECT", TARGET_SELECT);
  cJSON_AddNumberToObject(root, "TARGET_HID", TARGET_HID);
  cJSON_AddNumberToObject(root, "TARGET_BTHOME", TARGET_BTHOME);

  cJSON *dsArr = cJSON_CreateArray();
  cJSON *dlArr = cJSON_CreateArray();
  for (int i = 0; i < 8; i++)
  {
    cJSON_AddItemToArray(dsArr, cJSON_CreateNumber(Config::DEFAULT_SHORT[i]));
    cJSON_AddItemToArray(dlArr, cJSON_CreateNumber(Config::DEFAULT_LONG[i]));
  }
  cJSON_AddItemToObject(root, "defaultShort", dsArr);
  cJSON_AddItemToObject(root, "defaultLong", dlArr);

  cJSON *bondsArr = cJSON_CreateArray();
  for (const auto &b : _bondList)
    cJSON_AddItemToArray(bondsArr, cJSON_CreateString(b.c_str()));
  cJSON_AddItemToObject(root, "bonds", bondsArr);

  cJSON *kmArr = cJSON_CreateArray();
  for (int km = 0; km < 3; km++)
  {
    cJSON *kmObj = cJSON_CreateObject();
    cJSON *sArr = cJSON_CreateArray(), *lArr = cJSON_CreateArray();
    cJSON *stArr = cJSON_CreateArray(), *ltArr = cJSON_CreateArray();
    cJSON *smArr = cJSON_CreateArray(), *lmArr = cJSON_CreateArray();
    for (int i = 0; i < 8; i++)
    {
      const KeyEntry &se = cfg.shortEntries[km][i];
      const KeyEntry &le = cfg.longEntries[km][i];
      cJSON_AddItemToArray(sArr, cJSON_CreateNumber(se.key));
      cJSON_AddItemToArray(lArr, cJSON_CreateNumber(le.key));
      cJSON_AddItemToArray(stArr, cJSON_CreateNumber((uint8_t)se.target));
      cJSON_AddItemToArray(ltArr, cJSON_CreateNumber((uint8_t)le.target));
      cJSON_AddItemToArray(smArr, cJSON_CreateString(se.mac));
      cJSON_AddItemToArray(lmArr, cJSON_CreateString(le.mac));
    }
    cJSON_AddItemToObject(kmObj, "S", sArr);
    cJSON_AddItemToObject(kmObj, "L", lArr);
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

void WebUIConfigManager::_handleSave(httpd_req_t *req)
{
  std::string body;
  if (!_readBody(req, body))
  {
    httpd_resp_send_500(req);
    return;
  }

  Config &cfg = _configManager->config;

  for (int km = 0; km < 3; km++)
  {
    for (int i = 0; i < 8; i++)
    {
      char si[8];
      snprintf(si, sizeof(si), "%d_%d", km + 1, i);

      std::string sv = _formParam(body.c_str(), (std::string("s") + si).c_str());
      std::string lv = _formParam(body.c_str(), (std::string("l") + si).c_str());
      if (!sv.empty())
        cfg.shortEntries[km][i].key = (uint8_t)atoi(sv.c_str());
      if (!lv.empty())
        cfg.longEntries[km][i].key = (uint8_t)atoi(lv.c_str());

      // Short press target: "0"=select, "1[:<mac>]"=HID, "2"=BTHome
      std::string tsv = _formParam(body.c_str(), (std::string("ts") + si).c_str());
      if (!tsv.empty())
      {
        KeyEntry &se = cfg.shortEntries[km][i];
        if (tsv[0] == '1')
        {
          se.target = TARGET_HID;
          std::string m = (tsv.size() > 2) ? tsv.substr(2) : "";
          strncpy(se.mac, m.c_str(), sizeof(se.mac) - 1);
          se.mac[sizeof(se.mac) - 1] = '\0';
        }
        else if (tsv[0] == '2')
        {
          se.target = TARGET_BTHOME;
          se.mac[0] = '\0';
        }
        else
        {
          se.target = TARGET_SELECT;
          se.mac[0] = '\0';
        }
      }

      // Long press target
      std::string tlv = _formParam(body.c_str(), (std::string("tl") + si).c_str());
      if (!tlv.empty())
      {
        KeyEntry &le = cfg.longEntries[km][i];
        if (tlv[0] == '1')
        {
          le.target = TARGET_HID;
          std::string m = (tlv.size() > 2) ? tlv.substr(2) : "";
          strncpy(le.mac, m.c_str(), sizeof(le.mac) - 1);
          le.mac[sizeof(le.mac) - 1] = '\0';
        }
        else if (tlv[0] == '2')
        {
          le.target = TARGET_BTHOME;
          le.mac[0] = '\0';
        }
        else
        {
          le.target = TARGET_SELECT;
          le.mac[0] = '\0';
        }
      }
    }
  }

  std::string newName = _formParam(body.c_str(), "blename");
  _strTrim(newName);
  bool valid = newName.size() > 0 && newName.size() <= BLE_NAME_MAX_LEN;
  if (valid)
    for (char c : newName)
      if (c < 0x20 || c > 0x7E)
      {
        valid = false;
        break;
      }
  if (valid)
  {
    strncpy(cfg.bleName, newName.c_str(), sizeof(cfg.bleName) - 1);
    cfg.bleName[sizeof(cfg.bleName) - 1] = '\0';
  }

  std::string batParam = _formParam(body.c_str(), "battery_enabled");
  cfg.batteryEnabled = (batParam == "1");

  std::string blePSParam = _formParam(body.c_str(), "ble_power_saving");
  cfg.blePowerSaving = (blePSParam == "1");

  std::string maxConnParam = _formParam(body.c_str(), "max_ble_connections");
  int maxConn = atoi(maxConnParam.c_str());
  if (maxConn >= 1 && maxConn <= 3)
    cfg.maxBLEConnections = (uint8_t)maxConn;

  _configManager->saveConfig();

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

void WebUIConfigManager::_handleClearBonds(httpd_req_t *req)
{
  // BLE is stopped while the WiFi AP runs; write a flag and reboot so setup()
  // can delete bonds once NimBLE is re-initialised.
  _configManager->requestClearBonds();

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
void WebUIConfigManager::_handleUpdate(httpd_req_t *req)
{
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

  std::string header_buf;
  header_buf.reserve(512);
  std::string tail;
  tail.reserve(eblen + 6);

  while (remaining > 0 && ota_ok)
  {
    size_t to_read = std::min((size_t)remaining, BUF);
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
      }
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

  if (ota_ok && !tail.empty())
  {
    size_t pos = tail.rfind(endBoundary);
    size_t wlen = (pos != std::string::npos) ? pos : tail.size();
    if (wlen >= 2)
      wlen -= 2;
    if (wlen > 0 && esp_ota_write(ota_handle, tail.data(), wlen) != ESP_OK)
      ota_ok = false;
  }

  if (ota_ok)
    ota_ok = (esp_ota_end(ota_handle) == ESP_OK);
  else
    esp_ota_abort(ota_handle);
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
