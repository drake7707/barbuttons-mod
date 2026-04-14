#include "config/WebUIConfigManager.h"
#include "config/ConfigManager.h"
#include "cJSON.h"

void WebUIConfigManager::begin(const char *firmwareVersion)
{
  strncpy(_firmwareVersion, firmwareVersion, sizeof(_firmwareVersion) - 1);
  _firmwareVersion[sizeof(_firmwareVersion) - 1] = '\0';
}

// ---------------------------------------------------------------------------
// AP / HTTP server -- config mode
// ---------------------------------------------------------------------------
void WebUIConfigManager::beginConfigAP(ConfigManager *configManager,
                                        const std::vector<std::string> &bondList,
                                        int batVoltageMv, int batPercent)
{
  _configManager = configManager;
  _bondList      = bondList;
  _batVoltageMv  = batVoltageMv;
  _batPercent    = batPercent;
  _exitConfig    = false;

  _apNetif = esp_netif_create_default_wifi_ap();
  wifi_init_config_t wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&wifiInitConfig);

  wifi_config_t apConfig = {};
  strncpy((char *)apConfig.ap.ssid,     _apSsid, sizeof(apConfig.ap.ssid));
  strncpy((char *)apConfig.ap.password, _apPswd, sizeof(apConfig.ap.password));
  apConfig.ap.ssid_len       = (uint8_t)strlen(_apSsid);
  apConfig.ap.max_connection = 4;
  apConfig.ap.authmode       = WIFI_AUTH_WPA2_PSK;
  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &apConfig);
  esp_wifi_start();

  if (DEBUG)
    printf("[CONFIG] AP started: 192.168.4.1\n");

  httpd_config_t httpdConfig = HTTPD_DEFAULT_CONFIG();
  httpdConfig.stack_size        = 8192;
  httpdConfig.recv_wait_timeout = 60;
  httpdConfig.send_wait_timeout = 60;
  httpd_start(&_server, &httpdConfig);

  httpd_uri_t rootUri       = {"/",           HTTP_GET,  _s_root,       this};
  httpd_uri_t saveUri       = {"/save",        HTTP_POST, _s_save,       this};
  httpd_uri_t clearBondsUri = {"/clearbonds",  HTTP_POST, _s_clearbonds, this};
  httpd_uri_t updateUri     = {"/update",      HTTP_POST, _s_update,     this};
  httpd_register_uri_handler(_server, &rootUri);
  httpd_register_uri_handler(_server, &saveUri);
  httpd_register_uri_handler(_server, &clearBondsUri);
  httpd_register_uri_handler(_server, &updateUri);
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
  size_t firstNonSpace = s.find_first_not_of(" \t\r\n");
  if (firstNonSpace == std::string::npos)
  {
    s.clear();
    return;
  }
  s.erase(0, firstNonSpace);
  size_t lastNonSpace = s.find_last_not_of(" \t\r\n");
  if (lastNonSpace != std::string::npos)
    s.erase(lastNonSpace + 1);
}

std::string WebUIConfigManager::_urlDecode(const char *src, size_t len)
{
  std::string result;
  result.reserve(len);
  for (size_t i = 0; i < len; i++)
  {
    if (src[i] == '%' && i + 2 < len)
    {
      char hex[3] = {src[i + 1], src[i + 2], '\0'};
      result += (char)strtol(hex, nullptr, 16);
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
  size_t nameLength = strlen(name);
  const char *pos   = body;
  while (pos && *pos)
  {
    if (strncmp(pos, name, nameLength) == 0 && pos[nameLength] == '=')
    {
      const char *val    = pos + nameLength + 1;
      const char *end    = strchr(val, '&');
      size_t valueLength = end ? (size_t)(end - val) : strlen(val);
      return _urlDecode(val, valueLength);
    }
    pos = strchr(pos, '&');
    if (pos)
      pos++;
  }
  return std::string();
}

bool WebUIConfigManager::_readBody(httpd_req_t *req, std::string &out)
{
  const int total = req->content_len;
  if (total <= 0)
  {
    out.clear();
    return true;
  }
  out.resize(total);
  int received = 0;
  while (received < total)
  {
    const int bytesRead = httpd_req_recv(req, &out[received], total - received);
    if (bytesRead <= 0)
      return false;
    received += bytesRead;
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

  cJSON *root = cJSON_CreateObject();
  cJSON_AddStringToObject(root, "fwVer",            _firmwareVersion);
  cJSON_AddStringToObject(root, "bleName",           _configManager->getBleName());
  cJSON_AddStringToObject(root, "defaultBleName",    DEFAULT_BLE_NAME);
  cJSON_AddBoolToObject(  root, "batteryEnabled",    _configManager->isBatteryEnabled());
  cJSON_AddBoolToObject(  root, "blePowerSaving",    _configManager->allowBLEPowerSaving());
  cJSON_AddBoolToObject(  root, "batterySection",    !LEGACY);
  cJSON_AddNumberToObject(root, "maxBleConnections", _configManager->getMaxBLEConnections());
  cJSON_AddNumberToObject(root, "activeKeymap",      _configManager->getActiveKeymap());
  cJSON_AddNumberToObject(root, "batVoltageMv",      _batVoltageMv);
  cJSON_AddNumberToObject(root, "batPercent",        _batPercent);

  cJSON_AddNumberToObject(root, "TARGET_SELECT", TARGET_SELECT);
  cJSON_AddNumberToObject(root, "TARGET_HID",    TARGET_HID);
  cJSON_AddNumberToObject(root, "TARGET_BTHOME", TARGET_BTHOME);

  cJSON *defaultShortArr = cJSON_CreateArray();
  cJSON *defaultLongArr  = cJSON_CreateArray();
  for (int i = 0; i < 8; i++)
  {
    cJSON_AddItemToArray(defaultShortArr, cJSON_CreateNumber(Config::DEFAULT_SHORT[i]));
    cJSON_AddItemToArray(defaultLongArr,  cJSON_CreateNumber(Config::DEFAULT_LONG[i]));
  }
  cJSON_AddItemToObject(root, "defaultShort", defaultShortArr);
  cJSON_AddItemToObject(root, "defaultLong",  defaultLongArr);

  cJSON *bondsArr = cJSON_CreateArray();
  for (const auto &bond : _bondList)
    cJSON_AddItemToArray(bondsArr, cJSON_CreateString(bond.c_str()));
  cJSON_AddItemToObject(root, "bonds", bondsArr);

  cJSON *keymapArr = cJSON_CreateArray();
  for (int keymap = 0; keymap < 3; keymap++)
  {
    cJSON *keymapObj   = cJSON_CreateObject();
    cJSON *shortKeys   = cJSON_CreateArray(), *longKeys    = cJSON_CreateArray();
    cJSON *shortTargets = cJSON_CreateArray(), *longTargets = cJSON_CreateArray();
    cJSON *shortMacs   = cJSON_CreateArray(), *longMacs    = cJSON_CreateArray();
    for (int i = 0; i < 8; i++)
    {
      const KeyEntry &shortEntry = _configManager->rawShortEntry(keymap, i);
      const KeyEntry &longEntry  = _configManager->rawLongEntry(keymap, i);
      cJSON_AddItemToArray(shortKeys,    cJSON_CreateNumber(shortEntry.key));
      cJSON_AddItemToArray(longKeys,     cJSON_CreateNumber(longEntry.key));
      cJSON_AddItemToArray(shortTargets, cJSON_CreateNumber((uint8_t)shortEntry.target));
      cJSON_AddItemToArray(longTargets,  cJSON_CreateNumber((uint8_t)longEntry.target));
      cJSON_AddItemToArray(shortMacs,    cJSON_CreateString(shortEntry.mac));
      cJSON_AddItemToArray(longMacs,     cJSON_CreateString(longEntry.mac));
    }
    cJSON_AddItemToObject(keymapObj, "S",  shortKeys);
    cJSON_AddItemToObject(keymapObj, "L",  longKeys);
    cJSON_AddItemToObject(keymapObj, "ST", shortTargets);
    cJSON_AddItemToObject(keymapObj, "LT", longTargets);
    cJSON_AddItemToObject(keymapObj, "SM", shortMacs);
    cJSON_AddItemToObject(keymapObj, "LM", longMacs);
    cJSON_AddItemToArray(keymapArr, keymapObj);
  }
  cJSON_AddItemToObject(root, "km", keymapArr);

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

  for (int keymap = 0; keymap < 3; keymap++)
  {
    for (int i = 0; i < 8; i++)
    {
      char slotKey[8];
      snprintf(slotKey, sizeof(slotKey), "%d_%d", keymap + 1, i);

      std::string shortVal = _formParam(body.c_str(), (std::string("s") + slotKey).c_str());
      std::string longVal  = _formParam(body.c_str(), (std::string("l") + slotKey).c_str());
      if (!shortVal.empty())
        _configManager->rawShortEntry(keymap, i).key = (uint8_t)atoi(shortVal.c_str());
      if (!longVal.empty())
        _configManager->rawLongEntry(keymap, i).key  = (uint8_t)atoi(longVal.c_str());

      // Short press target: "0"=select, "1[:<mac>]"=HID, "2"=BTHome
      std::string shortTargetVal = _formParam(body.c_str(), (std::string("ts") + slotKey).c_str());
      if (!shortTargetVal.empty())
      {
        KeyEntry &shortEntry = _configManager->rawShortEntry(keymap, i);
        if (shortTargetVal[0] == '1')
        {
          shortEntry.target = TARGET_HID;
          std::string mac = (shortTargetVal.size() > 2) ? shortTargetVal.substr(2) : "";
          strncpy(shortEntry.mac, mac.c_str(), sizeof(shortEntry.mac) - 1);
          shortEntry.mac[sizeof(shortEntry.mac) - 1] = '\0';
        }
        else if (shortTargetVal[0] == '2')
        {
          shortEntry.target    = TARGET_BTHOME;
          shortEntry.mac[0]    = '\0';
        }
        else
        {
          shortEntry.target    = TARGET_SELECT;
          shortEntry.mac[0]    = '\0';
        }
      }

      // Long press target
      std::string longTargetVal = _formParam(body.c_str(), (std::string("tl") + slotKey).c_str());
      if (!longTargetVal.empty())
      {
        KeyEntry &longEntry = _configManager->rawLongEntry(keymap, i);
        if (longTargetVal[0] == '1')
        {
          longEntry.target = TARGET_HID;
          std::string mac = (longTargetVal.size() > 2) ? longTargetVal.substr(2) : "";
          strncpy(longEntry.mac, mac.c_str(), sizeof(longEntry.mac) - 1);
          longEntry.mac[sizeof(longEntry.mac) - 1] = '\0';
        }
        else if (longTargetVal[0] == '2')
        {
          longEntry.target  = TARGET_BTHOME;
          longEntry.mac[0]  = '\0';
        }
        else
        {
          longEntry.target  = TARGET_SELECT;
          longEntry.mac[0]  = '\0';
        }
      }
    }
  }

  std::string newBleName = _formParam(body.c_str(), "blename");
  _strTrim(newBleName);
  bool bleNameValid = newBleName.size() > 0 && newBleName.size() <= BLE_NAME_MAX_LEN;
  if (bleNameValid)
    for (char c : newBleName)
      if (c < 0x20 || c > 0x7E)
      {
        bleNameValid = false;
        break;
      }
  if (bleNameValid)
    _configManager->setBleName(newBleName.c_str());

  _configManager->setBatteryEnabled(_formParam(body.c_str(), "battery_enabled") == "1");
  _configManager->setBlePowerSaving(_formParam(body.c_str(), "ble_power_saving") == "1");

  int maxConnections = atoi(_formParam(body.c_str(), "max_ble_connections").c_str());
  if (maxConnections >= 1 && maxConnections <= 3)
    _configManager->setMaxBLEConnections((uint8_t)maxConnections);

  _configManager->saveConfig();

  static const char resp[] =
      "<!DOCTYPE html><html>"
      "<head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Saved!</h2><p>Rebooting&hellip;</p></body></html>";
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

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

  vTaskDelay(pdMS_TO_TICKS(800));
  esp_restart();
}

// Stream a multipart/form-data firmware upload directly to OTA flash.
void WebUIConfigManager::_handleUpdate(httpd_req_t *req)
{
  char contentType[256] = {};
  httpd_req_get_hdr_value_str(req, "Content-Type", contentType, sizeof(contentType));
  const char *boundaryPrefix = "boundary=";
  const char *boundaryStart  = strstr(contentType, boundaryPrefix);
  if (!boundaryStart)
  {
    httpd_resp_send_500(req);
    return;
  }
  boundaryStart += strlen(boundaryPrefix);

  std::string boundary("--");
  boundary += boundaryStart;
  std::string endBoundary(boundary + "--");
  size_t endBoundaryLength = endBoundary.size();

  const esp_partition_t *otaPartition = esp_ota_get_next_update_partition(nullptr);
  if (!otaPartition)
  {
    httpd_resp_send_500(req);
    return;
  }

  esp_ota_handle_t otaHandle;
  if (esp_ota_begin(otaPartition, OTA_WITH_SEQUENTIAL_WRITES, &otaHandle) != ESP_OK)
  {
    httpd_resp_send_500(req);
    return;
  }

  const size_t READ_BUF_SIZE = 1024;
  char readBuf[READ_BUF_SIZE];
  int  remaining   = req->content_len;
  bool otaSucceeded = true;
  bool inData      = false;

  std::string headerBuf;
  headerBuf.reserve(512);
  std::string tail;
  tail.reserve(endBoundaryLength + 6);

  while (remaining > 0 && otaSucceeded)
  {
    const size_t toRead = std::min((size_t)remaining, READ_BUF_SIZE);
    const int bytesGot  = httpd_req_recv(req, readBuf, toRead);
    if (bytesGot <= 0)
    {
      otaSucceeded = false;
      break;
    }
    remaining -= bytesGot;

    if (!inData)
    {
      headerBuf.append(readBuf, bytesGot);
      if (headerBuf.size() > 2048)
      {
        otaSucceeded = false;
        break;
      }
      size_t headerEnd = headerBuf.find("\r\n\r\n");
      if (headerEnd != std::string::npos)
      {
        inData = true;
        const char *dataStart  = headerBuf.c_str() + headerEnd + 4;
        size_t dataLength = headerBuf.size() - (headerEnd + 4);
        if (dataLength > 0)
        {
          tail.append(dataStart, dataLength);
          if (tail.size() > endBoundaryLength + 4)
          {
            size_t writeLength = tail.size() - (endBoundaryLength + 4);
            if (esp_ota_write(otaHandle, tail.data(), writeLength) != ESP_OK)
              otaSucceeded = false;
            tail.erase(0, writeLength);
          }
        }
        headerBuf.clear();
      }
    }
    else
    {
      tail.append(readBuf, bytesGot);
      if (tail.size() > endBoundaryLength + 4)
      {
        size_t writeLength = tail.size() - (endBoundaryLength + 4);
        if (esp_ota_write(otaHandle, tail.data(), writeLength) != ESP_OK)
          otaSucceeded = false;
        tail.erase(0, writeLength);
      }
    }
  }

  if (otaSucceeded && !tail.empty())
  {
    size_t boundaryPos = tail.rfind(endBoundary);
    size_t writeLength = (boundaryPos != std::string::npos) ? boundaryPos : tail.size();
    if (writeLength >= 2)
      writeLength -= 2;
    if (writeLength > 0 && esp_ota_write(otaHandle, tail.data(), writeLength) != ESP_OK)
      otaSucceeded = false;
  }

  if (otaSucceeded)
    otaSucceeded = (esp_ota_end(otaHandle) == ESP_OK);
  else
    esp_ota_abort(otaHandle);
  if (otaSucceeded)
    esp_ota_set_boot_partition(otaPartition);

  static const char respOk[] =
      "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10003; Update successful!</h2><p>Rebooting&hellip;</p></body></html>";
  static const char respFail[] =
      "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'></head>"
      "<body style='font-family:sans-serif;max-width:400px;margin:60px auto;text-align:center'>"
      "<h2>&#10007; Update failed.</h2><p><a href='/'>Try again</a></p></body></html>";

  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, otaSucceeded ? respOk : respFail, HTTPD_RESP_USE_STRLEN);

  if (otaSucceeded)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
  }
}
