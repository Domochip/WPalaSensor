#include "Core.h"
#include <EEPROM.h>
#include "../Main.h" //for VERSION define
#include "Version.h" //for BASE_VERSION define

#include "data/index.html.gz.h"
#include "data/pure-min.css.gz.h"
#include "data/side-menu.css.gz.h"
#include "data/side-menu.js.gz.h"

void Core::setConfigDefaultValues() {};
bool Core::parseConfigJSON(JsonDocument &doc, bool fromWebPage = false) { return true; };
String Core::generateConfigJSON(bool clearPassword = false) { return String(); };
String Core::generateStatusJSON()
{
  JsonDocument doc;

  char sn[9];
#ifdef ESP8266
  sprintf_P(sn, PSTR("%08x"), ESP.getChipId());
#else
  sprintf_P(sn, PSTR("%08x"), (uint32_t)(ESP.getEfuseMac() << 40 >> 40));
#endif
  unsigned long minutes = millis() / 60000;

  doc["sn"] = sn;
  doc["baseversion"] = BASE_VERSION;
  doc["version"] = VERSION;
  doc["uptime"] = String((byte)(minutes / 1440)) + 'd' + (byte)(minutes / 60 % 24) + 'h' + (byte)(minutes % 60) + 'm';
  doc["freeheap"] = ESP.getFreeHeap();
#ifdef ESP8266
  doc["freestack"] = ESP.getFreeContStack();
  doc["flashsize"] = ESP.getFlashChipRealSize();
#else
  doc["freestack"] = uxTaskGetStackHighWaterMark(nullptr);
#endif

  String gs;
  serializeJson(doc, gs);

  return gs;
}
bool Core::appInit(bool reInit = false)
{

// init ticker to check for update every 24h
#ifdef ESP8266
  _checkForUpdateTicker.attach(86400, [this]()
                               { this->_needCheckForUpdateTick = true; });
#else
  _checkForUpdateTicker.attach<typeof this>(86400, [](typeof this core)
                                            { core->_needCheckForUpdateTick = true; }, this);
#endif

  return true;
};
const PROGMEM char *Core::getHTMLContent(WebPageForPlaceHolder wp)
{
  switch (wp)
  {
  case status:
    return status0htmlgz;
    break;
  case config:
    return config0htmlgz;
    break;
  };
  return nullptr;
}
size_t Core::getHTMLContentSize(WebPageForPlaceHolder wp)
{
  switch (wp)
  {
  case status:
    return sizeof(status0htmlgz);
    break;
  case config:
    return sizeof(config0htmlgz);
    break;
  };
  return 0;
}
void Core::appInitWebServer(WebServer &server, bool &shouldReboot, bool &pauseApplication)
{
  // root is index
  server.on("/", HTTP_GET, [&server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, PSTR("text/html"), indexhtmlgz, sizeof(indexhtmlgz)); });

  // Ressources URLs
  server.on("/pure-min.css", HTTP_GET, [&server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.sendHeader(F("Cache-Control"), F("max-age=604800, public"));
    server.send_P(200, PSTR("text/css"), puremincssgz, sizeof(puremincssgz)); });

  server.on("/side-menu.css", HTTP_GET, [&server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.sendHeader(F("Cache-Control"), F("max-age=604800, public"));
    server.send_P(200, PSTR("text/css"), sidemenucssgz, sizeof(sidemenucssgz)); });

  server.on("/side-menu.js", HTTP_GET, [&server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.sendHeader(F("Cache-Control"), F("max-age=604800, public"));
    server.send_P(200, PSTR("text/javascript"), sidemenujsgz, sizeof(sidemenujsgz)); });

  server.on("/fw.html", HTTP_GET, [this, &server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, PSTR("text/html"), fwhtmlgz, sizeof(fwhtmlgz)); });

  // Get Update Infos ---------------------------------------------------------
  server.on("/gui", HTTP_GET, [this, &server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.send(200, F("application/json"), getUpdateInfos(server.hasArg("refresh"))); });

  // Update Firmware from Github ----------------------------------------------
  server.on("/update", HTTP_POST, [this, &shouldReboot, &pauseApplication, &server]()
            {
              shouldReboot = updateFirmware(server.arg("plain").c_str());

              SERVER_KEEPALIVE_FALSE()
              if (shouldReboot)
                server.send(200, F("text/html"), F("Firmware Successfully Updated"));
              else
                server.send(500, F("text/html"), F("Firmware Update Failed")); });

  // Firmware POST URL allows to push new firmware ----------------------------
  server.on(
      "/fw", HTTP_POST, [&shouldReboot, &pauseApplication, &server]()
      {
    shouldReboot = !Update.hasError();
    if (shouldReboot)
    {
      SERVER_KEEPALIVE_FALSE()
      server.send(200, F("text/html"), F("Firmware Successfully Updated"));
    }
    else
    {
      // Upload failed so restart to Run Application in loop
      pauseApplication = false;
      // Prepare response
      String errorMsg;
#ifdef ESP8266
      errorMsg = Update.getErrorString();
#else
      errorMsg = Update.errorString();
#endif
      SERVER_KEEPALIVE_FALSE()
      server.send(500, F("text/html"), errorMsg);
    } },
      [&pauseApplication, &server]()
      {
        HTTPUpload &upload = server.upload();

        if (upload.status == UPLOAD_FILE_START)
        {
          // stop to Run Application in loop
          pauseApplication = true;

#ifdef LOG_SERIAL
          // Set Update onError callback
          Update.onError([](uint8_t err)
                         { Update.printError(LOG_SERIAL); });

          LOG_SERIAL.printf("Update Start: %s\n", upload.filename.c_str());
#endif

#ifdef ESP8266
          Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000);
#else
          Update.begin();
#endif
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          Update.write(upload.buf, upload.currentSize);
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
#ifdef LOG_SERIAL
            LOG_SERIAL.printf("Update Success: %uB\n", upload.totalSize);
#endif
        }

#ifdef ESP8266
        // Feed the dog otherwise big firmware won't pass
        ESP.wdtFeed();
#endif
        yield();
      });

  // reboot POST --------------------------------------------------------------
  server.on("/rbt", HTTP_POST, [&shouldReboot, &server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.send_P(200,PSTR("text/html"),PSTR("Reboot command received"));
    shouldReboot = true; });

  // reboot RescueMode POST ---------------------------------------------------
  server.on("/rbtrsc", HTTP_POST, [&shouldReboot, &server]()
            {
    SERVER_KEEPALIVE_FALSE()
    server.send_P(200,PSTR("text/html"),PSTR("Reboot in rescue command received"));
    //Set EEPROM for Rescue mode flag
    EEPROM.begin(4);
    EEPROM.write(0, 1);
    EEPROM.end();
    shouldReboot = true; });

  // 404 on not found ---------------------------------------------------------
  server.onNotFound([&server]()
                    {
    SERVER_KEEPALIVE_FALSE()
    server.send(404); });
}

void Core::appRun()
{
  if (_needCheckForUpdateTick)
  {
    _needCheckForUpdateTick = false;
    // check for update
    checkForUpdate();
  }
}

Core::Core(char appId, String appName) : Application(appId, appName)
{
  _applicationList[Application::Applications::Core] = this;
}

void Core::checkForUpdate()
{
  String githubURL = "https://api.github.com/repos/" APPLICATION1_MANUFACTURER "/" APPLICATION1_MODEL "/releases/latest";

  WiFiClientSecure clientSecure;
  HTTPClient http;

  clientSecure.setInsecure();
  http.begin(clientSecure, githubURL);
  int httpCode = http.GET();
  if (httpCode == 200)
  {
    WiFiClient *stream = http.getStreamPtr();
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, *stream);
    if (!error)
    {
      JsonVariant jv;
      if ((jv = doc["tag_name"]).is<const char *>())
        strlcpy(_lastFirmwareInfos.version, jv, sizeof(_lastFirmwareInfos.version));
      if ((jv = doc["name"]).is<const char *>())
      {
        // find the first space and copy the rest to title
        char *space = strchr(jv, ' ');
        if (space)
          strlcpy(_lastFirmwareInfos.title, space + 1, sizeof(_lastFirmwareInfos.title));
        else
          _lastFirmwareInfos.title[0] = 0;
      }

      if ((jv = doc["published_at"]).is<const char *>())
        strlcpy(_lastFirmwareInfos.releaseDate, jv, sizeof(_lastFirmwareInfos.releaseDate));
      else
        _lastFirmwareInfos.releaseDate[0] = 0;

      if ((jv = doc["body"]).is<const char *>())
      {
        // copy body to summary until "\r\n\r\n##"
        const char *body = jv.as<const char *>();
        const char *end = strstr(body, "\r\n\r\n##");
        if (end)
        {
          size_t len = end - body;
          if (len >= sizeof(_lastFirmwareInfos.summary))
            len = sizeof(_lastFirmwareInfos.summary) - 1;

          // Copy the string considering multi-byte characters
          strncpy(_lastFirmwareInfos.summary, body, len);
          _lastFirmwareInfos.summary[len] = 0;
        }
        else
        {
          size_t bodyLen = strlen(body);
          if (bodyLen < sizeof(_lastFirmwareInfos.summary))
          {
            strncpy(_lastFirmwareInfos.summary, body, bodyLen);
            _lastFirmwareInfos.summary[bodyLen] = 0;
          }
          else
            _lastFirmwareInfos.summary[0] = 0;
        }
      }
    }
    else
      _lastFirmwareInfos.version[0] = 0;
  }
  else
    _lastFirmwareInfos.version[0] = 0;

  http.end();
}

String Core::getUpdateInfos(bool refresh)
{
  if (refresh)
    checkForUpdate();

  JsonDocument doc;

  doc["installed_version"] = VERSION;
  if (_lastFirmwareInfos.version[0])
  {
    doc["latest_version"] = _lastFirmwareInfos.version;
    doc["title"] = _lastFirmwareInfos.title;
    doc["release_date"] = _lastFirmwareInfos.releaseDate;
    doc["release_summary"] = _lastFirmwareInfos.summary;
    doc["release_url"] = String(F("https://github.com/" APPLICATION1_MANUFACTURER "/" APPLICATION1_MODEL "/releases/tag/")) + _lastFirmwareInfos.version;
  }

  String infos;
  serializeJson(doc, infos);

  return infos;
}

bool Core::updateFirmware(const char *version)
{
  char versionToFlash[8];

  if (version && !strcmp(version, "latest"))
  {
    checkForUpdate();
    if (_lastFirmwareInfos.version[0] and versionCompare(_lastFirmwareInfos.version, VERSION) > 0)
      strlcpy(versionToFlash, _lastFirmwareInfos.version, sizeof(versionToFlash));
    else
      return false;
  }
  else if (version && version[0])
    strlcpy(versionToFlash, version, sizeof(versionToFlash));
  else
    return false;

  WiFiClientSecure clientSecure;
  clientSecure.setInsecure();

  String fwUrl(F("https://github.com/" APPLICATION1_MANUFACTURER "/" APPLICATION1_MODEL "/releases/download/"));
  fwUrl = fwUrl + versionToFlash + '/' + APPLICATION1_MODEL + '.' + versionToFlash + F(".bin");

#ifdef LOG_SERIAL
  LOG_SERIAL.print(F("Trying to Update from URL: "));
  LOG_SERIAL.println(fwUrl);
#endif

  HTTPClient https;
  https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  https.begin(clientSecure, fwUrl);
  int httpCode = https.GET();

  if (httpCode != 200)
  {
#ifdef LOG_SERIAL
    LOG_SERIAL.print(F("Failed to download file, httpCode: "));
    LOG_SERIAL.println(httpCode);
#endif
    https.end();
    return false;
  }

  // starting here we have a valid httpCode (200)

  // get the stream
  WiFiClient *stream = https.getStreamPtr();
  int contentLength = https.getSize();

#ifdef LOG_SERIAL
  // Set Update onError callback
  Update.onError([](uint8_t err)
                 { Update.printError(LOG_SERIAL); });

  LOG_SERIAL.println(F("Firmware file found, Update Start"));
#endif

#ifdef ESP8266
  Update.begin(contentLength);
#else
  Update.begin();
#endif

  Update.writeStream(*stream);

  if (Update.end())
  {
#ifdef LOG_SERIAL
    LOG_SERIAL.printf("Update Success: %uB\n", contentLength);
#endif
  }

  https.end();

  return !Update.hasError();
}

int8_t Core::versionCompare(const char *version1, const char *version2)
{
  int8_t result = 0;

  // Create copies of the version strings
  char *v1 = strdup(version1);
  char *v2 = strdup(version2);

  // Tokenize the version strings
  char *token1 = strtok(v1, ".");
  char *token2 = strtok(v2, ".");

  while (token1 && token2)
  {
    int v1 = atoi(token1);
    int v2 = atoi(token2);

    if (v1 > v2)
    {
      result = 1;
      break;
    }
    else if (v1 < v2)
    {
      result = -1;
      break;
    }

    // Move to the next token
    token1 = strtok(NULL, ".");
    token2 = strtok(NULL, ".");
  }

  // Clean up
  free(v1);
  free(v2);

  return result;
}