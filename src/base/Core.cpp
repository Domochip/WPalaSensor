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
String Core::generateConfigJSON(bool forSaveFile = false) { return String(); };
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
  doc[F("baseversion")] = BASE_VERSION;
  doc[F("version")] = VERSION;
  doc[F("uptime")] = String((byte)(minutes / 1440)) + 'd' + (byte)(minutes / 60 % 24) + 'h' + (byte)(minutes % 60) + 'm';
  doc[F("freeheap")] = ESP.getFreeHeap();
#ifdef ESP8266
  doc[F("freestack")] = ESP.getFreeContStack();
  doc[F("flashsize")] = ESP.getFlashChipRealSize();
#else
  doc[F("freestack")] = uxTaskGetStackHighWaterMark(nullptr);
#endif

  String gs;
  serializeJson(doc, gs);

  return gs;
}
bool Core::appInit(bool reInit = false)
{
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
  server.on("/", HTTP_GET,
            [&server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.send_P(200, PSTR("text/html"), indexhtmlgz, sizeof(indexhtmlgz));
            });

  // Ressources URLs
  server.on(F("/pure-min.css"), HTTP_GET,
            [&server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.sendHeader(F("Cache-Control"), F("max-age=604800, public"));
              server.send_P(200, PSTR("text/css"), puremincssgz, sizeof(puremincssgz));
            });

  server.on(F("/side-menu.css"), HTTP_GET,
            [&server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.sendHeader(F("Cache-Control"), F("max-age=604800, public"));
              server.send_P(200, PSTR("text/css"), sidemenucssgz, sizeof(sidemenucssgz));
            });

  server.on(F("/side-menu.js"), HTTP_GET,
            [&server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.sendHeader(F("Cache-Control"), F("max-age=604800, public"));
              server.send_P(200, PSTR("text/javascript"), sidemenujsgz, sizeof(sidemenujsgz));
            });

  server.on(F("/fw.html"), HTTP_GET,
            [this, &server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.send_P(200, PSTR("text/html"), fwhtmlgz, sizeof(fwhtmlgz));
            });

  // Get Latest Update Info ---------------------------------------------------------
  server.on(
      F("/glui"), HTTP_GET,
      [this, &server]()
      {
        SERVER_KEEPALIVE_FALSE()
        server.send(200, F("application/json"), getLatestUpdateInfoJson());
      });

  // Update Firmware from Github ----------------------------------------------
  server.on(
      F("/update"), HTTP_POST,
      [this, &shouldReboot, &server]()
      {
        String msg;

        server.chunkedResponseModeStart(200, PSTR("text/plain"));

        // Define the progress callback function
        std::function<void(size_t, size_t)> progressCallback = [&server](size_t progress, size_t total)
        {
          uint8_t percent = (progress * 100) / total;
          LOG_SERIAL_PRINTF_P(PSTR("Progress: %d%%\n"), percent);
          server.sendContent((String(F("p:")) + percent + '\n').c_str());
        };

        // Call the updateFirmware function with the progress callback
        shouldReboot = updateFirmware(server.arg(F("plain")).c_str(), msg, progressCallback);
        if (shouldReboot)
          server.sendContent(F("s:true\n"));
        else
          server.sendContent(String(F("s:false\nm:")) + msg + '\n');

        server.chunkedResponseFinalize();
      });

  // Firmware POST URL allows to push new firmware ----------------------------
  server.on(
      F("/fw"), HTTP_POST,
      [&shouldReboot, &pauseApplication, &server]()
      {
        shouldReboot = !Update.hasError();

        String msg;

        if (shouldReboot)
          msg = F("Update successful");
        else
        {
          msg = F("Update failed: ");
#ifdef ESP8266
          msg = Update.getErrorString();
#else
          msg = Update.errorString();
#endif
          Update.clearError();
          // Update failed so restart to Run Application in loop
          pauseApplication = false;
        }

        LOG_SERIAL_PRINTLN(msg);

        SERVER_KEEPALIVE_FALSE()
        server.send(shouldReboot ? 200 : 500, F("text/html"), msg);
      },
      [&pauseApplication, &server]()
      {
        HTTPUpload &upload = server.upload();

        if (upload.status == UPLOAD_FILE_START)
        {
          // stop to Run Application in loop
          pauseApplication = true;

          LOG_SERIAL_PRINTF_P(PSTR("Update Start: %s\n"), upload.filename.c_str());

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
          Update.end(true);
        }

#ifdef ESP8266
        // Feed the dog otherwise big firmware won't pass
        ESP.wdtFeed();
#endif
        yield();
      });

  // reboot POST --------------------------------------------------------------
  server.on(F("/rbt"), HTTP_POST,
            [&shouldReboot, &server]()
            {
              if (server.hasArg(F("rescue")))
              {
                // Set EEPROM for Rescue mode flag
                EEPROM.begin(4);
                EEPROM.write(0, 1);
                EEPROM.end();
              }
              SERVER_KEEPALIVE_FALSE()
              server.send_P(200, PSTR("text/html"), PSTR("Reboot command received"));
              shouldReboot = true;
            });

  // 404 on not found ---------------------------------------------------------
  server.onNotFound(
      [&server]()
      {
        SERVER_KEEPALIVE_FALSE()
        server.send(404);
      });
}

void Core::appRun()
{
}

Core::Core(char appId, String appName) : Application(appId, appName)
{
  _applicationList[CoreApp] = this;
}

bool Core::getLastestUpdateInfo(char (*version)[10], char (*title)[64] /* = nullptr */, char (*releaseDate)[11] /* = nullptr */, char (*summary)[256] /* = nullptr */)
{
  // version is mandatory
  if (!version)
    return false;

  String githubURL = F("https://api.github.com/repos/" APPLICATION1_MANUFACTURER "/" APPLICATION1_MODEL "/releases/latest");

  WiFiClientSecure clientSecure;
  HTTPClient http;

  clientSecure.setInsecure();
  http.begin(clientSecure, githubURL);
  int httpCode = http.GET();

  // check for http error
  if (httpCode != 200)
  {
    http.end();
    return false;
  }

  // httpCode is 200, we can continue
  WiFiClient *stream = http.getStreamPtr();
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, *stream);

  // check for json error
  if (error)
  {
    http.end();
    return false;
  }

  // json is valid, we can continue
  JsonVariant jv;

  if ((jv = doc[F("tag_name")]).is<const char *>())
    strlcpy(*version, jv.as<const char *>(), sizeof(*version));

  // check if we got a version
  if (!strlen(*version))
  {
    http.end();
    return false;
  }

  // we got a version, we can continue
  if (title && (jv = doc[F("name")]).is<const char *>())
  {
    // find the first space and copy the rest to title
    if (const char *space = strchr(jv, ' '))
      strlcpy(*title, space + 1, sizeof(*title));
  }

  if (releaseDate && (jv = doc[F("published_at")]).is<const char *>())
    strlcpy(*releaseDate, jv.as<const char *>(), 11); // copy the part part only (10 chars)

  if (summary && (jv = doc[F("body")]).is<const char *>())
  {
    // copy body to summary until "\r\n\r\n##"
    const char *body = jv.as<const char *>();
    const char *end = strstr(body, "\r\n\r\n##");
    if (end)
    {
      size_t len = std::min<size_t>(end - body + 1, sizeof(*summary)); // +1 to include the null terminator
      strlcpy(*summary, body, len);
    }
    else
      strlcpy(*summary, body, sizeof(*summary));
  }

  http.end();

  return true;
}

String Core::getLatestUpdateInfoJson()
{
  JsonDocument doc;

  doc[F("installed_version")] = VERSION;

  char version[10] = {0};
  char title[64] = {0};
  char releaseDate[11] = {0};
  char summary[256] = {0};

  if (getLastestUpdateInfo(&version, &title, &releaseDate, &summary))
  {
    doc[F("latest_version")] = version;
    doc[F("title")] = title;
    doc[F("release_date")] = releaseDate;
    doc[F("release_summary")] = summary;
    doc[F("release_url")] = String(F("https://github.com/" APPLICATION1_MANUFACTURER "/" APPLICATION1_MODEL "/releases/tag/")) + version;
  }

  String info;
  serializeJson(doc, info);

  return info;
}

bool Core::updateFirmware(const char *version, String &retMsg, std::function<void(size_t, size_t)> progressCallback /* = nullptr */)
{
  char versionToFlash[8] = {0};

  if (version && !strcmp(version, "latest"))
  {
    char _version[10] = {0};

    if (getLastestUpdateInfo(&_version) and versionCompare(_version, VERSION) > 0)
      strlcpy(versionToFlash, _version, sizeof(versionToFlash));
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
  fwUrl = fwUrl + versionToFlash + '/' + F(APPLICATION1_MODEL) + '.' + versionToFlash + F(".bin");

  LOG_SERIAL_PRINTF_P(PSTR("Trying to Update from URL: %s\n"), fwUrl.c_str());

  HTTPClient https;
  https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  https.begin(clientSecure, fwUrl);
  int httpCode = https.GET();

  if (httpCode != 200)
  {
    https.end();

    retMsg = F("Failed to download file, httpCode: ");
    retMsg += httpCode;

    LOG_SERIAL_PRINTLN(retMsg);

    return false;
  }

  // starting here we have a valid httpCode (200)

  // get the stream
  WiFiClient *stream = https.getStreamPtr();
  int contentLength = https.getSize();

  LOG_SERIAL_PRINTF_P(PSTR("Update Start: %s (Online Update)\n"), (String(F(APPLICATION1_MODEL)) + '.' + versionToFlash + F(".bin")).c_str());

  if (progressCallback)
    Update.onProgress(progressCallback);

#ifdef ESP8266
  Update.begin(contentLength);
#else
  Update.begin();
#endif

  Update.writeStream(*stream);

  Update.end();

  https.end();

  bool success = !Update.hasError();
  if (success)
    LOG_SERIAL_PRINTLN(F("Update successful"));
  else
  {
#ifdef ESP8266
    retMsg = Update.getErrorString();
#else
    retMsg = Update.errorString();
#endif
    LOG_SERIAL_PRINTF_P(PSTR("Update failed: %s\n"), retMsg.c_str());
    Update.clearError();
  }

  return success;
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