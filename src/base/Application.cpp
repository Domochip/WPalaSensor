#include "Application.h"

Application *Application::_applicationList[3] = {nullptr, nullptr, nullptr};

Application::Application(AppId appId) : _appId(appId)
{
  _applicationList[_appId] = this;
}

char Application::getAppIdChar(AppId appId)
{
  return '0' + appId;
}

String Application::getAppIdName(AppId appId)
{
  if (appId == CoreApp)
    return F("Core");

  if (appId == WifiManApp)
    return F("WiFi");

  return F(CUSTOM_APP_MODEL);
}

bool Application::saveConfig()
{
  File configFile = LittleFS.open(String('/') + getAppIdName(_appId) + F(".json"), "w");
  if (!configFile)
  {
    LOG_SERIAL_PRINTLN(F("Failed to open config file for writing"));
    return false;
  }

  configFile.print(generateConfigJSON(true));
  configFile.close();
  return true;
}

bool Application::loadConfig()
{
  // special exception for Core, there is no Core.json file to Load
  if (_appId == CoreApp)
    return true;

  bool result = false;
  File configFile = LittleFS.open(String('/') + getAppIdName(_appId) + F(".json"), "r");
  if (configFile)
  {

    JsonDocument jsonDoc;

    DeserializationError deserializeJsonError = deserializeJson(jsonDoc, configFile);

    // if deserialization failed, then log error and save current config (default values)
    if (deserializeJsonError)
    {

      LOG_SERIAL_PRINTF_P(PSTR("deserializeJson() failed : %s\n"), deserializeJsonError.c_str());

      saveConfig();
    }
    else
    { // otherwise pass it to application
      result = parseConfigJSON(jsonDoc);
    }
    configFile.close();
  }

  return result;
}

bool Application::getLastestUpdateInfo(String &version, String &title, String &releaseDate, String &summary)
{
  String githubURL = F("https://api.github.com/repos/" CUSTOM_APP_MANUFACTURER "/" CUSTOM_APP_MODEL "/releases/latest");

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

  // We need to parse the JSON response without loading the whole response in memory

  uint8_t maxKeyLength = 16; // longest key is "\"published_at\":"" =>15 chars
  String keyBuffer;          // Shifting buffer used to find keys
  uint8_t treeLevel = 0;     // used to skip unwanted data
  bool keyFound = false;     // used to know if we found a key we are looking for
  String *targetString = nullptr;
  size_t targetStringSize = 0;

  // sometime the stream is not yet ready (no data available yet)
  for (byte i = 0; i < 200 && stream->available() == 0; i++) // available include an optimistic_yield of 100us
    ;

  // while there is data to read
  while (http.connected() && stream->available())
  {
    // read the next character
    char c = stream->read();

    // if c is a brace or bracket, increment or decrement the treeLevel
    if (c == '{' || c == '[')
      treeLevel++;
    else if (c == '}' || c == ']')
      treeLevel--;

    // if we are not at the first treeLevel, skip the character
    // (there is some "name" key in assets that we don't want to parse)
    if (treeLevel > 1)
      continue;

    // if keyBuffer is full, shift it to the left by one character
    if (keyBuffer.length() == maxKeyLength)
      keyBuffer.remove(0, 1);

    // add the new character at the end
    keyBuffer.concat(c);

    keyFound = false;

    // if we found the key "tag_name"
    if (c == ':' && keyBuffer.endsWith(F("\"tag_name\":")))
    {
      keyFound = true;
      targetString = &version;
      targetStringSize = 9;
    }

    // if we found the key "name"
    if (c == ':' && keyBuffer.endsWith(F("\"name\":")))
    {
      keyFound = true;
      targetString = &title;
      targetStringSize = 63;
    }

    // if we found the key "published_at"
    if (c == ':' && keyBuffer.endsWith(F("\"published_at\":")))
    {
      keyFound = true;
      targetString = &releaseDate;
      targetStringSize = 10;
    }

    // if we found the key "body"
    if (c == ':' && keyBuffer.endsWith(F("\"body\":")))
    {
      keyFound = true;
      targetString = &summary;
      targetStringSize = 255;
    }

    if (keyFound)
    {
      // read until the next quote (should be next to semicolon)
      stream->readStringUntil('"');

      // for name/title key, skip text until the first space
      if (targetString == &title)
        stream->readStringUntil(' ');

      // read the value
      while (stream->available())
      {
        c = stream->read();

        if (c == '"' && !targetString->endsWith(F("\\")))
          break;

        if (targetString->endsWith(F("\\")) && c == 'n')
          (*targetString)[targetString->length() - 1] = '\n';
        else if (targetString->endsWith(F("\\")) && c == 'r')
          (*targetString)[targetString->length() - 1] = '\r';
        else if (targetString->endsWith(F("\\")) && c == '"')
          (*targetString)[targetString->length() - 1] = '"';
        else if (targetString->length() < targetStringSize)
          targetString->concat(c);

        // for summary, stop at "\r\n\r\n##"
        if (targetString == &summary && targetString->endsWith(F("\r\n\r\n##")))
        {
          // remove the last 6 characters
          targetString->remove(targetString->length() - 6);
          // avoid adding more text in summary
          targetStringSize = targetString->length();
        }
      }
    }
  }

  http.end();

  return version.length() > 0;
}

String Application::getLatestUpdateInfoJson(bool forWebPage /* = false */)
{
  JsonDocument doc;

  doc[F("installed_version")] = VERSION;

  String version, title, releaseDate, summary;

  if (getLastestUpdateInfo(version, title, releaseDate, summary))
  {
    doc[F("latest_version")] = version;
    doc[F("title")] = title;
    doc[F("release_summary")] = summary;
    doc[F("release_url")] = String(F("https://github.com/" CUSTOM_APP_MANUFACTURER "/" CUSTOM_APP_MODEL "/releases/tag/")) + version;

    if (forWebPage)
      doc[F("release_date")] = releaseDate;
  }

  String info;
  serializeJson(doc, info);

  return info;
}

bool Application::updateFirmware(const char *version, String &retMsg, std::function<void(size_t, size_t)> progressCallback /* = nullptr */)
{
  if (!version || !version[0])
  {
    retMsg = F("No version provided");
    return false;
  }

  WiFiClientSecure clientSecure;
  clientSecure.setInsecure();

  String fwUrl(F("https://github.com/" CUSTOM_APP_MANUFACTURER "/" CUSTOM_APP_MODEL "/releases/download/"));
#ifdef ESP8266
  fwUrl = fwUrl + version + '/' + F(CUSTOM_APP_MODEL) + '.' + version + F(".bin");
#else
  fwUrl = fwUrl + version + '/' + F(CUSTOM_APP_MODEL) + F(".esp32") + '.' + version + F(".bin");
#endif

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

  LOG_SERIAL_PRINTF_P(PSTR("Update Start: %s (Online Update)\n"), (String(F(CUSTOM_APP_MODEL)) + '.' + version + F(".bin")).c_str());

  if (progressCallback)
    Update.onProgress(progressCallback);

  Update.begin(contentLength);

  // sometime the stream is not yet ready (no data available yet)
  // and writeStream start by a peek which then fail
  for (byte i = 0; i < 200 && stream->available() == 0; i++) // available include an optimistic_yield of 100us
    ;
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

String Application::getStatusJSON()
{
  return generateStatusJSON();
}

void Application::init(bool skipExistingConfig)
{
  bool result = true;

  LOG_SERIAL_PRINTF_P(PSTR("Start %s : "), getAppIdName(_appId).c_str());

  setConfigDefaultValues();

  if (!skipExistingConfig)
    result = loadConfig();

  // Execute specific Application Init Code
  result = appInit() && result;

  LOG_SERIAL_PRINTLN(result ? F("OK") : F("FAILED"));
}

void Application::initWebServer(WebServer &server)
{
  char url[16];

  // HTML Status handler
  sprintf_P(url, PSTR("/status%c.html"), getAppIdChar(_appId));
  server.on(url, HTTP_GET,
            [this, &server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.send_P(200, PSTR("text/html"), getHTMLContent(status), getHTMLContentSize(status));
            });

  // HTML Config handler
  sprintf_P(url, PSTR("/config%c.html"), getAppIdChar(_appId));
  server.on(url, HTTP_GET,
            [this, &server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Content-Encoding"), F("gzip"));
              server.send_P(200, PSTR("text/html"), getHTMLContent(config), getHTMLContentSize(config));
            });

  // JSON Status handler
  sprintf_P(url, PSTR("/gs%c"), getAppIdChar(_appId));
  server.on(url, HTTP_GET,
            [this, &server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Cache-Control"), F("no-cache"));
              server.send(200, F("text/json"), generateStatusJSON());
            });

  // JSON Config handler
  sprintf_P(url, PSTR("/gc%c"), getAppIdChar(_appId));
  server.on(url, HTTP_GET,
            [this, &server]()
            {
              SERVER_KEEPALIVE_FALSE()
              server.sendHeader(F("Cache-Control"), F("no-cache"));
              server.send(200, F("text/json"), generateConfigJSON());
            });

  sprintf_P(url, PSTR("/sc%c"), getAppIdChar(_appId));
  server.on(url, HTTP_POST,
            [this, &server]()
            {
              // All responses have keep-alive set to false
              SERVER_KEEPALIVE_FALSE()

              // config json are received in POST body (arg("plain"))

              // Deserialize it
              JsonDocument doc;
              DeserializationError error = deserializeJson(doc, server.arg(F("plain")));
              if (error)
              {
                server.send(400, F("text/html"), F("Malformed JSON"));
                return;
              }

              // Parse it using the application method
              if (!parseConfigJSON(doc, true))
              {
                server.send(400, F("text/html"), F("Invalid Configuration"));
                return;
              }

              // Save it
              if (!saveConfig())
              {
                server.send(500, F("text/html"), F("Configuration hasn't been saved"));
                return;
              }

              // Everything went fine, Send client answer
              server.send(200);
              _reInit = true;
            });

  // Execute Specific Application Web Server initialization
  appInitWebServer(server);
}

void Application::run()
{
  if (_reInit)
  {
    LOG_SERIAL_PRINTF_P(PSTR("ReStart %s : "), getAppIdName(_appId).c_str());

    if (appInit(true))
      LOG_SERIAL_PRINTLN(F("OK"));
    else
      LOG_SERIAL_PRINTLN(F("FAILED"));

    _reInit = false;
  }

  appRun();
}