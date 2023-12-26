#include "Application.h"

#if ENABLE_STATUS_EVTSRC

void Application::statusEventSourceHandler(ESP8266WebServer &server)
{
  uint8_t subPos = 0;

  // Find the subscription for this client
  while (subPos < STATUS_EVTSRC_MAX_CLIENTS &&
         (!_statusEvtSrcClient[subPos] ||
          _statusEvtSrcClient[subPos].remoteIP() != server.client().remoteIP() ||
          _statusEvtSrcClient[subPos].remotePort() != server.client().remotePort()))
    subPos++;

  // If no subscription found
  if (subPos == STATUS_EVTSRC_MAX_CLIENTS)
  {
    subPos = 0;
    // Find a free slot
    while (subPos < STATUS_EVTSRC_MAX_CLIENTS && _statusEvtSrcClient[subPos])
      subPos++;

    // If there is no more slot available
    if (subPos == STATUS_EVTSRC_MAX_CLIENTS)
      return server.send(500);
  }

  server.client().setSync(true); // disable Nagle's algorithm and flush immediately

  // create/update subscription
  _statusEvtSrcClient[subPos] = server.client();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); // the payload can go on forever
  server.sendContent_P(PSTR("HTTP/1.1 200 OK\nContent-Type: text/event-stream;\nConnection: keep-alive\nCache-Control: no-cache\nAccess-Control-Allow-Origin: *\n\n"));

#if DEVELOPPER_MODE && defined(LOG_SERIAL)
  LOG_SERIAL.printf_P(PSTR("statusEventSourceHandler - client #%d (%s:%d) registered\n"), subPos, server.client().remoteIP().toString().c_str(), server.client().remotePort());
#endif
}

void Application::statusEventSourceBroadcast(const String &message, const String &eventType) // default eventType is "message"
{
  for (uint8_t i = 0; i < STATUS_EVTSRC_MAX_CLIENTS; i++)
  {
    if (_statusEvtSrcClient[i])
    {
      _statusEvtSrcClient[i].printf_P(PSTR("event: %s\ndata: %s\n\n"), eventType.c_str(), message.c_str());

#if DEVELOPPER_MODE && defined(LOG_SERIAL)
      LOG_SERIAL.printf_P(PSTR("statusEventSourceBroadcast - event sent to client #%d\n"), i);
#endif
    }
  }
}

#if ENABLE_STATUS_EVTSRC_KEEPALIVE
void Application::statusEventSourceKeepAlive()
{
  for (uint8_t i = 0; i < STATUS_EVTSRC_MAX_CLIENTS; i++)
  {
    if (_statusEvtSrcClient[i])
    {
      _statusEvtSrcClient[i].println(F(":keepalive\n\n"));

#if DEVELOPPER_MODE && defined(LOG_SERIAL)
      LOG_SERIAL.printf_P(PSTR("statusEventSourceKeepAlive - keep-alive sent to client #%d (%s:%d)\n"), i, _statusEvtSrcClient[i].remoteIP().toString().c_str(), _statusEvtSrcClient[i].remotePort());
#endif
    }
  }
}
#endif // ENABLE_STATUS_EVTSRC_KEEPALIVE
#endif // ENABLE_STATUS_EVTSRC

bool Application::saveConfig()
{
  File configFile = LittleFS.open(String('/') + _appName + F(".json"), "w");
  if (!configFile)
  {
#ifdef LOG_SERIAL
    LOG_SERIAL.println(F("Failed to open config file for writing"));
#endif
    return false;
  }

  configFile.print(generateConfigJSON(true));
  configFile.close();
  return true;
}

bool Application::loadConfig()
{
  // special exception for Core, there is no Core.json file to Load
  if (_appId == '0')
    return true;

  bool result = false;
  File configFile = LittleFS.open(String('/') + _appName + F(".json"), "r");
  if (configFile)
  {

    int memoryAllocation = JSON_DOC_MEM_STEP;

    while (memoryAllocation <= JSON_DOC_MAX_MEM_SIZE)
    {
      DynamicJsonDocument jsonDoc(memoryAllocation);

      configFile.seek(0);

      auto deserializeJsonError = deserializeJson(jsonDoc, configFile);
      // if parsing OK, pass it to application then stop loop
      if (deserializeJsonError.code() == DeserializationError::Ok)
      {
        parseConfigJSON(jsonDoc);
        result = true;
        break;
      }
      // if parsing result is not a NoMemory, there is a problem in JSON
      if (deserializeJsonError.code() != DeserializationError::NoMemory)
      {
#ifdef LOG_SERIAL
        LOG_SERIAL.print(F("deserializeJson() failed : "));
        LOG_SERIAL.println(deserializeJsonError.c_str());
#endif
        break;
      }

      // there, we need to increase memorySize and loop
      memoryAllocation += JSON_DOC_MEM_STEP;
    }

    configFile.close();
  }

  // if loading failed, then run a Save to write a good one
  if (!result)
    saveConfig();

  return result;
}

void Application::init(bool skipExistingConfig)
{
  bool result = true;

#ifdef LOG_SERIAL
  LOG_SERIAL.print(F("Start "));
  LOG_SERIAL.print(_appName);
  LOG_SERIAL.print(F(" : "));
#endif

  setConfigDefaultValues();

  if (!skipExistingConfig)
    result = loadConfig();

  // Execute specific Application Init Code
  result = appInit() && result;

#ifdef LOG_SERIAL
  if (result)
    LOG_SERIAL.println(F("OK"));
  else
    LOG_SERIAL.println(F("FAILED"));
#endif
}

void Application::initWebServer(ESP8266WebServer &server, bool &shouldReboot, bool &pauseApplication)
{
  char url[16];

  // HTML Status handler
  sprintf_P(url, PSTR("/status%c.html"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            {
    server.keepAlive(false);
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, PSTR("text/html"), getHTMLContent(status), getHTMLContentSize(status)); });

  // HTML Config handler
  sprintf_P(url, PSTR("/config%c.html"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            {
    server.keepAlive(false);
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, PSTR("text/html"), getHTMLContent(config), getHTMLContentSize(config)); });

  // HTML fw handler
  sprintf_P(url, PSTR("/fw%c.html"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            {
    server.keepAlive(false);
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, PSTR("text/html"), getHTMLContent(fw), getHTMLContentSize(fw)); });

  // HTML discover handler
  sprintf_P(url, PSTR("/discover%c.html"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            {
    server.keepAlive(false);
    server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, PSTR("text/html"), getHTMLContent(discover), getHTMLContentSize(discover)); });

  // JSON Status handler
  sprintf_P(url, PSTR("/gs%c"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            {
    server.keepAlive(false);
    server.sendHeader(F("Cache-Control"), F("no-cache"));
    server.send(200, F("text/json"), generateStatusJSON()); });

  // JSON Config handler
  sprintf_P(url, PSTR("/gc%c"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            {
    server.keepAlive(false);
    server.sendHeader(F("Cache-Control"), F("no-cache"));
    server.send(200, F("text/json"), generateConfigJSON()); });

  sprintf_P(url, PSTR("/sc%c"), _appId);
  server.on(url, HTTP_POST, [this, &server]()
            {
    if (!parseConfigWebRequest(server))
      return;

    //then save
    bool result = saveConfig();

    //Send client answer
    if (result)
    {
      server.keepAlive(false);
      server.send(200);
      _reInit = true;
    }
    else{
      server.keepAlive(false);
      server.send(500, F("text/html"), F("Configuration hasn't been saved"));
    } });

#if ENABLE_STATUS_EVTSRC
  // register EventSource Uri
  sprintf_P(url, PSTR("/statusEvt%c"), _appId);
  server.on(url, HTTP_GET, [this, &server]()
            { statusEventSourceHandler(server); });
#if ENABLE_STATUS_EVTSRC_KEEPALIVE
  // send keep alive event every 60 seconds
  _statusEvtSrcKeepAliveTicker.attach_scheduled(60.0f, [this]()
                                                { _needStatusEvtSrcKeepAlive = true; });
#endif
#endif

  // Execute Specific Application Web Server initialization
  appInitWebServer(server, shouldReboot, pauseApplication);
}

void Application::run()
{
  if (_reInit)
  {
#ifdef LOG_SERIAL
    LOG_SERIAL.print(F("ReStart "));
    LOG_SERIAL.print(_appName);
    LOG_SERIAL.print(F(" : "));
#endif

    if (appInit(true))
    {
#ifdef LOG_SERIAL
      LOG_SERIAL.println(F("OK"));
    }
    else
    {
      LOG_SERIAL.println(F("FAILED"));
#endif
    }

    _reInit = false;
  }

#if ENABLE_STATUS_EVTSRC
#if ENABLE_STATUS_EVTSRC_KEEPALIVE
  if (_needStatusEvtSrcKeepAlive)
  {
    _needStatusEvtSrcKeepAlive = false;
    statusEventSourceKeepAlive();
  }
#endif
#endif

  appRun();
}