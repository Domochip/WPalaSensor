#ifndef Application_h
#define Application_h

#include "../Main.h"
#include <LittleFS.h>
#ifdef ESP8266
#include <ESP8266WebServer.h>
using WebServer = ESP8266WebServer;
#define SERVER_KEEPALIVE_FALSE() server.keepAlive(false);
#include <ESP8266HTTPClient.h>
#else
#include <WebServer.h>
#define SERVER_KEEPALIVE_FALSE()
#include <HTTPClient.h>
#include <Update.h>
#endif
#include <ArduinoJson.h>
#include <Ticker.h>

class Application
{
protected:
  typedef enum
  {
    CoreApp,
    WifiManApp,
    CustomApp
  } Applications;

  static Application *_applicationList[3]; // static list of all applications

  typedef enum
  {
    status,
    config
  } WebPageForPlaceHolder;

  char _appId;
  String _appName;
  bool _reInit = false;

  // already built methods
  bool saveConfig();
  bool loadConfig();

  static bool getLastestUpdateInfo(char (*version)[10], char (*title)[64] = nullptr, char (*releaseDate)[11] = nullptr, char (*summary)[256] = nullptr);
  static String getLatestUpdateInfoJson();
  static bool updateFirmware(const char *version, String &retMsg, std::function<void(size_t, size_t)> progressCallback = nullptr);

  // specialization required from the application
  virtual void setConfigDefaultValues() = 0;
  virtual bool parseConfigJSON(JsonDocument &doc, bool fromWebPage = false) = 0;
  virtual String generateConfigJSON(bool forSaveFile = false) = 0;
  virtual String generateStatusJSON() = 0;
  virtual bool appInit(bool reInit = false) = 0;
  virtual const PROGMEM char *getHTMLContent(WebPageForPlaceHolder wp) = 0;
  virtual size_t getHTMLContentSize(WebPageForPlaceHolder wp) = 0;
  virtual void appInitWebServer(WebServer &server, bool &shouldReboot, bool &pauseApplication) = 0;
  virtual void appRun() = 0;

public:
  // already built methods
  Application(char appId, String appName) : _appId(appId), _appName(appName) {}

  String getStatusJSON();
  void init(bool skipExistingConfig);
  void initWebServer(WebServer &server, bool &shouldReboot, bool &pauseApplication);
  void run();
};

#endif