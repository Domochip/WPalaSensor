#ifndef Core_h
#define Core_h

#include "../Main.h"
#include "Application.h"
#ifdef ESP8266
#include <ESP8266HTTPClient.h>
#else
#include <HTTPClient.h>
#include <Update.h>
#endif

#include "data/status0.html.gz.h"
#include "data/config0.html.gz.h"
#include "data/fw.html.gz.h"

class Core : public Application
{
private:
  typedef struct
  {
    char version[8] = {0};
    char title[64] = {0};
    char releaseDate[11] = {0};
    char summary[255] = {0};
  } LastFirmwareInfos;

  LastFirmwareInfos _lastFirmwareInfos;
  Ticker _checkForUpdateTicker;
  bool _needCheckForUpdateTick = false;

  void setConfigDefaultValues();
  bool parseConfigJSON(JsonDocument &doc, bool fromWebPage);
  String generateConfigJSON(bool forSaveFile);
  String generateStatusJSON();
  bool appInit(bool reInit);
  const PROGMEM char *getHTMLContent(WebPageForPlaceHolder wp);
  size_t getHTMLContentSize(WebPageForPlaceHolder wp);
  void appInitWebServer(WebServer &server, bool &shouldReboot, bool &pauseApplication);
  void appRun();

public:
  Core(char appId, String appName);

  void checkForUpdate();
  String getUpdateInfos(bool refresh = false);
  bool updateFirmware(const char *version, String &retMsg, std::function<void(size_t, size_t)> progressCallback = nullptr);
  static int8_t versionCompare(const char *version1, const char *version2);
};

#endif