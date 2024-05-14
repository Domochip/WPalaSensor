#ifndef Core_h
#define Core_h

#include "../Main.h"
#include "Application.h"
#ifdef ESP32
#include <Update.h>
#endif

#include "data/status0.html.gz.h"
#include "data/config0.html.gz.h"
#include "data/fw0.html.gz.h"
#include "data/discover0.html.gz.h"

class Core : public Application
{
private:
  void setConfigDefaultValues();
  void parseConfigJSON(JsonDocument &doc);
  bool parseConfigWebRequest(ESP8266WebServer &server);
  String generateConfigJSON(bool clearPassword);
  String generateStatusJSON();
  bool appInit(bool reInit);
  const PROGMEM char *getHTMLContent(WebPageForPlaceHolder wp);
  size_t getHTMLContentSize(WebPageForPlaceHolder wp);
  void appInitWebServer(ESP8266WebServer &server, bool &shouldReboot, bool &pauseApplication);
  void appRun(){};

public:
  Core(char appId, String fileName) : Application(appId, fileName) {}
};

#endif