#ifndef WPalaSensor_h
#define WPalaSensor_h

#include "Main.h"
#include "base/MQTTMan.h"
#include "base/Application.h"

const char appDataPredefPassword[] PROGMEM = "ewcXoCt4HHjZUvY1";

#include "data/status2.html.gz.h"
#include "data/config2.html.gz.h"

#ifdef ESP8266
#include <ESP8266HTTPClient.h>
#else
#include <HTTPClient.h>
#endif
#include <SPI.h>
#include <math.h>
#include <Ticker.h>
#include "SingleDS18B20.h"
#include <MCP4725.h>

class WPalaSensor : public Application
{
private:
  // -------------------- HomeAutomation Classes --------------------

#define HA_HTTP_JEEDOM 0
#define HA_HTTP_FIBARO 1
#define HA_HTTP_HOMEASSISTANT 2

  typedef struct
  {
    byte type = HA_HTTP_HOMEASSISTANT;
    char hostname[64 + 1] = {0};
    bool tls = false;
    int temperatureId = 0;
    char secret[183 + 1] = {0}; // store Home Assistant long lived access token or Jeedom API key or Fibaro password

    struct
    {
      char username[64 + 1] = {0};
    } fibaro;

    struct
    {
      char entityId[64 + 1] = {0};
    } homeassistant;

    uint32_t cboxIp = 0;
  } HTTP;

  typedef struct
  {
    char hostname[64 + 1] = {0};
    uint32_t port = 0;
    char username[32 + 1] = {0};
    char password[64 + 1] = {0};
    char baseTopic[64 + 1] = {0};
    bool hassDiscoveryEnabled = true;
    char hassDiscoveryPrefix[32 + 1] = {0};

    char temperatureTopic[64 + 1] = {0};

    char cboxT1Topic[32 + 1] = {0};
  } MQTT;

#define HA_PROTO_DISABLED 0
#define HA_PROTO_HTTP 1
#define HA_PROTO_MQTT 2

#define CBOX_PROTO_DISABLED 0
#define CBOX_PROTO_HTTP 1
#define CBOX_PROTO_MQTT 2

  typedef struct
  {
    byte maxFailedRequest = 0; // number of failed requests to HA before failover to local temperature sensor

    byte protocol = HA_PROTO_DISABLED;
    uint16_t temperatureTimeout = 0;
    byte cboxProtocol = CBOX_PROTO_DISABLED;
    uint16_t cboxTemperatureTimeout = 0;

    HTTP http;
    MQTT mqtt;
  } HomeAutomation;

  // --------------------

  uint8_t _refreshPeriod;
  double _steinhartHartCoeffs[3] = {0, 0, 0};
  HomeAutomation _ha;

  // Run variables

  SingleDS18B20 _ds18b20;
  MCP4725 _dac;

  bool _needRefresh = false;
  Ticker _refreshTicker;
  byte _skipTick = 0;
  bool _needPublishHassDiscovery = false;
  bool _needPublishUpdate = false;
  Ticker _publishUpdateTicker;

  // Used in refresh method for logic and calculation

  // to avoid delta calculation on first call (then 20° is applied at first call and then delta is calculated at least during 2nd call)
  bool _firstRefreshTick = true;

  float _haTemperature = 20.0;
  unsigned long _haTemperatureMillis = 0;
  int _haRequestResult = 0;
  float _owTemperature = 0.0;
  bool _haTemperatureUsed = false;
  float _lastTemperatureUsed = 20.0;

  float _stoveTemperature = 20.0;
  unsigned long _stoveTemperatureMillis = 0;
  int _stoveRequestResult = 0;

  float _stoveDelta = 0.0;
  float _pushedTemperature = 20.0;

  WiFiClient _wifiClient; // used by _mqttMan
  MQTTMan _mqttMan;

  void setDac(float temperature, bool EEPROM = false);
  void setDac(int resistance, bool EEPROM = false);
  void refresh();
  void mqttConnectedCallback(MQTTMan *mqttMan, bool firstConnection);
  void mqttCallback(char *topic, uint8_t *payload, unsigned int length);
  bool mqttPublishHassDiscovery();
  bool mqttPublishUpdate();

  void setConfigDefaultValues();
  bool parseConfigJSON(JsonDocument &doc, bool fromWebPage);
  String generateConfigJSON(bool forSaveFile);
  String generateStatusJSON();
  bool appInit(bool reInit);
  const PROGMEM char *getHTMLContent(WebPageForPlaceHolder wp);
  size_t getHTMLContentSize(WebPageForPlaceHolder wp);
  void appInitWebServer(WebServer &server);
  void appRun();

public:
  WPalaSensor();
};

#endif
