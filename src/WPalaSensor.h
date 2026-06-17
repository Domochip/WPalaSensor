#ifndef WPalaSensor_h
#define WPalaSensor_h

#include "Main.h"
#include "base/WifiMan.h"
#include "base/MQTTMan.h"
#include "base/Application.h"

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

  enum class HaHttpType : uint8_t
  {
    Jeedom = 0,
    Fibaro = 1,
    HomeAssistant = 2
  };

  enum class HaProtocol : uint8_t
  {
    Disabled = 0,
    Http = 1,
    Mqtt = 2
  };

  enum class CBoxProtocol : uint8_t
  {
    Disabled = 0,
    Http = 1,
    Mqtt = 2
  };

  typedef struct
  {
    HaHttpType type = HaHttpType::HomeAssistant;
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

  typedef struct
  {
    HaProtocol protocol = HaProtocol::Disabled;
    uint16_t temperatureTimeout = 0;
    CBoxProtocol cboxProtocol = CBoxProtocol::Disabled;
    uint16_t cboxTemperatureTimeout = 0;

    HTTP http;
    MQTT mqtt;
  } HomeAutomation;

  // --------------------

  uint8_t _refreshPeriod;
  // Steinhart-Hart coefficient A
  double _shA = 0;
  // Steinhart-Hart coefficient B
  double _shB = 0;
  // Steinhart-Hart coefficient C
  double _shC = 0;
  HomeAutomation _ha;

  // Run variables

  SingleDS18B20 _ds18b20;
  MCP4725 _dac;

  bool _needRefresh = false;
  Ticker _refreshTicker;
  uint8_t _skipTick = 0;
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
  void fetchHaTemperatureHttp();
  void fetchCBoxTemperatureHttp();
  void refresh();
  void mqttConnectedCallback(MQTTMan *mqttMan, bool firstConnection);
  void mqttCallback(char *topic, uint8_t *payload, unsigned int length);
  bool mqttPublishHassDiscovery();
  void mqttPublishHassDiscovery(HassDiscoveryCtx &ctx);

  void setConfigDefaultValues();
  bool parseConfigJSON(JsonVariant json, bool fromWebPage = false);
  void validateConfig() override;
  void fillConfigJSON(JsonVariant json, bool forSaveFile = false);
  void fillStatusJSON(JsonVariant json);
  bool appInit(bool reInit = false);
  void appInitWebServer(WebServer &server);
  void appRun();

public:
  WPalaSensor();
};

#endif
