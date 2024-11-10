#include "WPalaSensor.h"

//-----------------------------------------------------------------------
// Steinhart–Hart reverse function
//-----------------------------------------------------------------------
void WPalaSensor::setDac(float temperature)
{
  // convert temperature from Celsius to Kevin degrees
  float temperatureK = temperature + 273.15;

  // calculate and return resistance value based on provided temperature
  double x = (1 / _digipotsNTC.steinhartHartCoeffs[2]) * (_digipotsNTC.steinhartHartCoeffs[0] - (1 / temperatureK));
  double y = sqrt(pow(_digipotsNTC.steinhartHartCoeffs[1] / (3 * _digipotsNTC.steinhartHartCoeffs[2]), 3) + pow(x / 2, 2));
  setDac((int)(exp(pow(y - (x / 2), 1.0F / 3) - pow(y + (x / 2), 1.0F / 3))));
}
//-----------------------------------------------------------------------
// Set DAC equivalent resistance
//-----------------------------------------------------------------------
void WPalaSensor::setDac(int resistance)
{
  // calculate DAC value
  uint32_t value = 8200 + 2200;
  value *= 4096;
  value /= 8200 + resistance;

  _dac.setValue(value);
}

//-----------------------------------------------------------------------
// Main Refresh method
//-----------------------------------------------------------------------
void WPalaSensor::refresh()
{
  // LOG
  LOG_SERIAL_PRINTLN(F("refresh"));

  // if MQTT protocol is enabled and connected then publish Core, Wifi and WPalaControl status
  if ((_ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT) && _mqttMan.connected())
  {
    String baseTopic = _ha.mqtt.baseTopic;
    MQTTMan::prepareTopic(baseTopic);
    // remove the last char of baseTopic which is a '/'
    baseTopic.remove(baseTopic.length() - 1);

    JsonDocument doc;
    doc[getAppIdName(CoreApp)] = serialized(_applicationList[CoreApp]->getStatusJSON());
    doc[getAppIdName(WifiManApp)] = serialized(_applicationList[WifiManApp]->getStatusJSON());
    doc[getAppIdName(CustomApp)] = serialized(getStatusJSON());

    String strJson;
    serializeJson(doc, strJson);

    _mqttMan.publish(baseTopic.c_str(), strJson.c_str(), true);
  }

  if (_skipTick)
  {
    _skipTick--;
    return;
  }

  float temperatureToDisplay = 20.0;

  // read temperature from the local sensor
  _owTemperature = _ds18b20.readTemp();
  if (_owTemperature == 12.3456F)
    _owTemperature = 20.0; // if reading of local sensor failed so push 20°C
  else
  {
    // round it to tenth
    _owTemperature *= 10;
    _owTemperature = round(_owTemperature);
    _owTemperature /= 10;
  }

  // if HomeAutomation protocol is HTTP and WiFi is connected
  if (_ha.protocol == HA_PROTO_HTTP && WiFi.isConnected())
  {

    WiFiClient client;
    WiFiClientSecure clientSecure;

    HTTPClient http;

    // set timeOut
    http.setTimeout(5000);

    // try to get HomeAutomation sensor value -----------------

    // build the complete URI
    String completeURI = String(F("http")) + (_ha.http.tls ? F("s") : F("")) + F("://") + _ha.http.hostname;
    switch (_ha.http.type)
    {
    case HA_HTTP_JEEDOM:
      completeURI = completeURI + F("/core/api/jeeApi.php?apikey=") + _ha.http.secret + F("&type=cmd&id=") + _ha.http.temperatureId;
      break;
    case HA_HTTP_FIBARO:
      completeURI = completeURI + F("/api/devices?id=") + _ha.http.temperatureId;
      break;
    case HA_HTTP_HOMEASSISTANT:
      completeURI = completeURI + F("/api/states/") + _ha.http.homeassistant.entityId;
      break;
    }

    // if not TLS then use client, else use clientSecure
    if (!_ha.http.tls)
      http.begin(client, completeURI);
    else
    {
      clientSecure.setInsecure();
      http.begin(clientSecure, completeURI);
    }

    // For Fibaro, Pass authentication if specified in configuration
    if (_ha.http.type == HA_HTTP_FIBARO && _ha.http.fibaro.username[0])
      http.setAuthorization(_ha.http.fibaro.username, _ha.http.secret);

    // For HomeAssistant, Pass long-lived access token and set content type
    if (_ha.http.type == HA_HTTP_HOMEASSISTANT)
    {
      http.addHeader(F("Authorization"), String(F("Bearer ")) + _ha.http.secret);
      http.addHeader(F("Content-Type"), F("application/json"));
    }

    // send request
    _haRequestResult = http.GET();

    if (_haRequestResult == 200)
    {
      WiFiClient *stream = http.getStreamPtr();

      char payload[60] = {0};
      int nb = 0;

      switch (_ha.http.type)
      {
      case HA_HTTP_JEEDOM:

        nb = stream->readBytes(payload, 5);
        payload[nb] = 0;
        break;

      case HA_HTTP_FIBARO:

        while (http.connected() && stream->find("\"value\""))
        {
          // go to first next double quote (or return false if a comma appears first)
          if (stream->findUntil("\"", ","))
          {
            // read value (read until next doublequote)
            nb = stream->readBytesUntil('"', payload, sizeof(payload) - 1);
            payload[nb] = 0;
          }
        }
        break;

      case HA_HTTP_HOMEASSISTANT:

        while (http.connected() && stream->find("\"state\""))
        {
          // go to first next double quote (or return false if a comma appears first)
          if (stream->findUntil("\"", ","))
          {
            // read value (read until next doublequote)
            nb = stream->readBytesUntil('"', payload, sizeof(payload) - 1);
            payload[nb] = 0;
          }
        }
        break;
      }

      // if we readed some bytes
      if (nb)
      {
        // convert to float
        float haTemperature = atof(payload);
        String strHATemperature(payload);
        strHATemperature.replace("0", "");

        // if we got a correct value
        if (haTemperature != 0.0F || strHATemperature == ".")
        {
          // round it to tenth
          haTemperature *= 10;
          haTemperature = round(haTemperature);
          haTemperature /= 10;

          // place it in global _haTemperature and store millis
          _haTemperature = haTemperature;
          _haTemperatureMillis = millis();
        }
      }
    }
    http.end();
  }

  // if ConnectionBox protocol is HTTP and WiFi is connected
  if (_ha.cboxProtocol == CBOX_PROTO_HTTP && WiFi.isConnected())
  {
    WiFiClient client;

    HTTPClient http;

    // set timeOut
    http.setTimeout(5000);

    // try to get current stove temperature info ----------------------
    http.begin(client, String(F("http://")) + IPAddress(_ha.http.cboxIp).toString() + F("/cgi-bin/sendmsg.lua?cmd=GET%20TMPS"));

    // send request
    _stoveRequestResult = http.GET();
    // if we get successfull HTTP answer
    if (_stoveRequestResult == 200)
    {
      WiFiClient *stream = http.getStreamPtr();

      // if we found T1 in answer
      if (stream->find("\"T1\""))
      {
        char payload[8];
        // read until the comma into payload variable
        int nb = stream->readBytesUntil(',', payload, sizeof(payload) - 1);
        payload[nb] = 0; // end payload char[]
        // if we readed some bytes
        if (nb)
        {
          // look for start position of T1 value
          byte posTRW = 0;
          while ((payload[posTRW] == ' ' || payload[posTRW] == ':' || payload[posTRW] == '\t') && posTRW < nb)
            posTRW++;

          // convert to float
          float stoveTemperature = atof(payload + posTRW);
          String strStoveTemperature(payload + posTRW);
          strStoveTemperature.replace("0", "");

          // if we got a correct value
          if (stoveTemperature != 0.0F || strStoveTemperature == ".")
          {
            // place it in global _stoveTemperature and store millis
            _stoveTemperature = stoveTemperature;
            _stoveTemperatureMillis = millis();
          }
        }
      }
    }
    http.end();
  }

  // if Home Automation protocol is defined and temperature is not too old
  if (_ha.protocol != HA_PROTO_DISABLED && (_haTemperatureMillis + _ha.temperatureTimeout * 1000) > millis())
  {
    temperatureToDisplay = _haTemperature;
    _haTemperatureUsed = true;
  }
  else
  {
    temperatureToDisplay = _owTemperature;
    _haTemperatureUsed = false;
  }

  // if Connection Box protocol is defined and stove temperature arrived after last refresh and not first refresh tick
  if (_ha.cboxProtocol != CBOX_PROTO_DISABLED && (_stoveTemperatureMillis + _refreshPeriod * 1000) > millis() && !_firstRefreshTick)
  {
    // adjust delta
    _stoveDelta += (_lastTemperatureUsed - _stoveTemperature) / 2.5F;
  }
  // else if stove temperature is too old (older for more than timeout) then reset delta
  else if ((_stoveTemperatureMillis + _ha.cboxTemperatureTimeout * 1000) <= millis())
  {
    _stoveDelta = 0;
  }

  // Set DigiPot position according to resistance calculated from temperature to display with delta
  setDualDigiPot(temperatureToDisplay + _stoveDelta);

  _lastTemperatureUsed = temperatureToDisplay;

  _pushedTemperature = temperatureToDisplay + _stoveDelta;

  // if first refresh tick then set flag to false
  if (_firstRefreshTick)
    _firstRefreshTick = false;

#if DEVELOPPER_MODE

  // publish to MQTT
  if (_mqttMan.connected())
  {
    // prepare base topic
    String baseTopic = _ha.mqtt.baseTopic;
    MQTTMan::prepareTopic(baseTopic);

    // publish oneWire temperature
    _mqttMan.publish((baseTopic + F("OWTemp")).c_str(), String(_owTemperature).c_str(), true);

    // publish Home Automation temperature
    _mqttMan.publish((baseTopic + F("HATemp")).c_str(), String(_haTemperature).c_str(), true);

    // publish temperature to display
    _mqttMan.publish((baseTopic + F("TempToDisplay")).c_str(), String(temperatureToDisplay).c_str(), true);

    // publish last used temperature
    _mqttMan.publish((baseTopic + F("LastTempUsed")).c_str(), String(_lastTemperatureUsed).c_str(), true);

    // publish Stove temperature
    _mqttMan.publish((baseTopic + F("StoveTemp")).c_str(), String(_stoveTemperature).c_str(), true);

    // publish Delta
    _mqttMan.publish((baseTopic + F("Delta")).c_str(), String(_stoveDelta).c_str(), true);

    // publish Pushed temperature
    _mqttMan.publish((baseTopic + F("PushedTemp")).c_str(), String(_pushedTemperature).c_str(), true);
  }
#endif
}

//------------------------------------------
// Connect then Subscribe to MQTT
void WPalaSensor::mqttConnectedCallback(MQTTMan *mqttMan, bool firstConnection)
{

  // Subscribe to needed topic
  // if Home Automation is configured for MQTT
  if (_ha.protocol == HA_PROTO_MQTT)
  {
    mqttMan->subscribe(_ha.mqtt.temperatureTopic);
  }

  // if Connection Box/PalaControl is configured for MQTT
  if (_ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    mqttMan->subscribe(_ha.mqtt.cboxT1Topic);
  }

  // subscribe to update/install topic
  String topic(_ha.mqtt.baseTopic);
  MQTTMan::prepareTopic(topic);
  topic += F("update/install");
  mqttMan->subscribe(topic.c_str());

  // raise flag to publish Home Assistant discovery data
  _needPublishHassDiscovery = true;
}

//------------------------------------------
// Callback used when an MQTT message arrived
void WPalaSensor::mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{
  // if Home Automation is configured for MQTT and topic match
  if (_ha.protocol == HA_PROTO_MQTT && !strcmp(topic, _ha.mqtt.temperatureTopic))
  {
    String strHATemperature;
    strHATemperature.reserve(length + 1);

    // convert payload to string
    for (unsigned int i = 0; i < length; i++)
      strHATemperature += (char)payload[i];

    // convert
    float haTemperature = strHATemperature.toFloat();

    // remove all 0 from str for conversion verification
    // (remove all 0 from the string, then only one dot should remain like "0.00")
    strHATemperature.replace("0", "");

    if (haTemperature != 0.0F || strHATemperature == ".")
    {
      // round it to tenth
      haTemperature *= 10;
      haTemperature = round(haTemperature);
      haTemperature /= 10;

      _haTemperature = haTemperature;
      _haTemperatureMillis = millis();
    }
  }

  // if Home Automation is configured for MQTT and topic match
  if (_ha.cboxProtocol == CBOX_PROTO_MQTT && !strcmp(topic, _ha.mqtt.cboxT1Topic))
  {
    String strStoveTemperature;
    strStoveTemperature.reserve(length + 1);

    // convert payload to string
    for (unsigned int i = 0; i < length; i++)
      strStoveTemperature += (char)payload[i];

    // if payload contains JSON, isolate T1 value
    if (strStoveTemperature[0] == '{')
    {
      strStoveTemperature = strStoveTemperature.substring(strStoveTemperature.indexOf(F("\"T1\":")) + 5);
      strStoveTemperature.trim();
      strStoveTemperature = strStoveTemperature.substring(0, strStoveTemperature.indexOf(','));
      strStoveTemperature.trim();
    }

    // convert
    float stoveTemperature = strStoveTemperature.toFloat();

    // remove all 0 from str for conversion verification
    // (remove all 0 from the string, then only one dot should remain like "0.00")
    strStoveTemperature.replace("0", "");

    if (stoveTemperature != 0.0F || strStoveTemperature == ".")
    {
      _stoveTemperature = stoveTemperature;
      _stoveTemperatureMillis = millis();
    }
  }

  // if topic ends with "/update/install"
  if (String(topic).endsWith(F("/update/install")))
  {
    String version;
    String retMsg;
    unsigned long lastProgressPublish = 0;

    // result topic is topic without the last 8 characters ("/install")
    String resTopic(topic);
    resTopic.remove(resTopic.length() - 8);

    if (length > 10)
      retMsg = F("Version is too long");
    else
    {
      // convert payload to String
      version.concat((char *)payload, length);

      // Define the progress callback function
      std::function<void(size_t, size_t)> progressCallback = [this, &resTopic, &lastProgressPublish](size_t progress, size_t total)
      {
        // if last progress publish is less than 500ms ago then return
        if (millis() - lastProgressPublish < 500)
          return;
        lastProgressPublish = millis();

        uint8_t percent = (progress * 100) / total;
        LOG_SERIAL_PRINTF_P(PSTR("Progress: %d%%\n"), percent);
        String payload = String(F("{\"update_percentage\":")) + percent + '}';
        _mqttMan.publish(resTopic.c_str(), payload.c_str(), true);
      };

      SystemState::shouldReboot = updateFirmware(version.c_str(), retMsg, progressCallback);
    }

    if (SystemState::shouldReboot)
      retMsg = F("{\"update_percentage\":null,\"in_progress\":true}");
    else
    {
      _mqttMan.publish(topic, (String(F("Update failed: ")) + retMsg).c_str(), true);
      retMsg = F("{\"in_progress\":false}");
    }

    // publish result
    _mqttMan.publish(resTopic.c_str(), retMsg.c_str(), true);
  }
}

//------------------------------------------
bool WPalaSensor::mqttPublishHassDiscovery()
{
  // if MQTT is not connected then return false
  if (!_mqttMan.connected())
    return false;

  LOG_SERIAL_PRINTLN(F("Publish Home Assistant Discovery data"));

  // variables
  JsonDocument jsonDoc;
  String device, availability, payload;
  String baseTopic;
  String uniqueIdPrefix;
  String uniqueId;
  String topic;

  // prepare base topic
  baseTopic = _ha.mqtt.baseTopic;
  MQTTMan::prepareTopic(baseTopic);

  // prepare unique id prefix
  uniqueIdPrefix = F(CUSTOM_APP_MODEL "_");
  uniqueIdPrefix += WiFi.macAddress();
  uniqueIdPrefix.replace(":", "");

  // ---------- Device ----------

  // prepare device JSON
  jsonDoc[F("configuration_url")] = F("http://" CUSTOM_APP_MODEL ".local");
  jsonDoc[F("identifiers")][0] = uniqueIdPrefix;
  jsonDoc[F("manufacturer")] = F(CUSTOM_APP_MANUFACTURER);
  jsonDoc[F("model")] = F(CUSTOM_APP_MODEL);
  jsonDoc[F("name")] = WiFi.getHostname();
  jsonDoc[F("sw_version")] = VERSION;
  serializeJson(jsonDoc, device); // serialize to device String
  jsonDoc.clear();                // clean jsonDoc

  // ----- Entities -----

  //
  // Connectivity entity
  //

  // prepare uniqueId, topic and payload for connectivity sensor
  uniqueId = uniqueIdPrefix;
  uniqueId += F("_Connectivity");

  topic = _ha.mqtt.hassDiscoveryPrefix;
  topic += F("/binary_sensor/");
  topic += uniqueId;
  topic += F("/config");

  // prepare payload for connectivity sensor
  jsonDoc[F("~")] = baseTopic.substring(0, baseTopic.length() - 1); // remove ending '/'
  jsonDoc[F("device_class")] = F("connectivity");
  jsonDoc[F("device")] = serialized(device);
  jsonDoc[F("entity_category")] = F("diagnostic");
  jsonDoc[F("object_id")] = F(CUSTOM_APP_MODEL "_connectivity");
  jsonDoc[F("state_topic")] = F("~/connected");
  jsonDoc[F("unique_id")] = uniqueId;
  jsonDoc[F("value_template")] = F("{{ iif(int(value) > 0, 'ON', 'OFF') }}");

  jsonDoc.shrinkToFit();
  serializeJson(jsonDoc, payload);

  // publish
  _mqttMan.publish(topic.c_str(), payload.c_str(), true);

  // clean
  jsonDoc.clear();
  payload = "";

  return true;
}

//------------------------------------------
// Publish update to MQTT
bool WPalaSensor::mqttPublishUpdate()
{
  if (!_mqttMan.connected())
    return false;

  // get update info from Core
  String updateInfo = getLatestUpdateInfoJson();

  String baseTopic;
  String topic;

  // prepare base topic
  baseTopic = _ha.mqtt.baseTopic;
  MQTTMan::prepareTopic(baseTopic);

  // This part of code is necessary because "payload_install" is not a template
  // and we need to get the version to install pushed from Home Assistant
  // so it is mandatory to update the Update entity each time we publish the update
  // parse JSON
  if (_ha.mqtt.hassDiscoveryEnabled)
  {
    JsonDocument doc;
    JsonVariant jv;
    DeserializationError error = deserializeJson(doc, updateInfo);
    // if there is no error and latest_version is available
    if (!error && (jv = doc[F("latest_version")]).is<const char *>())
    {
      // get version
      char version[10] = {0};
      strlcpy(version, jv.as<const char *>(), sizeof(version));

      doc.clear(); // clean doc

      // then publish updated Update autodiscovery

      // variables
      JsonDocument jsonDoc;
      String device, availability, payload;

      String uniqueIdPrefix;
      String uniqueId;

      // prepare unique id prefix
      uniqueIdPrefix = F(CUSTOM_APP_MODEL "_");
      uniqueIdPrefix += WiFi.macAddress();
      uniqueIdPrefix.replace(":", "");

      // ---------- Device ----------

      // prepare device JSON
      jsonDoc[F("configuration_url")] = F("http://" CUSTOM_APP_MODEL ".local");
      jsonDoc[F("identifiers")][0] = uniqueIdPrefix;
      jsonDoc[F("manufacturer")] = F(CUSTOM_APP_MANUFACTURER);
      jsonDoc[F("model")] = F(CUSTOM_APP_MODEL);
      jsonDoc[F("name")] = WiFi.getHostname();
      jsonDoc[F("sw_version")] = VERSION;
      serializeJson(jsonDoc, device); // serialize to device String
      jsonDoc.clear();                // clean jsonDoc

      // prepare availability JSON for entities
      jsonDoc[F("topic")] = F("~/connected");
      jsonDoc[F("value_template")] = F("{{ iif(int(value) > 0, 'online', 'offline') }}");
      serializeJson(jsonDoc, availability); // serialize to availability String
      jsonDoc.clear();                      // clean jsonDoc

      // ----- Entities -----

      //
      // Update entity
      //

      // prepare uniqueId, topic and payload for update sensor
      uniqueId = uniqueIdPrefix;
      uniqueId += F("_Update");

      topic = _ha.mqtt.hassDiscoveryPrefix;
      topic += F("/update/");
      topic += uniqueId;
      topic += F("/config");

      // prepare payload for update sensor
      jsonDoc[F("~")] = baseTopic.substring(0, baseTopic.length() - 1); // remove ending '/'
      jsonDoc[F("availability")] = serialized(availability);
      jsonDoc[F("command_topic")] = F("~/update/install");
      jsonDoc[F("device")] = serialized(device);
      jsonDoc[F("device_class")] = F("firmware");
      jsonDoc[F("entity_category")] = F("config");
      jsonDoc[F("object_id")] = F(CUSTOM_APP_MODEL);
      jsonDoc[F("payload_install")] = version;
      jsonDoc[F("state_topic")] = F("~/update");
      jsonDoc[F("unique_id")] = uniqueId;

      jsonDoc.shrinkToFit();
      serializeJson(jsonDoc, payload);

      // publish
      _mqttMan.publish(topic.c_str(), payload.c_str(), true);
    }
  }

  // calculate topic
  topic = baseTopic + F("update");

  // publish install in_progress (new in 2024.11)
  // I keep it here because I want to separate the two publish for retrocompatibility
  // if "in_progress" is in the same payload, Home Assistant 2024.10 and lower will ignore the payload
  // (to be moved to WBase around 2025-05)
  _mqttMan.publish(topic.c_str(), (String(F("{\"in_progress\":")) + (Update.isRunning() ? F("true") : F("false")) + '}').c_str(), true);

  // publish update info
  _mqttMan.publish(topic.c_str(), updateInfo.c_str(), true);

  return true;
}

//------------------------------------------
// Used to initialize configuration properties to default values
void WPalaSensor::setConfigDefaultValues()
{
  _refreshPeriod = 30;

  _digipotsNTC.rWTotal = 240.0;
  _digipotsNTC.steinhartHartCoeffs[0] = 0.001067860568;
  _digipotsNTC.steinhartHartCoeffs[1] = 0.0002269969431;
  _digipotsNTC.steinhartHartCoeffs[2] = 0.0000002641627999;
  _digipotsNTC.rBW5KStep = 19.0;
  _digipotsNTC.rBW50KStep = 190.0;
  _digipotsNTC.dp50kStepSize = 1;
  _digipotsNTC.dp5kOffset = 10;

  _ha.maxFailedRequest = 10;
  _ha.protocol = HA_PROTO_DISABLED;
  _ha.temperatureTimeout = 300;
  _ha.cboxProtocol = CBOX_PROTO_DISABLED;
  _ha.cboxTemperatureTimeout = 300;

  _ha.http.type = HA_HTTP_HOMEASSISTANT;
  _ha.http.hostname[0] = 0;
  _ha.http.tls = false;
  _ha.http.temperatureId = 0;
  _ha.http.secret[0] = 0;
  _ha.http.fibaro.username[0] = 0;
  _ha.http.homeassistant.entityId[0] = 0;
  _ha.http.cboxIp = 0;

  _ha.mqtt.hostname[0] = 0;
  _ha.mqtt.port = 1883;
  _ha.mqtt.username[0] = 0;
  _ha.mqtt.password[0] = 0;
  strcpy_P(_ha.mqtt.baseTopic, PSTR("$model$"));
  _ha.mqtt.hassDiscoveryEnabled = true;
  strcpy_P(_ha.mqtt.hassDiscoveryPrefix, PSTR("homeassistant"));
  _ha.mqtt.temperatureTopic[0] = 0;
  _ha.mqtt.cboxT1Topic[0] = 0;
}

//------------------------------------------
// Parse JSON object into configuration properties
bool WPalaSensor::parseConfigJSON(JsonDocument &doc, bool fromWebPage = false)
{
  JsonVariant jv;
  char tempPassword[183 + 1] = {0};

  if ((jv = doc[F("rp")]).is<JsonVariant>())
    _refreshPeriod = jv;

  if ((jv = doc[F("sha")]).is<JsonVariant>())
    _digipotsNTC.steinhartHartCoeffs[0] = jv;
  if ((jv = doc[F("shb")]).is<JsonVariant>())
    _digipotsNTC.steinhartHartCoeffs[1] = jv;
  if ((jv = doc[F("shc")]).is<JsonVariant>())
    _digipotsNTC.steinhartHartCoeffs[2] = jv;

  // Parse Home Automation config

  if ((jv = doc[F("haproto")]).is<JsonVariant>())
    _ha.protocol = jv;

  // if an home Automation protocol has been selected then get common param
  if (_ha.protocol != HA_PROTO_DISABLED)
  {
    if ((jv = doc[F("hamfr")]).is<JsonVariant>())
      _ha.maxFailedRequest = jv;

    if ((jv = doc[F("hatt")]).is<JsonVariant>())
      _ha.temperatureTimeout = jv;
  }

  // Parse ConnectionBox config
  if ((jv = doc[F("cbproto")]).is<JsonVariant>())
    _ha.cboxProtocol = jv;

  // if an ConnectionBox protocol has been selected then get common param
  if (_ha.cboxProtocol != CBOX_PROTO_DISABLED)
  {
    if ((jv = doc[F("cbtt")]).is<JsonVariant>())
      _ha.cboxTemperatureTimeout = jv;
  }

  // if home automation or CBox protocol is MQTT then get common mqtt params
  if (_ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {

    if ((jv = doc[F("hamhost")]).is<const char *>())
      strlcpy(_ha.mqtt.hostname, jv, sizeof(_ha.mqtt.hostname));
    if ((jv = doc[F("hamport")]).is<JsonVariant>())
      _ha.mqtt.port = jv;
    if ((jv = doc[F("hamu")]).is<const char *>())
      strlcpy(_ha.mqtt.username, jv, sizeof(_ha.mqtt.username));

    // put MQTT password into tempPassword
    if ((jv = doc[F("hamp")]).is<const char *>())
    {
      strlcpy(tempPassword, jv, sizeof(_ha.mqtt.password));

      // if not from web page or password is not the predefined one then copy it to _ha.mqtt.password
      if (!fromWebPage || strcmp_P(tempPassword, appDataPredefPassword))
        strcpy(_ha.mqtt.password, tempPassword);
    }
    if ((jv = doc[F("hambt")]).is<const char *>())
      strlcpy(_ha.mqtt.baseTopic, jv, sizeof(_ha.mqtt.baseTopic));

    _ha.mqtt.hassDiscoveryEnabled = doc[F("hamhassde")];
    if ((jv = doc[F("hamhassdp")]).is<const char *>())
      strlcpy(_ha.mqtt.hassDiscoveryPrefix, jv, sizeof(_ha.mqtt.hassDiscoveryPrefix));
  }

  // Now get Home Automation specific params
  switch (_ha.protocol)
  {
  case HA_PROTO_HTTP:

    if ((jv = doc[F("hahtype")]).is<JsonVariant>())
      _ha.http.type = jv;
    if ((jv = doc[F("hahhost")]).is<const char *>())
      strlcpy(_ha.http.hostname, jv, sizeof(_ha.http.hostname));
    _ha.http.tls = doc[F("hahtls")];
    if ((jv = doc[F("hahtempid")]).is<JsonVariant>())
      _ha.http.temperatureId = jv;

    switch (_ha.http.type)
    {
    case HA_HTTP_JEEDOM:

      // put apiKey into tempPassword
      if ((jv = doc[F("hahjak")]).is<const char *>())
      {
        strlcpy(tempPassword, jv, sizeof(_ha.http.secret));

        // if not from web page or received apiKey is not the predefined one then copy it to _ha.http.secret
        if (!fromWebPage || strcmp_P(tempPassword, appDataPredefPassword))
          strcpy(_ha.http.secret, tempPassword);
      }

      if (!_ha.http.hostname[0] || !_ha.http.secret[0])
        _ha.protocol = HA_PROTO_DISABLED;
      break;

    case HA_HTTP_FIBARO:

      if ((jv = doc[F("hahfuser")]).is<const char *>())
        strlcpy(_ha.http.fibaro.username, jv, sizeof(_ha.http.fibaro.username));

      // put Fibaropassword into tempPassword
      if ((jv = doc[F("hahfpass")]).is<const char *>())
      {
        strlcpy(tempPassword, jv, sizeof(_ha.http.secret));

        // if not from web page or password is not the predefined one then copy it to _ha.http.secret
        if (!fromWebPage || strcmp_P(tempPassword, appDataPredefPassword))
          strcpy(_ha.http.secret, tempPassword);
      }

      if (!_ha.http.hostname[0])
        _ha.protocol = HA_PROTO_DISABLED;
      break;

    case HA_HTTP_HOMEASSISTANT:

      // if hostname is not empty and doesn't contains ":" and tls not enabled then add ":8123" (if it fits)
      if (_ha.http.hostname[0] && !strchr(_ha.http.hostname, ':') && !_ha.http.tls && (strlen(_ha.http.hostname) + 5 < sizeof(_ha.http.hostname) - 1))
        strcat(_ha.http.hostname, ":8123");

      if ((jv = doc[F("hahhaei")]).is<const char *>())
        strlcpy(_ha.http.homeassistant.entityId, jv, sizeof(_ha.http.homeassistant.entityId));

      // put longLivedAccessToken into tempPassword
      if ((jv = doc[F("hahhallat")]).is<const char *>())
      {
        strlcpy(tempPassword, jv, sizeof(_ha.http.secret));

        // if not from web page or long-lived access token is not the predefined one then copy it to _ha.http.secret
        if (!fromWebPage || strcmp_P(tempPassword, appDataPredefPassword))
          strcpy(_ha.http.secret, tempPassword);
      }

      if (!_ha.http.hostname[0] || !_ha.http.homeassistant.entityId[0] || !_ha.http.secret[0])
        _ha.protocol = HA_PROTO_DISABLED;
      break;
    }

    break;
  case HA_PROTO_MQTT:

    if ((jv = doc[F("hamtemptopic")]).is<const char *>())
      strlcpy(_ha.mqtt.temperatureTopic, jv, sizeof(_ha.mqtt.temperatureTopic));

    if (!_ha.mqtt.hostname[0] || !_ha.mqtt.baseTopic[0] || !_ha.mqtt.temperatureTopic[0])
      _ha.protocol = HA_PROTO_DISABLED;
    break;
  }

  // Now get ConnectionBox specific params
  switch (_ha.cboxProtocol)
  {
  case CBOX_PROTO_HTTP:

    if ((jv = doc[F("cbhip")]).is<const char *>())
    {
      IPAddress ipParser;
      if (ipParser.fromString(jv.as<const char *>()))
        _ha.http.cboxIp = static_cast<uint32_t>(ipParser);
      else
        _ha.http.cboxIp = 0;
    }

    if (!_ha.http.cboxIp)
      _ha.cboxProtocol = CBOX_PROTO_DISABLED;
    break;

  case CBOX_PROTO_MQTT:

    if ((jv = doc[F("cbmt1topic")]).is<const char *>())
      strlcpy(_ha.mqtt.cboxT1Topic, jv, sizeof(_ha.mqtt.cboxT1Topic));

    if (!_ha.mqtt.hostname[0] || !_ha.mqtt.baseTopic[0] || !_ha.mqtt.cboxT1Topic[0])
      _ha.cboxProtocol = CBOX_PROTO_DISABLED;
    break;
  }

  return true;
}

//------------------------------------------
// Generate JSON from configuration properties
String WPalaSensor::generateConfigJSON(bool forSaveFile = false)
{
  JsonDocument doc;

  doc["rp"] = _refreshPeriod;

  doc[F("sha")] = serialized(String(_digipotsNTC.steinhartHartCoeffs[0], 16));
  doc[F("shb")] = serialized(String(_digipotsNTC.steinhartHartCoeffs[1], 16));
  doc[F("shc")] = serialized(String(_digipotsNTC.steinhartHartCoeffs[2], 16));

  doc[F("hamfr")] = _ha.maxFailedRequest;
  doc[F("haproto")] = _ha.protocol;
  doc[F("hatt")] = _ha.temperatureTimeout;

  // if for WebPage or protocol selected is HTTP
  if (!forSaveFile || _ha.protocol == HA_PROTO_HTTP)
  {
    doc[F("hahtype")] = _ha.http.type;
    doc[F("hahhost")] = _ha.http.hostname;
    doc[F("hahtls")] = _ha.http.tls;
    doc[F("hahtempid")] = _ha.http.temperatureId;

    // if Home Automation protocol selected is Jeedom
    if (_ha.http.type == HA_HTTP_JEEDOM)
    {
      if (forSaveFile)
        doc[F("hahjak")] = _ha.http.secret;
      else
        doc[F("hahjak")] = (const __FlashStringHelper *)appDataPredefPassword; // predefined special password (mean to keep already saved one)
    }

    // if Home Automation protocol selected is Fibaro
    if (_ha.http.type == HA_HTTP_FIBARO)
    {
      doc[F("hahfuser")] = _ha.http.fibaro.username;
      if (forSaveFile)
        doc[F("hahfpass")] = _ha.http.secret;
      else
        doc[F("hahfpass")] = (const __FlashStringHelper *)appDataPredefPassword; // predefined special password (mean to keep already saved one)
    }

    // if Home Automation protocol selected is HomeAssistant
    if (_ha.http.type == HA_HTTP_HOMEASSISTANT)
    {
      doc[F("hahhaei")] = _ha.http.homeassistant.entityId;
      if (forSaveFile)
        doc[F("hahhallat")] = _ha.http.secret;
      else
        doc[F("hahhallat")] = (const __FlashStringHelper *)appDataPredefPassword; // predefined special password (mean to keep already saved one)
    }
  }

  // if for WebPage or protocol selected is MQTT
  if (!forSaveFile || _ha.protocol == HA_PROTO_MQTT)
  {
    doc[F("hamtemptopic")] = _ha.mqtt.temperatureTopic;
  }

  doc[F("cbproto")] = _ha.cboxProtocol;
  doc[F("cbtt")] = _ha.cboxTemperatureTimeout;

  // if for WebPage or CBox protocol selected is HTTP
  if (!forSaveFile || _ha.cboxProtocol == CBOX_PROTO_HTTP)
  {
    if (_ha.http.cboxIp)
      doc[F("cbhip")] = IPAddress(_ha.http.cboxIp).toString();
  }

  // if for WebPage or CBox protocol selected is MQTT
  if (!forSaveFile || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    doc[F("cbmt1topic")] = _ha.mqtt.cboxT1Topic;
  }

  if (!forSaveFile || _ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    doc[F("hamhost")] = _ha.mqtt.hostname;
    doc[F("hamport")] = _ha.mqtt.port;
    doc[F("hamu")] = _ha.mqtt.username;
    if (forSaveFile)
      doc[F("hamp")] = _ha.mqtt.password;
    else
      doc[F("hamp")] = (const __FlashStringHelper *)appDataPredefPassword; // predefined special password (mean to keep already saved one)
    doc[F("hambt")] = _ha.mqtt.baseTopic;
    doc[F("hamhassde")] = _ha.mqtt.hassDiscoveryEnabled;
    doc[F("hamhassdp")] = _ha.mqtt.hassDiscoveryPrefix;
  }

  String gc;
  doc.shrinkToFit();
  serializeJson(doc, gc);

  return gc;
}

//------------------------------------------
// Generate JSON of application status
String WPalaSensor::generateStatusJSON()
{
  JsonDocument doc;

  // Home automation protocol
  if (_ha.protocol == HA_PROTO_HTTP)
    doc[F("haprotocol")] = F("HTTP");
  else if (_ha.protocol == HA_PROTO_MQTT)
    doc[F("haprotocol")] = F("MQTT");
  else
    doc[F("haprotocol")] = F("Disabled");

  // Home automation connection status
  if (_ha.protocol == HA_PROTO_HTTP)
    doc[F("hahttplastrespcode")] = _haRequestResult;
  else if (_ha.protocol == HA_PROTO_MQTT)
    doc[F("hamqttstatus")] = _mqttMan.getStateString();

  // Home Automation last temperature and age
  if (_ha.protocol != HA_PROTO_DISABLED)
  {
    doc[F("haslasttemp")] = String(_haTemperature, 2);
    doc[F("haslasttempage")] = ((millis() - _haTemperatureMillis) / 1000);
  }

  // WPalaControl/CBox protocol
  if (_ha.cboxProtocol == CBOX_PROTO_HTTP)
    doc[F("cboxprotocol")] = F("HTTP");
  else if (_ha.cboxProtocol == CBOX_PROTO_MQTT)
    doc[F("cboxprotocol")] = F("MQTT");
  else
    doc[F("cboxprotocol")] = F("Disabled");

  // WPalaControl/CBox connection status
  if (_ha.cboxProtocol == CBOX_PROTO_HTTP)
    doc[F("cboxhttplastrespcode")] = _stoveRequestResult;
  else if (_ha.cboxProtocol == CBOX_PROTO_MQTT)
    doc[F("cboxmqttstatus")] = _mqttMan.getStateString();

  // WPalaControl/CBox last temperature and age
  if (_ha.cboxProtocol != CBOX_PROTO_DISABLED)
  {
    doc[F("cboxlasttemp")] = String(_stoveTemperature, 2);
    doc[F("cboxlasttempage")] = ((millis() - _stoveTemperatureMillis) / 1000);
  }

  doc[F("onewiretemp")] = String(_owTemperature, 2);
  doc[F("onewiretempused")] = (_haTemperatureUsed ? F("No") : F("Yes"));

  doc[F("pushedtemp")] = String(_pushedTemperature, 2);
  doc[F("dgp50k")] = _mcp4151_50k.getPosition(0);
  doc[F("dgp5k")] = _mcp4151_5k.getPosition(0);

  String gs;
  doc.shrinkToFit();
  serializeJson(doc, gs);

  return gs;
}

//------------------------------------------
// code to execute during initialization and reinitialization of the app
bool WPalaSensor::appInit(bool reInit)
{
  // stop Tickers
  _refreshTicker.detach();
  _publishUpdateTicker.detach();

  // Stop MQTT
  _mqttMan.disconnect();

  // if MQTT used so configure it
  if (_ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    // prepare will topic
    String willTopic = _ha.mqtt.baseTopic;
    MQTTMan::prepareTopic(willTopic);
    willTopic += F("connected");

    // setup MQTT
    _mqttMan.setBufferSize(600); // max JSON size (Connectivity HAss discovery ~450)
    _mqttMan.setClient(_wifiClient).setServer(_ha.mqtt.hostname, _ha.mqtt.port);
    _mqttMan.setConnectedAndWillTopic(willTopic.c_str());
    _mqttMan.setConnectedCallback(std::bind(&WPalaSensor::mqttConnectedCallback, this, std::placeholders::_1, std::placeholders::_2));
    _mqttMan.setCallback(std::bind(&WPalaSensor::mqttCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Connect
    _mqttMan.connect(_ha.mqtt.username, _ha.mqtt.password);
  }

  // initialize run variables
  _haTemperatureMillis = millis(); // keep last received HA temperature fresh
  _haRequestResult = 0;
  _stoveTemperatureMillis = millis() - (1000 * _refreshPeriod); // delta is kept but will evolve only if a new value is received
  _stoveRequestResult = 0;

  // flag to force refresh to see immediate result (init and reinit)
  _needRefresh = true;

  // start refresh Ticker
#ifdef ESP8266
  _refreshTicker.attach(_refreshPeriod, [this]()
                        { this->_needRefresh = true; });
#else
  _refreshTicker.attach<typeof this>(_refreshPeriod, [](typeof this palaSensor)
                                     { palaSensor->_needRefresh = true; }, this);
#endif

  // flag to force publish update (init and reinit)
  _needPublishUpdate = true;

  // start publish update Ticker
#ifdef ESP8266
  _publishUpdateTicker.attach(86400, [this]()
                              { this->_needPublishUpdate = true; });
#else
  _publishUpdateTicker.attach<typeof this>(86400, [](typeof this palaSensor)
                                           { palaSensor->_needPublishUpdate = true; }, this);
#endif

  return _ds18b20.getReady();
}

//------------------------------------------
// Return HTML Code to insert into Status Web page
const PROGMEM char *WPalaSensor::getHTMLContent(WebPageForPlaceHolder wp)
{
  switch (wp)
  {
  case status:
    return status2htmlgz;
    break;
  case config:
    return config2htmlgz;
    break;
  default:
    return nullptr;
    break;
  };
  return nullptr;
}

// and his Size
size_t WPalaSensor::getHTMLContentSize(WebPageForPlaceHolder wp)
{
  switch (wp)
  {
  case status:
    return sizeof(status2htmlgz);
    break;
  case config:
    return sizeof(config2htmlgz);
    break;
  default:
    return 0;
    break;
  };
  return 0;
}

//------------------------------------------
// code to register web request answer to the web server
void WPalaSensor::appInitWebServer(WebServer &server)
{
  // GetDigiPot
  server.on(F("/gdp"), HTTP_GET,
            [this, &server]()
            {
              String dpJSON('{');
              dpJSON = dpJSON + F("\"r\":") + (_mcp4151_50k.getPosition(0) * _digipotsNTC.rBW50KStep + _mcp4151_5k.getPosition(0) * _digipotsNTC.rBW5KStep + _digipotsNTC.rWTotal);
#if DEVELOPPER_MODE
              dpJSON = dpJSON + F(",\"dp5k\":") + _mcp4151_5k.getPosition(0);
              dpJSON = dpJSON + F(",\"dp50k\":") + _mcp4151_50k.getPosition(0);
#endif
              dpJSON += '}';

              server.sendHeader(F("Cache-Control"), F("no-cache"));
              server.send(200, F("text/json"), dpJSON);
            });

  // SetDigiPot
  server.on(F("/sdp"), HTTP_POST,
            [this, &server]()
            {
#define TICK_TO_SKIP 20
              JsonDocument doc;
              DeserializationError error = deserializeJson(doc, server.arg(F("plain")));

              if (error)
              {
                server.send(400, F("text/html"), F("Malformed JSON"));
                return;
              }

              JsonVariant jv;

              // look for temperature to apply
              if ((jv = doc[F("temperature")]).is<JsonVariant>())
              {
                // convert and set it
                setDualDigiPot(jv.as<float>());
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for increase of digipots
              if (doc[F("up")].is<JsonVariant>())
              {
                // go one step up
                setDualDigiPot((int)(_mcp4151_50k.getPosition(0) * _digipotsNTC.rBW50KStep + _mcp4151_5k.getPosition(0) * _digipotsNTC.rBW5KStep + _digipotsNTC.rWTotal + _digipotsNTC.rBW5KStep));
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for decrease of digipots
              if (doc[F("down")].is<JsonVariant>())
              {
                // go one step down
                setDualDigiPot((int)(_mcp4151_50k.getPosition(0) * _digipotsNTC.rBW50KStep + _mcp4151_5k.getPosition(0) * _digipotsNTC.rBW5KStep + _digipotsNTC.rWTotal - _digipotsNTC.rBW5KStep));
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

#if DEVELOPPER_MODE

              // look for 5k digipot requested position
              if ((jv = doc[F("dp5k")]).is<JsonVariant>())
              {
                // convert and set it
                _mcp4151_5k.setPosition(0, jv);
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for 50k digipot requested position
              if ((jv = doc[F("dp50k")]).is<JsonVariant>())
              {
                // convert and set it
                _mcp4151_50k.setPosition(0, jv);
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for resistance to apply
              if ((jv = doc[F("resistance")]).is<JsonVariant>())
              {
                // convert resistance value and call right function
                setDualDigiPot(0, jv);
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }
#endif

              server.send(200);
            });
}

//------------------------------------------
// Run for application
void WPalaSensor::appRun()
{
  if (_ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    _mqttMan.loop();

    // if Home Assistant discovery enabled and publish is needed (and publish is successful)
    if (_ha.mqtt.hassDiscoveryEnabled && _needPublishHassDiscovery && mqttPublishHassDiscovery())
    {
      _needPublishHassDiscovery = false;
      _needRefresh = true;
    }

    if (_needPublishUpdate && mqttPublishUpdate())
      _needPublishUpdate = false;
  }

  if (_needRefresh)
  {
    _needRefresh = false;
    refresh();
  }
}

//------------------------------------------
// Constructor
WPalaSensor::WPalaSensor() : Application(CustomApp), _ds18b20(ONEWIRE_BUS_PIN), _dac(0x60)
{
  // Init I2c for DAC
  Wire.begin();
  // Init DigiPots @20°C
  _mcp4151_50k.setPosition(0, 61);
  _mcp4151_5k.setPosition(0, 5);
}