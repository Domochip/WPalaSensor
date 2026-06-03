#include "WPalaSensor.h"

//-----------------------------------------------------------------------
// Steinhart–Hart reverse function
//-----------------------------------------------------------------------
void WPalaSensor::setDac(float temperature, bool EEPROM /* = false*/)
{
  // convert temperature from Celsius to Kevin degrees
  float temperatureK = temperature + 273.15;

  // calculate and return resistance value based on provided temperature
  double x = (1 / _steinhartHartCoeffs[2]) * (_steinhartHartCoeffs[0] - (1 / temperatureK));
  double y = sqrt(pow(_steinhartHartCoeffs[1] / (3 * _steinhartHartCoeffs[2]), 3) + pow(x / 2, 2));
  setDac((int)(exp(cbrt(y - (x / 2)) - cbrt(y + (x / 2)))), EEPROM);
}
//-----------------------------------------------------------------------
// Set DAC equivalent resistance
//-----------------------------------------------------------------------
void WPalaSensor::setDac(int resistance, bool EEPROM /* = false*/)
{
  // calculate DAC value
  uint32_t value = 8200 + 2200;
  value *= 4096;
  value /= 8200 + resistance;

  // if EEPROM is true
  if (EEPROM)
  {
    if (_dac.readEEPROM() != value)
      _dac.writeDAC(value, true);
  }
  else
  {
    if (_dac.getValue() != value)
      _dac.setValue(value);
  }
}

//-----------------------------------------------------------------------
// Main Refresh method
//-----------------------------------------------------------------------
void WPalaSensor::refresh()
{
  // LOG
  LOG_SERIAL_PRINTLN(F("refresh"));

  // if MQTT protocol is enabled and connected then publish Core, Wifi and WPalaControl status
  if (_ha.protocol == HA_PROTO_MQTT && _mqttMan.connected())
  {
    JsonDocument json;
    _applicationList[CoreApp]->fillStatusJSON(json[getAppIdName(CoreApp)].to<JsonObject>());
    _applicationList[WifiManApp]->fillStatusJSON(json[getAppIdName(WifiManApp)].to<JsonObject>());
    fillStatusJSON(json[getAppIdName(CustomApp)].to<JsonObject>());

    _mqttMan.publish(_mqttMan.getBaseTopic(), json, true);
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
    char httpBuffer[320];

    // set timeOut
    http.setTimeout(5000);

    // try to get HomeAutomation sensor value -----------------

    // build the complete URI
    switch (_ha.http.type)
    {
    case HA_HTTP_JEEDOM:
      snprintf_P(httpBuffer, sizeof(httpBuffer), PSTR("http%s://%s/core/api/jeeApi.php?apikey=%s&type=cmd&id=%d"),
                 _ha.http.tls ? "s" : "", _ha.http.hostname, _ha.http.secret, _ha.http.temperatureId);
      break;
    case HA_HTTP_FIBARO:
      snprintf_P(httpBuffer, sizeof(httpBuffer), PSTR("http%s://%s/api/devices?id=%d"),
                 _ha.http.tls ? "s" : "", _ha.http.hostname, _ha.http.temperatureId);
      break;
    case HA_HTTP_HOMEASSISTANT:
      snprintf_P(httpBuffer, sizeof(httpBuffer), PSTR("http%s://%s/api/states/%s"),
                 _ha.http.tls ? "s" : "", _ha.http.hostname, _ha.http.homeassistant.entityId);
      break;
    }

    // if not TLS then use client, else use clientSecure
    if (!_ha.http.tls)
      http.begin(client, httpBuffer);
    else
    {
      clientSecure.setInsecure();
      http.begin(clientSecure, httpBuffer);
    }

    // For Fibaro, Pass authentication if specified in configuration
    if (_ha.http.type == HA_HTTP_FIBARO && _ha.http.fibaro.username[0])
      http.setAuthorization(_ha.http.fibaro.username, _ha.http.secret);

    // For HomeAssistant, Pass long-lived access token and set content type
    if (_ha.http.type == HA_HTTP_HOMEASSISTANT)
    {
      snprintf_P(httpBuffer, sizeof(httpBuffer), PSTR("Bearer %s"), _ha.http.secret);
      http.addHeader(F("Authorization"), httpBuffer);
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
        char *endPtr;
        float haTemperature = strtof(payload, &endPtr);

        // if we got a correct value (at least one character was parsed)
        if (endPtr != payload)
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
    char cboxUrl[64];
    IPAddress cboxIp(_ha.http.cboxIp);
    snprintf_P(cboxUrl, sizeof(cboxUrl), PSTR("http://%d.%d.%d.%d/cgi-bin/sendmsg.lua?cmd=GET%%20TMPS"),
               cboxIp[0], cboxIp[1], cboxIp[2], cboxIp[3]);
    http.begin(client, cboxUrl);

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
          char *endPtr;
          float stoveTemperature = strtof(payload + posTRW, &endPtr);

          // if we got a correct value (at least one character was parsed)
          if (endPtr != payload + posTRW)
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

  // Set DAC position according to resistance calculated from temperature to display with delta
  setDac(temperatureToDisplay + _stoveDelta);

  _lastTemperatureUsed = temperatureToDisplay;

  _pushedTemperature = temperatureToDisplay + _stoveDelta;

  // if first refresh tick then set flag to false
  if (_firstRefreshTick)
    _firstRefreshTick = false;

#if DEVELOPPER_MODE

  // publish to MQTT
  if (_mqttMan.connected())
  {
    char topic[MQTTMan::baseTopicSize + 14]; // 14 = len("/TempToDisplay")
    char val[9];

    // publish oneWire temperature
    snprintf_P(topic, sizeof(topic), PSTR("%s/OWTemp"), _mqttMan.getBaseTopic());
    dtostrf(_owTemperature, 1, 2, val);
    _mqttMan.publish(topic, val, true);

    // publish Home Automation temperature
    snprintf_P(topic, sizeof(topic), PSTR("%s/HATemp"), _mqttMan.getBaseTopic());
    dtostrf(_haTemperature, 1, 2, val);
    _mqttMan.publish(topic, val, true);

    // publish temperature to display
    snprintf_P(topic, sizeof(topic), PSTR("%s/TempToDisplay"), _mqttMan.getBaseTopic());
    dtostrf(temperatureToDisplay, 1, 2, val);
    _mqttMan.publish(topic, val, true);

    // publish last used temperature
    snprintf_P(topic, sizeof(topic), PSTR("%s/LastTempUsed"), _mqttMan.getBaseTopic());
    dtostrf(_lastTemperatureUsed, 1, 2, val);
    _mqttMan.publish(topic, val, true);

    // publish Stove temperature
    snprintf_P(topic, sizeof(topic), PSTR("%s/StoveTemp"), _mqttMan.getBaseTopic());
    dtostrf(_stoveTemperature, 1, 2, val);
    _mqttMan.publish(topic, val, true);

    // publish Delta
    snprintf_P(topic, sizeof(topic), PSTR("%s/Delta"), _mqttMan.getBaseTopic());
    dtostrf(_stoveDelta, 1, 2, val);
    _mqttMan.publish(topic, val, true);

    // publish Pushed temperature
    snprintf_P(topic, sizeof(topic), PSTR("%s/PushedTemp"), _mqttMan.getBaseTopic());
    dtostrf(_pushedTemperature, 1, 2, val);
    _mqttMan.publish(topic, val, true);
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
    mqttMan->subscribe(_ha.mqtt.temperatureTopic);

  // if Connection Box/PalaControl is configured for MQTT
  if (_ha.cboxProtocol == CBOX_PROTO_MQTT)
    mqttMan->subscribe(_ha.mqtt.cboxT1Topic);

  // subscribe to update/install topic
  String topic(_mqttMan.getBaseTopic());
  topic += F("/update/install");
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
    // copy payload into a null-terminated buffer
    char haTemperatureBuffer[7];
    unsigned int len = min(length, 6u);
    memcpy(haTemperatureBuffer, payload, len);
    haTemperatureBuffer[len] = 0;

    // convert
    char *endPtr;
    float haTemperature = strtof(haTemperatureBuffer, &endPtr);

    // if we got a correct value (at least one character was parsed)
    if (endPtr != haTemperatureBuffer)
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
    const uint8_t *valueStart = payload;
    unsigned int valueLen = length;

    // if payload contains JSON, isolate T1 value
    if (length > 0 && payload[0] == '{')
    {
      // search for "T1": in payload
      for (unsigned int i = 0; i + 5 <= length; i++)
      {
        if (memcmp(payload + i, "\"T1\":", 5) == 0)
        {
          valueStart = payload + i + 5;

          // skip whitespace before value
          while (valueStart < (payload + length) && (*valueStart == ' ' || *valueStart == '\t' || *valueStart == '\r' || *valueStart == '\n'))
            valueStart++;

          valueLen = 0;

          // count float text until first non-float char
          while ((valueStart + valueLen) < (payload + length))
          {
            char c = valueStart[valueLen];
            if ((c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E')
              valueLen++;
            else
              break;
          }

          break;
        }
      }
    }

    if (valueLen > 0)
    {
      // copy into null-terminated buffer for strtof
      char stoveBuffer[7];
      unsigned int bufLen = min(valueLen, 6u);
      memcpy(stoveBuffer, valueStart, bufLen);
      stoveBuffer[bufLen] = 0;

      // convert
      char *endPtr;
      float stoveTemperature = strtof(stoveBuffer, &endPtr);

      // if we got a correct value (at least one character was parsed)
      if (endPtr != stoveBuffer)
      {
        _stoveTemperature = stoveTemperature;
        _stoveTemperatureMillis = millis();
      }
    }
  }

  // if topic ends with "/update/install"
  const size_t topicLen = strlen(topic);
  if (topicLen >= 15 && memcmp_P(topic + topicLen - 15, PSTR("/update/install"), 15) == 0)
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

  // Helper lambda to prepare entity topic
  auto prepareHassDiscoveryTopic = [&](const String &type, const String &uniqueId)
  {
    String topic;
    topic.reserve(strlen(_ha.mqtt.hassDiscoveryPrefix) + type.length() + uniqueId.length() + 9); // 9 = "/" + "/" + "/config"
    topic += _ha.mqtt.hassDiscoveryPrefix;
    topic += F("/");
    topic += type;
    topic += F("/");
    topic += uniqueId;
    topic += F("/config");
    return topic;
  };

  // variables
  JsonDocument json;
  String device;
  String uniqueIdPrefix;
  String uniqueId;
  String topic;

  // prepare unique id prefix
  uniqueIdPrefix = F(CUSTOM_APP_MODEL "_");
  uniqueIdPrefix += WifiMan::getMacAddress();
  uniqueIdPrefix.replace(":", "");

  // default availability JSON
  const __FlashStringHelper *availabilityJSON = F("{\"topic\":\"~/connected\",\"value_template\":\"{{ iif(int(value) > 0, 'online', 'offline') }}\"}");

  // ---------- Device ----------

  // prepare device JSON
  deserializeJson(json, F("{"
                          "\"configuration_url\":\"http://" CUSTOM_APP_MODEL ".local\","
                          "\"manufacturer\":\"" CUSTOM_APP_MANUFACTURER "\","
                          "\"model\":\"" CUSTOM_APP_MODEL "\","
                          "\"sw_version\":\"" VERSION "\""
                          "}"));
  json[F("identifiers")][0] = uniqueIdPrefix;
  json[F("name")] = WiFi.getHostname();
  serializeJson(json, device); // serialize to device String

  // ----- Entities -----

  //
  // Connectivity entity
  //

  // prepare uniqueId, topic and payload for connectivity sensor
  uniqueId = uniqueIdPrefix + F("_Connectivity");

  topic = prepareHassDiscoveryTopic(F("binary_sensor"), uniqueId);

  // prepare payload for connectivity sensor
  deserializeJson(json, F("{"
                          "\"default_entity_id\":\"binary_sensor." CUSTOM_APP_MODEL "_connectivity\","
                          "\"device_class\":\"connectivity\","
                          "\"entity_category\":\"diagnostic\","
                          "\"object_id\":\"" CUSTOM_APP_MODEL "_connectivity\","
                          "\"state_topic\":\"~/connected\","
                          "\"value_template\": \"{{ iif(int(value) > 0, 'ON', 'OFF') }}\""
                          "}"));
  json[F("~")] = _mqttMan.getBaseTopic();
  json[F("device")] = serialized(device);
  json[F("unique_id")] = uniqueId;

  // publish
  _mqttMan.publish(topic.c_str(), json, true);

  //
  // Update entity
  //

  // prepare uniqueId, topic and payload for update sensor
  uniqueId = uniqueIdPrefix + F("_Update");

  topic = prepareHassDiscoveryTopic(F("update"), uniqueId);

  // prepare payload for update sensor
  deserializeJson(json, F("{"
                          "\"command_topic\":\"~/update/install\","
                          "\"default_entity_id\":\"update." CUSTOM_APP_MODEL "\","
                          "\"device_class\":\"firmware\","
                          "\"entity_category\":\"config\","
                          "\"object_id\":\"" CUSTOM_APP_MODEL "\","
                          "\"payload_install\":\"latest\","
                          "\"state_topic\":\"~/update\""
                          "}"));
  json[F("~")] = _mqttMan.getBaseTopic();
  json[F("availability")] = serialized(availabilityJSON);
  json[F("device")] = serialized(device);
  json[F("unique_id")] = uniqueId;

  // publish
  _mqttMan.publish(topic.c_str(), json, true);

  return true;
}

//------------------------------------------
// Publish update to MQTT
bool WPalaSensor::mqttPublishUpdate()
{
  if (!_mqttMan.connected())
    return false;

  // get update info
  JsonDocument updateInfo;
  fillLatestUpdateInfoJson(updateInfo);

  // calculate update topic
  String topic = _mqttMan.getBaseTopic();
  topic += F("/update");

  // publish install in_progress (new in 2024.11)
  // I keep it here because I want to separate the two publish for retrocompatibility
  // if "in_progress" is in the same payload, Home Assistant 2024.10 and lower will ignore the payload
  // (to be moved to WBase around 2025-05)
  _mqttMan.publish(topic.c_str(), (String(F("{\"in_progress\":")) + (Update.isRunning() ? F("true") : F("false")) + '}').c_str(), true);

  // publish update info
  _mqttMan.publish(topic.c_str(), updateInfo, true);

  return true;
}

//------------------------------------------
// Used to initialize configuration properties to default values
void WPalaSensor::setConfigDefaultValues()
{
  _refreshPeriod = 30;

  _steinhartHartCoeffs[0] = 0.0010418107149703;
  _steinhartHartCoeffs[1] = 0.0002249955839098;
  _steinhartHartCoeffs[2] = 0.0000003246246447;

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
bool WPalaSensor::parseConfigJSON(JsonVariant json, bool fromWebPage /* = false */)
{
  JsonVariant jv;

  if ((jv = json[F("rp")]).is<JsonVariant>())
    _refreshPeriod = jv;

  if ((jv = json[F("sha")]).is<JsonVariant>())
    _steinhartHartCoeffs[0] = jv;
  if ((jv = json[F("shb")]).is<JsonVariant>())
    _steinhartHartCoeffs[1] = jv;
  if ((jv = json[F("shc")]).is<JsonVariant>())
    _steinhartHartCoeffs[2] = jv;

  // Parse Home Automation config

  if ((jv = json[F("haproto")]).is<JsonVariant>())
    _ha.protocol = jv;

  // if an home Automation protocol has been selected then get common param
  if (_ha.protocol != HA_PROTO_DISABLED)
  {
    if ((jv = json[F("hamfr")]).is<JsonVariant>())
      _ha.maxFailedRequest = jv;

    if ((jv = json[F("hatt")]).is<JsonVariant>())
      _ha.temperatureTimeout = jv;
  }

  // Parse ConnectionBox config
  if ((jv = json[F("cbproto")]).is<JsonVariant>())
    _ha.cboxProtocol = jv;

  // if an ConnectionBox protocol has been selected then get common param
  if (_ha.cboxProtocol != CBOX_PROTO_DISABLED)
  {
    if ((jv = json[F("cbtt")]).is<JsonVariant>())
      _ha.cboxTemperatureTimeout = jv;
  }

  // if home automation or CBox protocol is MQTT then get common mqtt params
  if (_ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {

    if ((jv = json[F("hamhost")]).is<const char *>())
      strlcpy(_ha.mqtt.hostname, jv, sizeof(_ha.mqtt.hostname));
    if ((jv = json[F("hamport")]).is<JsonVariant>())
      _ha.mqtt.port = jv;
    if ((jv = json[F("hamu")]).is<const char *>())
      strlcpy(_ha.mqtt.username, jv, sizeof(_ha.mqtt.username));

    parseSecret(json[F("hamp")], _ha.mqtt.password, sizeof(_ha.mqtt.password), fromWebPage);
    if ((jv = json[F("hambt")]).is<const char *>())
      strlcpy(_ha.mqtt.baseTopic, jv, sizeof(_ha.mqtt.baseTopic));

    _ha.mqtt.hassDiscoveryEnabled = json[F("hamhassde")];
    if ((jv = json[F("hamhassdp")]).is<const char *>())
      strlcpy(_ha.mqtt.hassDiscoveryPrefix, jv, sizeof(_ha.mqtt.hassDiscoveryPrefix));
  }

  // Now get Home Automation specific params
  switch (_ha.protocol)
  {
  case HA_PROTO_HTTP:

    if ((jv = json[F("hahtype")]).is<JsonVariant>())
      _ha.http.type = jv;
    if ((jv = json[F("hahhost")]).is<const char *>())
      strlcpy(_ha.http.hostname, jv, sizeof(_ha.http.hostname));
    _ha.http.tls = json[F("hahtls")];
    if ((jv = json[F("hahtempid")]).is<JsonVariant>())
      _ha.http.temperatureId = jv;

    switch (_ha.http.type)
    {
    case HA_HTTP_JEEDOM:

      parseSecret(json[F("hahjak")], _ha.http.secret, sizeof(_ha.http.secret), fromWebPage);
      break;

    case HA_HTTP_FIBARO:

      if ((jv = json[F("hahfuser")]).is<const char *>())
        strlcpy(_ha.http.fibaro.username, jv, sizeof(_ha.http.fibaro.username));

      parseSecret(json[F("hahfpass")], _ha.http.secret, sizeof(_ha.http.secret), fromWebPage);
      break;

    case HA_HTTP_HOMEASSISTANT:

      if ((jv = json[F("hahhaei")]).is<const char *>())
        strlcpy(_ha.http.homeassistant.entityId, jv, sizeof(_ha.http.homeassistant.entityId));

      parseSecret(json[F("hahhallat")], _ha.http.secret, sizeof(_ha.http.secret), fromWebPage);
      break;
    }

    break;
  case HA_PROTO_MQTT:

    if ((jv = json[F("hamtemptopic")]).is<const char *>())
      strlcpy(_ha.mqtt.temperatureTopic, jv, sizeof(_ha.mqtt.temperatureTopic));
    break;
  }

  // Now get ConnectionBox specific params
  switch (_ha.cboxProtocol)
  {
  case CBOX_PROTO_HTTP:

    if ((jv = json[F("cbhip")]).is<const char *>())
    {
      IPAddress ipParser;
      if (ipParser.fromString(jv.as<const char *>()))
        _ha.http.cboxIp = static_cast<uint32_t>(ipParser);
      else
        _ha.http.cboxIp = 0;
    }
    break;

  case CBOX_PROTO_MQTT:

    if ((jv = json[F("cbmt1topic")]).is<const char *>())
      strlcpy(_ha.mqtt.cboxT1Topic, jv, sizeof(_ha.mqtt.cboxT1Topic));
    break;
  }

  return true;
}

//------------------------------------------
// Normalise and validate configuration — disable protocols with missing required fields
void WPalaSensor::validateConfig()
{
  // Normalise HomeAssistant hostname: append ":8123" if no port is specified and TLS is off
  if (_ha.protocol == HA_PROTO_HTTP && _ha.http.type == HA_HTTP_HOMEASSISTANT)
    if (_ha.http.hostname[0] && !strchr(_ha.http.hostname, ':') && !_ha.http.tls && (strlen(_ha.http.hostname) + 5 < sizeof(_ha.http.hostname) - 1))
      strcat(_ha.http.hostname, ":8123");

  // Disable HA protocol if required fields are missing
  switch (_ha.protocol)
  {
  case HA_PROTO_HTTP:
    switch (_ha.http.type)
    {
    case HA_HTTP_JEEDOM:
      if (!_ha.http.hostname[0] || !_ha.http.secret[0])
        _ha.protocol = HA_PROTO_DISABLED;
      break;
    case HA_HTTP_FIBARO:
      if (!_ha.http.hostname[0])
        _ha.protocol = HA_PROTO_DISABLED;
      break;
    case HA_HTTP_HOMEASSISTANT:
      if (!_ha.http.hostname[0] || !_ha.http.homeassistant.entityId[0] || !_ha.http.secret[0])
        _ha.protocol = HA_PROTO_DISABLED;
      break;
    }
    break;
  case HA_PROTO_MQTT:
    if (!_ha.mqtt.hostname[0] || !_ha.mqtt.baseTopic[0] || !_ha.mqtt.temperatureTopic[0])
      _ha.protocol = HA_PROTO_DISABLED;
    break;
  }

  // Disable CBox protocol if required fields are missing
  switch (_ha.cboxProtocol)
  {
  case CBOX_PROTO_HTTP:
    if (!_ha.http.cboxIp)
      _ha.cboxProtocol = CBOX_PROTO_DISABLED;
    break;
  case CBOX_PROTO_MQTT:
    if (!_ha.mqtt.hostname[0] || !_ha.mqtt.baseTopic[0] || !_ha.mqtt.cboxT1Topic[0])
      _ha.cboxProtocol = CBOX_PROTO_DISABLED;
    break;
  }
}

//------------------------------------------
// Generate JSON from configuration properties
void WPalaSensor::fillConfigJSON(JsonVariant json, bool forSaveFile /* = false */)
{
  json["rp"] = _refreshPeriod;

  json[F("sha")] = serialized(String(_steinhartHartCoeffs[0], 16));
  json[F("shb")] = serialized(String(_steinhartHartCoeffs[1], 16));
  json[F("shc")] = serialized(String(_steinhartHartCoeffs[2], 16));

  json[F("hamfr")] = _ha.maxFailedRequest;
  json[F("haproto")] = _ha.protocol;
  json[F("hatt")] = _ha.temperatureTimeout;

  // if for WebPage or protocol selected is HTTP
  if (!forSaveFile || _ha.protocol == HA_PROTO_HTTP)
  {
    json[F("hahtype")] = _ha.http.type;
    json[F("hahhost")] = _ha.http.hostname;
    json[F("hahtls")] = _ha.http.tls;
    json[F("hahtempid")] = _ha.http.temperatureId;

    // if Home Automation protocol selected is Jeedom
    if (_ha.http.type == HA_HTTP_JEEDOM)
    {
      fillSecret(json, F("hahjak"), _ha.http.secret, forSaveFile);
    }

    // if Home Automation protocol selected is Fibaro
    if (_ha.http.type == HA_HTTP_FIBARO)
    {
      json[F("hahfuser")] = _ha.http.fibaro.username;
      fillSecret(json, F("hahfpass"), _ha.http.secret, forSaveFile);
    }

    // if Home Automation protocol selected is HomeAssistant
    if (_ha.http.type == HA_HTTP_HOMEASSISTANT)
    {
      json[F("hahhaei")] = _ha.http.homeassistant.entityId;
      fillSecret(json, F("hahhallat"), _ha.http.secret, forSaveFile);
    }
  }

  // if for WebPage or protocol selected is MQTT
  if (!forSaveFile || _ha.protocol == HA_PROTO_MQTT)
  {
    json[F("hamtemptopic")] = _ha.mqtt.temperatureTopic;
  }

  json[F("cbproto")] = _ha.cboxProtocol;
  json[F("cbtt")] = _ha.cboxTemperatureTimeout;

  // if for WebPage or CBox protocol selected is HTTP
  if (!forSaveFile || _ha.cboxProtocol == CBOX_PROTO_HTTP)
  {
    if (_ha.http.cboxIp)
      json[F("cbhip")] = WifiMan::ipToCString(_ha.http.cboxIp);
  }

  // if for WebPage or CBox protocol selected is MQTT
  if (!forSaveFile || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    json[F("cbmt1topic")] = _ha.mqtt.cboxT1Topic;
  }

  if (!forSaveFile || _ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    json[F("hamhost")] = _ha.mqtt.hostname;
    json[F("hamport")] = _ha.mqtt.port;
    json[F("hamu")] = _ha.mqtt.username;
    fillSecret(json, F("hamp"), _ha.mqtt.password, forSaveFile);
    json[F("hambt")] = _ha.mqtt.baseTopic;
    json[F("hamhassde")] = _ha.mqtt.hassDiscoveryEnabled;
    json[F("hamhassdp")] = _ha.mqtt.hassDiscoveryPrefix;
  }
}

//------------------------------------------
// Generate JSON of application status
void WPalaSensor::fillStatusJSON(JsonVariant json)
{
  // Home automation protocol
  if (_ha.protocol == HA_PROTO_HTTP)
    json[F("haprotocol")] = F("HTTP");
  else if (_ha.protocol == HA_PROTO_MQTT)
    json[F("haprotocol")] = F("MQTT");
  else
    json[F("haprotocol")] = F("Disabled");

  // Home automation connection status
  if (_ha.protocol == HA_PROTO_HTTP)
    json[F("hahttplastrespcode")] = _haRequestResult;
  else if (_ha.protocol == HA_PROTO_MQTT)
    json[F("hamqttstatus")] = _mqttMan.getStateString();

  // Home Automation last temperature and age
  char buf[10];
  if (_ha.protocol != HA_PROTO_DISABLED)
  {
    json[F("haslasttemp")] = dtostrf(_haTemperature, 0, 2, buf);
    json[F("haslasttempage")] = ((millis() - _haTemperatureMillis) / 1000);
  }

  // WPalaControl/CBox protocol
  if (_ha.cboxProtocol == CBOX_PROTO_HTTP)
    json[F("cboxprotocol")] = F("HTTP");
  else if (_ha.cboxProtocol == CBOX_PROTO_MQTT)
    json[F("cboxprotocol")] = F("MQTT");
  else
    json[F("cboxprotocol")] = F("Disabled");

  // WPalaControl/CBox connection status
  if (_ha.cboxProtocol == CBOX_PROTO_HTTP)
    json[F("cboxhttplastrespcode")] = _stoveRequestResult;
  else if (_ha.cboxProtocol == CBOX_PROTO_MQTT)
    json[F("cboxmqttstatus")] = _mqttMan.getStateString();

  // WPalaControl/CBox last temperature and age
  if (_ha.cboxProtocol != CBOX_PROTO_DISABLED)
  {
    json[F("cboxlasttemp")] = dtostrf(_stoveTemperature, 0, 2, buf);
    json[F("cboxlasttempage")] = ((millis() - _stoveTemperatureMillis) / 1000);
  }

  json[F("onewiretemp")] = dtostrf(_owTemperature, 0, 2, buf);
  json[F("onewiretempused")] = (_haTemperatureUsed ? F("No") : F("Yes"));

  json[F("pushedtemp")] = dtostrf(_pushedTemperature, 0, 2, buf);
  json[F("dac")] = _dac.getValue();
}

//------------------------------------------
// code to execute during initialization and reinitialization of the app
bool WPalaSensor::appInit(bool reInit /* = false */)
{
  // stop Tickers
  _refreshTicker.detach();
  _publishUpdateTicker.detach();

  // Stop MQTT
  _mqttMan.disconnect();

  // Init DAC
  _dac.begin();
  // configure DAC EEPROM for next boot (will be set only if EEPROM value is not the same)
  setDac(20.0F, true);

  // if MQTT used so configure it
  if (_ha.protocol == HA_PROTO_MQTT || _ha.cboxProtocol == CBOX_PROTO_MQTT)
  {
    // setup MQTT
    _mqttMan.setBufferSize(600); // max JSON size (Connectivity HAss discovery ~450)
    _mqttMan.setClient(_wifiClient).setServer(_ha.mqtt.hostname, _ha.mqtt.port);
    _mqttMan.setBaseTopic(_ha.mqtt.baseTopic);
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
// code to register web request answer to the web server
void WPalaSensor::appInitWebServer(WebServer &server)
{
  // GetDAC
  server.on(F("/gdac"), HTTP_GET,
            [this, &server]()
            {
              String dpJSON('{');
              dpJSON = dpJSON + F("\"res\":") + (((10400 * 4096) / _dac.getValue()) - 8200);
#if DEVELOPPER_MODE
              dpJSON = dpJSON + F(",\"dac\":") + _dac.getValue();
#endif
              dpJSON += '}';

              server.sendHeader(F("Cache-Control"), F("no-cache"));
              server.send(200, F("text/json"), dpJSON);
            });

  // SetDAC
  server.on(F("/sdac"), HTTP_POST,
            [this, &server]()
            {
#define TICK_TO_SKIP 20
              JsonDocument json;
              DeserializationError error = deserializeJson(json, server.arg(F("plain")));

              if (error)
              {
                server.send(400, F("text/html"), F("Malformed JSON"));
                return;
              }

              JsonVariant jv;

              // look for temperature to apply
              if ((jv = json[F("temperature")]).is<JsonVariant>())
              {
                // convert and set it
                setDac(jv.as<float>());
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for increase of dac
              if (json[F("up")].is<JsonVariant>())
              {
                // go one step up
                _dac.setValue(_dac.getValue() + 1);
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for decrease of dac
              if (json[F("down")].is<JsonVariant>())
              {
                // go one step down
                _dac.setValue(_dac.getValue() - 1);
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

#if DEVELOPPER_MODE

              // look for dac requested position
              if ((jv = json[F("dac")]).is<JsonVariant>())
              {
                // convert and set it
                _dac.setValue(jv);
                // go for refresh tick skipped (time to look a value on stove)
                _skipTick = TICK_TO_SKIP;
              }

              // look for resistance to apply
              if ((jv = json[F("resistance")]).is<JsonVariant>())
              {
                // convert resistance value and call right function
                setDac(jv.as<int>());
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
}