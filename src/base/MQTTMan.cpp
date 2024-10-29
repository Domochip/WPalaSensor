#include "MQTTMan.h"

void MQTTMan::prepareTopic(String &topic)
{
    if (topic.indexOf(F("$sn$")) != -1)
    {
        char sn[9];
#ifdef ESP8266
        sprintf_P(sn, PSTR("%08x"), ESP.getChipId());
#else
        sprintf_P(sn, PSTR("%08x"), (uint32_t)(ESP.getEfuseMac() << 40 >> 40));
#endif
        topic.replace(F("$sn$"), sn);
    }

    if (topic.indexOf(F("$mac$")) != -1)
        topic.replace(F("$mac$"), WiFi.macAddress());

    if (topic.indexOf(F("$model$")) != -1)
        topic.replace(F("$model$"), APPLICATION1_NAME);

    // check for final slash
    if (topic.length() && topic.charAt(topic.length() - 1) != '/')
        topic += '/';
}

bool MQTTMan::connect(bool firstConnection)
{
    char sn[9];
#ifdef ESP8266
    sprintf_P(sn, PSTR("%08x"), ESP.getChipId());
#else
    sprintf_P(sn, PSTR("%08x"), (uint32_t)(ESP.getEfuseMac() << 40 >> 40));
#endif

    // generate clientID
    String clientID(F(APPLICATION1_NAME));
    clientID += sn;

    // Connect
    char *username = (_username[0] ? _username : nullptr);
    char *password = (_username[0] ? _password : nullptr);
    char *willTopic = (_connectedAndWillTopic[0] ? _connectedAndWillTopic : nullptr);
    const char *willMessage = (_connectedAndWillTopic[0] ? "0" : nullptr);
    PubSubClient::connect(clientID.c_str(), username, password, willTopic, 0, true, willMessage);

    if (connected())
    {
        if (_connectedAndWillTopic[0])
            publish(_connectedAndWillTopic, "1", true);

        // Subscribe to needed topic
        if (_connectedCallBack)
            _connectedCallBack(this, firstConnection);
    }

    return connected();
}

MQTTMan &MQTTMan::setConnectedAndWillTopic(const char *topic)
{
    if (!topic)
        _connectedAndWillTopic[0] = 0;
    else if (strlen(topic) < sizeof(_connectedAndWillTopic))
        strcpy(_connectedAndWillTopic, topic);

    return *this;
}

MQTTMan &MQTTMan::setConnectedCallback(CONNECTED_CALLBACK_SIGNATURE connectedCallback)
{
    _connectedCallBack = connectedCallback;
    return *this;
}

MQTTMan &MQTTMan::setDisconnectedCallback(DISCONNECTED_CALLBACK_SIGNATURE disconnectedCallback)
{
    _disconnectedCallBack = disconnectedCallback;
    return *this;
}

bool MQTTMan::connect(const char *username, const char *password)
{
    // check logins
    if (username && strlen(username) >= sizeof(_username))
        return false;
    if (password && strlen(password) >= sizeof(_password))
        return false;

    if (username)
        strcpy(_username, username);
    else
        _username[0] = 0;

    if (password)
        strcpy(_password, password);
    else
        _password[0] = 0;

    return connect(true);
}

void MQTTMan::disconnect()
{
    // publish disconnected just before disconnect...
    if (_connectedAndWillTopic[0])
        publish(_connectedAndWillTopic, "0", true);

    // Stop MQTT Reconnect
    _mqttReconnectTicker.detach();
    // Disconnect
    if (connected()) // Issue #598 : disconnect() crash if client not yet set
    {
        PubSubClient::disconnect();
        // call disconnected callback if set
        if (_disconnectedCallBack)
            _disconnectedCallBack();
    }
}

bool MQTTMan::publishToConnectedTopic(const char *payload)
{
    if (_connectedAndWillTopic[0])
        return publish(_connectedAndWillTopic, payload, true);
    return false;
}

String MQTTMan::getStateString()
{
    switch (state())
    {
    case MQTT_CONNECTION_TIMEOUT:
        return F("Timed Out");
    case MQTT_CONNECTION_LOST:
        return F("Lost");
    case MQTT_CONNECT_FAILED:
        return F("Failed");
    case MQTT_DISCONNECTED:
        return F("Disconnected");
    case MQTT_CONNECTED:
        return F("Connected");
    case MQTT_CONNECT_BAD_PROTOCOL:
        return F("Bad Protocol Version");
    case MQTT_CONNECT_BAD_CLIENT_ID:
        return F("Incorrect ClientID");
    case MQTT_CONNECT_UNAVAILABLE:
        return F("Server Unavailable");
    case MQTT_CONNECT_BAD_CREDENTIALS:
        return F("Bad Credentials");
    case MQTT_CONNECT_UNAUTHORIZED:
        return F("Unauthorized Connection");
    default:
        return F("Unknown");
    }
}

bool MQTTMan::loop()
{
    if (state() != MQTT_DISCONNECTED)
    {
        // evaluate connection status and call disconnected callback if needed
        // if we are not connected, reconnect ticker not started nor _needMqttReconnect flag raised and disconnected callback set
        if (!connected() && !(_mqttReconnectTicker.active() || _needMqttReconnect) && _disconnectedCallBack)
            _disconnectedCallBack();

        if (_needMqttReconnect)
        {
            _needMqttReconnect = false;

            LOG_SERIAL_PRINT(F("MQTT Reconnection : "));

            bool res = connect(false);

            LOG_SERIAL_PRINTLN(res ? F("OK") : F("Failed"));
        }

        // if not connected and reconnect ticker not started
        if (!connected() && !_mqttReconnectTicker.active())
        {
            LOG_SERIAL_PRINTLN(F("MQTT Disconnected"));
            // set Ticker to reconnect after 20 or 60 sec (Wifi connected or not)
#ifdef ESP8266
            _mqttReconnectTicker.once((WiFi.isConnected() ? 20 : 60), [this]()
                                      { _needMqttReconnect = true; });

#else
            _mqttReconnectTicker.once<typeof this>((WiFi.isConnected() ? 20 : 60), [](typeof this mqttMan)
                                                   { mqttMan->_needMqttReconnect = true; }, this);
#endif
        }

        return PubSubClient::loop();
    }
    return true;
}