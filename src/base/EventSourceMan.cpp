#include "EventSourceMan.h"

#if EVTSRC_ENABLED

void EventSourceMan::eventSourceHandler(WebServer &server)
{
    uint8_t subPos = 0;

    // Find the subscription for this client
    while (subPos < EVTSRC_MAX_CLIENTS &&
           (!_EventSourceClientList[subPos] ||
            _EventSourceClientList[subPos].remoteIP() != server.client().remoteIP() ||
            _EventSourceClientList[subPos].remotePort() != server.client().remotePort()))
        subPos++;

    // If no subscription found
    if (subPos == EVTSRC_MAX_CLIENTS)
    {
        subPos = 0;
        // Find a free slot
        while (subPos < EVTSRC_MAX_CLIENTS && _EventSourceClientList[subPos])
            subPos++;

        // If there is no more slot available
        if (subPos == EVTSRC_MAX_CLIENTS)
            return server.send(500);
    }

#ifdef ESP8266
    server.client().setSync(true); // disable Nagle's algorithm and flush immediately
#else
    server.client().setNoDelay(true);
#endif

    // create/update subscription
    _EventSourceClientList[subPos] = server.client();
    server.setContentLength(CONTENT_LENGTH_UNKNOWN); // the payload can go on forever
    server.sendContent_P(PSTR("HTTP/1.1 200 OK\nContent-Type: text/event-stream;\nConnection: keep-alive\nCache-Control: no-cache\nAccess-Control-Allow-Origin: *\n\n"));

#if DEVELOPPER_MODE
    LOG_SERIAL_PRINTF_P(PSTR("statusEventSourceHandler - client #%d (%s:%d) registered\n"), subPos, server.client().remoteIP().toString().c_str(), server.client().remotePort());
#endif
}

#if EVTSRC_KEEPALIVE_ENABLED
void EventSourceMan::eventSourceKeepAlive()
{
    for (uint8_t i = 0; i < EVTSRC_MAX_CLIENTS; i++)
    {
        if (_EventSourceClientList[i])
        {
            _EventSourceClientList[i].println(F(":keepalive\n\n"));

#if DEVELOPPER_MODE
            LOG_SERIAL_PRINTF_P(PSTR("statusEventSourceKeepAlive - keep-alive sent to client #%d (%s:%d)\n"), i, _EventSourceClientList[i].remoteIP().toString().c_str(), _EventSourceClientList[i].remotePort());
#endif
        }
    }
}
#endif

void EventSourceMan::initEventSourceServer(char appId, WebServer &server)
{
    String url(F("/statusEvt"));
    url += appId;
    // register EventSource Uri
    server.on(url, HTTP_GET, [this, &server]()
              { eventSourceHandler(server); });
#if EVTSRC_KEEPALIVE_ENABLED
    // send keep alive event every 60 seconds
#ifdef ESP8266
    _eventSourceKeepAliveTicker.attach(60, [this]()
                                       { _needEventSourceKeepAlive = true; });
#else
    _eventSourceKeepAliveTicker.attach<typeof this>(60, [](typeof this eventSourceMan)
                                                    { eventSourceMan->_needEventSourceKeepAlive = true; }, this);
#endif
#endif
}

void EventSourceMan::eventSourceBroadcast(const String &message, const String &eventType) // default eventType is "message"
{
    for (uint8_t i = 0; i < EVTSRC_MAX_CLIENTS; i++)
    {
        if (_EventSourceClientList[i])
        {
            _EventSourceClientList[i].printf_P(PSTR("event: %s\ndata: %s\n\n"), eventType.c_str(), message.c_str());

#if DEVELOPPER_MODE
            LOG_SERIAL_PRINTF_P(PSTR("statusEventSourceBroadcast - event sent to client #%d\n"), i);
#endif
        }
    }
}

#if EVTSRC_KEEPALIVE_ENABLED
void EventSourceMan::run()
{
    if (_needEventSourceKeepAlive)
    {
        _needEventSourceKeepAlive = false;
        eventSourceKeepAlive();
    }
}
#endif // EVTSRC_KEEPALIVE_ENABLED

#endif // EVTSRC_ENABLED