#ifndef Main_h
#define Main_h

#include <Arduino.h>

// Helper macros to convert a define to a string
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// ----- Should be modified for your application -----
#define APPLICATION1_MANUFACTURER "Domochip" // also used as Github user for OTA
#define APPLICATION1_MODEL "WPalaSensor"     // also used as Github repository for OTA
#define APPLICATION1_CLASS WPalaSensor
#define VERSION_NUMBER "4.1.0"

#define APPLICATION1_NAME TOSTRING(APPLICATION1_CLASS)     // stringified class name
#define APPLICATION1_HEADER TOSTRING(APPLICATION1_CLASS.h) // calculated header file "{APPLICATION1_NAME}.h"
#define DEFAULT_AP_SSID APPLICATION1_NAME                  // Default Access Point SSID "{APPLICATION1_NAME}{4 digits of ChipID}"
#define DEFAULT_AP_PSK APPLICATION1_NAME "Pass"            // Default Access Point Password "{APPLICATION1_NAME}Pass"

// Control EventSourceMan code (To be used by Application if EventSource server is needed)
#define EVTSRC_ENABLED 0
#define EVTSRC_MAX_CLIENTS 2
#define EVTSRC_KEEPALIVE_ENABLED 0

#ifdef ESP8266

// Pin 12, 13 and 14 are used by DigiPot Bus
// Pins used for DigiPot Select
#define MCP4151_5k_SSPIN D2
#define MCP4151_50k_SSPIN D1
// Pin for 1Wire DS18B20 bus
#define ONEWIRE_BUS_PIN D4

#else

// Pin 19, 23 and 18 are used by DigiPot Bus
// Pins used for DigiPot Select
#define MCP4151_5k_SSPIN 21
#define MCP4151_50k_SSPIN 22
// Pin for 1Wire DS18B20 bus
#define ONEWIRE_BUS_PIN 16

#endif

// Enable developper mode (fwdev webpage and SPIFFS is used)
#define DEVELOPPER_MODE 0

// Log Serial Object
#define LOG_SERIAL Serial
// Choose Log Serial Speed
#define LOG_SERIAL_SPEED 115200

// Choose Pin used to boot in Rescue Mode
// #define RESCUE_BTN_PIN 16

// Status LED
// #define STATUS_LED_SETUP pinMode(XX, OUTPUT);pinMode(XX, OUTPUT);
// #define STATUS_LED_OFF digitalWrite(XX, HIGH);digitalWrite(XX, HIGH);
// #define STATUS_LED_ERROR digitalWrite(XX, HIGH);digitalWrite(XX, HIGH);
// #define STATUS_LED_WARNING digitalWrite(XX, HIGH);digitalWrite(XX, HIGH);
// #define STATUS_LED_GOOD digitalWrite(XX, HIGH);digitalWrite(XX, HIGH);

// construct Version text
#if DEVELOPPER_MODE
#define VERSION VERSION_NUMBER "-DEV"
#else
#define VERSION VERSION_NUMBER
#endif

#endif