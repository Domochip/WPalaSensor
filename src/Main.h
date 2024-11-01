#ifndef Main_h
#define Main_h

#include <Arduino.h>

// Helper macros to convert a define to a string
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// ----- Should be modified for your application -----
#define CUSTOM_APP_MANUFACTURER "Domochip" // also used as Github user for OTA
#define CUSTOM_APP_MODEL "WPalaSensor"     // also used as Github repository for OTA
#define CUSTOM_APP_CLASS WPalaSensor
#define VERSION_NUMBER "4.1.1"

#define CUSTOM_APP_NAME TOSTRING(CUSTOM_APP_CLASS)     // stringified class name
#define CUSTOM_APP_HEADER TOSTRING(CUSTOM_APP_CLASS.h) // calculated header file "{CUSTOM_APP_NAME}.h"
#define DEFAULT_AP_SSID CUSTOM_APP_MODEL               // Default Access Point SSID "{CUSTOM_APP_MODEL}"
#define DEFAULT_AP_PSK "pass"                          // Default Access Point Password

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

// Enable developper mode
#define DEVELOPPER_MODE 0

// Log Serial Object
#define LOG_SERIAL Serial
// Choose Log Serial Speed
#define LOG_SERIAL_SPEED 115200

// Log Serial Macros
#ifdef LOG_SERIAL
#define LOG_SERIAL_PRINT(...) LOG_SERIAL.print(__VA_ARGS__)
#define LOG_SERIAL_PRINTLN(...) LOG_SERIAL.println(__VA_ARGS__)
#define LOG_SERIAL_PRINTF(...) LOG_SERIAL.printf(__VA_ARGS__)
#define LOG_SERIAL_PRINTF_P(...) LOG_SERIAL.printf_P(__VA_ARGS__)
#else
#define LOG_SERIAL_PRINT(...)
#define LOG_SERIAL_PRINTLN(...)
#define LOG_SERIAL_PRINTF(...)
#define LOG_SERIAL_PRINTF_P(...)
#endif

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