#ifndef Mcp4725_h
#define Mcp4725_h

#include <Arduino.h>
#include <Wire.h>

class Mcp4725
{
public:
    Mcp4725(uint8_t address);
    void begin();
    uint16_t readDAC(bool EEPROM);
    void writeDACFastMode(uint16_t value);
    void writeDAC(uint16_t value, bool EEPROM);

private:
    uint8_t _address;
};

#endif // Mcp4725_h