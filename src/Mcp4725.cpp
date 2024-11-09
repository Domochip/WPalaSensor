#include "Mcp4725.h"

Mcp4725::Mcp4725(uint8_t address)
{
    _address = address;
}

void Mcp4725::begin()
{
    Wire.begin();
}

uint16_t Mcp4725::readDAC(bool EEPROM)
{
    // TODO: Implement this function
}

void Mcp4725::writeDACFastMode(uint16_t value)
{
    Wire.beginTransmission(_address);
    Wire.write((value >> 8) & 0x0F);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

void Mcp4725::writeDAC(uint16_t value, bool EEPROM)
{
    Wire.beginTransmission(_address);
    Wire.write(EEPROM ? 0x60 : 0x40);
    Wire.write(value >> 4);
    Wire.write((value & 0xF) << 4);
    Wire.endTransmission();
}