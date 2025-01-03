//
//    FILE: MCP4726.cpp
//  AUTHOR: Rob Tillaart
// PURPOSE: Arduino library for 12 bit I2C DAC - MCP4726
// VERSION: 0.4.0
//     URL: https://github.com/RobTillaart/MCP4726

#include "MCP4726.h"

//  registerMode
#define MCP4726_DAC 0x40
#define MCP4726_DACEEPROM 0x60

MCP4726::MCP4726(const uint8_t deviceAddress, TwoWire *wire)
{
  _deviceAddress = deviceAddress;
  _wire = wire;
  _lastValue = 0;
}

bool MCP4726::begin()
{
  if ((_deviceAddress < 0x60) || (_deviceAddress > 0x67))
    return false;
  if (!isConnected())
    return false;

  _lastValue = readDAC();
  return true;
}

bool MCP4726::isConnected()
{
  _wire->beginTransmission(_deviceAddress);
  return (_wire->endTransmission() == 0);
}

uint8_t MCP4726::getAddress()
{
  return _deviceAddress;
}

int MCP4726::setValue(const uint16_t value)
{
  if (value == _lastValue)
    return MCP4726_OK;
  if (value > MCP4726_MAXVALUE)
    return MCP4726_VALUE_ERROR;
  int rv = _writeFastMode(value);
  if (rv == 0)
    _lastValue = value;
  return rv;
}

uint16_t MCP4726::getValue()
{
  return _lastValue;
}

//  unfortunately it is not possible to write a different value
//  to the DAC and EEPROM simultaneously or write EEPROM only.
int MCP4726::writeDAC(const uint16_t value, const bool EEPROM)
{
  if (value > MCP4726_MAXVALUE)
    return MCP4726_VALUE_ERROR;
  while (!ready())
    ;
  int rv = _writeRegisterMode(value, EEPROM ? MCP4726_DACEEPROM : MCP4726_DAC);
  if (rv == 0)
    _lastValue = value;
  return rv;
}

//  ready checks if the last write to EEPROM has been written.
//  until ready all writes to the MCP4726 are ignored!
bool MCP4726::ready()
{
  yield();
  uint8_t buffer[1];
  _readRegister(buffer, 1);
  return ((buffer[0] & 0x80) > 0);
}

uint16_t MCP4726::readDAC()
{
  while (!ready())
    ;
  uint8_t buffer[3];
  _readRegister(buffer, 3);
  uint16_t value = buffer[1];
  value = value << 4;
  value = value + (buffer[2] >> 4);
  return value;
}

uint16_t MCP4726::readEEPROM()
{
  while (!ready())
    ;
  uint8_t buffer[6];
  _readRegister(buffer, 6);
  uint16_t value = buffer[4];
  value = value << 4;
  value = value + (buffer[5] >> 4);
  return value;
}

//  PAGE 18 DATASHEET
int MCP4726::_writeFastMode(const uint16_t value)
{
  uint8_t l = value & 0xFF;
  uint8_t h = ((value / 256) & 0x0F); //  set C0 = C1 = 0, no PDmode

  _wire->beginTransmission(_deviceAddress);
  _wire->write(h);
  _wire->write(l);
  return _wire->endTransmission();
}

//  PAGE 19 DATASHEET
//  reg = MCP4726_DAC | MCP4726_EEPROM
int MCP4726::_writeRegisterMode(const uint16_t value, uint8_t reg)
{
  uint8_t h = (value / 16);
  uint8_t l = (value & 0x0F) << 4;
  _wire->beginTransmission(_deviceAddress);
  _wire->write(reg);
  _wire->write(h);
  _wire->write(l);
  return _wire->endTransmission();
}

//  PAGE 20 DATASHEET
//  typical 3 or 5 bytes
uint8_t MCP4726::_readRegister(uint8_t *buffer, const uint8_t length)
{
  _wire->beginTransmission(_deviceAddress);
  int rv = _wire->endTransmission();
  if (rv != 0)
    return 0; //  error

  //  readBytes will always be equal or smaller to length
  uint8_t readBytes = _wire->requestFrom(_deviceAddress, length);
  uint8_t cnt = 0;
  while (cnt < readBytes)
  {
    buffer[cnt++] = _wire->read();
  }
  return readBytes;
}

//  -- END OF FILE --
