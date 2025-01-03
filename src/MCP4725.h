#pragma once
//
//    FILE: MCP4725.h
//  AUTHOR: Rob Tillaart
// PURPOSE: Arduino library for 12 bit I2C DAC - MCP4725
// VERSION: 0.4.0
//     URL: https://github.com/RobTillaart/MCP4725

#include "Wire.h"
#include "Arduino.h"

#define MCP4725_VERSION (F("0.4.0"))

//  CONSTANTS
#define MCP4725_MAXVALUE 4095

//  ERRORS
#define MCP4725_OK 0
#define MCP4725_VALUE_ERROR -999
#define MCP4725_REG_ERROR -998
#define MCP4725_NOT_CONNECTED -997

#define MCP4725_MIDPOINT 2048

class MCP4725
{
public:
  //  address = 0x60..0x67
  explicit MCP4725(const uint8_t deviceAddress, TwoWire *wire = &Wire);

  bool begin();
  bool isConnected();
  uint8_t getAddress();

  //  uses writeFastMode
  int setValue(const uint16_t value = 0);
  //  returns last value set - cached - much faster than readDAC();
  uint16_t getValue();

  //  0..100.0% - input checked.
  //  will set the closest integer value in range 0..4095
  int setPercentage(float percentage = 0);
  //  due to rounding the returned value can differ slightly.
  float getPercentage();

  //  unfortunately it is not possible to write a different value
  //  to the DAC and EEPROM simultaneously or write EEPROM only.
  int writeDAC(const uint16_t value, const bool EEPROM = false);
  //  ready checks if the last write to EEPROM has been written.
  //  until ready all writes to the MCP4725 are ignored!
  bool ready();
  uint16_t readDAC();
  uint16_t readEEPROM();
  uint32_t getLastWriteEEPROM(); //  returns timestamp

private:
  uint8_t _deviceAddress;
  uint16_t _lastValue;
  int _writeFastMode(const uint16_t value);
  uint32_t _lastWriteEEPROM;

  int _writeRegisterMode(const uint16_t value, uint8_t reg);
  uint8_t _readRegister(uint8_t *buffer, const uint8_t length);

  TwoWire *_wire;
};

//  -- END OF FILE --
