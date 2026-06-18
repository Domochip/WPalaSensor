
#include "SingleDS18B20.h"

//----------------------------------------------------------------------
// --- SingleDS18B20 Class---
//----------------------------------------------------------------------

//-----------------------------------------------------------------------
// DS18X20 Read ScratchPad command
boolean SingleDS18B20::readScratchPad(byte addr[], byte data[])
{

  boolean crcScratchPadOK = false;

  // read scratchpad (if 3 failures occurs, then return the error
  for (byte i = 0; i < 3; i++)
  {
    // read scratchpad of the current device
    _ow.reset();
    _ow.select(addr);
    _ow.write(0xBE); // Read ScratchPad
    for (byte j = 0; j < 9; j++)
    { // read 9 bytes
      data[j] = _ow.read();
    }
    if (_ow.crc8(data, 8) == data[8])
    {
      crcScratchPadOK = true;
      i = 3; // end for loop
    }
  }

  return crcScratchPadOK;
}
//------------------------------------------
// DS18X20 Write ScratchPad command
void SingleDS18B20::writeScratchPad(byte addr[], byte th, byte tl, byte cfg)
{

  _ow.reset();
  _ow.select(addr);
  _ow.write(0x4E); // Write ScratchPad
  _ow.write(th);   // Th 80°C
  _ow.write(tl);   // Tl 0°C
  _ow.write(cfg);  // Config
}
//------------------------------------------
// DS18X20 Copy ScratchPad command
void SingleDS18B20::copyScratchPad(byte addr[])
{

  _ow.reset();
  _ow.select(addr);
  _ow.write(0x48); // Copy ScratchPad
}
//------------------------------------------
// DS18X20 Start Temperature conversion
void SingleDS18B20::startConvertT(byte addr[])
{
  _ow.reset();
  _ow.select(addr);
  _ow.write(0x44); // start conversion
}

//------------------------------------------
// Constructor
SingleDS18B20::SingleDS18B20(uint8_t owPin) : _ow(owPin)
{

  // find first temp sensor ----------
  _ow.reset_search();

  while (_ow.search(_owROMCode))
  {

    // if ROM received is correct or not a Temperature sensor THEN continue to next device
    if ((_ow.crc8(_owROMCode, 7) == _owROMCode[7]) && (_owROMCode[0] == 0x10 || _owROMCode[0] == 0x22 || _owROMCode[0] == 0x28))
    {
      _tempSensorFound = true;
      break;
    }
  }

  if (!_tempSensorFound)
    return;

  _tempSensorFound = false;
  // configure sensor ----------
  byte data[9];

  // if scratchPad read failed then stop
  if (!readScratchPad(_owROMCode, data))
    return;

  // if config is not correct
  if (data[2] != 0x50 || data[3] != 0x00 || data[4] != 0x5F)
  {

    writeScratchPad(_owROMCode, 0x50, 0x00, 0x5F); // write ScratchPad with Th=80°C, Tl=0°C, Config 11bit resolution
    if (!readScratchPad(_owROMCode, data))
      return;                   // if scratchPad read failed then stop
    copyScratchPad(_owROMCode); // so we finally can copy scratchpad to memory
  }

  _tempSensorFound = true;
}

//------------------------------------------
// function that return true if SingleDS18B20 is ready for temperature reading
bool SingleDS18B20::getReady()
{
  return _tempSensorFound;
}

//------------------------------------------
// function that get temperature from a DS18X20 (run convertion, get scratchpad then calculate temperature)
float SingleDS18B20::readTemp()
{

  if (!getReady())
    return NAN;

  byte data[9];

  startConvertT(_owROMCode);

  // wait for conversion end (DS18B20 are powered)
  while (_ow.read_bit() == 0)
  {
    delay(10);
  }

  // if read of scratchpad failed (implicit 3 times) then return special fake value
  if (!readScratchPad(_owROMCode, data))
    return NAN;

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (_owROMCode[0] == 0x10)
  {                 // type S temp Sensor
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10)
    {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  }
  else
  {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return (float)raw / 16.0;
}
