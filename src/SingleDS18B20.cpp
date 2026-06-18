
#include "SingleDS18B20.h"

//----------------------------------------------------------------------
// --- SingleDS18B20 Class---
//----------------------------------------------------------------------

//-----------------------------------------------------------------------
// DS18X20 Read ScratchPad command
bool SingleDS18B20::readScratchPad(uint8_t data[])
{
  // try to read scratchpad 3 times
  for (uint8_t i = 0; i < 3; i++)
  {
    // read scratchpad of the current device
    _ow.reset();
    _ow.select(_owROMCode);
    _ow.write(0xBE);                // Read ScratchPad
    for (uint8_t j = 0; j < 9; j++) // read 9 bytes
      data[j] = _ow.read();

    if (_ow.crc8(data, 8) == data[8])
      return true; // if CRC is OK then return true
  }

  return false;
}
//------------------------------------------
// DS18X20 Write ScratchPad command
void SingleDS18B20::writeScratchPad(uint8_t th, uint8_t tl, uint8_t cfg)
{

  _ow.reset();
  _ow.select(_owROMCode);
  _ow.write(0x4E); // Write ScratchPad
  _ow.write(th);   // Th 80°C
  _ow.write(tl);   // Tl 0°C
  _ow.write(cfg);  // Config
}
//------------------------------------------
// DS18X20 Copy ScratchPad command
void SingleDS18B20::copyScratchPad()
{

  _ow.reset();
  _ow.select(_owROMCode);
  _ow.write(0x48); // Copy ScratchPad
}
//------------------------------------------
// DS18X20 Start Temperature conversion
void SingleDS18B20::startConvertT()
{
  _ow.reset();
  _ow.select(_owROMCode);
  _ow.write(0x44); // start conversion
}

//------------------------------------------
// Constructor
SingleDS18B20::SingleDS18B20(uint8_t owPin) : _ow(owPin)
{

  // find first temp sensor ----------
  bool sensorLocated = false;
  _ow.reset_search();

  while (_ow.search(_owROMCode))
  {

    // if ROM received is correct or a Temperature sensor THEN we found a sensor
    if ((_ow.crc8(_owROMCode, 7) == _owROMCode[7]) && (_owROMCode[0] == 0x10 || _owROMCode[0] == 0x22 || _owROMCode[0] == 0x28))
    {
      sensorLocated = true;
      break;
    }
  }

  if (!sensorLocated)
    return;

  // configure sensor ----------
  uint8_t data[9];

  // if scratchPad read failed then stop
  if (!readScratchPad(data))
    return;

  // if config is not correct
  if (data[2] != 0x50 || data[3] != 0x00 || data[4] != 0x5F)
  {
    writeScratchPad(0x50, 0x00, 0x5F); // write ScratchPad with Th=80°C, Tl=0°C, Config 11bit resolution
    if (!readScratchPad(data))
      return;         // if scratchPad read failed then stop
    copyScratchPad(); // so we finally can copy scratchpad to memory
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

  uint8_t data[9];

  startConvertT();

  // wait for conversion end (DS18B20 are powered)
  while (_ow.read_bit() == 0)
    delay(10);

  // if read of scratchpad failed (implicit 3 times) then return NaN
  if (!readScratchPad(data))
    return NAN;

  // Convert the data to actual temperature
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
    uint8_t cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return raw / 16.0f;
}
