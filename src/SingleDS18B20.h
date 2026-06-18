#ifndef SingleDS18B20_h
#define SingleDS18B20_h

#include <OneWire.h>

class SingleDS18B20
{
private:
  OneWire _ow;
  bool _tempSensorFound = false;
  byte _owROMCode[8];

  bool readScratchPad(byte data[]);
  void writeScratchPad(byte th, byte tl, byte cfg);
  void copyScratchPad();
  void startConvertT();

public:
  SingleDS18B20(uint8_t owPin);
  bool getReady();
  float readTemp();
};

#endif
