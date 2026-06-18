#ifndef SingleDS18B20_h
#define SingleDS18B20_h

#include <OneWire.h>

class SingleDS18B20
{
private:
  OneWire _ow;
  bool _tempSensorFound = false;
  uint8_t _owROMCode[8];

  bool readScratchPad(uint8_t data[]);
  void writeScratchPad(uint8_t th, uint8_t tl, uint8_t cfg);
  void copyScratchPad();
  void startConvertT();

public:
  SingleDS18B20(uint8_t owPin);
  bool getReady();
  float readTemp();
};

#endif
