#include "Ponoor_L6470Library.h"
#include "prephral_config.hpp"

int AutoDriver::_numBoards;

// Constructors
AutoDriver::AutoDriver(int position, int CSPin, int resetPin, int busyPin) {
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = busyPin;
  _numBoards++;
}

AutoDriver::AutoDriver(int position, int CSPin, int resetPin) {
  _CSPin = CSPin;
  _position = position;
  _resetPin = resetPin;
  _busyPin = -1;
  _numBoards++;
}

void AutoDriver::SPIPortConnect(ConcreteSPI *SPIPort) { _SPI = SPIPort; }

int AutoDriver::busyCheck(void) {
  if (_busyPin == -1) {
    if (getParam(REG_STATUS) & 0x0002)
      return 0;
    else
      return 1;
  } else {
    if (gpio.digitalRead(_busyPin) == HIGH)
      return 0;
    else
      return 1;
  }
}
