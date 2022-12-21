//
//    FILE: AC_Current.h
//  AUTHOR: Rob Tillaart, Viktor Balint, Tim Bishop
// VERSION: 0.1.0
//    DATE: 2022-10-28
// PURPOSE: Arduino library for the Grove I2C AC Current Sensor
//     URL:
//

#include "I2C_AC_Current.h"

//  SUPPORTED COMMANDS
#define SHT2x_GET_TEMPERATURE_NO_HOLD   0xF3
#define SHT2x_GET_HUMIDITY_NO_HOLD      0xF5
#define SHT2x_SOFT_RESET                0xFE
#define SHT2x_WRITE_USER_REGISTER       0xE6
#define SHT2x_READ_USER_REGISTER        0xE7

#define SHT2x_ADDRESS                   0x40

AC_Current::AC_Current()
{
  _rawTemperature = 0;
  _rawHumidity    = 0;
  _error          = SHT2x_OK;
  _status         = 0;
  _resolution     = 0;
}

#if defined(ESP8266) || defined(ESP32)
bool AC_Current::begin(const int dataPin, const int clockPin)
{
  _wire = &Wire;
  if ((dataPin < 255) && (clockPin < 255))
  {
    _wire->begin(dataPin, clockPin);
  } else {
    _wire->begin();
  }
  return reset();
}
#endif


bool AC_Current::begin(TwoWire *wire)
{
  _wire = wire;
  return reset();
}


bool AC_Current::isConnected()
{
  _wire->beginTransmission(SHT2x_ADDRESS);
  int rv = _wire->endTransmission();
  if (rv != 0) _error = SHT2x_ERR_NOT_CONNECT;
  return (rv == 0);
}


bool AC_Current::read()
{
  uint8_t buffer[3];

  //  TEMPERATURE
  writeCmd(SHT2x_GET_TEMPERATURE_NO_HOLD);
  delay(85); 

  if (readBytes(3, (uint8_t*) &buffer[0], 90) == false)
  {
    _error = SHT2x_ERR_READBYTES;
    return false;
  }
  if (crc8(buffer, 2) != buffer[2])
  {
    _error = SHT2x_ERR_CRC_TEMP;
  //  return false;  // do not fail yet
  }
  _rawTemperature  = buffer[0] << 8;
  _rawTemperature += buffer[1];
  _rawTemperature &= 0xFFFC;

  _status = buffer[1] & 0x0003;
  if (_status == 0xFF)  // TODO  != 0x01  (need HW to test)
  {
    _error = SHT2x_ERR_READBYTES;
    return false;
  }
  
  //  HUMIDITY
  writeCmd(SHT2x_GET_HUMIDITY_NO_HOLD);
  delay(29); 

  if (readBytes(3, (uint8_t*) &buffer[0], 30) == false)
  {
    return false;
  }
  if (crc8(buffer, 2) != buffer[2])
  {
    _error = SHT2x_ERR_CRC_HUM;
  //    return false;  // do not fail yet
  }
  _rawHumidity  = buffer[0] << 8;
  _rawHumidity += buffer[1];
  _rawHumidity &= 0xFFFC;     //  TODO is this mask OK? as humidity is max 12 bit..

  _status = buffer[1] & 0x0003;
  if (_status == 0xFF)        //  TODO  != 0x02  (need HW to test)
  {
    _error = SHT2x_ERR_READBYTES;
    return false;
  }

  _error = SHT2x_OK;
  return true;
}


float AC_Current::getCurrent()
{
  // par 6.2
  return -46.85 + (175.72 / 65536.0) * _rawTemperature;
}


float AC_Current::getPF()
{
  // par  6.1
  return -6.0 + (125.0 / 65536.0) * _rawHumidity;
}


bool AC_Current::reset()
{
  bool b = writeCmd(SHT2x_SOFT_RESET);
  return b;
}


//////////////////////////////////////////////////////////
//
//  PRIVATE
//
uint8_t AC_Current::crc8(const uint8_t *data, uint8_t len)
{
  // CRC-8 formula from page 14 of SHT spec pdf
  // Sensirion_Humidity_Sensors_SHT2x_CRC_Calculation.pdf
  const uint8_t POLY = 0x31;
  uint8_t crc = 0x00;

  for (uint8_t j = len; j; --j)
  {
    crc ^= *data++;

    for (uint8_t i = 8; i; --i)
    {
      crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
    }
  }
  return crc;
}


bool AC_Current::writeCmd(uint8_t cmd)
{
  _wire->beginTransmission(SHT2x_ADDRESS);
  _wire->write(cmd);
  if (_wire->endTransmission() != 0)
  {
    _error = SHT2x_ERR_WRITECMD;
    return false;
  }
  return true;
}


bool AC_Current::writeCmd(uint8_t cmd, uint8_t value)
{
  _wire->beginTransmission(SHT2x_ADDRESS);
  _wire->write(cmd);
  _wire->write(value);
  if (_wire->endTransmission() != 0)
  {
    _error = SHT2x_ERR_WRITECMD;
    return false;
  }
  return true;
}


bool AC_Current::readBytes(uint8_t n, uint8_t *val, uint8_t maxDuration)
{
  _wire->requestFrom((uint8_t)SHT2x_ADDRESS, (uint8_t) n);
  uint32_t start = millis();
  while (_wire->available() < n)
  {
    if (millis() - start > maxDuration)
    {
      _error = SHT2x_ERR_READBYTES;
      return false;
    }
    yield();
  }

  for (uint8_t i = 0; i < n; i++)
  {
    val[i] = _wire->read();
  }
  _error = SHT2x_OK;
  return true;
}
