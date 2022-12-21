#pragma once
//
//    FILE: I2C_AC_Current.h
//  AUTHOR: Rob Tillaart, Viktor Balint, Tim Bishop
// VERSION: 0.1.0
//    DATE: 2022-10-28
// PURPOSE: Arduino library for the Grove I2C AC Current Sensor
//     URL:
//    BASE: Based on SHT20 temperature sensor driver https://github.com/RobTillaart/SHT2x


#include <Arduino.h>
#include <Wire.h>

//  error codes
//  kept in sync with SHT31 library
#define SHT2x_OK                      0x00
#define SHT2x_ERR_WRITECMD            0x81
#define SHT2x_ERR_READBYTES           0x82
#define SHT2x_ERR_NOT_CONNECT         0x84
#define SHT2x_ERR_CRC_TEMP            0x85
#define SHT2x_ERR_CRC_HUM             0x86
#define SHT2x_ERR_CRC_STATUS          0x87 
#define SHT2x_ERR_RESOLUTION          0x8A

class AC_Current
{
public:
  AC_Current();

#if defined(ESP8266) || defined(ESP32)
  bool begin(const int dataPin, const int clockPin);
#endif
  bool begin(TwoWire *wire);

  //  check sensor is reachable over I2C
  bool isConnected();

  //  read must be called get getTemperature / getHumidity
  bool read();

  float    getCurrent();
  float    getPF();

    //  might take up to 15 milliseconds.
  bool reset();

  private:
  uint8_t   crc8(const uint8_t *data, uint8_t len);

  bool      writeCmd(uint8_t cmd);
  bool      writeCmd(uint8_t cmd, uint8_t value);
  bool      readBytes(uint8_t n, uint8_t *val, uint8_t maxDuration);
  TwoWire* _wire;

  uint16_t  _rawHumidity;
  uint16_t  _rawTemperature;

  uint8_t   _status;

  uint8_t   _error;
  uint8_t   _resolution;
};
