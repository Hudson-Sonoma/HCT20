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


class device_config {
public:
  uint8_t  version = 0x01;
  uint8_t  flags   = 0x00;
  uint16_t counts_per_50_hz_cycle = 0;
  uint16_t counts_per_60_hz_cycle = 0;
  uint16_t count_cal_phase_A = 60;
  uint16_t count_cal_phase_B = 60;
  uint16_t count_cal_phase_C = 60;
  uint16_t seconds_per_sample= 1;
  uint8_t  reserved = 0x00;
  uint8_t  crc = 0x00;

  device_config() {
  }
  // needs a 15 byte buffer
  void get_bytes(uint8_t *bytes);

  // needs a 16 byte buffer, last byte has crc
  void get_bytes_with_crc(uint8_t *bytes);

  // needs a 16 byte buffer, last byte has crc
  bool load_from_bytes(uint8_t *bytes) ;

  void print_summary() ;

};


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

  uint16_t getFWVersion();
  float    getCurrent();
  float    getCurrent_A();
  float    getCurrent_B();
  float    getCurrent_C();
  float    getPF();
  float   getPF_A();
  float   getPF_B();
  float   getPF_C();
<<<<<<< HEAD
  uint16_t getPhaseCount_A();
  uint16_t getPhaseCount_B();
  uint16_t getPhaseCount_C();
  uint16_t getDeviceTemperatureK();
=======
>>>>>>> bc1e099 (report per channel data)

  bool reset();

  bool writeConfig(device_config *config);
  bool readConfig(device_config *config);

  uint8_t   _error;
  private:
  bool      writeCmd(uint8_t cmd);
  bool      writeCmd(uint8_t cmd, uint8_t value);
  bool      readBytes(uint8_t n, uint8_t *val, uint8_t maxDuration);
  bool      writeBytes(uint8_t numBytes, uint8_t *data);
  TwoWire* _wire;

  uint16_t  _rawHumidity;
  uint16_t  _rawTemperature;

<<<<<<< HEAD
   uint16_t _fw_version;
=======
>>>>>>> bc1e099 (report per channel data)
   uint16_t _raw_A_current;
   uint16_t _raw_B_current;
   uint16_t _raw_C_current;
   uint16_t _raw_A_pf;
   uint16_t _raw_B_pf;
   uint16_t _raw_C_pf;
<<<<<<< HEAD
   uint16_t _phase_count_A;
   uint16_t _phase_count_B;
   uint16_t _phase_count_C;
   uint16_t _device_temperature_kelvin;
=======
>>>>>>> bc1e099 (report per channel data)

  uint8_t   _status;

  uint8_t   _resolution;
};

uint8_t   crc8(const uint8_t *data, uint8_t len);
