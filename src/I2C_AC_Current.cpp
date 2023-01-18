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
  #define CHANNEL_DATA_LEN (12*2) // 12 uint16_t in 24 bytes
  uint8_t buffer[CHANNEL_DATA_LEN];

  //  TEMPERATURE
  writeCmd(SHT2x_GET_TEMPERATURE_NO_HOLD);
  //delay(85);  Not needed for HCT20

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

  //  HUMIDITY
  writeCmd(SHT2x_GET_HUMIDITY_NO_HOLD);
  //delay(29);  Not needed for HCT20

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


  writeCmd(0xD3);
  if (readBytes(CHANNEL_DATA_LEN, (uint8_t*) &buffer[0], 30) == false)
  {
    return false;
  }
  if (crc8(buffer, CHANNEL_DATA_LEN-2) != buffer[CHANNEL_DATA_LEN-1])
  {
    _error = SHT2x_ERR_CRC_HUM;
    //Serial.print("CRC error: calc_crc16: "); Serial.print(crc8(buffer, CHANNEL_DATA_LEN-2)); Serial.print(" xmit_crc16: "); Serial.println(buffer[CHANNEL_DATA_LEN-1]);
    // print serial buffer
    // for (int i = 0; i < CHANNEL_DATA_LEN; i++) {
    //   Serial.print(buffer[i], HEX); Serial.print(" ");
    // }
    // Serial.println();
    return false;  
  }
  _fw_version = buffer[0] << 8;
  _fw_version += buffer[1];
  _raw_A_current = buffer[2] << 8;
  _raw_A_current += buffer[3];
  _raw_B_current = buffer[4] << 8;
  _raw_B_current += buffer[5];
  _raw_C_current = buffer[6] << 8;
  _raw_C_current += buffer[7];
  _raw_A_pf = buffer[8] << 8;
  _raw_A_pf += buffer[9];
  _raw_B_pf = buffer[10] << 8;
  _raw_B_pf += buffer[11];
  _raw_C_pf = buffer[12] << 8;
  _raw_C_pf += buffer[13];
  _phase_count_A = buffer[14] << 8;
  _phase_count_A += buffer[15];
  _phase_count_B = buffer[16] << 8;
  _phase_count_B += buffer[17];
  _phase_count_C = buffer[18] << 8;
  _phase_count_C += buffer[19];
  _device_temperature_kelvin = buffer[20] << 8;
  _device_temperature_kelvin += buffer[21];

  _error = SHT2x_OK;
  return true;
}

uint16_t AC_Current::getFWVersion()
{
  return _fw_version;
}

float AC_Current::getCurrent()
{
  // par 6.2
  return -46.85 + (175.72 / 65536.0) * _rawTemperature;
}
float AC_Current::getCurrent_A()
{
  return -46.85 + (175.72 / 65536.0) * _raw_A_current;
}
float AC_Current::getCurrent_B()
{
  return -46.85 + (175.72 / 65536.0) * _raw_B_current;
}
float AC_Current::getCurrent_C()
{
  return -46.85 + (175.72 / 65536.0) * _raw_C_current;
}


float AC_Current::getPF()
{
  // par  6.1
  return (-6.0 + (125.0 / 65536.0) * _rawHumidity)/100.0;
}
float AC_Current::getPF_A()
{
  return (-6.0 + (125.0 / 65536.0) * _raw_A_pf)/100.0;
}
float AC_Current::getPF_B()
{
  return (-6.0 + (125.0 / 65536.0) * _raw_B_pf)/100.0;
}
float AC_Current::getPF_C()
{
  return (-6.0 + (125.0 / 65536.0) * _raw_C_pf)/100.0;
}

uint16_t AC_Current::getPhaseCount_A()
{
  return _phase_count_A;
}
uint16_t AC_Current::getPhaseCount_B()
{
  return _phase_count_B;
}
uint16_t AC_Current::getPhaseCount_C()
{
  return _phase_count_C;
}

uint16_t AC_Current::getDeviceTemperatureK()
{
  return _device_temperature_kelvin;
}


bool AC_Current::reset()
{
  bool b = writeCmd(SHT2x_SOFT_RESET);
  return b;
}


bool AC_Current::readConfig(device_config *config)
{
  writeCmd(0xD0);
  uint8_t buf[16];
  if (readBytes(16, buf, 30) == false)
  {
    _error = 1;
    return false;
  }
  if (crc8( buf, 15) != buf[15])
  {
    _error = SHT2x_ERR_CRC_HUM;
    return false;  
  }
  if (!config->load_from_bytes(buf)) {
    _error = 2;
    return false;
  }
  return true;
}

bool AC_Current::writeConfig(device_config *config)
{
  uint8_t buf[16];
  config->get_bytes(buf);
  buf[15] = crc8(buf, 15);
  writeCmd(0xD1);
  if (writeBytes(16, buf) == false)
  {
    _error = 1;
    return false;
  }
  return true;
}


// {
//   writeCmd(0xD0);
//   if (readBytes(sizeof(ac_current_config_t), (uint8_t*) config, 30) == false)
//   {
//     return false;
//   }
//   if (crc8(config, 9) != config.crc)
//   {
//     _error = SHT2x_ERR_CRC_HUM;
//     return false;  
//   }
//   return true;
// }


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

bool AC_Current::writeBytes(uint8_t numBytes, uint8_t *data)
{
  _wire->beginTransmission(SHT2x_ADDRESS);
  for (uint8_t i = 0; i < numBytes; i++)
  {
    _wire->write(data[i]);
  }
  if (_wire->endTransmission() != 0)
  {
    _error = 10;
    return false;
  }
  return true;
}


uint8_t crc8(const uint8_t *data, uint8_t len)
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


  // needs a 15 byte buffer
  void device_config::get_bytes(uint8_t *bytes) {
    //memcpy(bytes, this, sizeof(device_config));
    bytes[0] = version;
    bytes[1] = flags;
    bytes[2] = counts_per_50_hz_cycle >> 8;
    bytes[3] = counts_per_50_hz_cycle & 0xFF;
    bytes[4] = counts_per_60_hz_cycle >> 8;
    bytes[5] = counts_per_60_hz_cycle & 0xFF;
    bytes[6] = count_cal_phase_A >> 8;
    bytes[7] = count_cal_phase_A & 0xFF;
    bytes[8] = count_cal_phase_B >> 8;
    bytes[9] = count_cal_phase_B & 0xFF;
    bytes[10] = count_cal_phase_C >> 8;
    bytes[11] = count_cal_phase_C & 0xFF;
    bytes[12] = seconds_per_sample >> 8;
    bytes[13] = seconds_per_sample & 0xFF;
    bytes[14] = reserved;
  }

  // needs a 16 byte buffer, last byte has crc
  void device_config::get_bytes_with_crc(uint8_t *bytes) {
    get_bytes(bytes);
    bytes[15] = crc8(bytes, 15);
  }

  // needs a 16 byte buffer, last byte has crc
  bool device_config::load_from_bytes(uint8_t *bytes) {
    if (crc8(bytes, 15) == bytes[15]) {
      if (bytes[0] != version) {
        Serial.println("device_config: version mismatch");
        return false;
      }
      version = bytes[0];
      flags = bytes[1];
      counts_per_50_hz_cycle = (bytes[2] << 8) | bytes[3];
      counts_per_60_hz_cycle = (bytes[4] << 8) | bytes[5];
      count_cal_phase_A = (bytes[6] << 8) | bytes[7];
      count_cal_phase_B = (bytes[8] << 8) | bytes[9];
      count_cal_phase_C = (bytes[10] << 8) | bytes[11];
      seconds_per_sample = (bytes[12] << 8) | bytes[13];
      reserved = bytes[14];
      crc = bytes[15];
      return true;
    }
    return false;
  }

  void device_config::print_summary() {
            Serial.print(version); Serial.print(" "); Serial.print(counts_per_50_hz_cycle); Serial.print(" "); Serial.print(counts_per_60_hz_cycle); Serial.print(" ");
            Serial.print(count_cal_phase_A); Serial.print(" "); Serial.print(count_cal_phase_B); Serial.print(" "); Serial.print(count_cal_phase_C); Serial.print(" ");
            Serial.print(seconds_per_sample); 
  }


