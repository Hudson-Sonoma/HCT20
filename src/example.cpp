#include <Arduino.h>
#include <Wire.h>
#include "I2C_AC_Current.h"

AC_Current hct20;
device_config config;

void setup()
{
  delay(3000);
  Serial.begin(115200);
  hct20.begin(21,22);  // SDA, SCL. 21,22 for ESP32
  
  if (!hct20.readConfig(&config)) {
    Serial.print("Failed to read config via I2C: "); Serial.print(hct20._error); Serial.println();
  }
  config.print_summary();
  config.count_cal_phase_A = 90;
  config.count_cal_phase_B = 90;  // This is to calibrate the PF sensing. Set this to the getPhaseCount_B() value when you know that the PF is 1.00
  config.count_cal_phase_C = 90;

  if(!hct20.writeConfig(&config)) {
    Serial.print("Failed to write config via I2C: "); Serial.print(hct20._error); Serial.println();
  }

  Serial.println();
}

void loop()
{
   if (Serial.available() > 0) {    // as soon as the first byte is received on Serial
    char c = Serial.read();       // read the data from serial.
    if (c == 'r' || c == 'R') {

      bool success = hct20.read();
      if (!success) {
          Serial.println("Read failed");
      } else {
          Serial.print("v");
          Serial.print(hct20.getFWVersion());
          Serial.print(" Total Current: ");
          Serial.print(hct20.getCurrent());
          Serial.print(" PF: ");
          Serial.print(hct20.getPF());

          Serial.print("\tChannel Current: ");
          Serial.print(hct20.getCurrent_A()); Serial.print(",\t");
          Serial.print(hct20.getCurrent_B()); Serial.print(",\t");
          Serial.print(hct20.getCurrent_C()); Serial.print(",\t");
          Serial.print(" PF: ");
          Serial.print(hct20.getPF_A()); Serial.print(",\t");
          Serial.print(hct20.getPF_B()); Serial.print(",\t");
          Serial.print(hct20.getPF_C()); 
          Serial.print(" phase count: ");
          Serial.print(hct20.getPhaseCount_A()); Serial.print(",\t");
          Serial.print(hct20.getPhaseCount_B()); Serial.print(",\t");
          Serial.print(hct20.getPhaseCount_C()); Serial.print(" T");
          Serial.print(hct20.getDeviceTemperatureK());
          Serial.println("");
      }
    }
    if ( c == 'c') {
      hct20.readConfig(&config);
    }
    if ( c == 'C') {
      config.count_cal_phase_B = 90;  // This is to calibrate the PF sensing. Set this to the getPhaseCount_B() number when you know that the PF is 1.00
      config.print_summary();
      Serial.println();
      hct20.writeConfig(&config);
    }
   }

    delay(1000);
}


