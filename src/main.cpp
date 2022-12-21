#include <Arduino.h>
#include <Wire.h>
#include "I2C_AC_Current.h"

AC_Current hct20;

void setup()
{
  Serial.begin(115200);

  hct20.begin(21,22);  // SDA, SCL. 21,22 for ESP32

}


void loop()
{

    hct20.read();
    Serial.print("Current: ");
    Serial.print(hct20.getCurrent());
    Serial.print(" PF: ");
    Serial.println(hct20.getPF());

    delay(1000);

}


