/**
 * @file sensor_rak1901.cpp
 * @author Raul Ceron (recerpin@posgrado.upv.es)
 * @brief
 * @version 0.1
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Arduino.h>
#include <stdint.h>
#include "sensor_rak1901.hpp"

#ifdef RAK1901_REAL

#include "rak1901.h"

#define TEMPERATURE_MIN 12.7
#define HUMIDITY_MIN 45.5
rak1901 rak1901;

void rak1901_Init(void)
{
  // begin for I2C
  Wire.begin();

  // check if sensor Rak1901 is working
  Serial.printf("RAK1901 init %s\r\n", rak1901.init() ? "Success" : "Fail");
}

float temperature_Read(void)
{
  static float temp = TEMPERATURE_MIN;

  if (rak1901.update())
  {
    temp = rak1901.temperature();
  }
  else
  {
    Serial.println("Please plug in the sensor RAK1901 and Reboot");
  }
  return temp;
}

float humidity_Read(void)
{
  static float humidity = HUMIDITY_MIN;

  if (rak1901.update())
  {
    humidity = rak1901.humidity();
  }
  else
  {
    Serial.println("Please plug in the sensor RAK1901 and Reboot");
  }
  return humidity;
}

#endif
