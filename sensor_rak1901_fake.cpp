/**
 * @file sensor_rak1901_fake.cpp
 * @author Raul Ceron (recerpin@posgrado.upv.es)
 * @brief
 * @version 0.1
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <stdint.h>
#include "sensor_rak1901.hpp"

#define TEMPERATURE_MIN 12.7
#define TEMPERATURE_MAX 37.2
#define TEMPERATURE_STEP 0.06

#define HUMIDITY_MIN 45.5
#define HUMIDITY_MAX 80.2
#define HUMIDITY_STEP 0.65

#ifndef RAK1901_REAL

void rak1901_Init(void)
{
}

float temperature_Read(void)
{
  static float temp = TEMPERATURE_MIN;
  static uint8_t direction = 1; // 1 = up, 0 = down

  if (direction == 1)
  {
    temp += TEMPERATURE_STEP;
    if (temp >= TEMPERATURE_MAX)
    {
      temp = TEMPERATURE_MAX;
      direction = 0;
    }
  }
  else
  {
    temp -= TEMPERATURE_STEP;
    if (temp <= TEMPERATURE_MIN)
    {
      temp = TEMPERATURE_MIN;
      direction = 1;
    }
  }
  return temp;
}

float humidity_Read(void)
{
  static float temp = HUMIDITY_MIN;
  static uint8_t direction = 1; // 1 = up, 0 = down

  if (direction == 1)
  {
    temp += HUMIDITY_STEP;
    if (temp >= HUMIDITY_MAX)
    {
      temp = HUMIDITY_MAX;
      direction = 0;
    }
  }
  else
  {
    temp -= HUMIDITY_STEP;
    if (temp <= HUMIDITY_MIN)
    {
      temp = HUMIDITY_MIN;
      direction = 1;
    }
  }
  return temp;
}

#endif
