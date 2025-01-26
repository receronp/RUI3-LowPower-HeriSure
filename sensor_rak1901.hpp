/**
 * @file sensor_rak1901.hpp
 * @author Raul Ceron (recerpin@posgrado.upv.es)
 * @brief
 * @version 0.1
 * @date 2025-01-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#define RAK1901_REAL // comment to use the "fake" module

void rak1901_Init(void);
float temperature_Read(void);
float humidity_Read(void);

#endif // TEMPERATURE_H
