#ifndef SENSORS_H
#define SENSORS_H

#include <SharpDistSensor.h>

#include "SharpIR.h"
#include "globals.h"

#ifndef NO_HC_SR04
float HC_SR04_range();
#endif

// void Analog_Range_A4();

// // FRONT_LEFT_IR
uint16_t FRONT_LEFT_shortIR_reading();

// // FRONT_RIGHT_IR
uint16_t FRONT_RIGHT_shortIR_reading();

// // BACK_LEFT_IR
uint16_t BACK_LEFT_longIR_reading();

// // BACK_RIGHT_IR
uint16_t BACK_RIGHT_longIR_reading();

#ifndef NO_READ_GYRO
void GYRO_reading(double T);
#endif

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK();
#endif

#endif  // SENSORS_H
