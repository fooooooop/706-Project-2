#ifndef UTILITIES_H
#define UTILITIES_H

#include "globals.h"
#include "sensors.h"

void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
void speed_change_smooth();
double GYRO_controller(double gyro_target, double kp, double ki, double kd);
double PT_controller(double kp, double ki, double kd);
double IR_controller(double IR_target, enum DRIVE IR_mode, enum DIRECTION left_right, double kp, double ki, double kd);
void fan_on();


#endif  // UTILITIES_H
