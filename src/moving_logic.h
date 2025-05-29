#ifndef MOVING_LOGIC_H
#define MOVING_LOGIC_H

#include "globals.h"
#include "utilities.h"
#include "sensors.h"

// Motor control functions
void enable_motors();
void disable_motors();
void stop_motors();
void forward();
void reverse();
void ccw();
void cw();
void strafe_left();
void strafe_right();

// Custom functions
bool find_light();
void rotate_findlight();
void forward_light(double angle_target);
void turn_angle(double target);
void avoid_obstacle(double angle_target, bool *leftside, bool *rightside);

#endif  // MOVING_LOGIC_H
