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
void find_corner();
void turn_angle(double target);
void forward_target(double target_sidewall, double target, enum DIRECTION left_right, enum SPEED boostit);
void reverse_target(double target_sidewall, double target, enum DIRECTION left_right, enum SPEED boostit);
void strafe_target(double target, enum DIRECTION left_right, enum SPEED boostit);
void forward_right();
void forward_left();

#endif  // MOVING_LOGIC_H
