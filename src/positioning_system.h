#ifndef POSITIONING_SYSTEM_H
#define POSITIONING_SYSTEM_H

#include <Arduino.h>

#define TABLE_WIDTH 122.0   // in cm
#define TABLE_LENGTH 198.5  // in cm

struct Position {
  float x;          // Lateral coordinate (cm)
  float y;          // Longitudinal coordinate (cm)
  float theta;      // Orientation (degrees)
  bool frontClose;  // True if ultrasonic sensor detects a front wall
};

void initPositioning();
Position updatePosition();

#endif  // POSITIONING_SYSTEM_H
