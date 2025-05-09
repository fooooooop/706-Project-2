#include "positioning_system.h"

#include <math.h>

#include "sensors.h"

const float IR_NEAR_THRESHOLD = 15.0;     // cm
const float ULTRASONIC_THRESHOLD = 50.0;  // cm
const float OFFSET_IR_LEFT = 3.0;         // cm
const float OFFSET_IR_RIGHT = 3.0;        // cm
const float ROBOT_FRONT_OFFSET = 11.5;
const float ROBOT_BACK_OFFSET = 11.5;     // cm
const float IR_SENSOR_SEPARATION = 10.0;  // cm

void initPositioning() {
  // TO DO
}

Position updatePosition() {
  Position pos;

  float irFL = FRONT_LEFT_shortIR_reading();
  float irFR = FRONT_RIGHT_shortIR_reading();
  float irBL = BACK_LEFT_longIR_reading();
  float irBR = BACK_RIGHT_longIR_reading();
  float ultra = HC_SR04_range();

  //-- Y COORDINATE --//
  // If the ultrasonic reading is below our threshold, we assume a front wall is
  // present.
  bool frontClose = (ultra < ULTRASONIC_THRESHOLD);
  pos.frontClose = frontClose;

  // If a front wall is detected, use the ultrasonic reading (plus a front
  // offset) to estimate the distance from the front wall. Otherwise, use the
  // ultrasonic reading (plus a back offset) to estimate the distance from the
  // back wall.
  pos.y = frontClose ? ultra + ROBOT_FRONT_OFFSET
                     : TABLE_LENGTH - ROBOT_BACK_OFFSET;

  //-- X COORDINATE --//
  // Determine which side is near a wall by checking the IR sensor readings.
  bool leftNear = (irFL < IR_NEAR_THRESHOLD) || (irBL < IR_NEAR_THRESHOLD);
  bool rightNear = (irFR < IR_NEAR_THRESHOLD) || (irBR < IR_NEAR_THRESHOLD);

  if (leftNear && !rightNear) {
    // Near left wall: x is the average left sensor reading plus an offset.
    float leftAvg = (irFL + irBL) / 2.0;
    pos.x = leftAvg + OFFSET_IR_LEFT;
  } else if (rightNear && !leftNear) {
    // Near right wall: x is computed as table width minus the average right
    // sensor reading plus offset.
    float rightAvg = (irFR + irBR) / 2.0;
    pos.x = TABLE_WIDTH - (rightAvg + OFFSET_IR_RIGHT);
  } else if (leftNear && rightNear) {
    // In a corner: average the two estimates.
    float leftEst = (irFL + irBL) / 2.0 + OFFSET_IR_LEFT;
    float rightEst = TABLE_WIDTH - ((irFR + irBR) / 2.0 + OFFSET_IR_RIGHT);
    pos.x = (leftEst + rightEst) / 2.0;
  } else {
    // If neither side is clearly near a wall, default to the middle of the
    // table.
    pos.x = TABLE_WIDTH / 2.0;
  }

  //-- THETA --//
  if (leftNear) {
    // For the left side: if the front left reads further than the back left,
    // the robot is rotated away from the wall.
    float diff = irFL - irBL;
    pos.theta = atan2(diff, IR_SENSOR_SEPARATION) * (180.0 / PI);
  } else if (rightNear) {
    // For the right side, invert the sign so that positive theta means a
    // similar rotation as on the left.
    float diff = irFR - irBR;
    pos.theta = -atan2(diff, IR_SENSOR_SEPARATION) * (180.0 / PI);
  } else {
    pos.theta = 0;
  }

  return pos;
}
