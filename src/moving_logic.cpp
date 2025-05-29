#include "moving_logic.h"

#define PT_KP 0.15
#define PT_KI 0.0
#define PT_KD 0.0
#define GYRO_KP 0.0
#define GYRO_KI 0.0
#define GYRO_KD 0.0
#define AVOID_KP 0.15
#define AVOID_KI 0.0
#define AVOID_KD 0.0

#define OBSTACLE_DETECT 150

void enable_motors() {
  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_front_motor.attach(right_front);
}

void disable_motors() {
  left_front_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_front_motor.detach();

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void stop_motors() {
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void forward() {
  while (HC_SR04_range() > 55) {
    GYRO_controller(0, 0, 0, 0);
    left_front_motor.writeMicroseconds(1500 + speed_val + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + gyro_u);
  }

  // Stop Motor ----//
  stop_motors();
}

void reverse() {
  while (Serial.read() != 'v') {
    GYRO_controller(0, 20.5, 0, 0);
    left_front_motor.writeMicroseconds(1500 - speed_val + gyro_u);
    left_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u);
    right_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u);
    right_front_motor.writeMicroseconds(1500 + speed_val + gyro_u);

    // if (((double)FRONT_LEFT_shortIR_reading() < OBSTACLE_DETECT) ||
    //     ((double)FRONT_RIGHT_shortIR_reading() < OBSTACLE_DETECT) ||
    //     ((double)HC_SR04_range() < OBSTACLE_DETECT))
    //   avoid_obstacle();
  }

  // Stop Motor ----//
  stop_motors();
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  // left_front_motor.writeMicroseconds(1500 - speed_val);
  // left_rear_motor.writeMicroseconds(1500 + speed_val);
  // right_rear_motor.writeMicroseconds(1500 + speed_val);
  // right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  // left_front_motor.writeMicroseconds(1500 + speed_val);
  // left_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_rear_motor.writeMicroseconds(1500 - speed_val);
  // right_front_motor.writeMicroseconds(1500 + speed_val);
}

//-------------------------------------------------//
//------------Custom Functions---------------------//
//-------------------------------------------------//

void turn_angle(double target) {
  bool gyro_exit = false;
  bool gyro_timestart = false;
  double gyro_timer = 0;
  double gyro_err_pos;
  double gyro_bounds = 10;

  while (gyro_exit == false) {
    gyro_err_pos = GYRO_controller(target, 4.75, 0, 0);

    // Send Power to Motors-----//
    left_front_motor.writeMicroseconds(1500 + gyro_u);
    left_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_rear_motor.writeMicroseconds(1500 + gyro_u);
    right_front_motor.writeMicroseconds(1500 + gyro_u);

    // Exit Condition-----//
    if ((abs(gyro_err_pos) < gyro_bounds) && (gyro_timestart != true)) {
      // Checks to see if yss is within exit threshold, start timer
      gyro_timestart = true;
      gyro_timer = millis();
    }
    if ((abs(gyro_err_pos) > gyro_bounds) && (gyro_timestart == true)) {
      // Checks to see if yss falls outside of exit threshold
      // If it does, then restart timer
      gyro_timestart = false;
    } else if ((millis() - gyro_timer > 750.0) &&
               (abs(gyro_err_pos) < gyro_bounds) && (gyro_timestart == true)) {
      // Else, if yss is within threshold for a certain amount of time (check
      // first condition), exit controller
      gyro_exit = true;
    }
  }
}

void avoid_obstacle(double angle_target) {
  stop_motors();

  int side_detect = 100;

  // Initialize readings so they'll only be read once
  double US_reading;
  double IR_FRONTRIGHT_reading;
  double IR_FRONTLEFT_reading;
  double IR_BACKRIGHT_reading;
  double IR_BACKLEFT_reading;

  if (!((double)BACK_RIGHT_longIR_reading() < side_detect)) {  // Obstacle DOES NOT exist on right side

    do {  // Strafe Right until front doesn't see obstacle
      // Sensor Readings
      US_reading = (double)HC_SR04_range();
      IR_FRONTRIGHT_reading = (double)FRONT_RIGHT_shortIR_reading();
      IR_FRONTLEFT_reading = (double)FRONT_LEFT_shortIR_reading();
      IR_BACKRIGHT_reading = (double)BACK_RIGHT_longIR_reading();

      // If an obstacle suddenly appears on the right.
      if (IR_BACKRIGHT_reading < side_detect) break;

      // Start Strafing------------//
      GYRO_controller(angle_target, GYRO_KP, GYRO_KI, GYRO_KD);
      PT_controller(PT_KP, PT_KI, PT_KD);
      // AVOID controller is linked to avoid_obstacle()
      AVOID_controller(OBSTACLE_DETECT, US_reading, IR_FRONTRIGHT_reading, IR_FRONTLEFT_reading, AVOID_KP, AVOID_KI, AVOID_KD);

      left_front_motor.writeMicroseconds(1500 + gyro_u + PT_u - AVOID_u + speed_val);
      left_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u - AVOID_u - speed_val);
      right_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u + AVOID_u - speed_val);
      right_front_motor.writeMicroseconds(1500 + gyro_u + PT_u + AVOID_u + speed_val);

    } while ((IR_FRONTLEFT_reading < OBSTACLE_DETECT) ||
             (IR_FRONTRIGHT_reading < OBSTACLE_DETECT) ||
             (US_reading < OBSTACLE_DETECT));

  } else if (!((double)BACK_LEFT_longIR_reading() < side_detect)) {  // Obstacle DOES NOT exist on left side

    do {  // Strafe Left until front doesn't see obstacle
      // Sensor Readings
      US_reading = (double)HC_SR04_range();
      IR_FRONTRIGHT_reading = (double)FRONT_RIGHT_shortIR_reading();
      IR_FRONTLEFT_reading = (double)FRONT_LEFT_shortIR_reading();
      IR_BACKLEFT_reading = (double)BACK_LEFT_longIR_reading();

      // If an obstacle suddenly appears on the left.
      if (IR_BACKLEFT_reading < side_detect) break;

      // Start Strafing------------//
      GYRO_controller(angle_target, GYRO_KP, GYRO_KI, GYRO_KD);
      PT_controller(PT_KP, PT_KI, PT_KD);
      // AVOID controller is linked to avoid_obstacle()
      AVOID_controller(OBSTACLE_DETECT, US_reading, IR_FRONTRIGHT_reading, IR_FRONTLEFT_reading, AVOID_KP, AVOID_KI, AVOID_KD);

      left_front_motor.writeMicroseconds(1500 + gyro_u + PT_u - AVOID_u - speed_val);
      left_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u - AVOID_u + speed_val);
      right_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u + AVOID_u + speed_val);
      right_front_motor.writeMicroseconds(1500 + gyro_u + PT_u + AVOID_u - speed_val);

    } while ((IR_FRONTLEFT_reading < OBSTACLE_DETECT) ||
             (IR_FRONTRIGHT_reading < OBSTACLE_DETECT) ||
             (US_reading < OBSTACLE_DETECT));
  }
}

void forward_light(double angle_target) {
  float detection_threshold =
      600;  // light is clearly detected when the PT reading is 50.

  // Initialize readings so they'll only be read once
  double US_reading;
  double IR_FRONTRIGHT_reading;
  double IR_FRONTLEFT_reading;

  uint16_t PT_FRONTRIGHT_reading;
  uint16_t PT_FRONTLEFT_reading;

  while (1) {
    GYRO_controller(angle_target, GYRO_KP, GYRO_KI, GYRO_KD);
    PT_controller(PT_KP, PT_KI, PT_KD);

    // Sensor Readings
    US_reading = (double)HC_SR04_range();
    IR_FRONTRIGHT_reading = (double)FRONT_RIGHT_shortIR_reading();
    IR_FRONTLEFT_reading = (double)FRONT_LEFT_shortIR_reading();
    PT_FRONTRIGHT_reading = FRONT_RIGHT_PT_reading();
    PT_FRONTLEFT_reading = FRONT_LEFT_PT_reading();

    // Break Loop Conditions
    // if ((PT_FRONTRIGHT_reading > detection_threshold) && (US_reading <
    // OBSTACLE_DETECT)) break; if ((PT_FRONTLEFT_reading > detection_threshold)
    // && (US_reading < OBSTACLE_DETECT)) break;
    if (((PT_FRONTRIGHT_reading + PT_FRONTLEFT_reading) / 2.0 >
         detection_threshold) &&
        (US_reading < OBSTACLE_DETECT))
      break;

    // Avoid Obstacle
    if (((IR_FRONTLEFT_reading < OBSTACLE_DETECT) ||
         (IR_FRONTRIGHT_reading < OBSTACLE_DETECT) ||
         (US_reading < OBSTACLE_DETECT)) &&
        !((PT_FRONTRIGHT_reading + PT_FRONTLEFT_reading) / 2.0 >
          detection_threshold))
      avoid_obstacle(angle_target);

    // Move Towards Light
    left_front_motor.writeMicroseconds(1500 + speed_val + gyro_u + PT_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + PT_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + PT_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + gyro_u + PT_u);
  }

  // Stop Motor ----//
  stop_motors();
  delay(500);
}

bool find_light() {
  float detection_threshold =
      50;  // light is clearly detected when the PT reading is 50.

  bool front_left_detected;  // 0 when the front left PT is not detecting light, 1 if it is
  bool front_right_detected;  // 0 when the front right PT is not detecting light, 1 if it is

  // Step 1: Check which PTs detect light
  front_left_detected = (FRONT_LEFT_PT_reading() > detection_threshold) ? 1 : 0;
  front_right_detected = (FRONT_RIGHT_PT_reading() > detection_threshold) ? 1 : 0;

  // Step 2: If no PT can see light, CW until the front right and front left
  // ones can.
  if (!front_left_detected && !front_right_detected) {  // This part CW until the front PTs detect light
    while (!(front_left_detected && front_right_detected)) {
      front_left_detected = (FRONT_LEFT_PT_reading() > detection_threshold) ? 1 : 0;
      front_right_detected = (FRONT_RIGHT_PT_reading() > detection_threshold) ? 1 : 0;
      cw();
    }
    stop_motors();
    delay(500);
  }

  return (front_left_detected || front_right_detected);
}

void rotate_findlight() {
  // I don't think we should use this function, shit's ass.

  // Take an angle reading and "zero" the robot---//
  // Set Gyro zero voltage
  int i;
  float sum = 0;
  int sensorValue = 0;
  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    sensorValue = analogRead(A3);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
  for (int i = 1; i < 10; i++) {
    GYRO_reading(100);
  }
  currentAngle = 0;
  //----------------------------------------------//

  // Rotate Robot to "zero" angle
  turn_angle(179);
  // Start rotating to find where lights/fires are based on robot angle, will
  // use front right PT as reference
  int k = 0;
  int speed_rotate = 115;
  while (currentAngle > -160) {
    GYRO_reading(50);  // Keep track of current angle
    PT_value[k] = FRONT_RIGHT_PT_reading();
    k++;
    // Rotate ccw, sorry I didn't wanna mess with the actual function cuz I
    // wanted to control how fast it went instead of relying on speed_val
    left_front_motor.writeMicroseconds(1500 - speed_rotate);
    left_rear_motor.writeMicroseconds(1500 - speed_rotate);
    right_rear_motor.writeMicroseconds(1500 - speed_rotate);
    right_front_motor.writeMicroseconds(1500 - speed_rotate);
  }
  stop_motors();
  delay(500);

  // Find first max value in array = where *a* light/fire are
  int first_max_value = 0;
  int first_max_value_index = 0;

  // 1) Find the top two *local* peaks by value
  int idx1 = -1, idx2 = -1;  // indices of the 1st and 2nd peaks
  int val1 = -1, val2 = -1;  // their values

  for (int i = 1; i + 1 < k; ++i) {
    int v = PT_value[i];
    // is this a local peak?
    if (v > PT_value[i - 1] && v > PT_value[i + 1]) {
      if (v > val1) {
        // shift old top into second place
        val2 = val1;
        idx2 = idx1;
        // promote new top
        val1 = v;
        idx1 = i;
      } else if (v > val2) {
        val2 = v;
        idx2 = i;
      }
    }
  }

  if (idx2 < 0) {
    val1 = val2 = -1;
    idx1 = idx2 = 0;
    for (int i = 0; i < k; ++i) {
      int v = PT_value[i];
      if (v > val1) {
        val2 = val1;
        idx2 = idx1;
        val1 = v;
        idx1 = i;
      } else if (v > val2) {
        val2 = v;
        idx2 = i;
      }
    }
  }

  int first_fire_angle = 180 + 20 - round((double)idx1 / k * 360.0);
  int second_fire_angle = 180 + 20 - round((double)idx2 / k * 360.0);

  turn_angle(first_fire_angle);
  forward_light(first_fire_angle);
  return;

  //-------- OLD MAX VALUE FINDER
  // for (int i = 0; i < k; i++) {
  //   if (PT_value[i] > first_max_value){
  //     first_max_value = PT_value[i];
  //     first_max_value_index = i;
  //   }
  // }
  // PT_value[first_max_value_index] = 0; // "Erase it" to find next light/fire
  // Find second max value in array = where *another* light/fire is
  // Need to implement a better function because it might just find the first
  // light again int second_max_value = 0; int second_max_value_index = 0;

  // for (int i = 0; i < k; i++) {
  //   if (PT_value[i] > second_max_value){
  //     second_max_value = PT_value[i];
  //     second_max_value_index = i;
  //   }
  // }

  // Find angle of our fires
  // The +180 is because of how we defined our coordinate system
  // The +20 because our sensors aren't accurate lol
  // int first_fire_angle =
  //     180 + 20 - round((double)first_max_value_index / k * 360.0);
  // int second_fire_angle =
  //     180 + 20 - round((double)second_max_value_index / k * 360.0);

  // turn_angle(first_fire_angle);
  // forward_light(first_fire_angle);

  // return;
}