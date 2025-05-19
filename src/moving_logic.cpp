#include "moving_logic.h"

#define FORWARD_BOUND 7
#define BACKWARD_BOUND 166
#define SAMPLING_TIME 125

#define PT_KP 0.4
#define PT_KI 0.0
#define PT_KD 0.0
#define GYRO_KP 0.0
#define GYRO_KI 0.0
#define GYRO_KD 0.0

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
  while (HC_SR04_range() > 40) {
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

    // if (((double)FRONT_LEFT_shortIR_reading() < obstacle_detect) ||
    //     ((double)FRONT_RIGHT_shortIR_reading() < obstacle_detect) ||
    //     ((double)HC_SR04_range() < obstacle_detect))
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

  // Setup for check if Light/Fire is already there
  float detection_threshold = 970; // light is clearly detected in front

  if ((!((double)BACK_RIGHT_longIR_reading() < obstacle_detect)) || 
       ((double)BACK_RIGHT_longIR_reading() > 10000)) {  // Obstacle DOES NOT exist on right side
    
    do {  // Strafe Right until front doesn't see obstacle
      // Start Strafing------------//
      GYRO_controller(angle_target, GYRO_KP, GYRO_KI, GYRO_KD);
      PT_controller(PT_KP, PT_KI, PT_KD);

      left_front_motor.writeMicroseconds(1500 + gyro_u + PT_u + speed_val);
      left_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u - speed_val);
      right_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u - speed_val);
      right_front_motor.writeMicroseconds(1500 + gyro_u + PT_u + speed_val);

      // If an obstacle suddenly appears on the right.
      if ((double)BACK_RIGHT_longIR_reading() < obstacle_detect) break;

      // Check if Light/Fire is already there
      if (FRONT_RIGHT_PT_reading() > detection_threshold) return;

    } while (((double)FRONT_LEFT_shortIR_reading() < obstacle_detect) ||
             ((double)FRONT_RIGHT_shortIR_reading() < obstacle_detect) ||
             ((double)HC_SR04_range() < obstacle_detect));

  } else if ((!((double)BACK_LEFT_longIR_reading() < obstacle_detect)) ||
              ((double)BACK_LEFT_longIR_reading() > 10000)) {  // Obstacle DOES NOT exist on left side

    do {  // Strafe Left until front doesn't see obstacle
      // Start Strafing------------//
      GYRO_controller(angle_target, GYRO_KP, GYRO_KI, GYRO_KD);
      PT_controller(PT_KP, PT_KI, PT_KD);

      left_front_motor.writeMicroseconds(1500 + gyro_u + PT_u - speed_val);
      left_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u + speed_val);
      right_rear_motor.writeMicroseconds(1500 + gyro_u + PT_u + speed_val);
      right_front_motor.writeMicroseconds(1500 + gyro_u + PT_u - speed_val);

      // If an obstacle suddenly appears on the left.
      if ((double)BACK_LEFT_longIR_reading() < obstacle_detect) break;

      // Check if Light/Fire is already there
      if (FRONT_RIGHT_PT_reading() > detection_threshold) return;

    } while (((double)FRONT_LEFT_shortIR_reading() < obstacle_detect) ||
             ((double)FRONT_RIGHT_shortIR_reading() < obstacle_detect) ||
             ((double)HC_SR04_range() < obstacle_detect));
  } 
}

void forward_light(double angle_target) {
  float detection_threshold = 970; // light is clearly detected in front
  bool front_right_detected = 0;  // 0 when the front right PT is not detecting light, 1 if it is

  while (!(front_right_detected)) {
    GYRO_controller(angle_target, GYRO_KP, GYRO_KI, GYRO_KD);
    PT_controller(PT_KP, PT_KI, PT_KD);

    left_front_motor.writeMicroseconds(1500 + speed_val + gyro_u + PT_u);
    left_rear_motor.writeMicroseconds(1500 + speed_val + gyro_u + PT_u);
    right_rear_motor.writeMicroseconds(1500 - speed_val + gyro_u + PT_u);
    right_front_motor.writeMicroseconds(1500 - speed_val + gyro_u + PT_u);

    if ((((double)FRONT_LEFT_shortIR_reading() < obstacle_detect) ||
         ((double)FRONT_RIGHT_shortIR_reading() < obstacle_detect) ||
         ((double)HC_SR04_range() < obstacle_detect)) && !(front_right_detected))
      avoid_obstacle(angle_target);

    // Check which PTs detect light
    front_right_detected = (FRONT_RIGHT_PT_reading() > detection_threshold) ? 1 : 0;
  }

  // Stop Motor ----//
  stop_motors();
  delay(500);
}

bool find_light() {
  float detection_threshold = 50; // light is clearly detected when the PT reading is 50.

  bool front_left_detected;  // 0 when the front left PT is not detecting light, 1 if it is
  bool front_right_detected;  // 0 when the front right PT is not detecting light, 1 if it is

  // Step 1: Check which PTs detect light
  front_left_detected = (FRONT_LEFT_PT_reading() > detection_threshold) ? 1 : 0;
  front_right_detected = (FRONT_RIGHT_PT_reading() > detection_threshold) ? 1 : 0;

  // Step 2: If no PT can see light, CW until the front right and front left ones can.
  if (!front_left_detected && !front_right_detected) { // This part CW until the front PTs detect light
    while (!(front_left_detected && front_right_detected)) {
      front_left_detected = (FRONT_LEFT_PT_reading() > detection_threshold) ? 1 : 0;
      front_right_detected = (FRONT_RIGHT_PT_reading() > detection_threshold) ? 1 : 0;
      cw();
    }
    stop_motors();
    delay(300);
  }

  return (front_left_detected || front_right_detected);
}

void rotate_findlight(){
  // I don't think we should use this function, shit's ass.

  // Take an angle reading and "zero" the robot---//
  // Set Gyro zero voltage
  int i;
  float sum = 0;
  int sensorValue = 0;
  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at
                             // still, to calculate the zero-drift
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
  // Start rotating to find where lights/fires are based on robot angle, will use front right PT as reference
  int k = 0;
  int speed_rotate = 115;
  while (currentAngle > -160) {
    GYRO_reading(50); // Keep track of current angle
    PT_value[k] = FRONT_RIGHT_PT_reading();
    k++;
    // Rotate ccw, sorry I didn't wanna mess with the actual function cuz I wanted to control how fast it went
    // instead of relying on speed_val
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
  for (int i = 0; i < k; i++) {
    if (PT_value[i] > first_max_value){
      first_max_value = PT_value[i];
      first_max_value_index = i;
    }
  }
  PT_value[first_max_value_index] = 0; // "Erase it" to find next light/fire

  // Find second max value in array = where *another* light/fire is
  // Need to implement a better function because it might just find the first light again
  int second_max_value = 0;
  int second_max_value_index = 0;
  for (int i = 0; i < k; i++) {
    if (PT_value[i] > second_max_value){
      second_max_value = PT_value[i];
      second_max_value_index = i;
    }
  }

  // Find angle of our fires
  // The +180 is because of how we defined our coordinate system
  // The +20 because our sensors aren't accurate lol
  int first_fire_angle = 180+20 - round((double)first_max_value_index/k * 360.0); 
  int second_fire_angle = 180+20 - round((double)second_max_value_index/k * 360.0);

  turn_angle(first_fire_angle);
  forward_light(first_fire_angle);

  return;
}