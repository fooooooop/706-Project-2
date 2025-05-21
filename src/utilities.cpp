#include "utilities.h"

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 500) speed_val = 500;
  delay(50);
  speed_change = 0;
}

double GYRO_controller(double gyro_target, double kp, double ki, double kd) {
  //Setup!------------------------//

  // Time variables
  double t_current = 0;
  double gyro_t = 50;

  // General error variables
  double gyro_currentSensor;
  double gyro_err_current;

  // For Derivative controller
  double dt;
  double de;
  double dedt;

  //Main Code Start!------------------------//
  // To get dt for Integral and Derivative controllers
  t_current = millis();
  dt = (t_current - gyro_t_previous) / 1000;
  gyro_t_previous = t_current;

  // Gyro reading
  GYRO_reading(gyro_t); 
  gyro_currentSensor = currentAngle;

  // Proportional controller
  gyro_err_current = gyro_target - gyro_currentSensor;

  // Integral controller
  if (gyro_u < 200) { // Anti-integral windup
    gyro_err_mem += gyro_err_current;
  }
  
  // Derivative controller
  de = (abs(gyro_err_current) - abs(gyro_err_previous));
  gyro_err_previous = gyro_err_current;

  dedt = (de / dt);

  // PID controller
  (kp*gyro_err_current + ki*gyro_err_mem + kd*dedt) > 300 ? gyro_u = 300 : 
  (kp*gyro_err_current + ki*gyro_err_mem + kd*dedt) < -300 ? gyro_u = -300 :
  gyro_u = kp*gyro_err_current + ki*gyro_err_mem + kd*dedt;

  return gyro_err_current;
}

double PT_controller(double kp, double ki, double kd){
  //Setup!------------------------//

  // Clamp
  double clamp_effort = 300;

  // Time variables
  double t_current = 0;

  // General error variables
  double right_side_reading;
  double left_side_reading;
  double PT_err_current;

  // For Derivative controller
  double dt;
  double de;
  double dedt;

  //Main Code Start!------------------------//
  // To get dt for Integral and Derivative controllers
  t_current = millis();
  dt = (t_current - PT_t_previous) / 1000;
  PT_t_previous = t_current;

  // PT reading
  right_side_reading = FRONT_RIGHT_PT_reading() + RIGHT_PT_reading();
  left_side_reading = FRONT_LEFT_PT_reading() + LEFT_PT_reading();

  // Proportional controller
  PT_err_current = right_side_reading-left_side_reading;

  // Integral controller
  if (PT_u < 200) { // Anti-integral windup
    PT_err_mem += PT_err_current;
  }
  
  // Derivative controller
  de = (abs(PT_err_current) - abs(PT_err_previous));
  PT_err_previous = PT_err_current;

  dedt = (de / dt);

  // PID controller
  (kp*PT_err_current + ki*PT_err_mem + kd*dedt) > clamp_effort ? PT_u = clamp_effort : 
  (kp*PT_err_current + ki*PT_err_mem + kd*dedt) < -clamp_effort ? PT_u = -clamp_effort :
  PT_u = kp*PT_err_current + ki*PT_err_mem + kd*dedt;

  return PT_err_current;
}

double IR_controller(double IR_target, enum DRIVE IR_mode, enum DIRECTION left_right, double kp, double ki, double kd) {
  //Setup!------------------------//

  //IR_mode changes what efforts are given to the motor
  //This is needed for the corner finding function so that the front and back wheels have
  //their own controllers
  // For Project 2: JUST USE AWD! (Only long IR sensors are on the sides right now)

  //left_right states which side IR sensor to use

  // Clamp
  double clamp_effort = 300;

  // Time variables
  double t_current;

  // General error variables
  double IR_currentSensor;
  double IR_err_current;

  // For Derivative controller
  double dt;
  double de;
  double dedt;

  //Main Code Start!------------------------//
  // To get dt for Integral and Derivative controllers
  t_current = millis();
  dt = (t_current - IR_t_previous) / 1000;
  IR_t_previous = t_current;

  // IR reading + Proportional Controller
  if (IR_mode == FWD) {

    // Uses only Front IR Sensors //
    if (left_right == LEFT) {
      // Less than 13cm from LEFT wall
      IR_currentSensor = (double)FRONT_LEFT_shortIR_reading();
      IR_err_current = (IR_target - IR_currentSensor) * -1; 
  
    } else if (left_right == RIGHT) {
      // Less than 13cm from RIGHT wall
      IR_currentSensor = (double)FRONT_RIGHT_shortIR_reading();
      IR_err_current = IR_target - IR_currentSensor; 
  
    }
    //---------------------------//

  } else if (IR_mode == RWD) {

    // Uses only the Back IR Sensors //
    if (left_right == LEFT){
      // More than 32cm from LEFT wall
      IR_currentSensor = (double)BACK_LEFT_longIR_reading();
      IR_err_current = (IR_target - IR_currentSensor) * -1; 
  
    } else if (left_right == RIGHT){
      // More than 32cm from RIGHT wall
      IR_currentSensor = (double)BACK_RIGHT_longIR_reading();
      IR_err_current = (IR_target - IR_currentSensor); 

    } 
    //------------------------------//

  } else if (IR_mode == AWD) {

    // Will use all sensors
    // USE FOR THE STRAIGHT LINE
    // Honestly we could do two separate controllers for the front and back wheels and see how that goes?
    // But that's highkey kinda hard, and also, the short IR sensors will be useless in the middle anyways
    if (IR_target < 130) {
      if (left_right == LEFT){
        IR_currentSensor = (double)FRONT_LEFT_shortIR_reading();
        IR_err_current = (IR_target - IR_currentSensor) * -1; 
    
      } else if (left_right == RIGHT){
        IR_currentSensor = (double)FRONT_RIGHT_shortIR_reading();
        IR_err_current = IR_target - IR_currentSensor; 
    
      }
    } else {
      if (left_right == LEFT){
        IR_currentSensor = (double)BACK_LEFT_longIR_reading();
        IR_err_current = (IR_target - IR_currentSensor) * -1; 
    
      } else if (left_right == RIGHT){
        IR_currentSensor = (double)BACK_RIGHT_longIR_reading();
        IR_err_current = (IR_target - IR_currentSensor);

      }
    }
    //------------------------------//

  }

  // Integral controller

  if (abs(IR_u) < clamp_effort) { 
    // Anti-integral windup
    IR_err_mem += IR_err_current;
  }

  if (abs(IRFront_u) < clamp_effort) { 
    // Anti-integral windup
    IR_err_mem_front += IR_err_current;
  }

  if (abs(IRBack_u) < clamp_effort) { 
    // Anti-integral windup
    IR_err_mem_back += IR_err_current;
  }
  
  // Derivative controller
  de = (abs(IR_err_current) - abs(IR_err_previous));
  IR_err_previous = IR_err_current;

  dedt = (de / dt);

  // PID controller
  if (IR_mode == FWD) {
    // Add clamps
    if ((kp*IR_err_current + ki*IR_err_mem_front + kd*dedt) > clamp_effort) {IRFront_u = clamp_effort;} else
    if ((kp*IR_err_current + ki*IR_err_mem_front + kd*dedt) < -clamp_effort) {IRFront_u = -clamp_effort;} else
    {IRFront_u = (kp*IR_err_current + ki*IR_err_mem_front + kd*dedt);}
  } else if (IR_mode == RWD) {
    // Add clamps
    if ((kp*IR_err_current + ki*IR_err_mem_back + kd*dedt) > clamp_effort) {IRBack_u = clamp_effort;} else
    if ((kp*IR_err_current + ki*IR_err_mem_back + kd*dedt) < -clamp_effort) {IRBack_u = -clamp_effort;} else
    {IRBack_u = (kp*IR_err_current + ki*IR_err_mem_back + kd*dedt);}
  } else {
    // Add clamps
    if ((kp*IR_err_current + ki*IR_err_mem + kd*dedt) > clamp_effort) {IR_u = clamp_effort;} else
    if ((kp*IR_err_current + ki*IR_err_mem + kd*dedt) < -clamp_effort) {IR_u = -clamp_effort;} else
    {IR_u = (kp*IR_err_current + ki*IR_err_mem + kd*dedt);}
  }

  return IR_err_current;
}

void fan_on() {
  digitalWrite(4, HIGH); // what is digital 4 pin lol
  bool fan_is_on = 1;
  unsigned long firstTime = millis();
  // sleep for 10 seconds
  while(fan_is_on) {
    if (millis() >= (firstTime + 10000)) { // more than 10 seconds since the timer started
      digitalWrite(4, LOW);
      fan_is_on = 0;
    }
  }
}