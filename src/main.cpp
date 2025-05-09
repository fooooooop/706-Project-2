#include <Arduino.h>

#include "globals.h"
#include "positioning_system.h"
#include "state_machine.h"

// Instantiate servo objects and global variables
Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;
Servo turret_motor;

short IR_value[SENSOR_LOG_SIZE];
short IR_pos[SENSOR_LOG_SIZE];
short US_value[SENSOR_LOG_SIZE];
short k = 0;
long global_timesnap = 0;

// Speed Control
int speed_val = 165;
int speed_change = 0;

int pos = 0;  // turret pos?

// Gyro Parameters
float gyroSupplyVoltage = 5;  // supply voltage for gyro
float gyroZeroVoltage = 0;    // the value of voltage when gyro is zero
float gyroSensitivity =
    0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold =
    1;  // because of gyro drifting, defining rotation angular velocity less
        // than this value will not be ignored
float gyroRate = 0;  // read out value of sensor in voltage
float currentAngle =
    0;  // current angle calculated by angular velocity integral on
float maxGyroDrift = 0;

// Controller Efforts
double gyro_u = 0;
double IR_u = 0;
double IRFront_u = 0;
double IRBack_u = 0;

// Controller Integral Memory
double IR_err_mem = 0;
double IR_err_mem_back = 0;
double IR_err_mem_front = 0;
double gyro_err_mem = 0;

// Controller Derivative Memory
double IR_t_previous = 0;
double IR_err_previous = 0;
double gyro_t_previous = 0;
double gyro_err_previous = 0;

// Instanstiate IR sensor objects
SharpDistSensor FrontLeftIR(FRONT_LEFT_IR, MEDIAN_WINDOW_FL);
SharpDistSensor FrontRightIR(FRONT_RIGHT_IR, MEDIAN_WINDOW_FR);
SharpDistSensor BackLeftIR(BACK_LEFT_IR, MEDIAN_WINDOW_BL);
SharpDistSensor BackRightIR(BACK_RIGHT_IR, MEDIAN_WINDOW_BR);

void setup(void) {
  // Attach turret servo (example pin 11)
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup ultrasonic trigger pin
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize USB Serial for debugging and Serial1 for wireless commands
  Serial.begin(115200);   // Debug output
  Serial1.begin(115200);  // HC‑12 wireless commands

  // Set IR sensor models
  FrontLeftIR.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);
  FrontRightIR.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);
  BackLeftIR.setModel(SharpDistSensor::GP2Y0A21F_5V_DS);
  BackRightIR.setModel(SharpDistSensor::GP2Y0A21F_5V_DS);

  // Set Gyro zero voltage
  int i;
  float sum = 0;
  int sensorValue = 0;

  pinMode(A3, INPUT);
  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at
                             // still, to calculate the zero-drift
  {
    sensorValue = analogRead(A3);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
  initPositioning();
  // Debug startup messages
  dualPrintln("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  dualPrintln("Setup....");
  delay(1000);
}

void loop(void) {
  static STATE machine_state = INITIALISING;
  // Finite-state machine
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:
      machine_state = running();
      break;
    case STOPPED:
      machine_state = stopped();
      break;
  }
}