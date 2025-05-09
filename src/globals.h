#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Servo.h>

#include "SharpDistSensor.h"
#include "SharpIR.h"
#include "dual_serial.h"

#define MEDIAN_WINDOW_FR 5
#define MEDIAN_WINDOW_FL 5
#define MEDIAN_WINDOW_BR 5
#define MEDIAN_WINDOW_BL 5

#define SENSOR_LOG_SIZE 500

extern short IR_value[SENSOR_LOG_SIZE];
extern short IR_pos[SENSOR_LOG_SIZE];
extern short US_value[SENSOR_LOG_SIZE];
extern short k;
extern long global_timesnap;

// State machine state
enum STATE { INITIALISING, RUNNING, STOPPED };

// Motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// IR sensor pins
const int FRONT_LEFT_IR = A4;

const int FRONT_RIGHT_IR = A14;

const int BACK_LEFT_IR = A5;

const int BACK_RIGHT_IR = A6;

extern SharpDistSensor FrontLeftIR;
extern SharpDistSensor FrontRightIR;
extern SharpDistSensor BackLeftIR;
extern SharpDistSensor BackRightIR;

// IR left-right/wheel-drive states
enum DRIVE { AWD, FWD, RWD };
enum DIRECTION { LEFT, RIGHT, IDGAF };
enum SPEED { SLOW, FAST };

// Ultrasonic sensor pins and parameters
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const unsigned int MAX_DIST = 23200;

// Gyro parameters
extern float gyroSupplyVoltage;  // supply voltage for gyro
extern float gyroZeroVoltage;    // the value of voltage when gyro is zero
extern float gyroSensitivity;    // gyro sensitivity unit is (mv/degree/second)
                                 // get from datasheet
extern float
    rotationThreshold;  // because of gyro drifting, defining rotation angular
                        // velocity less than this value will not be ignored
extern float gyroRate;  // read out value of sensor in voltage
extern float
    currentAngle;  // current angle calculated by angular velocity integral on
extern float maxGyroDrift;

// Declare servo objects
extern Servo left_front_motor;
extern Servo left_rear_motor;
extern Servo right_rear_motor;
extern Servo right_front_motor;
extern Servo turret_motor;

// Global variables for speed control and turret position
extern int speed_val;
extern int speed_change;
extern int pos;

// Controller variables
extern double gyro_u;
extern double IR_u;
extern double IRFront_u;
extern double IRBack_u;

extern double IR_err_mem;
extern double IR_err_mem_back;
extern double IR_err_mem_front;
extern double gyro_err_mem;

extern double IR_t_previous;
extern double IR_err_previous;
extern double gyro_t_previous;
extern double gyro_err_previous;

// Serial port pointer
extern HardwareSerial *SerialCom;

#endif  // GLOBALS_H
