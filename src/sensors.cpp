#include "sensors.h"

#define MEDIAN_WINDOW 3
#define OVERALL_IR_OFFSET 69
#define FRONT_LEFT_IR_OFFSET 0
#define FRONT_RIGHT_IR_OFFSET -60
#define BACK_LEFT_IR_OFFSET 10
#define BACK_RIGHT_IR_OFFSET -31

#ifndef NO_HC_SR04

// float HC_SR04_range() {
//   const int n = MEDIAN_WINDOW;  // Number of samples for the average
//   float sum = 0;
//   int validReadings = 0;

//   for (int i = 0; i < n; i++) {
//     unsigned long t1, t2, pulse_width;
//     float cm;

//     // Trigger the sensor
//     digitalWrite(TRIG_PIN, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(TRIG_PIN, LOW);

//     // Wait for echo to start
//     t1 = micros();
//     while (digitalRead(ECHO_PIN) == 0) {
//       t2 = micros();
//       pulse_width = t2 - t1;
//       if (pulse_width > (MAX_DIST + 1000)) {
//         dualPrintln("HC-SR04: NOT found");
//         cm = 0;
//         break;
//       }
//     }

//     // Measure the echo pulse duration
//     t1 = micros();
//     while (digitalRead(ECHO_PIN) == 1) {
//       t2 = micros();
//       pulse_width = t2 - t1;
//       if (pulse_width > (MAX_DIST + 1000)) {
//         dualPrintln("HC-SR04: Out of range");
//         cm = 0;
//         break;
//       }
//     }
//     t2 = micros();
//     pulse_width = t2 - t1;

//     cm = pulse_width / 58.0;

//     if (pulse_width <= MAX_DIST && cm > 0) {
//       sum += cm;
//       validReadings++;
//     }

//     delay(50);  // Small delay between measurements
//   }

//   if (validReadings == 0) {
//     return 0;
//   }

//   return sum / validReadings;  // Return average
// }

float HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Trigger the sensor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for echo to start
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      dualPrintln("HC-SR04: NOT found");
      return 0;
    }
  }

  // Measure the length of the echo pulse
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      dualPrintln("HC-SR04: Out of range");
      return 0;
    }
  }
  t2 = micros();
  pulse_width = t2 - t1;

  cm = pulse_width / 58.0;

  if (pulse_width > MAX_DIST) {
    dualPrintln("HC-SR04: Out of range");
  } else {
    // dualPrintln("HC-SR04:");
    // dualPrintln(cm);
    // dualPrintln("cm");
  }

  return cm;
}
#endif

// Return IR sensor readings
uint16_t FRONT_LEFT_shortIR_reading() {
  return FrontLeftIR.getDist() + OVERALL_IR_OFFSET + FRONT_LEFT_IR_OFFSET;
}

uint16_t FRONT_RIGHT_shortIR_reading() {
  return FrontRightIR.getDist() + OVERALL_IR_OFFSET + FRONT_RIGHT_IR_OFFSET;
}

uint16_t BACK_LEFT_longIR_reading() {
  return BackLeftIR.getDist() + OVERALL_IR_OFFSET + BACK_LEFT_IR_OFFSET;
}

uint16_t BACK_RIGHT_longIR_reading() {
  return BackRightIR.getDist() + OVERALL_IR_OFFSET + BACK_RIGHT_IR_OFFSET;
}

#ifndef NO_READ_GYRO
void GYRO_reading(double T) {
  // T is delay for loop

  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(A3) * gyroSupplyVoltage) / 1023;
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5);
  // read out voltage divided the gyro sensitivity to calculate the angular
  // velocity
  float angularVelocity = gyroRate / gyroSensitivity;
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold ||
      angularVelocity <= -rotationThreshold) {
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000.0 / T);
    currentAngle += angleChange;
  }

  if (abs(angularVelocity) > maxGyroDrift) maxGyroDrift = abs(angularVelocity);

  // keep the angle between 0-360
  // 183 threshold so we can do 180 degree turns
  if (currentAngle < -195) {
    currentAngle += 360;
  } else if (currentAngle > 195) {
    currentAngle -= 360;
  }

  delay(T);
  return;
}
#endif

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  int Lipo_level_cal;
  int raw_lipo;

  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    dualPrint("Lipo level:");
    dualPrint(Lipo_level_cal);
    dualPrint("%");
    dualPrintln("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
    dualPrintln("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
    dualPrintln("!Lipo is Overcharged!!!");
    else {
      dualPrintln(
          "Lipo voltage too LOW, any lower and the lipo will be damaged");
      dualPrint("Please Re-charge Lipo:");
      dualPrint(Lipo_level_cal);
      dualPrintln("%");
    }
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif
