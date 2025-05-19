#include "serial_command.h"

#include "dual_serial.h"
#include "moving_logic.h"
#include "positioning_system.h"
#include "utilities.h"

// woah im adding such big change - test for git pushing

// --- Wireless Command Parsing ---
// This function now listens on Serial1 (the HC‑12)

void read_serial_command() {
  if (Serial1.available()) {
    char val = Serial1.read();
    dualPrint("Received command: ");
    dualPrintln(String(val));
    dualPrint("Speed: ");
    dualPrint(speed_val);
    dualPrintln(" ms");

    switch (val) {
      case 'w':  // Move Forward
      case 'W':
        forward();
        dualPrintln("Forward executed");
        break;
      case 's':  // Move Backward
      case 'S':
        reverse();
        dualPrintln("Reverse executed");
        break;
      case 'q':  // Strafe Left
      case 'Q':
        strafe_left();
        dualPrintln("Strafe Left executed");
        break;
      case 'e':  // Strafe Right
      case 'E':
        strafe_right();
        dualPrintln("Strafe Right executed");
        break;
      case 'a':  // Rotate Counter-Clockwise
      case 'A':
        ccw();
        dualPrintln("Rotate CCW executed");
        break;
      case 'd':  // Rotate Clockwise
      case 'D':
        cw();
        dualPrintln("Rotate CW executed");
        break;
      case '-':  // Decrease Speed
      case '_':
        speed_change = -50;
        dualPrintln("Speed decreased");
        break;
      case '=':
      case '+':  // Increase Speed
        speed_change = 50;
        dualPrintln("Speed increased");
        break;

      case 'b':  // Rotate on the spot
      case 'B':
        dualPrintln("Rotating");
        turn_angle(-90);
        break;

      case 'r':  // Request for status report
      case 'R':
        dualPrintln("Status report requested");
        dualPrintln(
            "=== Mega Status Report ===");  // Keeping this HC-12 specific line
        dualPrint("Speed value: ");
        dualPrintln(speed_val);
        dualPrintln(gyro_u);  // Keeping gyro_u only on Serial
        break;

      case 'x':  // Initiate forward_left() **CANNOT LEAVE BY PRESSING V OR 9**
      case 'X':
        dualPrintln("Loop forward_left initiated");
        forward_light(0);
        dualPrintln("Loop forward_left done!");
        break;

      // Case '1' to print Front Left IR sensor reading continously until '9' is
      // pressed  to stpo
      case '1':
        dualPrintln("Front Left IR sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Front left IR reading: ");
          dualPrint(FRONT_LEFT_shortIR_reading());
          dualPrintln("mm");
          delay(100);
        }
        dualPrintln("Front Left IR sensor reading done!");
        break;

        // Case '2' to print Front Right IR sensor reading continously until '9'
        // is pressed  to stop
      case '2':
        dualPrintln("Front Right IR sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Front right IR reading: ");
          dualPrint(FRONT_RIGHT_shortIR_reading());
          dualPrintln("mm");
          delay(100);
        }
        dualPrintln("Front Right IR sensor reading done!");
        break;

        // Case '3' to print Back Left IR sensor reading continously until '9'
        // is pressed  to stop
      case '3':
        dualPrintln("Back Left IR sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Back left IR reading: ");
          dualPrint(BACK_LEFT_longIR_reading());
          dualPrintln(" mm");
          delay(100);
        }
        dualPrintln("Back Left IR sensor reading done!");
        break;

        // Case '4' to print Back Right IR sensor reading continously until '9'
        // is pressed  to stop
      case '4':
        dualPrintln("Back Right IR sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Back right IR reading: ");
          dualPrint(BACK_RIGHT_longIR_reading());
          dualPrintln(" mm");
          delay(100);
        }
        dualPrintln("Back Right IR sensor reading done!");
        break;

      case '0':
        dualPrintln("Ultrasonic Sensor reading initiated");
        while (Serial1.read() != '9') {
          float distance = HC_SR04_range();
          dualPrint("Ultrasonic Distance: ");
          dualPrint(distance);
          dualPrintln(" mm");
          delay(200);
        }
        dualPrintln("Ultrasonic reading done!");
        break;

      case '5':
        dualPrintln("Front Left PT sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Front left PT reading: ");
          dualPrint(FRONT_LEFT_PT_reading());
          dualPrintln(" mm");
          delay(100);
        }
        dualPrintln("Front Left PT sensor reading done!");
        break;
      case '6':
        dualPrintln("Front Right PT sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Front right PT reading: ");
          dualPrint(FRONT_RIGHT_PT_reading());
          dualPrintln(" mm");
          delay(100);
        }
        dualPrintln("Front Right PT sensor reading done!");
        break;
      case '7':
        dualPrintln("Left PT sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Left PT reading: ");
          dualPrint(LEFT_PT_reading());
          dualPrintln(" mm");
          delay(100);
        }
        dualPrintln("Left PT sensor reading done!");
        break;

      case '8':
        dualPrintln("Right PT sensor reading initiated");
        while (Serial1.read() != '9') {
          dualPrint("Right PT reading: ");
          dualPrint(RIGHT_PT_reading());
          dualPrintln(" mm");
          delay(100);
        }
        dualPrintln("Right PT sensor reading done!");
        break;

      case 'c':  // Initiate forward_right() **CANNOT LEAVE BY PRESSING V OR 9**
      case 'C':
        dualPrintln("Rotate light initiated");
        rotate_findlight();
        dualPrintln("Rotate light done");
        break;

      case 'p':
      case 'P':
        dualPrintln("Find light initiated");
        // Find First Light
        find_light();
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

        forward_light(0);
        forward();
        dualPrintln("First light found, turning on fan");
        fan_on();
        // delay(10000);

        
        // Find Second Light and "zero" the robot---//
        find_light();
        currentAngle = 0;
        //----------------------------------------------//

        forward_light(0);
        forward();
        fan_on();
        // delay(10000);
        dualPrintln("Find light done");    
        break;

      default:
        stop_motors();
        dualPrintln("Stop executed");
        break;

    }
  }
}
