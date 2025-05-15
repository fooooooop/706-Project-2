#include "serial_command.h"

#include "dual_serial.h"
#include "moving_logic.h"
#include "positioning_system.h"

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
        forward_left();
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

      case '5':
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

      case '0':
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
        dualPrintln("Loop forward_right initiated");
        forward_right();
        dualPrintln("Loop forward_right done!");
        break;

      case 'p':
      case 'P':
        while (1) {
          dualPrintln("Find light initiated");
          find_light();
          dualPrintln("Find light done");
        }
        break;

      default:
        stop_motors();
        dualPrintln("Stop executed");
        break;
    }
  }
}
