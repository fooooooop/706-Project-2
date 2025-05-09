#include "state_machine.h"

#include "dual_serial.h"  // <-- Add this line
#include "moving_logic.h"
#include "positioning_system.h"
#include "sensors.h"
#include "serial_command.h"
#include "utilities.h"

STATE initialising() {
  dualPrintln("INITIALISING....");
  delay(1000);  // Allow time to view message
  dualPrintln("Enabling Motors...");
  enable_motors();
  dualPrintln("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  static unsigned long previous_millis = 0;

  // Read wireless command from HC‑12 (Serial1)
  read_serial_command();

  // Blink the built-in LED quickly to indicate running state
  fast_flash_double_LED_builtin();

  // Perform periodic tasks every 500ms
  if (millis() - previous_millis > 500) {
    previous_millis = millis();
    // dualPrintln("RUNNING---------");
    // speed_change_smooth();
    // dualPrintln(speed_val);
    // GYRO_controller(0,0,0,0);
    // dualPrintln(gyro_u);
    // dualPrintln(currentAngle);
    // dualPrintln(IR_u);
    // dualPrintln(IRFront_u);
    // dualPrintln(IRBack_u);
    // dualPrint("IR Sensor Front Right Short: ");
    // dualPrintln(FRONT_RIGHT_shortIR_reading());
    // dualPrint("IR Sensor Front Left Short: ");
    // dualPrintln(FRONT_LEFT_shortIR_reading());
    // dualPrint("IR Sensor Back Left Long: ");
    // dualPrintln(BACK_LEFT_longIR_reading());
    // dualPrint("IR Sensor Back Right  Long: ");
    // dualPrintln(BACK_RIGHT_longIR_reading());

#ifndef NO_READ_GYRO
    // GYRO_reading(500);
    // Can't trust the readings from this cuz the current angle is now a global
    // variable Running this will mess with the GYRO controller
#endif

#ifndef NO_HC_SR04
    // HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif

    // Update turret position as an example
    turret_motor.write(pos);
    pos = (pos == 0) ? 45 : 0;
  }

  return RUNNING;
}

STATE stopped() {
  static byte counter_lipo_voltage_ok = 0;
  static unsigned long previous_millis = 0;

  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {  // Print every 500ms
    previous_millis = millis();
    dualPrintln("STOPPED---------");

#ifndef NO_BATTERY_V_OK
    if (is_battery_voltage_OK()) {
      dualPrint("Lipo OK waiting, counter: ");
      dualPrintln(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  // Ensure voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        dualPrintln("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}
