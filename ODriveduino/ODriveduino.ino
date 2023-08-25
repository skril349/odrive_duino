// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

//variables
int motornum;
int state = 0;
int test;
float move_to;
// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 100.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 30.0f << '\n';

    odrive_serial << "w axis" << axis << ".trap_traj.config.accel_lim " << 50.0f << '\n';
    odrive_serial << "w axis" << axis << ".trap_traj.config.decel_lim " << 50.0f << '\n';
    odrive_serial << "w axis" << axis << ".trap_traj.config.vel_lim " << 100.0f << '\n';

    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("calibration");
  motornum = '0' - '0';
  int requested_state;

  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << "0" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << "0" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << "0" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;

  delay(15000);

  motornum = '1' - '0';
requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
  

  Serial.println("presiona 0 si quieres hacer un ensayo de tracción y 1 si quieres uno de twist: ");


}
void loop() {
  switch (state) {
    case 0:
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '0' || c == '1') {
          test = c;
          Serial.println("introduce el número de vueltas que quieres que de el motor");
          state = 1;
        }
      }

      break;
    case 1:
      if (Serial.available()) {

        String input = Serial.readStringUntil('\n'); // Lee la entrada hasta encontrar un salto de línea ('\n')
        // Intenta convertir la entrada a un número entero (int)
        float value = atof(input.c_str());
        Serial.println(value);
        if (value > 0) {
          move_to = value;
          Serial.println("nos movemos a");
          state = 2;
        }
      }
      break;

    case 2:
      if (test == '1') {

      }
      if (test == '0') {
        Serial.println("moving trajectory");
        odrive_serial << "w axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ" << '\n';
        delay(5);
        odrive_serial << "t 0 " << move_to << '\n';
        Serial.println("presiona 0 si quieres hacer un ensayo de tracción y 1 si quieres uno de twist: ");
        state = 0;
      }
      break;
    default:
      break;
      // statements
  }
}
