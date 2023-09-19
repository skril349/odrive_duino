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
float turns;

HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;



// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX

//SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);



//functions
float GetIntensity(int motor_number) {
  odrive_serial << "r axis" << motor_number << ".motor.current_control.Iq_measured\n";

  return readString().toFloat();
}

float GetVelocity(int motor_number) {
  odrive_serial << "r axis" << motor_number << ".encoder.vel_estimate\n";

  return readString().toFloat();
}

float GetPosition(int motor_number) {
  odrive_serial << "r axis" << motor_number << ".encoder.pos_estimate\n";

  return readString().toFloat();
}


String readString() {
  String str = "";
  static const unsigned long timeout = 1000;
  unsigned long timeout_start = millis();
  for (;;) {
    while (!odrive_serial.available()) {
      if (millis() - timeout_start >= timeout) {
        return str;
      }
    }
    char c = odrive_serial.read();
    if (c == '\n')
      break;
    str += c;
  }
  return str;
}

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
  odrive_serial << "w axis0.controller.config.vel_limit " << 30.0f << '\n';
  odrive_serial << "w axis0.motor.config.current_lim " << 25.0f << '\n';
  odrive_serial << "w axis0.trap_traj.config.accel_lim " << 10.0f << '\n';
  odrive_serial << "w axis0.trap_traj.config.decel_lim " << 10.0f << '\n';
  odrive_serial << "w axis0.trap_traj.config.vel_lim " << 30.0f << '\n';

  odrive_serial << "w axis1.controller.config.vel_limit " << 30.0f << '\n';
  odrive_serial << "w axis1.motor.config.current_lim " << 25.0f << '\n';

  odrive_serial << "w axis1.trap_traj.config.accel_lim " << 10.0f << '\n';
  odrive_serial << "w axis1.trap_traj.config.decel_lim " << 10.0f << '\n';
  odrive_serial << "w axis1.trap_traj.config.vel_lim " << 30.0f << '\n';

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


  int motornum2 = '1' - '0';
  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum2, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum2, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << "1" << ": Requesting state " << requested_state << '\n';
  if (!odrive.run_state(motornum2, requested_state, false /*don't wait*/)) return;


  Serial.println("presiona `t` si quieres hacer un ensayo de tracción y `r` si quieres uno de twist & tracción: ");


}
void loop() {
  switch (state) {
    case 0:
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 't' || c == 'r') {
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
      if (test == 'r') {
        Serial.println("Selecciona el numera de vueltas");
        state = 3;

      }
      if (test == 't') {
        Serial.println("moving trajectory");
        odrive_serial << "w axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ" << '\n';
        delay(5);
        odrive_serial << "t 0 " << move_to << '\n';
        Serial.println("presiona 't' si quieres hacer un ensayo de tracción y 'r' si quieres uno de twist & tracción: ");
        state = 0;
      }
      break;
    case 3:
      if (Serial.available() > 0) {

        String input = Serial.readStringUntil('\n'); // Lee la entrada hasta encontrar un salto de línea ('\n')
        // Intenta convertir la entrada a un número entero (int)
        float value = atof(input.c_str());
        Serial.println(value);
        if (value >= 0) {
          turns = value;
          Serial.println("rotamos a:");
          state = 4;
        }
      }
      break;

    case 4:
      Serial.println("moving trajectory");
      odrive_serial << "w axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ" << '\n';
      odrive_serial << "w axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ" << '\n';
      delay(5);
      odrive_serial << "t 0 " << move_to << '\n';
      odrive_serial << "t 1 " << turns << '\n';

      Serial.println("presiona 0 si quieres hacer un ensayo de tracción y 1 si quieres uno de twist: ");
      state = 5;
      break;
    case 5:
      odrive_serial << "w axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ" << '\n';
      //7 float intensity = GetIntensity(0);
      float velocity = GetVelocity(0);
      float pos = GetPosition(0);
      if (velocity <100 && pos < 100) {
       if (velocity>-100 && pos > -100) {
        Serial.print("vel:");
        Serial.print(velocity);
        Serial.print(", ");
        // Serial.print("intens:");
        // Serial.print(intensity);
        //  Serial.print(", ");
        Serial.print("pos:");
        Serial.print(pos);
        Serial.println();
        delay(10);
      }

      }

      if (Serial.available() > 0) {
        state = 0;
      }
      break;
    default:
      break;
      // statements
  }
}
