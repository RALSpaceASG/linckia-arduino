/*
 * ------------------------------
 *  Linckia Arduino Code
 * ------------------------------
 * Group: RAL Space Autonomous Systems Group
 * Contact: Aron Kisdi <aron.kisdi@stfc.ac.uk>
 * Organisation: RAL Space, STFC
 * ------------------------------
 */

#define LINCKIA_VERSION "6.6"
#define LINCKIA_VERSION_DATE "24 May 2016"

// imports
#include <Metro.h>

// compatibility with Arduino < 1.6.7
typedef byte uint8_t;

static const int routeren = 2; //Digital pin 2 for enabling 3.3V regulator

// User commands over serial
byte command[6];                        // raw input from serial buffer, 6 bytes
enum command_t {MOTOR = 1, SERVO, SENSOR, SENSOR_INTERVAL, PING, REBOOT, VERSION};

//Metro objects for schedueled tasks
unsigned long ulActuatorsInterval = 100;
unsigned long ulSensorsInterval   = 1000;

Metro MActuators = Metro(ulActuatorsInterval);
Metro MSensors   = Metro(ulSensorsInterval);

void Return(int ID, int value, int value1, int value2) {
  Serial.write(255);
  Serial.write(ID);
  Serial.write(value);
  Serial.write(value1);
  Serial.write(value2);
  Serial.write(254);
}

void MoveActuators() {
  //Moves actuator by a calculated increment every 10 milliseconds
  moveServos();
  moveMotors();
}

//************************************
// Get packets through Serial and
// execute commands based on payload
//
//************************************//

int ReadCommand() {
  if (Serial.available() < 8) {
    return -1;
  }

  if (Serial.read() != 255) {
    // seek to a startbyte
    while (Serial.available() >= 8 && Serial.read() != 255);
    // or else leave
    if (Serial.available() < 8) {
      return -2;
    }
  }

  Serial.readBytes(command, 6);

  if (Serial.read() != 254) {
    // error wrong stopbyte
    return -2;
  }

  // if we have reached here it means
  // we have a valid command packet
  return 0;
}

void HandleCommand(byte command[6]) {
  switch (command[0]) {
    case MOTOR:
      MotorCommand(command);
      break;
    case SERVO:
      ServoCommand(command);
      break;
    case SENSOR:
    {
      ReadAnalog(command[1]);
      break;
    }
    case SENSOR_INTERVAL:
    {
      SensorInterval(command);
      break;
    }
    case PING:
    {
      Return(104,0,0,0);
      break;
    }
    case REBOOT:
    {
      digitalWrite(routeren, LOW);
      delay(100);
      digitalWrite(routeren, HIGH);
      break;
    }
    case VERSION:
    {
      Serial.write(255);
      Serial.println(LINCKIA_VERSION);
      Serial.println(LINCKIA_VERSION_DATE);
      Serial.println(__DATE__);
      Serial.write(254);
      break;
    }
    default:
    {
      // invalid command
      Return(103, command[0], 0, 0);
      break;
    }
  }
}

//********************************
// ************MAIN***************
//
// Work is done here
//********************************

void setup() {
  // router reset pin
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  ServoSetup();
  MotorSetup();
  AnalogPinSetup();  //set up I/O
  // Open the serial connection, baud 9600 (max: 115200)
  Serial.begin(9600);
}

void loop() {
  int err = ReadCommand();
  if (!err) {
    HandleCommand(command);
  }

  if (MActuators.check()) {
    MoveActuators();
  }
}
