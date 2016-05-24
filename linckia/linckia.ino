/*
 * ------------------------------
 *  Linckia Arduino Code
 * ------------------------------
 * Group: RAL Space Autonomous Systems Group
 * Contact: Aron Kisdi <aron.kisdi@stfc.ac.uk>
 * Organisation: RAL Space, STFC
 * ------------------------------
 */

#define LINCKIA_VERSION "6.7"
#define LINCKIA_VERSION_DATE "24 May 2016"

// imports
#include <Metro.h>

// compatibility with Arduino < 1.6.7
typedef byte uint8_t;

static const int routeren = 2; //Digital pin 2 for enabling 3.3V regulator

// User commands over serial
byte command[6];                   // command from received serial packets
byte return_packet_buffer[6];      // buffer to store return packets
enum command_t {MOTOR = 1, SERVO, SENSOR, SENSOR_INTERVAL, PING, REBOOT, VERSION};

//Metro objects for schedueled tasks
unsigned long ulActuatorsInterval = 100;
unsigned long ulSensorsInterval   = 1000;

Metro MActuators = Metro(ulActuatorsInterval);
Metro MSensors   = Metro(ulSensorsInterval);

#ifdef LINCKIA_COMMAND_TIMEOUT
// Metro to check that we receive commands regularly
Metro MCommandTimeout = Metro(LINCKIA_COMMAND_TIMEOUT);
#endif

void makeReturnPacket(byte id, byte field2 = 0, byte field3 = 0, byte field4 = 0) {
  return_packet_buffer[0] = 255;
  return_packet_buffer[1] = id;
  return_packet_buffer[2] = field2;
  return_packet_buffer[3] = field3;
  return_packet_buffer[4] = field4;
  return_packet_buffer[5] = 254;
}

void sendReturnPacket() {
  Serial.write(return_packet_buffer, 6);
}

void ReturnPacket(byte id, byte field2 = 0, byte field3 = 0, byte field4 = 0) {
  makeReturnPacket(id, field2, field3, field4);
  sendReturnPacket();
}

//**********************************
//
// Tasks to communicate with motors
//
//**********************************

void StopAll() {
  MotorStopAll();
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
      ReturnPacket(104,0,0,0);
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
      ReturnPacket(103, command[0], 0, 0);
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

  // Open the serial connection, max baud 115200
  Serial.begin(115200);  // Command Serial port

  // set up I/O
  ServoSetup();
  MotorSetup();
  AnalogPinSetup();
}

void loop() {
  int err = ReadCommand();
  if (!err) {
    HandleCommand(command);
#ifdef LINCKIA_COMMAND_TIMEOUT
    MCommandTimeout.reset();
#endif
  }

  if (MActuators.check()) {
    MoveActuators();
  }

#ifdef LINCKIA_COMMAND_TIMEOUT
  // this will be true if we haven't
  // been resetting often enough
  if (MCommandTimeout.check()) {
    StopAll();
  }
#endif
}
