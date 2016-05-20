/*
 * ------------------------------
 *  Linckia Arduino Code
 * ------------------------------
 * Group: RAL Space Autonomous Systems Group
 * Contact: Aron Kisdi aron.kisdi@stfc.ac.uk
 * Organisation: RAL Space, STFC
 * ------------------------------
 */

#define LINCKIA_VERSION "6.5"
#define LINCKIA_VERSION_DATE "04 Apr 2016"

// Import the Arduino Servo library
#include <Metro.h>

static const int routeren = 2; //Digital pin 2 for enabling 3.3V regulator

// User commands over serial
byte command[6];                        // raw input from serial buffer, 6 bytes
enum command_t {MOTOR = 1, SERVO, SENSOR, SENSOR_INTERVAL, PING, REBOOT, VERSION};

//feedback
int feedback;             // reading from analog port
int feedback10;           // variable for storing first two digits of feedback
int feedback100;          // variable for storing 100s decimal places of feedback
int analogpin;            // variable for setting which analog pin to read

//Metro objects for schedueled tasks
int moveint = 100;
int mfeedon = 0;
int mfeedint = 1000;
Metro MMove = Metro(moveint);
Metro motorfeedback = Metro(mfeedint);

//USER DEFINED FUNCTIONS
void PinSetup()
{
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  pinMode(2,OUTPUT);
  digitalWrite(2, HIGH);
}

void ReadAnalog(int targetPin){
  feedback = analogRead(targetPin);
  feedback100 = feedback/100;
  feedback10 = feedback-feedback100*100;
  //Return(1,targetPin,feedback100,feedback10);
  if (feedback100 < 100){
   if (feedback10 < 100){
     Return(1,targetPin,feedback100,feedback10);
   }
  }
}

void CommandReceived(byte command[6]){
  switch (command[0]) {
    case MOTOR:
      motor_command(command);
      break;
    case SERVO:
      servo_command(command);
      break;
    case SENSOR:
    {
      analogpin = command[1];
      ReadAnalog(analogpin);
      break;
    }
    case SENSOR_INTERVAL:
    {
      //TODO Start stop readout
      if (command[1] == 1) { //command for changeg feedback interval
        mfeedon = command[2]; //turn it on if 1 turn it off if 0
        //command[3] not used
        mfeedint = (command[4]*100)+command[5];
        motorfeedback.interval(mfeedint);
      }
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

void MoveActuators(){
  //Moves actuator by a calculated increment every 10 milliseconds
  if (MMove.check() == 1)
  {
    moveServos();
    moveMotors();
  }
}

void CheckSensors()
{
  if (motorfeedback.check() == 1) //time to check motor feedback
  {
    if(mfeedon == 1) //if it is on 0 is off 1 is on
    {
      static int j;
      ReadAnalog(j);
      j = j + 1;
      if(j == 5)
      {
        j = 0;
      }
    }
  }
}

void Return(int ID, int value, int value1, int value2){
  Serial.write(255);
  Serial.write(ID);
  Serial.write(value);
  Serial.write(value1);
  Serial.write(value2);
  Serial.write(254);
}

void ReadSerial(){
  // Check serial input (min 8 bytes in buffer)
   if (Serial.available() > 7){
     // Read the first byte
     byte startbyte = Serial.read();
     // If it's really the startbyte (255) ...
     if (startbyte == 255){
       // Read command
       for (int i=0; i<6; i++){
         command[i] = Serial.read();
       }
       byte stopbyte = Serial.read();
       if (stopbyte == 254){
         //Return(command[0],command[1],command[2],command[3]);
         CommandReceived(command);
       } //end of accept command
       else{
         //Error wrong Stopbyte
         Return(102,stopbyte,0,0);
         //Reject or accept command, debug ???
       } //end of stopbyte check
    } //end of accept startbyte
     else{
       //Error wrong Startbyte
       Return(101,startbyte,0,0);
     } //end of reject startbyte
  } //end if serial.available
}

//ARDUINO SETUP and LOOP functions

void setup() {
  servo_setup();
  motor_setup();
  PinSetup();  //set up I/O
  // Open the serial connection, baud 9600 (max: 115200)
  Serial.begin(9600);
}

void loop()
{
  ReadSerial();
  MoveActuators();
}
