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

int routeren = 2; //Digital pin 2 for enabling 3.3V regulator

// User commands over serial
int command[6];                        // raw input from serial buffer, 6 bytes
char data[8] = {255,0,0,0,0,0,0,254};
int startbyte;                         // start byte, begin reading input
int stopbyte;                          // end of command

//Iterators and time
int movtime;              //timed command value
int i;                    // iterator command error check
int j = 0;                // iterator (get feedback)
int k;                    // iterator move joint

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

void CommandReceived(int command[6]){
  // check each part of the command
      //Return(command[0],command[1],command[2],command[3]);

      //MOTOR COMMAND
      if (command[0] == 1){
        motor = command[1]; // 1-4
        motordir = command[2]; //0 or 1
        if (motordir == 1){
          pwm = -command[3]; //-100 to 0
        }
        if (motordir == 0){
          pwm = command[3]; // 0 to 100
        }
        movtime = ((command[4])*1000)+((command[5])*10); //milliseconds ??? WARNING THIS CAN BE HIGHER THAN 32767!rollover?

        motorsdir[motor-1] = motordir;
        motortargets[motor-1] = pwm;
        motormovtimes[motor-1] = movtime;
        //Return(103,motor,motordir,pwm);
      }
      //SERVO COMMAND
      if (command[0] == 2){
        servo = command[1]; // 1-3
        pos = command[2]; // 0-180
        movtime = ((command[4])*1000)+((command[5])*10); //milliseconds ??? WARNING THIS CAN BE HIGHER THAN 32767!rollover?

        servotargets[servo-1] = pos;
        servomovtimes[servo-1] = movtime;
      }
      //SENSOR direct command
      if (command[0] == 3){
        analogpin = command[1];
        ReadAnalog(analogpin);
      }

      // automatic sensor interval command
      if (command[0] == 4){
        //TODO Start stop readout
        if (command[1] == 1){ //command for changeg feedback interval
          mfeedon = command[2]; //turn it on if 1 turn it off if 0
          //command[3] not used
          mfeedint = (command[4]*100)+command[5];
          motorfeedback.interval(mfeedint);
        }
      }

      //ping
      if (command[0] == 5){
        Return(104,0,0,0);
      } // End of Handshake

      //restart router
      if (command[0] == 6){
        digitalWrite(routeren, LOW);
        delay(100);
        digitalWrite(routeren, HIGH);
      } // End of Handshake

      // version information
      if (command[0] == 7){
        Serial.write(255);
        Serial.println(LINCKIA_VERSION);
        Serial.println(LINCKIA_VERSION_DATE);
        Serial.println(__DATE__);
        Serial.write(254);
      }
}

void MoveActuators(){
  //Moves actuator by a calculated increment every 10 milliseconds
  if (MMove.check() == 1){
    moveServos();
    moveMotors();
  }
}

void CheckSensors(){
  if (motorfeedback.check() == 1) { //time to check motor feedback
    if(mfeedon == 1){ //if it is on 0 is off 1 is on
      ReadAnalog(j);
      j = j + 1;
      if(j==5){
        j = 0;
      }
    } //end of checking if it is on
  } //end of metro check
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
     startbyte = Serial.read();
     // If it's really the startbyte (255) ...
     if (startbyte == 255){
       // Read command
       for (i=0;i<6;i++){
         command[i] = Serial.read();
       }
       stopbyte = Serial.read();
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
