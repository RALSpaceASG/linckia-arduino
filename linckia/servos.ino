#include <Servo.h>

Servo servo1; //Digital pin 3
Servo servo2; //Digital pin 11
Servo servo3; //Digital pin 6

// Common servo setup values
#define MIN_PULSE 1050 // minimum servo position, us microseconds
#define MAX_PULSE 1950 // maximum servo position, us microseconds

// Servos
int servo;                                                // target servo variable
int pos;                                                  // servo position
int servos[3] = {0,0,0};                                  //current postition of servos
int servotargets[3] = {90,90,0};                          //target position of servos
int servomovtimes[3] = {0,0,0};                           //time remaining to get to target position
float servospeed;                                         //temporary value for servo speed calculation

void ServoSetup()
{
  servo1.attach(3,  MIN_PULSE, MAX_PULSE); //attach servo 1 pwm to Digital pin 3
  servo2.attach(11, MIN_PULSE, MAX_PULSE); //attach servo 2 pwm to Digital pin 11
  servo3.attach(6,  MIN_PULSE, MAX_PULSE); //attach servo 3 pwm to Digital pin 6
}

void servoMove(int servo, int pos){
  switch (servo) {
    case 1:
      servo1.write(pos);       // move servo1 to 'pos'
      break;
    case 2:
      servo2.write(pos);       // move servo2 to 'pos'
      break;
    case 3:
      servo3.write(pos);       // move servo3 to 'pos'
      break;
  }
}

void ServoCommand(byte command[6])
{
  int servo = command[1] - 1; // 1-3
  servotargets[servo] = command[2]; // 0-180
  servomovtimes[servo] = ((command[4])*1000)+((command[5])*10); //milliseconds ??? WARNING THIS CAN BE HIGHER THAN 32767!rollover?
}

void moveServos()
{
  for (int k = 0; k < 3; k++) //cycle through pwm outputs
  {
    if (servos[k] != servotargets[k]) //if servo is not at new command target
    {
      if (servomovtimes[k] < moveint) //if command time is less than 100 milisecond
      {
        servos[k] = servotargets[k];
        servoMove(k+1,servos[k]); //move immediately
        servomovtimes[k] = 0;
      } //end of if time is less than moveint milisecond
      else //else if time > moveint milisecond
      {
        servospeed = ((float)(servotargets[k] - servos[k])*moveint)/servomovtimes[k]; //calculated servo speed pwmchange/100ms
        servos[k] = servos[k]+ (int) (servospeed); // update servo matrix
        servoMove(k+1,servos[k]); // move servo at calculted speed for 10ms
        servomovtimes[k] = servomovtimes[k]-moveint;
      }
    }
  }
}
