#include <SoftPWM.h>

int motor1ph = 4; //motor 1 phase Digital pin 4
int motor1en = 9; //motor 1 enable Digital pin 9 PWM
int motor2ph = 12; //motor 2 phase Digital pin 12
int motor2en = 10; //motor 2 enable Digital pin 10 PWM
int motor3ph = 8; //motor 3 phase Digital pin 8
int motor3en = 5; //motor 3 enable Digital pin 5 PWM
int motor4ph = 7; //motor 4 phase Digital pin 7
int motor4en = 13; //motor 4 enable Digital pin 13 PWM

//Motors
int motor;                         // target motor variable
int pwm;                           // motor speed pwm
int power;
int motordir;                      // motor direction
int motors[4] = {0,0,0,0};         //speed of motor pwm line
int motorsdir[4] = {0,0,0,0};      //direction of motor
int motortargets[4] = {0,0,0,0};   //target speed of motor
int motormovtimes[4] = {0,0,0,0};  //time remaining to get to target speed
float motorspeed;                  //temporary value of motor speed calucaltion

void motor_setup()
{
  pinMode(motor1ph,OUTPUT);
  SoftPWMSet(motor1en,0);
  pinMode(motor2ph,OUTPUT);
  SoftPWMSet(motor2en,0);
  pinMode(motor3ph,OUTPUT);
  SoftPWMSet(motor3en,0);
  pinMode(motor4ph,OUTPUT);
  SoftPWMSet(motor4en,0);

  SoftPWMBegin();
  SoftPWMSetFadeTime(motor1en, 1000, 1000);
  SoftPWMSetFadeTime(motor2en, 1000, 1000);
  SoftPWMSetFadeTime(motor3en, 1000, 1000);
  SoftPWMSetFadeTime(motor4en, 1000, 1000);
}

void MoveMotor(int motor, int motordir, int pwm)
{
  switch (motor)
  {
    case 1:
      if (motordir == 0)
      {
        digitalWrite(motor1ph,LOW);   //set motor direction
        SoftPWMSetPolarity(motor1en,SOFTPWM_NORMAL);
      }
      if (motordir == 1)
      {
        digitalWrite(motor1ph,HIGH);   //set motor direction
        SoftPWMSetPolarity(motor1en,SOFTPWM_INVERTED);
        //SoftPWMSetPercent(motor1en,100-pwm);           //set motor speed
      }
      SoftPWMSetPercent(motor1en,pwm);
      break;

    case 2:
      if (motordir == 0)
      {
        digitalWrite(motor2ph,LOW);   //set motor direction
        SoftPWMSetPolarity(motor2en,SOFTPWM_NORMAL);
        //SoftPWMSetPercent(motor2en,pwm);           //set motor speed
      }
      if (motordir == 1)
      {
        digitalWrite(motor2ph,HIGH);   //set motor direction
        SoftPWMSetPolarity(motor2en,SOFTPWM_INVERTED);
        //SoftPWMSetPercent(motor2en,100-pwm);           //set motor speed
      }
      SoftPWMSetPercent(motor2en,pwm);
      break;

    case 3:
      if (motordir == 0)
      {
        digitalWrite(motor3ph,LOW);   //set motor direction
        SoftPWMSetPolarity(motor3en,SOFTPWM_NORMAL);
        //SoftPWMSetPercent(motor3en,pwm);           //set motor speed
      }
      if (motordir == 1)
      {
        digitalWrite(motor3ph,HIGH);   //set motor direction
        SoftPWMSetPolarity(motor3en,SOFTPWM_INVERTED);
        //SoftPWMSetPercent(motor3en,100-pwm);           //set motor speed
      }
      SoftPWMSetPercent(motor3en,pwm);
      break;

    case 4:
      if (motordir == 0)
      {
        digitalWrite(motor4ph,LOW);   //set motor direction
        SoftPWMSetPolarity(motor4en,SOFTPWM_NORMAL);
        //SoftPWMSetPercent(motor4en,pwm);           //set motor speed
      }
      if (motordir == 1)
      {
        digitalWrite(motor4ph,HIGH);   //set motor direction
        SoftPWMSetPolarity(motor4en,SOFTPWM_INVERTED);
        //SoftPWMSetPercent(motor4en,100-pwm);           //set motor speed
      }
      SoftPWMSetPercent(motor4en,pwm);
      break;
  }
}

void moveMotors()
{
  for (int k = 0; k < 4; k++) //cycle through motor outputs
  {
    if (motors[k] != motortargets[k]) //if motor is not at new command target
    {
      if (motormovtimes[k] < moveint) //if command time is less than 100 milisecond
      {
        motors[k] = motortargets[k];
        power = abs(motors[k]);
        MoveMotor(k+1,motorsdir[k],power); // move motor immediatly
        motormovtimes[k] = 0;
      } //end of if time is less than moveint milisecond
      else //else if time > moveint milisecond
      {
        motorspeed = ((float)(motortargets[k] - motors[k])*moveint)/motormovtimes[k]; //calculated motor speed pwmchange/100ms
        motors[k] = motors[k]+ (int) (motorspeed); // update motor matrix
        power = abs(motors[k]);
        if (motors[k] <0){
          motorsdir[k]=1;
        }
        else
        {
          motorsdir[k]=0;
        }
        MoveMotor(k+1,motorsdir[k],power); // move motor at calculted speed for 10ms
        motormovtimes[k] = motormovtimes[k]-moveint;
      }
    }
  }
}