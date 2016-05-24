#include <Servo.h>

Servo servo_devices[3];

static const uint8_t servo_pins[3] = {3, 11, 6};

// maximum and minimum servo positions in microseconds
static const int servo_min_pulse = 1050;
static const int servo_max_pulse = 1950;

// variable storage
int servo_current[3]  = {0, 0, 0}; //current postition of servos
int servo_target[3]   = {90,90,0}; //target position of servos
int servo_movtimes[3] = {0, 0, 0}; //time remaining to get to target position

void ServoSetup() {
  for (int i = 0; i < 3; i++) {
    servo_devices[i].attach(servo_pins[i], servo_min_pulse, servo_max_pulse);
  }
}

void servoMove(int servo, int pos) {
  servo_devices[servo+1].write(pos);
}

void ServoCommand(byte command[6]) {
  int servo = command[1] - 1; // 1-3
  servo_target[servo] = command[2]; // 0-180
  servo_movtimes[servo] = ((command[4])*1000)+((command[5])*10); //milliseconds ??? WARNING THIS CAN BE HIGHER THAN 32767!rollover?
}

void moveServos() {
  for (int k = 0; k < 3; k++) //cycle through pwm outputs
  {
    if (servo_current[k] != servo_target[k]) //if servo is not at new command target
    {
      if (servo_movtimes[k] < ulActuatorsInterval) //if command time is less than 100 milisecond
      {
        servo_current[k] = servo_target[k];
        servoMove(k+1,servo_current[k]); //move immediately
        servo_movtimes[k] = 0;
      } //end of if time is less than ulActuatorsInterval milisecond
      else //else if time > ulActuatorsInterval milisecond
      {
        float servospeed = ((float)(servo_target[k] - servo_current[k])*ulActuatorsInterval)/servo_movtimes[k]; //calculated servo speed pwmchange/100ms
        servo_current[k] = servo_current[k]+ (int) (servospeed); // update servo matrix
        servoMove(k+1,servo_current[k]); // move servo at calculted speed for 10ms
        servo_movtimes[k] = servo_movtimes[k]-ulActuatorsInterval;
      }
    }
  }
}
