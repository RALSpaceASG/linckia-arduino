#include <SoftPWM.h>

static const size_t num_motors = 4;

uint8_t motor_phase_pins[num_motors] = {4, 12, 8, 7};
uint8_t motor_enpwm_pins[num_motors] = {9, 10, 5, 13};

int motor_current[num_motors];     //speed of motor pwm line
int motor_target[num_motors];     //target speed of motor
int motor_dir[num_motors];         //direction of motor
int motor_movtime[num_motors];    //time remaining to get to target speed

void MotorSetup()
{
  for (int m = 0; m < num_motors; m++) {
    pinMode(motor_phase_pins[m], OUTPUT);
    SoftPWMSet(motor_enpwm_pins[m], 0);
    SoftPWMSetFadeTime(motor_enpwm_pins[m], 1000, 1000);
  }
}

void MotorCommand(byte command[6])
{
  int motor = command[1] - 1; // 1-4
  motor_dir[motor] = command[2]; //0 or 1
  motor_target[motor] = motor_dir[motor] ? command[3] : -command[3]; // invert if necessary
  motor_movtime[motor] = ((command[4])*1000)+((command[5])*10); //milliseconds ??? WARNING THIS CAN BE HIGHER THAN 32767!rollover?
}

void moveMotor(int motor, int target)
{
  byte pwm = abs(target);
  bool dir = (target < 0);

  uint8_t ph = motor_phase_pins[motor];
  uint8_t en = motor_enpwm_pins[motor];

  uint8_t pin_state = dir == 0 ? LOW : HIGH;
  uint8_t pwm_state = dir == 0 ? SOFTPWM_NORMAL : SOFTPWM_INVERTED;

  // configure pins
  digitalWrite(ph, pin_state);
  SoftPWMSetPolarity(en, pwm_state);

  // set voltage
  SoftPWMSetPercent(en, pwm);
}

void moveMotors()
{
  for (int k = 0; k < 4; k++) //cycle through motor outputs
  {
    if (motor_current[k] != motor_target[k]) //if motor is not at new command target
    {
      if (motor_movtime[k] < moveint) //if command time is less than 100 milisecond
      {
        motor_current[k] = motor_target[k];
        moveMotor(k, motor_current[k]); // move motor immediatly
        motor_movtime[k] = 0;
      } //end of if time is less than moveint milisecond
      else //else if time > moveint milisecond
      {
        float motorspeed = ((float)(motor_target[k] - motor_current[k])*moveint)/motor_movtime[k]; //calculated motor speed pwmchange/100ms
        motor_current[k] += (int)motorspeed; // update motor matrix

        moveMotor(k, motor_current[k]); // move motor at calculted speed for 10ms
        motor_movtime[k] = motor_movtime[k] - moveint;
      }
    }
  }
}
