#include "Arduino.h"
#include "DCMotor.h"

DCMotor::DCMotor(int PWMMOTOR, int HBEN1, int HBEN2)
{
  _PWMMOTOR = PWMMOTOR;
  _HBEN1 = HBEN1;
  _HBEN2 = HBEN2;
  pinMode(_PWMMOTOR, OUTPUT);
  pinMode(_HBEN1, OUTPUT);
  pinMode(_HBEN2, OUTPUT);
 /*
 * set up of the pins being used to control the motor
 */
    
}

void DCMotor::motorDir(bool mx, bool my)
{
   digitalWrite(_HBEN1,mx);
   digitalWrite(_HBEN2,my);
   /*
   *  sets up the direction of the motor
   */
}

void DCMotor::pwmMotor(int pwm)
{
   analogWrite(_PWMMOTOR, pwm);
   /*
   * sets the pwm output voltage
   */
}

void DCMotor::rpmMotor(int rpm)
{
   rpm = map(rpm, 0, RPMMAX, 0, PWMMAX);
   pwmMotor(rpm);
   /*
   * maps the range of the possible rpm of the motor
   * to the range of the pwm
   */
}

void DCMotor::motorStop()
{
    motorDir(0, 0);
}
