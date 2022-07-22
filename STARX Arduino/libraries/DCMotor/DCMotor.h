/*
* \file DCMotor.h
* DCMOtor library purpose is to control the speed of the motor and it's direction
* any motor to be controlled needs one analog pin and two digital pins
*/
#ifndef DCMotor_h
#define DCMotor_h
#include "Arduino.h"
class DCMotor
{
  public: 
    /*
    * Set up 
    */
    DCMotor(int PWMMOTOR, int HBEN1,int HBEN2);
    /*
    *  to control the speed of the motor you will need to use the PWMMOTOR
    *  which is an analog pin and HBEN1 and HBEN2 are the two pins that 
    *  dictate the direction of the motor and should be connected to the input
    *  pins of the H-bridge
    */
    void motorDir(bool mx, bool my);
    /*
    *  mx and my are the two parameters that contorl the direction 
    *  of the motor; motorDir(0,1) being "Clockwise" and motorDir(1,0) 
    *  being "Counterclockwise" respectively. Design the circuit accordingly
    */
    void pwmMotor(int pwm);
    /*
    *  pwmMotor sets the speed of the motor by setting pwm.
    *  pwm is the Pulse Width Modulation output voltage and goes from 0 to 255
    */
    void rpmMotor(int rpm);
    /*
    *  sets the speed to the desireed rpm
    */
    void motorStop();
    /*
    *  stops the motor
    */
    void 

  private:
    int _PWMMOTOR, _HBEN1, _HBEN2;
    int RPMMAX = 16511;
    /*
    *  max rpm can vary according to the motor specifications and the power supply
    *  delivered to the H-brigde power pin 
    */
    int PWMMAX = 255;
};

#endif