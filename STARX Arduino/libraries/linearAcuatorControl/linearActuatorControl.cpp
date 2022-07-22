/*
* \file linearActuatorControl.cpp
* Defines linearActuatorControl definitions in order to control the speed and length of linear actuators.
* Code tested on Ultra motion servo cylinder A1CZ8C-B0M0E0 for the hip and A1CZ9C-B0M0E0 for the knee.
*/

#include "Arduino.h"
#include "linearActuatorControl.h"

linearActuatorControl::linearActuatorControl(int avnpin, int in1, int in2, int iopin)
{
    /*
    * Set up of the pins being used to control the linear actuator.
    */
    mAvnpin = avnpin;
    mIn1 = in1;
    mIn2 = in2;
    mIopin = iopin;
    pinMode(mAvnpin, OUTPUT);
    pinMode(mIn1, OUTPUT);
    pinMode(mIn2, OUTPUT);
    pinMode(mIopin, INPUT);
}

void linearActuatorControl::setVelocity(int velocity)
{
    // Ensure that velocity is between 0-255 (indicating valid digital voltage range).
    if (velocity <= 255 && velocity >= 0)
    {
        mVelocity = velocity;   // Speed must be between 0-255.
    }
    else
    {
        Serial.print("ERROR: Please enter velocity value between 0 to 255.");
        exit(0);
    }
}

void linearActuatorControl::updateVelocity()
{
    analogWrite(mAvnpin, mVelocity); // Changes speed on values 0-255.
}

void linearActuatorControl::setSetpoint(int position)
{
    // Ensure that position is between 0-255 (indicating valid digital voltage range).
    if (position <= 255 && position >= 0)
    {
        mSetpoint = position;   // Position must be between 0-255.
    }
    else
    {
        Serial.print("ERROR: Please enter position value between 0 to 1023.");
        exit(0);
    }
}

void linearActuatorControl::updateSetpoint()
{
    fetchCurrentLength();
    updateGoalLength();
    
    if (mDifference > mCurrentLength) // Setpoint is greater length than current length.
    {
        retract(); // retract the stroke
    }
    else if (mDifference < mCurrentLength) // Setpoint is less than length of current length.
    {
        extend(); // extend the stroke
    }
    else // Setpoint is eqivalent length than current length.
    {
        // Do nothing.
    }
}

void linearActuatorControl::fetchCurrentLength()
{
    // Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, a value of 
    // fromHigh to toHigh, values in-between to values in-between, etc.
    mCurrentLength = analogRead(map(mIopin, 0, 1023, 0, 255));     // Reads the current input value between 0 - 255.
    Serial.print("Current Linear Actuator Length: ");
    Serial.print(mCurrentLength);
}

void linearActuatorControl::updateGoalLength()
{
    mDifference = mSetpoint - mCurrentLength;    // Finds if we need to extend/contract from.
    Serial.print("Goal Linear Actuator Length: ");
    Serial.print(mDifference);
}

void linearActuatorControl::extend() 
{
    Serial.println("Extending...");
    digitalWrite(mIn1, HIGH);
    digitalWrite(mIn2, LOW);
}

void linearActuatorControl::retract() 
{
    Serial.println("Retracting...");
    digitalWrite(mIn1, LOW);
    digitalWrite(mIn2, HIGH);
}
