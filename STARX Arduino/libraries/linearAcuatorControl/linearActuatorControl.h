/*
* \file linearActuatorControl.h
* linearActuatorControl library purpose is to control the speed and length of linear actuator.
* Code tested on Ultra motion servo cylinder A1CZ8C-B0M0E0 for the hip and A1CZ9C-B0M0E0 for the knee.
*/

#ifndef linearActuatorControl_h
#define linearActuatorControl_h
#include "Arduino.h"
class linearActuatorControl
{
    public: 
        /*
        * Set up constructor.
        */
        linearActuatorControl(int avnpin, int in1, int in2, int iopin);

        /*
        * Sets the velocity of the linear actuator.
        * /param velocity Int value between 0-255 that dictates the new velocity of the linear actuator.
        */
        void setVelocity(int velocity);

        /*
        * Update the velocity of the linear actuator by analog write mVelocity to mAnvpin.
        */
        void updateVelocity();

        /*
        * Update length of the linear actuator at this instance of time.
        * /param position Int value between 0-255 that dictates the new setpoint of the linear actuator.
        */
        void setSetpoint(int position);

        /*
        * Update the linear actuator length to new mSetpoint.
        */
        void updateSetpoint();

    private:
        // Pins
        int mAvnpin = 0;
        int mIn1 = 0;
        int mIn2 = 0;
        int mIopin = 0;
        
        // Position and Velocity calculations.
        int mVelocity = 0;
        int mCurrentLength = 0;
        double mDifference = 0;
        double mSetpoint = 0;

        /*
        * Helper function to fetch the linear actuator's current length.
        */
        void fetchCurrentLength();

        /*
        * Helper function to update the difference between linear actuator's current length and new mSetpoint.
        */
        void updateGoalLength();

        /*
        * Extends the linear actuator length by digital write mIn1 High and mIn2 Low.
        */
        void extend();

        /*
        * Contracts the linear actuator length by digital write mIn1 Low and mIn2 High.
        */
        void retract();
};

#endif
