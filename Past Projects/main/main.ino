#include "linearActuatorControl.h"

// Array of Movement label values:
// Array indexing: {Hip setpoint, Hip velocity}
float mPreswing[] = {213.0335049, 23.17901726};      // Terminal_Stance_to_Pre_Swing
float mToeOff[] = {205.4560785, 77.04381417};      // Pre_Swing_to_Toe_Off
float mTerminalSwing[] = {158.644927, 101.3723864};      // Toe_Off_to_Terminal_Swing
float mHealStrike[] = {139.2596156, 34.37931672};      // Terminal_Swing_to_Heal_Strike
float mLoadingResponse[] = {172.402608, 38.20719649};      // Heal_Strike_to_Loading_Response
float mMidStance[] = {197.4034404, 22.16240141};      // Loading_Response_to_Mid_Stance
float mTerminalStance[] = {209.5427921, 18.92111077};  // Mid_Stance_to_Terminal_Stance

// Construct a linearActuator object for Hip. (Hip Actuator)
// Parameters set at: anvpin, in1, in2, iopin
linearActuatorControl hip(7, 3, 4, 5);
    
/*
* Setup code that runs once before loop execution function.
*/
void setup() 
{

}

/*
* Loop code that runs repeatedly on every tick of the microprocessor clock cycle.
*/
void loop()
{
    Serial.println("Choose number between 0-6 (Pre-swing to Terminal Stance) to set linear actuator to?");     // Prompt user for input.
    while (Serial.available() == 0) 
    {
        // Wait for User to Input Data
    }
    int svmLabel = Serial.parseInt(); // Read the data the user has input
    finiteStateMachine(svmLabel, hip);
}

/*
* Linear Actuator Function - Finite State Machine that changes position and speed of linear actuator depending on SVM label.
* /param svmLabel Int between 0-8 that indicates phase in walking gait cycle.
* /param knee linearActuatorControl object that represents knee linear actuator.
* /param hip linearActuatorControl object that represents hip linear actuator.
*/
void finiteStateMachine(int svmLabel, linearActuatorControl hip)
{
    switch (svmLabel)
    {
        case 0:     // Terminal_Stance_to_Pre_Swing
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mPreswing[1]);
            hip.updateVelocity();
            hip.setSetpoint(mPreswing[0]);
            hip.updateSetpoint();
            break;
        case 1:     // Pre_Swing_to_Toe_Off
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mToeOff[1]);
            hip.updateVelocity();
            hip.setSetpoint(mToeOff[0]);
            hip.updateSetpoint();
            break;
        case 2:     // Toe_Off_to_Terminal_Swing
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mTerminalSwing[1]);
            hip.updateVelocity();
            hip.setSetpoint(mTerminalSwing[0]);
            hip.updateSetpoint();
            break;
        case 3:     // Terminal_Swing_to_Heal_Strike
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mHealStrike[1]);
            hip.updateVelocity();
            hip.setSetpoint(mHealStrike[0]);
            hip.updateSetpoint();
            break;
        case 4:     // Heal_Strike_to_Loading_Response
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mLoadingResponse[1]);
            hip.updateVelocity();
            hip.setSetpoint(mLoadingResponse[0]);
            hip.updateSetpoint();
            break;
        case 5:     // Loading_Response_to_Mid_Stance
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mMidStance[1]);
            hip.updateVelocity();
            hip.setSetpoint(mMidStance[0]);
            hip.updateSetpoint();
            break;
        case 6:     // Mid_Stance_to_Terminal_Stance
            // Hip linear actuator speed and setpoint set.
            hip.setVelocity(mTerminalStance[1]);
            hip.updateVelocity();
            hip.setSetpoint(mTerminalStance[0]);
            hip.updateSetpoint();
            break;
    }
}
