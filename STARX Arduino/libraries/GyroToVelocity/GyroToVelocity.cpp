#include "Arduino.h"
#include "GyroToVelocity.h"


//Suit dimensions in inches
const float hipActMidMntAttchDist = 13;  // Distance from hip flexion joint to hip mid - mount attachment location
const float hipActMidMntLen = 4;     //Distance from center of thigh segment to hip actuator mid - mount attachment(length of hip actuator mid - mount)
const float hipActUpMntDist = 4;     //Distance to upper mount of hip actuator
const float hipFlexJntDist = 5;      //Inferior Distance to Hip Flexion Joint
float hipActThetaFromNormal;


const float kneeLwrMntDist = 4;            // Distance from center of leg segment to actuator attachment
const float kneeUpMntDist = 2.5;         //Distance from center of leg segment to actuator attachment
const float kneeLwrMntDistFromKnee = 3;        //Distance along the suit leg from knee to lower mount(on shank)
const float kneeUpMntDistFromKnee = 14.5;      // Distance along the suit leg from knee to upper mount(on thigh)
float kneeActThetaFromNormal;

const float actUnretractedLen = 12.14;
bool HIPorKNEE = 0;
float theta[5];
float len[3];
float linearVelocity;
void HorK(bool mode)
{
    HIPorKNEE = mode;
}
float geometry(float angularVel, float currentLength)
{
    if (HIPorKNEE == HIP)
    {
        currentLength = currentLength + actUnretractedLen;
        angularVel = angularVel * PI / 180;
        theta[0] = 180 * (atan(hipActMidMntAttchDist / hipActMidMntLen)) / PI;
        theta[1] = 180 * (atan(hipActUpMntDist / hipFlexJntDist)) / PI;
        len[0] = sqrt(pow(hipActMidMntAttchDist, 2) + pow(hipActMidMntLen, 2));
        len[1] = sqrt(pow(hipActUpMntDist, 2) + pow(hipFlexJntDist, 2));
        theta[2] = 180 * (acos((pow(len[1], 2) + pow(len[0], 2) - pow(currentLength, 2)) / (2 * len[0] * len[1]))) / PI;
        len[2] = sqrt(pow(len[0], 2) + pow(hipFlexJntDist, 2) - 2 * len[0] * hipFlexJntDist * cos(PI * (theta[1] + theta[2]) / 180));
        theta[3] = 180 * (acos((pow(len[0], 2) + pow(len[2], 2) - pow(hipFlexJntDist, 2)) / (2 * len[0] * len[2]))) / PI;
        theta[4] = 180 * (acos((pow(len[2], 2) + pow(currentLength, 2) - pow(hipActUpMntDist, 2)) / (2 * len[2] * currentLength))) / PI;
        if (currentLength > 19)
        {
            theta[3] = theta[3] * (-1);
        }
        hipActThetaFromNormal = abs(theta[0] + theta[3] + theta[4] - 90);
        linearVelocity = hipActMidMntLen * angularVel / (cos(PI * hipActThetaFromNormal / 180));
        return linearVelocity;
    }
    else if (HIPorKNEE == KNEE)
    {
        currentLength = currentLength + actUnretractedLen;
        angularVel = angularVel * PI / 180;
        theta[0] = 180 * (atan(kneeLwrMntDistFromKnee / kneeLwrMntDist)) / PI;
        len[0] = sqrt(pow(kneeUpMntDist, 2) + pow(kneeUpMntDistFromKnee, 2));
        len[1] = sqrt(pow(kneeLwrMntDist, 2) + pow(kneeLwrMntDistFromKnee, 2));
        theta[1] = 180 * (acos((pow(currentLength, 2) + pow(len[1], 2) - pow(len[0], 2)) / (2 * currentLength * len[1]))) / PI;
        kneeActThetaFromNormal = abs(theta[0] + theta[1] - 90);
        linearVelocity = kneeLwrMntDist * angularVel / cos(PI * kneeActThetaFromNormal / 180);
        return linearVelocity;
    }
}