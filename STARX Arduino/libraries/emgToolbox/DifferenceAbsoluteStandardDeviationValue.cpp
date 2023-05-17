/**
 * \file DifferenceAbsoluteStandardDeviationValue.cpp
 *
 * \author Jordan Hybki (@jh9587)
 * 
 * Pseudocode referenced from MATLAB EMG Feature Extraction Toolbox:
 * https://www.mathworks.com/matlabcentral/fileexchange/71514-emg-feature-extraction-toolbox
 * 
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "emgToolbox.h"

double emgToolbox::DASDV()
{
    double Y = 0;
    for (int a = 0; a < mSize-1; a++)
    {
        // Sum the next value minus current value to the power of two.
        Y = Y + pow(fabs(mArr[a+1] - mArr[a]), 2);
    }
    int denominator = (mSize-1);
    Y = Y / denominator;
    return sqrt(Y);
}

