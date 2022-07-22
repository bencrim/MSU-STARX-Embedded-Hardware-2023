/**
 * \file MeanAbsoluteValue.cpp
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

double emgToolbox::MAV()
{
    double sum = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Find the sum of absolute value for current value.
        sum += fabs(mArr[a]);
    }
    return sum/mSize;
}
