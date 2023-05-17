/**
 * \file DifferenceAbsoluteMeanValue.cpp
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


double emgToolbox::DAMV()
{
    double Y = 0;
    for (int a = 0; a < mSize-1; a++)
    {
        // Find the sum of absolute value from next value minus previous value.
        Y = Y + fabs(mArr[a+1] - mArr[a]);
    }
    return Y / (mSize - 1);
}
