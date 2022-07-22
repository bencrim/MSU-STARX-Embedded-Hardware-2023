/**
 * \file AverageAmplitudeChange.cpp
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

double emgToolbox::AAC()
{
    double Y = 0;
    for (int a=0; a < mSize-1; a++)
    {
        // Find absolute value of next value minus current value in mArr.
        Y = Y + fabs(mArr[a+1] - mArr[a]);
        
    }
    return Y / mSize;
}
