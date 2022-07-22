/**
 * \file VOrder.cpp
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

double emgToolbox::VO()
{
    double sum = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Find the power of current value with mOpts value.
        // Sum these values until reaching end of singal EMG.
        sum += pow(mArr[a], mOpts);
    }
    double fraction = 1/(double)(mSize);
    double Y = fraction * sum;
    double fraction2 = 1/(double)(mOpts);
    return pow(Y, fraction2);
}
