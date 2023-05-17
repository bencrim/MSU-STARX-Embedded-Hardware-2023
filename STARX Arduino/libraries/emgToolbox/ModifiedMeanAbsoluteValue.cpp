/**
 * \file ModifiedMeanAbsoluteValue.cpp
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

double emgToolbox::MMAV()
{
    double Y = 0;
    double w = 0;
    for (int a = 1; a < mSize+1; a++)
    {
        // Find weight value.
        if (a >= 0.25 * mSize && a <= 0.75 * mSize)
        {
            w = 1;
        }
        else
        {
            w = 0.5;
        }
        // Set the counter to weight value * previous value.
        Y = Y + (w * fabs(mArr[a-1]));
    }
    double fraction = 1/(double)mSize;
    return fraction * Y;
}
