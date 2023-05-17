/**
 * \file ModifiedMeanAbsoluteValue2.cpp
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

double emgToolbox::MMAV2()
{
    double Y = 0;
    double w = 0;
    for (int a = 1; a < mSize+1; a++)
    {
        // Find weight value range.
        if (a >= 0.25 * mSize && a <= 0.75 * mSize)
        {
            w = 1;
        }
        else if (a < 0.25 * mSize)
        {
            w = (4 * a) / (double)mSize;
        }
        else
        {
            w = 4 * (a - (double)mSize) / (double)mSize;
        }
        // Set the counter to weight * absolute value of previous value.
        Y = Y + (w * fabs(mArr[a-1]));
    }
    double fraction = 1/(double)mSize;
    return fraction * Y;
}

