/**
 * \file MeanAbsoluteDeviation.cpp
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

/*
Mean Absolute Deviation of emg signal.
@return result double of Mean Absolute Deviation.
*/
double emgToolbox::MAD()
{
    double finalMean = mean(mArr, mSize);
    double sum = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Find the sum of changes to current value minus the signals mean value.
        sum += fabs(mArr[a] - finalMean);
    }
    double fraction = 1/(double)mSize;
    return fraction * sum;
}

