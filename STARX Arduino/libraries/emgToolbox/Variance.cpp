/**
 * \file Variance.cpp
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

double emgToolbox::VAR()
{
    double finalMean = mean(mArr, mSize);
    double sum = 0;
    // Find the variance of EMG singal.
    for (int a = 0; a < mSize; a++)
    {
        // Sum the current value minus the mean of EMG singal, to the power of two.
        sum += pow(mArr[a] - finalMean, 2);
    }
    // Population variance by 1/(size-1).
    double fraction = 1/(double)(mSize-1);
    return fraction * sum;
}
