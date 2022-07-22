/**
 * \file TemporalMoment.cpp
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

double emgToolbox::TM()
{
    double sum = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Find the current value power to mOpts as a sum of each value
        // in mArr.
        sum += pow(mArr[a], mOpts);
    }
    double fraction = 1/(double)mSize;
    return fraction * sum;
    
}
