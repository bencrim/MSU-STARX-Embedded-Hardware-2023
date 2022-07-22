/**
 * \file RootMeanSquare.cpp
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


double emgToolbox::RMS()
{
    // Calculate mean.
    double sum = 0;
    for (int a = 0; a < mSize; ++a) 
    {
        // Sum the current value to the power of two.
        sum += pow(mArr[a], 2);
    }
    return sqrt(sum/(double)mSize);
}


