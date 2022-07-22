/**
 * \file EnhancedWaveLength.cpp
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
Enhanced Wave Length of emg signal.
@return result double of Enhanced Wave Length.
*/
double emgToolbox::EWL()
{
    double exp = 0;
    double Y = 0;
    for (int a = 2; a < mSize+1; a++)
    {
        // Find the power to use.
        if (a >= (0.2 * mSize) && a <= (0.8 * mSize))
        {
            exp = 0.75;
        }
        else
        {
            exp = 0.5;
        }
        // Find the dum of the absolute value of previous value to two previous values ago.
        // Then find the power of the exponent found above.
        Y += pow(fabs(mArr[a-1]-mArr[a-2]), exp);
    }
    return Y;
}

