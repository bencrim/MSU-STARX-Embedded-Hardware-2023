/**
 * \file NewZeroCrossing.cpp
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

double emgToolbox::FZC()
{
    double T = 0;
    double sum = 0;
    int result = 0;
    // Compute T.
    for (int a = 0; a < 10; a++)
    {
        sum += mArr[a];
    }
    T = 4 * (0.1 * sum);
    // Compute proposed zero crossing.
    for (int a = 1; a < mSize-1; a++)
    {
        if ( (mArr[a] > T && mArr[a+1] < T) || (mArr[a] < T && mArr[a+1] > T) )
        {
            result = result + 1;
        }
    }
    return result;

}
