/**
 * \file SimpleSquareIntegral.cpp
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

double emgToolbox::SSI()
{
    double sum = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Find summation of current value to the power of two.
        sum += pow(mArr[a], 2);
    }
    return sum;
}
