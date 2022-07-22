/**
 * \file EnhancedMeanAbsoluteValue.cpp
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


double emgToolbox::EMAV()
{
    double exp = 0;
    double Y = 0;
    double result = 0;
    // Counters for complex number arithmetic.
    double real = 0;
    double imag = 0;
    for (int a = 1; a < mSize+1; a++)
    {
        // Find which power to use.
        if (a >= (0.2 * mSize) && a <= (0.8 * mSize))
        {
            exp = 0.75;
        }
        else
        {
            exp = 0.5;
        }
        // Complex number calculations
        if (mArr[a-1] < 0)
        {
            // Negative number means there is a imaginary number
            // when finding powers. So wrap negative in fabs and 
            // add to imaginary counter.
            Y += (pow(fabs(mArr[a-1]), exp));
        }
        else
        {
            // Add to the real number counter.
            Y += pow(mArr[a-1], exp);
        }
    }
    return Y / mSize;
}
