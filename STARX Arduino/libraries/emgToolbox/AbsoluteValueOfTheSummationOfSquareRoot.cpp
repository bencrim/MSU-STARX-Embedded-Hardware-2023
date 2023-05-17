/**
 * \file AbsoluteValueOfTheSummationOfSquareRoot.cpp
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


double emgToolbox::ASS()
{
    // Counters for complex number arithmetic.
    double real = 0;
    double imag = 0;
    for (int a=0; a < mSize; a++)
    {
        // Complex number calculations.
        if (mArr[a] < 0)
        {
            // Negative number means there is a imaginary number
            // when finding square rooting. So wrap negative in fabs and 
            // add to imaginary counter.
            imag += sqrt(fabs(mArr[a]));
        }
        else
        {
            // Add to the real number counter.
            real += sqrt(mArr[a]);
        }
        
    }
    // Convert complex number to its size for single return value.
    return sqrt(pow(real,2)+pow(imag,2));
}

