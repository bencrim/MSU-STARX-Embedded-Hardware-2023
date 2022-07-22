/**
 * \file LogDetector.cpp
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

double emgToolbox::LD()
{
    double Y = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Sum log base of the absolute value of current value.
        Y = Y + log(fabs(mArr[a]));
    }
    return exp(Y / mSize);
}
