/**
 * \file DifferenceVarianceValue.cpp
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


double emgToolbox::DVARV()
{
    double Y = 0;
    for (int a = 0; a < mSize-1; a++)
    {
        // Find the square of the difference between next value and current value.
        // Summate the different and increment array until reaching the end.
        Y = Y + pow(fabs(mArr[a+1] - mArr[a]), 2);
    }
    int denominator = (mSize-2);
    return Y / denominator;
}
