/**
 * \file MaximumFractalLength.cpp
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

double emgToolbox::MFL()
{
    double Y = 0;
    for (int a = 0; a < (mSize - 1); a++)
    {
        // Find the sum of the next power minus current power to power of two.
        Y = Y + pow(mArr[a+1] - mArr[a], 2);
    }
    return log10(sqrt(Y));
}
