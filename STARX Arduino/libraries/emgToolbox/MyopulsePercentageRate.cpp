/**
 * \file MyopulsePercentageRate.cpp
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
#include <iostream>

#include "emgToolbox.h"

double emgToolbox::MYOP()
{
    double Y = 0;
    for (int a = 0; a < mSize; a++)
    {
        if (fabs(mArr[a]) >= mOpts)
        {
            // Counter to when mOpts is greater than or equal to current value.
            Y += 1;
        }
    }
    return Y / (double)mSize;

}
