/**
 * \file LogDifferenceAbsoluteStandardDeviationValue.cpp
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


double emgToolbox::LDASDV()
{
    double Y = 0;
    for (int a = 0; a < (mSize - 1); a++)
    {
        // Sum the next value minus current value to the power of 2.
        Y = Y + pow(fabs(mArr[a+1] - mArr[a]), 2);
    }
    return log(sqrt(Y / (mSize-1) ));
}
