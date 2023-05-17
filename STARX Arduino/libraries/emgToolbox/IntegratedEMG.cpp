/**
 * \file IntegratedEMG.cpp
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
Integrated EMG of emg signal.
@return result double of Integrated EMG.
*/
double emgToolbox::IEMG()
{
    double exp = 0;
    double Y = 0;
    for (int a = 0; a < mSize; a++)
    {
        // Find the sum of the absolute value of current value.
        Y += fabs(mArr[a]);

    }
    return Y;
}
