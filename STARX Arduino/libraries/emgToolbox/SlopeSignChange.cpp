/**
 * \file SlopeSignChange.cpp
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

double emgToolbox::SSC()
{
    double results = 0;
    for (int a = 1; a < mSize-1; a++)
    {
        // Counter for when the slope sign changes in EMG singal.
        if (((mArr[a] > mArr[a-1] && mArr[a] > mArr[a+1]) || (mArr[a] < mArr[a-1] && mArr[a] < mArr[a+1])) && ((fabs(mArr[a] - mArr[a+1]) >= mOpts) || (fabs(mArr[a] - mArr[a-1]) >= mOpts)) )
        {
            results = results + 1;
        }
    }
    return results;
}
