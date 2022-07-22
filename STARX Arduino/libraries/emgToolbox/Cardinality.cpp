/**
 * \file Cardinality.cpp
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


double emgToolbox::CARD()
{
    // Sort from smallest to largest values. Using merge sort.
    mergeSort(mArr, 0, mSize - 1);
    double Y = 0;
    for (int a = 0; a < mSize - 1; a++)
    {
        if (fabs(mArr[a]-mArr[a+1]) > mOpts)
        {
            // Counter for when change in current value versus next value is greater than opts.
            Y += 1;
        }
    }
    return Y;
}
