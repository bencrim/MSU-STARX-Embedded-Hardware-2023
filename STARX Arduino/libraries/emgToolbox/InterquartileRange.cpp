/**
 * \file InterquartileRange.cpp
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


double emgToolbox::IQR()
{
    mergeSort(mArr, 0, mSize-1);
    // Find first quartile.
    double Q1 = getMedian(mArr, 0, mSize / 2 - 1);
    // Find median.
    double Q2 = getMedian(mArr, 0, mSize - 1);
    // Find third quartile.
    double Q3;
    if (mSize % 2 == 0) 
    {
        Q3 = getMedian(mArr, mSize / 2, mSize - 1);
    } else {
        Q3 = getMedian(mArr, mSize / 2 + 1, mSize - 1);
    }
    return Q3-Q1;

}
