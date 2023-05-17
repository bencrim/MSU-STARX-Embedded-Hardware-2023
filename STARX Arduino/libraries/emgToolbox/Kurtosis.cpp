/**
 * \file Kurtosis.cpp
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

double emgToolbox::kurtosisNumerator(double mean) 
{
    double var = 0.0;
    for (int a = 0; a < mSize; ++a) 
    {
        var += pow(mArr[a] - mean, 4);
    }
    var = var / (mSize);
    return var;
}


double emgToolbox::kurtosisDenominator(double mean) 
{
    double var = 0.0;
    for (int a = 0; a < mSize; ++a) 
    {
        var += pow(mArr[a] - mean, 2);
    }
    var = var / (mSize);
    var = pow(var, 2);
    return var;
}



double emgToolbox::KURT()
{
    // Calculate mean.
    double finalMean = mean(mArr, mSize);
    // Calculate modified standard deviations for numerator and denominator.
    double numerator = kurtosisNumerator(finalMean);
    double denominator = kurtosisDenominator(finalMean);
    return numerator/denominator;
}


