/**
 * \file Skewness.cpp
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

double emgToolbox::skewnessNumerator(double mean) 
{
    double var = 0.0;
    for (int a = 0; a < mSize; ++a) 
    {
        var += pow(mArr[a] - mean, 3);
    }
    double fraction = 1/(double)mSize;
    var = var * fraction;
    return var;
}

double emgToolbox::skewnessDenominator(double mean) 
{
    double var = 0.0;
    for (int a = 0; a < mSize; ++a) 
    {
        var += pow(mArr[a] - mean, 2);
    }
    double fraction = 1/(double)mSize;
    var = var * fraction;
    var = sqrt(var);
    var = pow(var, 3);
    return var;
}


double emgToolbox::SKEW()
{
    // Calculate mean.
    double finalMean = mean(mArr, mSize);
    // Calculate modified standard deviation for numerator and denominator.
    double numerator = skewnessNumerator(finalMean);
    double denominator = skewnessDenominator(finalMean);
    return numerator/denominator;
}




