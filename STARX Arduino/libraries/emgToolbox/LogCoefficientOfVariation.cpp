/**
 * \file LogCoefficientOfVariation.cpp
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
Log Coefficient Of Variation of emg signal.
@return result double of Log Coefficient Of Variation .
*/
double emgToolbox::LCOV()
{
    // Calculate mean.
    double finalMean = mean(mArr, mSize);
    // Calculate standard deviation.
    double finalStandardDev = standardDev(mArr, mSize, finalMean);
    return log(finalStandardDev/finalMean);
}
