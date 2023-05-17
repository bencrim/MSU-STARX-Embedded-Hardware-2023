/**
 * \file CoefficientOfVariation.cpp
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

double emgToolbox::COV()
{
    // Calculate mean.
    double finalMean = mean(mArr, mSize);
    // Calculate standard deviation.
    double finalStandardDev = standardDev(mArr, mSize, finalMean);
    return finalStandardDev/finalMean;
}
