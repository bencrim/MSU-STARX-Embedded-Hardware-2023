/**
 * \file AverageEnergy.cpp
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

double emgToolbox::ME()
{
    double Y = 0;
    for (int a=0; a < mSize; a++)
    {
        // Fnd sum of array to the power of 2.
        Y += pow(mArr[a], 2);
        
    }
    return Y / mSize;
}

