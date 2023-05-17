/**
 * \file WaveformLength.cpp
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

double emgToolbox::WL()
{
    double sum = 0;
    for (int a = 1; a < mSize; a++)
    {
        // Find the waveform length by suming the absolute value of 
        // next value minus previous value.
        sum += fabs(mArr[a] - mArr[a-1]);
    }
    return sum;
}
