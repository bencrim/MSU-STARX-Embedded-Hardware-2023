/**
 * \file AbsoluteValueOfTheSummationOfExpRoot.cpp
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
#include <complex>
#include "emgToolbox.h"


double emgToolbox::ASM()
{
    double exp = 0;
    double Y = 0;
    // Counters for complex number arithmetic.
    double real = 0;
    double imag = 0;
    double result = 0;
    for (int a = 1; a < mSize+1; a++)
    {
        // Find the power value.
        if (a >= (0.25 * mSize) && a <= (0.75 * mSize))
        {
            exp = 0.5;
        }
        else
        {
            exp = 0.75;
        }
        // Complex number calculations
        std::complex<double> z2(mArr[a-1], 0);
        auto complexNum = std::pow(z2, exp); // mArr[a-1] ^ exp.

        imag += complexNum.imag();
        real += complexNum.real();
    }
    // Convert complex number to its size for single return value.
    result = sqrt(pow(real,2)+pow(imag,2));

    result = result/mSize;
    return result;
}
