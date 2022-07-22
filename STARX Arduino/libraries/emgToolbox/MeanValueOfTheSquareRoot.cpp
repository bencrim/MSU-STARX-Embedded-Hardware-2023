/**
 * \file MeanValueOfTheSquareRoot.cpp
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


double emgToolbox::MSR()
{
    // Counter for complex numbers.
    double imag = 0;
    double real = 0;
    
    for (int a = 0; a < mSize; a++)
    {
        // Complex number calculations.
        std::complex<double> z2(mArr[a], 0);
        std::complex<double> complexNum = std::pow(z2, 0.5); // mArr[a] ^ (1/2).

        imag += complexNum.imag();
        real += complexNum.real();

    }
    // Convert complex number to its size for single return value.
    double result = sqrt(pow(real,2)+pow(imag,2));
    return result/mSize;
}
