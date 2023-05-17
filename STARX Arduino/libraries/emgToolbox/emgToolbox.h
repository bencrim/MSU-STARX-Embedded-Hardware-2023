/**
 * \file emgToolbox.h
 *
 * \author Jordan Hybki (@jh9587)
 * 
 * Library to interface with 39 emg feature extraction algorithms.
 * Pseudocode referenced from MATLAB EMG Feature Extraction Toolbox:
 * https://www.mathworks.com/matlabcentral/fileexchange/71514-emg-feature-extraction-toolbox
 * 
 */


#ifndef emgToolbox_h
#define emgToolbox_h

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "Arduino.h"

//!  emgToolbox class. 
/*!
   This class offers 39 different time series feature extractions for Electromyography (EMG) signals.
*/
 
class emgToolbox
{
  public:
    //! A constructor that initalises input for EMG feature calculations.
    /*!
      \param arr Array of doubles that stores EMG singal.
      \param size Int of the size of array that stores EMG signal.
      \param opts Double of the extra parameter required for some feature calculations.
    */
    emgToolbox(double *arr, int size, double opts)
    {
        // Store input as respective member variable of class emgToolbox.
        mArr = arr;
        mSize = size;
        mOpts = opts;
    }

    //! Finds the absolute value of the summation of exp root for EMG signal.
    /*!
      \return result Double of the absolute value of the summation of exp root.
    */
    double ASM();
    //! Finds the absolute value of the summation of square root for EMG signal.
    /*!
      \return result Double of the absolute value of the summation of square root.
    */
    double ASS();
    //! Finds the average amplitude change for EMG signal.
    /*!
      \return result Double of the average amplitude change.
    */
    double AAC();
    //! Finds the average amplitude change for EMG signal.
    /*!
      \return result Double of the average amplitude change.
    */
    double ME();
    //! Finds the cardinality for EMG signal.
    /*!
      \return result Double of the cardinality.
    */
    double CARD();
    //! Finds the coefficient of variation for EMG signal.
    /*!
      \return result Double of the coefficient of variation.
    */
    double COV();
    //! Finds the difference absolute mean value for EMG signal.
    /*!
      \return result Double of the difference absolute mean value.
    */
    double DAMV();
    //! Finds the difference absolute standard deviation value for EMG signal.
    /*!
      \return result Double of the difference absolute standard deviation value.
    */
    double DASDV();
    //! Finds the difference variance value for EMG signal.
    /*!
      \return result Double of the difference variance value.
    */
    double DVARV();
    //! Finds the enhanced mean absolute value for EMG signal.
    /*!
      \return result Double of the enhanced mean absolute value.
    */
    double EMAV();
    //! Finds the enhanced wave length for EMG signal.
    /*!
      \return result Double of the enhanced wave length.
    */
    double EWL();
    //! Finds the integrated EMG for EMG signal.
    /*!
      \return result Double of the integrated EMG.
    */
    double IEMG();
    //! Finds the interquartile range for EMG signal.
    /*!
      \return result Double of the interquartile range.
    */
    double IQR();
    //! Finds the kurtosis for EMG signal.
    /*!
      \return result Double of the kurtosis.
    */
    double KURT();
    //! Finds the log coefficient of variation for EMG signal.
    /*!
      \return result Double of the log coefficient of variation.
    */
    double LCOV();
    //! Finds the log detector for EMG signal.
    /*!
      \return result Double of the log detector.
    */
    double LD();
    //! Finds the log difference absolute mean value for EMG signal.
    /*!
      \return result Double of the log difference absolute mean value.
    */
    double LDAMV();
    //! Finds the log difference absolute standard deviation value for EMG signal.
    /*!
      \return result Double of the log difference absolute standard deviation value.
    */
    double LDASDV();
    //! Finds the log teager kaiser energy operator for EMG signal.
    /*!
      \return result Double of the log teager kaiser energy operator.
    */
    double LTKEO();
    //! Finds the myopulse percentage rate for EMG signal.
    /*!
      \return result Double of the myopulse percentage rate.
    */
    double MYOP();
    //! Finds the maximum fractal length for EMG signal.
    /*!
      \return result Double of the maximum fractal length.
    */
    double MFL();
    //! Finds the modified mean absolute value 2 for EMG signal.
    /*!
      \return result Double of the modified mean absolute value 2.
    */
    double MMAV2();
    //! Finds the modified mean absolute value for EMG signal.
    /*!
      \return result Double of the modified mean absolute value.
    */
    double MMAV();
    //! Finds the mean value of the square root for EMG signal.
    /*!
      \return result Double of the mean value of the square root.
    */
    double MSR();
    //! Finds the mean absolute value for EMG signal.
    /*!
      \return result Double of the mean absolute value.
    */
    double MAV();
    //! Finds the mean absolute deviation for EMG signal.
    /*!
      \return result Double of the mean absolute value.
    */
    double MAD();
    //! Finds the new zero crossing for EMG signal.
    /*!
      \return result Double of the new zero crossing .
    */
    double FZC();
    //! Finds the root mean square for EMG signal.
    /*!
      \return result Double of the root mean square.
    */
    double RMS();
    //! Finds the simple square integral for EMG signal.
    /*!
      \return result Double of the simple square integral.
    */
    double SSI();
    //! Finds the skewness for EMG signal.
    /*!
      \return result Double of the skewness.
    */
    double SKEW();
    //! Finds the slope sign change for EMG signal.
    /*!
      \return result Double of the slope sign change.
    */
    double SSC();
    //! Finds the standard deviation for EMG signal.
    /*!
      \return result Double of the standard deviation.
    */
    double SD();
    //! Finds the temporal moment for EMG signal.
    /*!
      \return result Double of the temporal moment.
    */
    double TM();
    //! Finds the variance for EMG signal.
    /*!
      \return result Double of the variance.
    */
    double VAR();
    //! Finds the variance of EMG for EMG signal.
    /*!
      \return result Double of the variance of EMG.
    */
    double VAREMG();
    //! Finds the VOrder for EMG signal.
    /*!
      \return result Double of the VOrder.
    */
    double VO();
    //! Finds the waveform length for EMG signal.
    /*!
      \return result Double of the waveform length.
    */
    double WL();
    //! Finds the wilson amplitude for EMG signal.
    /*!
      \return result Double of the wilson amplitude.
    */
    double WA();
    //! Finds the zero crossing for EMG signal.
    /*!
      \return result Double of the zero crossing.
    */
    double ZC();
    
  protected:

    
  private:
    // Array of double that stores emg singal.
    double *mArr;

    // Size of array that stores emg signal.
    int mSize;

    // Operands value that is required for some feature algorithms.
    double mOpts;

    //! Finds the numerator value of kurtosis for EMG signal.
    /*!
      \param mean Double of the average EMG singal value.
      \return result Double of the numerator value of kurtosis.
    */
    double kurtosisNumerator(double mean);
    //! Finds the denominator value of kurtosis for EMG signal.
    /*!
      \param mean Double of the average EMG singal value.
      \return result Double of the denominator value of kurtosis.
    */
    double kurtosisDenominator(double mean);
    //! Finds the numerator value of skewness for EMG signal.
    /*!
      \param mean Double of the average EMG singal value.
      \return result Double of the numerator value of skewness.
    */
    double skewnessNumerator(double mean);
    //! Finds the denominator value of skewness for EMG signal.
    /*!
      \param mean Double of the average EMG singal value.
      \return result Double of the denominator value of skewness.
    */
    double skewnessDenominator(double mean);

    //! Merges two subarrays of arr[]. First subarray is arr[l..m] and second subarray is arr[m+1..r]. Referenced here: https://www.geeksforgeeks.org/merge-sort/.
    /*!
      \param  arr Double array of emg signal values.
      \param  l Int of left index.
      \param  m Int of middle index.
      \param  r Int of right index.
    */
    void merge(double arr[], int l, int m, int r)
    {
        int i, j, k;
        int n1 = m - l + 1;
        int n2 = r - m;
    
        /* create temp arrays */
        double L[n1], R[n2];
    
        /* Copy data to temp arrays L[] and R[] */
        for (i = 0; i < n1; i++)
            L[i] = arr[l + i];
        for (j = 0; j < n2; j++)
            R[j] = arr[m + 1 + j];
    
        /* Merge the temp arrays back into arr[l..r]*/
        i = 0; // Initial index of first subarray
        j = 0; // Initial index of second subarray
        k = l; // Initial index of merged subarray
        while (i < n1 && j < n2) 
        {
            if (L[i] <= R[j]) 
            {
                arr[k] = L[i];
                i++;
            }
            else 
            {
                arr[k] = R[j];
                j++;
            }
            k++;
        }
    
        /* Copy the remaining elements of L[], if there
        are any */
        while (i < n1) 
        {
            arr[k] = L[i];
            i++;
            k++;
        }
    
        /* Copy the remaining elements of R[], if there
        are any */
        while (j < n2) 
        {
            arr[k] = R[j];
            j++;
            k++;
        }
    }

    //! Merge sort implementation on double arrays. Referenced here: https://www.geeksforgeeks.org/merge-sort/.
    /*!
      \param  arr Double array of emg signal values.
      \param  l Int of left index.
      \param  r Int of right index.
    */
    void mergeSort(double arr[], double l, double r)
    {
        if (l < r) 
        {
            // Same as (l+r)/2, but avoids overflow for
            // large l and h
            double m = l + (r - l) / 2;
    
            // Sort first and second halves
            mergeSort(arr, l, m);
            mergeSort(arr, m + 1, r);
            merge(arr, l, m, r);
        }
    }

    //! Finds the mean of EMG singal array.
    /*!
      \param  arr Double array of emg signal values.
      \param  size Int of the size of emg signal values.
      \return the average value of emg singal values.
    */
    double mean(double *arr, int size)
    {
        double sum = 0;
        for (int a = 0; a < size; ++a) {
            sum += arr[a];
        }
        return sum/size;
    }

    //! Finds the standard deviation of EMG singal array.
    /*!
      \param  arr Double array of emg signal values.
      \param  size Int of the size of emg signal values.
      \param  mean Double of the mean of EMG singal array.
      \return the stanard deviation value of emg singal values.
    */
    double standardDev(double arr[], int size, double mean) 
    {
        double var = 0.0;
        for (int a = 0; a < size; ++a) 
        {
            var += pow(arr[a] - mean, 2);
        }
        var = var / (size-1);
        return sqrt(var);
    }

    //! Finds the median of EMG singal array.
    /*!
      \param  arr Double array of emg signal values.
      \param  start Int of the starting index.
      \param  end Int of the ending index.
      \return the median of emg singal values.
    */
    double getMedian(double arr[], int start, int end) 
    {
        double median = 0;
        int size = end - start + 1;
        if (size % 2 == 0) 
        {
            median = (arr[start + size / 2 - 1] + arr[start + size / 2]) / 2;
        }
        else 
        {
            median = arr[start + (size - 1) / 2];
        }
        return median;
    }

};

#endif
