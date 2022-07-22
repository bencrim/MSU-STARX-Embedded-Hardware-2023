/**
 * \file pwmConfiguration.h
 *
 * \author Sparty
 * 
 * PWM library generates a single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller A, B, C or D pins.
 * Designed for Arduino Due with Atmel SAM3X8E ARM Cortex-M3 CPU.
 * 
 */


#ifndef pwmConfiguration_h
#define pwmConfiguration_h

#include "Arduino.h"

class pwmConfiguration
{
  public:
    /*
    * Set up invariants for pwm Setup that occurs no matter which configuration of PWM is desired, such as the
    * Peripheral clock and set desired clock rate.
    */
    pwmConfiguration();
    /*
    * Change Duty Cycle of PWM pin that has already been constructed at pwmChannel byte.
    * \param pwmDutyCycle float sets the duty cycle for desired PWM.
    * \param pwmChannel uint8_t sets Channel Duty Cycle Update Register channel. This byte can be in range of 0-7.
    */
    void changePWMDutyCycle(uint8_t pwmChannel, float pwmDutyCycle);
    /*
    * Change Frequency of PWM pin that has already been constructed at pwmChannel byte.
    * \param pwmDutyCycle float sets the duty cycle for desired PWM.
    * \param pwmChannel uint8_t sets C Channel Period Update Register channel. This byte can be in range of 0-7.
    */
    void changePWMFrequency(uint8_t pwmChannel, float pwmFrequency);
    /*
    * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller B pins only include DAC1, CAC0, A11,
    * A10, A9, A8, D53, D52, D22, D13 and D2.
    * \param pwmFrequency float sets the frequency for desired PWM.
    * \param pwmDutyCycle float sets the duty cycle for desired PWM.
    * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
    * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
    * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller B Disable Register and PWM Enable Register.
    */
    void setupPWMPinB(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle);
   /*
    * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller C pins only include D51, D50, D49, D48,
    * D47, D46, D45, D44, D41, D40, D39, D38, D37, D36, D35, D34, D33, D10, D9, D8, D7, D6, D5, D4 and D3.
    * \param pwmFrequency float sets the frequency for desired PWM.
    * \param pwmDutyCycle float sets the duty cycle for desired PWM.
    * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
    * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
    * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller C Disable Register and PWM Enable Register.
    */
    void setupPWMPinC(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle);
    /*
    * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller A pins only include A7, A6, A5, A4, A3, A2, A1, A0, D43, D42, D31,
    * D24, D23, D10, D4.
    * \param pwmFrequency float sets the frequency for desired PWM.
    * \param pwmDutyCycle float sets the duty cycle for desired PWM.
    * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
    * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
    * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller A Disable Register and PWM Enable Register.
    */
    void setupPWMPinA(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle);
    /*
    * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller D pins only include D32, D30, D29, D28, D27, D26, D25, D12 and D11.
    * \param pwmFrequency float sets the frequency for desired PWM.
    * \param pwmDutyCycle float sets the duty cycle for desired PWM.
    * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
    * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
    * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller D Disable Register and PWM Enable Register.
    */
    void setupPWMPinD(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle);

  private:
    // set this to match whatever prescaler value you want to set
    // (prescaler = 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1024)
    int prescaler = 256; 
  protected:
    /*
    * Set up invariants for pwm Setup that occurs no matter which configuration of PWM is desired, such as the
    * Peripheral clock and set desired clock rate.
    */
    void pwmSetupInvariants();


};

#endif
