/**
 * \file pwmConfiguration.cpp
 *
 * \author Sparty
 * 
 * PWM library generates a single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller A, B, C or D pins.
 * Designed for Arduino Due with Atmel SAM3X8E ARM Cortex-M3 CPU.
 * 
 */

#include "Arduino.h"
#include "pwmConfiguration.h"

/*
* Set up invariants for pwm Setup that occurs no matter which configuration of PWM is desired, such as the
* Peripheral clock and set desired clock rate.
*/
pwmConfiguration::pwmConfiguration()
{
  pwmSetupInvariants();
}


/*
 * Set up invariants for pwm Setup that occurs no matter which configuration of PWM is desired, such as the
 * Peripheral clock and set desired clock rate.
 */
void pwmConfiguration::pwmSetupInvariants()
{
  /*
   *  Peripheral Clock Enable 1 (REG_PMC_PCER1) enables PWM using peripheral clock controller.
   *  PIDx: Peripheral Clock x Enable (0 = No effect, 1 = Enables corresponding "Peripheral Identifiers" clock). Only need to do this once.
  */
  REG_PMC_PCER1 |= PMC_PCER1_PID36;

  /*
   *  Setting the clock rate to 84Mhz. The 0 and 1 value represents (84MHz/1). each linear divider can independently
   *  divide one of the clocks of the modulo n counter. The selection of the clock to be divided is made according to
   *  the PREA (PREB) field of the PWM Clock register (PWM_CLK). The resulting clock clkA (clkB) is the clock selected
   *  divided by DIVA (DIVB) field value. Only need to do this once.
  */
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);
}

/*
 * Change Duty Cycle of PWM pin that has already been constructed at pwmChannel byte.
 * \param pwmDutyCycle float sets the duty cycle for desired PWM.
 * \param pwmChannel uint8_t sets Channel Duty Cycle Update Register channel. This byte can be in range of 0-7.
 */
void pwmConfiguration::changePWMDutyCycle(uint8_t pwmChannel, float pwmDutyCycle)
{
  // To change the duty cycle during PWM operation, you'll need to use the REG_PWM_CDTYUPDx to update registers for the given channel.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CDTYUPD = pwmDutyCycle;
}

/*
 * Change Frequency of PWM pin that has already been constructed at pwmChannel byte.
 * \param pwmDutyCycle float sets the duty cycle for desired PWM.
 * \param pwmChannel uint8_t sets C Channel Period Update Register channel. This byte can be in range of 0-7.
 */
void pwmConfiguration::changePWMFrequency(uint8_t pwmChannel, float pwmFrequency)
{
  // To change the frequenct during PWM operation, you'll need to use the REG_PWM_CPRDUPDx to update registers for the given channel.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CPRDUPD = pwmFrequency;
}

/*
 * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller B pins only include DAC1, CAC0, A11,
 * A10, A9, A8, D53, D52, D22, D13 and D2.
 * \param pwmFrequency float sets the frequency for desired PWM.
 * \param pwmDutyCycle float sets the duty cycle for desired PWM.
 * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
 * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
 * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller B Disable Register and PWM Enable Register.
 */
void pwmConfiguration::setupPWMPinB(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle)
{
  /*
   * Parallel Input and Output Controller (PIO) Controller provides multiplexing of up to two peripheral functions on a single pin.
   * The selection is performed by writing PIO_ABSR (AB Select Register). For each pin, the corresponding bit after PIO for left hand side can determine if  
   * PIO controller A, B, C or D is selected. "P16" represents peripheral B type that has been selected, which is the PWM function.
   */
  //REG_PIOB_ABSR |= PIO_ABSR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_ABSR_Px (0x1u << x) //brief (PIO_ABSR) Peripheral A Select,
   * using this format allows easy change between #define PIO_ABSR_P0 (0x1u << 0) to #define PIO_ABSR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOB_ABSR |= (0x1u << lineIO);
  /*
   * Parallel Input and Output Controller C (PIOB) Disable Register (PDR) disables the PIO from controlling corresponding pins and enables
   * peripheral function control of the pin. Set PWM pin to output. "P16" represents peripheral B type that has been selected, which is the PWM function.
   */
  //REG_PIOB_PDR |= PIO_PDR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_PDR_Px (0x1u << x) //brief (PIO_PDR) PIO Disable,
   * using this format allows easy change between #define PIO_PDR_P0 (0x1u << 0) to #define PIO_PDR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOB_PDR |= (0x1u << lineIO);
  /*
   * Channel Mode Register (CMR) can range from the values of 0-7 and can configure Channel Prescaler (CPRE), Channel Alignment (CALG), Channel Polarity (CPOL),
   * Counter Event Selection (CES), Dead Time Generator Enable (DTE), Dead Time PWMHx Output Inverted (DTHI) and DeadTime PWMLx Output Inverted (DTLI).
   * Currently line of code enables signal PWMH0-7 (Channel 0-7) to be a single slope PWM and sets clock source as CLKA. We keep the right hand side the same so each
   * signal is synchronized to the same clock.For the digit on _CMRx look at the pinout diagram for PWMLx or PWMLHx (Just means PWM stars high/low).
   */
  //REG_PWM_CMR0 = PWM_CMR_CPRE_CLKA; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CMR = PWM_CMR_CPRE_CLKA;
  /*
   * Channel Period Register (CPR) is a waveform that is centre aligned, the output waveform depends on the channel counter source clock and can be
   * calculated by using the PWM master clock (MCK) divided by an X given prescaler value (X = 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1024):
   * (2 * X * CPRD) / (MCK). Refer to Arduino Due pdf for left align waveform if needed. This line sets PWM frequency to the relevant calculation
   * Example: Assume  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1) = 84 MHz / 1, (2 * X * CPRD) = 40 kHz, therefore 84 Mhz / 40 kHz = 2100.
   */
  //REG_PWM_CPRD0 = pwmFrequency; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CPRD = pwmFrequency;
  /*
   * Channel Duty Cycle Register (CDTYx) sets the waveform duty cycle of the PWM. This channel parameter is defined in the CDTY field of PWM_CDTY register.
   * If the waveform is centre aligned then duty cycle = [(period / 2) - (1 / (fchannel_x_clock*CDTY))] / period. Look in Arduino Due PDF for left align waveform if needed.
   * Example: To set a 50% duty cycle = pwmFrequency/2 = 2100/2 = 1050.
   */
  //REG_PWM_CDTY0 = pwmDutyCycle; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CDTY = pwmDutyCycle;
  /*
   * PWM Enable Register (PWM_ENAx) enables PWM output for channel x. As counters of synchronous channels must start at the same time, they are all enabled together
   * by enabling the channel 0 (by the CHID0 bit in PWM_ENA register). In the same way, they are all disabled together by disabling channel 0 (by the CHID0 bit in
   * PWM_DIS register). However, a synchronous channel x different from channel 0 can be enabled or disabled independently from others (by the CHIDx bit in PWM_ENA
   * and PWM_DIS registers). Again the digit is based on the pinout diagram --> PWMLx/PWMHx.
   */
   //REG_PWM_ENA = PWM_ENA_CHID0; is equivalent to below line. When parameter pwmChannelPos = 0.
  /*
   * In component_pwm.h found in Arduino Due the global variable for #define PWM_ENA_CHIDx (0x1u << x) //brief (PWM_ENA) Channel ID,
   * using this format allows easy change between #define PWM_ENA_CHID0 (0x1u << 0) to #define PWM_ENA_CHID7 (0x1u << 7) with given lineIO parameter.
   */
   REG_PWM_ENA = (0x1u << pwmChannelPos);
}


/*
 * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller C pins only include D51, D50, D49, D48,
 * D47, D46, D45, D44, D41, D40, D39, D38, D37, D36, D35, D34, D33, D10, D9, D8, D7, D6, D5, D4 and D3.
 * \param pwmFrequency float sets the frequency for desired PWM.
 * \param pwmDutyCycle float sets the duty cycle for desired PWM.
 * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
 * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
 * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller C Disable Register and PWM Enable Register.
 */
void pwmConfiguration::setupPWMPinC(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle)
{
  /*
   * Parallel Input and Output Controller (PIO) Controller provides multiplexing of up to two peripheral functions on a single pin.
   * The selection is performed by writing PIO_ABSR (AB Select Register). For each pin, the corresponding bit after PIO for left hand side can determine if  
   * PIO controller A, B, C or D is selected. "P16" represents peripheral C type that has been selected, which is the PWM function.
   */
  //REG_PIOC_ABSR |= PIO_ABSR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_ABSR_Px (0x1u << x) //brief (PIO_ABSR) Peripheral A Select,
   * using this format allows easy change between #define PIO_ABSR_P0 (0x1u << 0) to #define PIO_ABSR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOC_ABSR |= (0x1u << lineIO);
  /*
   * Parallel Input and Output Controller C (PIOC) Disable Register (PDR) disables the PIO from controlling corresponding pins and enables
   * peripheral function control of the pin. Set PWM pin to output. "P16" represents peripheral C type that has been selected, which is the PWM function.
   */
  //REG_PIOC_PDR |= PIO_PDR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_PDR_Px (0x1u << x) //brief (PIO_PDR) PIO Disable,
   * using this format allows easy change between #define PIO_PDR_P0 (0x1u << 0) to #define PIO_PDR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOC_PDR |= (0x1u << lineIO);
  /*
   * Channel Mode Register (CMR) can range from the values of 0-7 and can configure Channel Prescaler (CPRE), Channel Alignment (CALG), Channel Polarity (CPOL),
   * Counter Event Selection (CES), Dead Time Generator Enable (DTE), Dead Time PWMHx Output Inverted (DTHI) and DeadTime PWMLx Output Inverted (DTLI).
   * Currently line of code enables signal PWMH0-7 (Channel 0-7) to be a single slope PWM and sets clock source as CLKA. We keep the right hand side the same so each
   * signal is synchronized to the same clock.For the digit on _CMRx look at the pinout diagram for PWMLx or PWMLHx (Just means PWM stars high/low).
   */
  //REG_PWM_CMR0 = PWM_CMR_CPRE_CLKA; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CMR = PWM_CMR_CPRE_CLKA;
  /*
   * Channel Period Register (CPR) is a waveform that is centre aligned, the output waveform depends on the channel counter source clock and can be
   * calculated by using the PWM master clock (MCK) divided by an X given prescaler value (X = 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1024):
   * (2 * X * CPRD) / (MCK). Refer to Arduino Due pdf for left align waveform if needed. This line sets PWM frequency to the relevant calculation
   * Example: Assume  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1) = 84 MHz, (2 * X * CPRD) = 40 kHz, therefore 84 Mhz / 40 kHz = 2100.
   */
  //REG_PWM_CPRD0 = pwmFrequency; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CPRD = pwmFrequency;
  /*
   * Channel Duty Cycle Register (CDTYx) sets the waveform duty cycle of the PWM. This channel parameter is defined in the CDTY field of PWM_CDTY register.
   * If the waveform is centre aligned then duty cycle = [(period / 2) - (1 / (fchannel_x_clock*CDTY))] / period. Look in Arduino Due PDF for left align waveform if needed.
   * Example: To set a 50% duty cycle = pwmFrequency/2 = 2100/2 = 1050.
   */
  //REG_PWM_CDTY0 = pwmDutyCycle; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CDTY = pwmDutyCycle;
  /*
   * PWM Enable Register (PWM_ENAx) enables PWM output for channel x. As counters of synchronous channels must start at the same time, they are all enabled together
   * by enabling the channel 0 (by the CHID0 bit in PWM_ENA register). In the same way, they are all disabled together by disabling channel 0 (by the CHID0 bit in
   * PWM_DIS register). However, a synchronous channel x different from channel 0 can be enabled or disabled independently from others (by the CHIDx bit in PWM_ENA
   * and PWM_DIS registers). Again the digit is based on the pinout diagram --> PWMLx/PWMHx.
   */
   //REG_PWM_ENA = PWM_ENA_CHID0; is equivalent to below line. When parameter pwmChannelPos = 0.
  /*
   * In component_pwm.h found in Arduino Due the global variable for #define PWM_ENA_CHIDx (0x1u << x) //brief (PWM_ENA) Channel ID,
   * using this format allows easy change between #define PWM_ENA_CHID0 (0x1u << 0) to #define PWM_ENA_CHID7 (0x1u << 7) with given lineIO parameter.
   */
   REG_PWM_ENA = (0x1u << pwmChannelPos);
}


/*
 * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller A pins only include A7, A6, A5, A4, A3, A2, A1, A0, D43, D42, D31,
 * D24, D23, D10, D4.
 * \param pwmFrequency float sets the frequency for desired PWM.
 * \param pwmDutyCycle float sets the duty cycle for desired PWM.
 * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
 * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
 * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller A Disable Register and PWM Enable Register.
 */
void pwmConfiguration::setupPWMPinA(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle)
{
  /*
   * Parallel Input and Output Controller (PIO) Controller provides multiplexing of up to two peripheral functions on a single pin.
   * The selection is performed by writing PIO_ABSR (AB Select Register). For each pin, the corresponding bit after PIO for left hand side can determine if  
   * PIO controller A, B, C or D is selected. "P16" represents peripheral A type that has been selected, which is the PWM function.
   */
  //REG_PIOA_ABSR |= PIO_ABSR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_ABSR_Px (0x1u << x) //brief (PIO_ABSR) Peripheral A Select,
   * using this format allows easy change between #define PIO_ABSR_P0 (0x1u << 0) to #define PIO_ABSR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOA_ABSR |= (0x1u << lineIO);
  /*
   * Parallel Input and Output Controller A (PIOA) Disable Register (PDR) disables the PIO from controlling corresponding pins and enables
   * peripheral function control of the pin. Set PWM pin to output. "P16" represents peripheral A type that has been selected, which is the PWM function.
   */
  //REG_PIOC_PDR |= PIO_PDR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_PDR_Px (0x1u << x) //brief (PIO_PDR) PIO Disable,
   * using this format allows easy change between #define PIO_PDR_P0 (0x1u << 0) to #define PIO_PDR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOC_PDR |= (0x1u << lineIO);
  /*
   * Channel Mode Register (CMR) can range from the values of 0-7 and can configure Channel Prescaler (CPRE), Channel Alignment (CALG), Channel Polarity (CPOL),
   * Counter Event Selection (CES), Dead Time Generator Enable (DTE), Dead Time PWMHx Output Inverted (DTHI) and DeadTime PWMLx Output Inverted (DTLI).
   * Currently line of code enables signal PWMH0-7 (Channel 0-7) to be a single slope PWM and sets clock source as CLKA. We keep the right hand side the same so each
   * signal is synchronized to the same clock.For the digit on _CMRx look at the pinout diagram for PWMLx or PWMLHx (Just means PWM stars high/low).
   */
  //REG_PWM_CMR0 = PWM_CMR_CPRE_CLKA; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CMR = PWM_CMR_CPRE_CLKA;
  /*
   * Channel Period Register (CPR) is a waveform that is centre aligned, the output waveform depends on the channel counter source clock and can be
   * calculated by using the PWM master clock (MCK) divided by an X given prescaler value (X = 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1024):
   * (2 * X * CPRD) / (MCK). Refer to Arduino Due pdf for left align waveform if needed. This line sets PWM frequency to the relevant calculation
   * Example: Assume  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1) = 84 MHz, (2 * X * CPRD) = 40 kHz, therefore 84 Mhz / 40 kHz = 2100.
   */
  //REG_PWM_CPRD0 = pwmFrequency; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CPRD = pwmFrequency;
  /*
   * Channel Duty Cycle Register (CDTYx) sets the waveform duty cycle of the PWM. This channel parameter is defined in the CDTY field of PWM_CDTY register.
   * If the waveform is centre aligned then duty cycle = [(period / 2) - (1 / (fchannel_x_clock*CDTY))] / period. Look in Arduino Due PDF for left align waveform if needed.
   * Example: To set a 50% duty cycle = pwmFrequency/2 = 2100/2 = 1050.
   */
  //REG_PWM_CDTY0 = pwmDutyCycle; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CDTY = pwmDutyCycle;
  /*
   * PWM Enable Register (PWM_ENAx) enables PWM output for channel x. As counters of synchronous channels must start at the same time, they are all enabled together
   * by enabling the channel 0 (by the CHID0 bit in PWM_ENA register). In the same way, they are all disabled together by disabling channel 0 (by the CHID0 bit in
   * PWM_DIS register). However, a synchronous channel x different from channel 0 can be enabled or disabled independently from others (by the CHIDx bit in PWM_ENA
   * and PWM_DIS registers). Again the digit is based on the pinout diagram --> PWMLx/PWMHx.
   */
   //REG_PWM_ENA = PWM_ENA_CHID0; is equivalent to below line. When parameter pwmChannelPos = 0.
  /*
   * In component_pwm.h found in Arduino Due the global variable for #define PWM_ENA_CHIDx (0x1u << x) //brief (PWM_ENA) Channel ID,
   * using this format allows easy change between #define PWM_ENA_CHID0 (0x1u << 0) to #define PWM_ENA_CHID7 (0x1u << 7) with given lineIO parameter.
   */
   REG_PWM_ENA = (0x1u << pwmChannelPos);
}

/*
 * PWM single slope, centre aligned waveform initalisation for SAM3X Pins. Parallel I/O Controller D pins only include D32, D30, D29, D28, D27, D26, D25, D12 and D11.
 * \param pwmFrequency float sets the frequency for desired PWM.
 * \param pwmDutyCycle float sets the duty cycle for desired PWM.
 * \param pwmChannel uint8_t sets Channel for Channel Mode Register, Channel Period Register and Channel Duty Cycle Register. This byte can be in range of 0-7.
 * \param pwmChannelPos int is the same value of pwmChannel but in integer form. This integer can be in range of 0-7.
 * \param lineIO int sets pin for Parallel Input and Output Controller AB Select Register, Parallel Input and Output Controller D Disable Register and PWM Enable Register.
 */
void pwmConfiguration::setupPWMPinD(int lineIO, uint8_t pwmChannel, int pwmChannelPos, float pwmFrequency, float pwmDutyCycle)
{
  /*
   * Parallel Input and Output Controller (PIO) Controller provides multiplexing of up to two peripheral functions on a single pin.
   * The selection is performed by writing PIO_ABSR (AB Select Register). For each pin, the corresponding bit after PIO for left hand side can determine if  
   * PIO controller A, B, C or D is selected. "P16" represents peripheral D type that has been selected, which is the PWM function.
   */
  //REG_PIOD_ABSR |= PIO_ABSR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_ABSR_Px (0x1u << x) //brief (PIO_ABSR) Peripheral A Select,
   * using this format allows easy change between #define PIO_ABSR_P0 (0x1u << 0) to #define PIO_ABSR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOD_ABSR |= (0x1u << lineIO);
  /*
   * Parallel Input and Output Controller D (PIOA) Disable Register (PDR) disables the PIO from controlling corresponding pins and enables
   * peripheral function control of the pin. Set PWM pin to output. "P16" represents peripheral D type that has been selected, which is the PWM function.
   */
  //REG_PIOD_PDR |= PIO_PDR_P16; is equivalent to below line. When parameter lineIO is 16.
  /*
   * In component_pio.h found in Arduino Due the global variable for #define PIO_PDR_Px (0x1u << x) //brief (PIO_PDR) PIO Disable,
   * using this format allows easy change between #define PIO_PDR_P0 (0x1u << 0) to #define PIO_PDR_P31 (0x1u << 31) with given lineIO parameter.
   */
  REG_PIOD_PDR |= (0x1u << lineIO);
  /*
   * Channel Mode Register (CMR) can range from the values of 0-7 and can configure Channel Prescaler (CPRE), Channel Alignment (CALG), Channel Polarity (CPOL),
   * Counter Event Selection (CES), Dead Time Generator Enable (DTE), Dead Time PWMHx Output Inverted (DTHI) and DeadTime PWMLx Output Inverted (DTLI).
   * Currently line of code enables signal PWMH0-7 (Channel 0-7) to be a single slope PWM and sets clock source as CLKA. We keep the right hand side the same so each
   * signal is synchronized to the same clock.For the digit on _CMRx look at the pinout diagram for PWMLx or PWMLHx (Just means PWM stars high/low).
   */
  //REG_PWM_CMR0 = PWM_CMR_CPRE_CLKA; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CMR = PWM_CMR_CPRE_CLKA;
  /*
   * Channel Period Register (CPR) is a waveform that is centre aligned, the output waveform depends on the channel counter source clock and can be
   * calculated by using the PWM master clock (MCK) divided by an X given prescaler value (X = 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 or 1024):
   * (2 * X * CPRD) / (MCK). Refer to Arduino Due pdf for left align waveform if needed. This line sets PWM frequency to the relevant calculation
   * Example: Assume  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1) = 84 MHz, (2 * X * CPRD) = 40 kHz, therefore 84 Mhz / 40 kHz = 2100.
   */
  //REG_PWM_CPRD0 = pwmFrequency; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CPRD = pwmFrequency;
  /*
   * Channel Duty Cycle Register (CDTYx) sets the waveform duty cycle of the PWM. This channel parameter is defined in the CDTY field of PWM_CDTY register.
   * If the waveform is centre aligned then duty cycle = [(period / 2) - (1 / (fchannel_x_clock*CDTY))] / period. Look in Arduino Due PDF for left align waveform if needed.
   * Example: To set a 50% duty cycle = pwmFrequency/2 = 2100/2 = 1050.
   */
  //REG_PWM_CDTY0 = pwmDutyCycle; is equivalent to below line. When parameter pwmChannel = 0.
  PWM->PWM_CH_NUM[pwmChannel].PWM_CDTY = pwmDutyCycle;
  /*
   * PWM Enable Register (PWM_ENAx) enables PWM output for channel x. As counters of synchronous channels must start at the same time, they are all enabled together
   * by enabling the channel 0 (by the CHID0 bit in PWM_ENA register). In the same way, they are all disabled together by disabling channel 0 (by the CHID0 bit in
   * PWM_DIS register). However, a synchronous channel x different from channel 0 can be enabled or disabled independently from others (by the CHIDx bit in PWM_ENA
   * and PWM_DIS registers). Again the digit is based on the pinout diagram --> PWMLx/PWMHx.
   */
   //REG_PWM_ENA = PWM_ENA_CHID0; is equivalent to below line. When parameter pwmChannelPos = 0.
  /*
   * In component_pwm.h found in Arduino Due the global variable for #define PWM_ENA_CHIDx (0x1u << x) //brief (PWM_ENA) Channel ID,
   * using this format allows easy change between #define PWM_ENA_CHID0 (0x1u << 0) to #define PWM_ENA_CHID7 (0x1u << 7) with given lineIO parameter.
   */
   REG_PWM_ENA = (0x1u << pwmChannelPos);
}

