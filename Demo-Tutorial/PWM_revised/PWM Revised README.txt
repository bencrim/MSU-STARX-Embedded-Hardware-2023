7/26/2022 - Benjamin Crimmins

PWM_revised.ino is a sketch that is comprised of only the PWM features included in the main system.

This file was created by Bruno H. some time in SS2022, and it gives a good look at what is happening in the main linear actuator system as of this documents creation.

An important part of this program is the Interrupt Service Routine (ISR) that is used in lines 32-33. the enableInterrupt function comes from the imported library and
is a faster and more compatible version of the Arduino default attachInterrupt function. 
You can find more in depth information on ISRs here: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
