8/3/2022 - Benjamin Crimmins

This README file is designed to go over and explain each major section of the LinearActuatorControlSystem.ino file. Any major new additions
to the .ino file should also be elaborated in this file. 

Section 0 - Libraries (Lines 1-7)
The first 7 lines of the file are devoted to importing libraries that will be used later in the program. The only library that is not imported
from the internet is GyroToVelocity.h, which was written by Bruno H. when he was team lead in 2022. GyroToVelocity.h handles all the suit 
calculations that are specific to our design. I believe the formulas were originally written by Katharine Walters who was main engineer in 2022.



Section 1a - Variable definitions (Lines 9-18)
This section is where the simple Arduino constants such as PIN numbers and PWM values are defined. These constants are distinct from the others because constants or variables 
defined with the #define macro are constant across all files and are not restricted by scope. This means that if LinearActuatorControlSystem.ino were to be imported 
to another file, these variables could be referenced and used in the new file. Our program does not necessarily have to define these constants this way because we aren't 
importing this .ino to anything else, but it's safer to define these constants this way so there are absolutely no scope issues later. 

Section 1b - Array definitions (Lines 21-28)
The arrays defined in the section differ from the constants defined above because these were assigned under the const definition. This type of definition
was used because we are defining arrays that contain a specific type, in this case the double type. These arrays are used later in the program to differentiate 
between which gyro we are using. This is important because each MPU-6050 gyroscope has its own unique offset that must be accounted for. The values were obtained
with an example file included in the Adafruit Sensor Lab library.

Section 1c - Interrupt Arrays (Lines 34-36)
These arrays are defined as unsigned ints of varying sizes and types. Unsigned ints are the same as ints in that they store a 2 byte value, but they only store positive values
which doubles the normal range of values. The number in the assignment macro determines how many bits that variable will be, so pwm_start[PWM_NUM] will be an array that stores
values that can be up to 32 bits large in the positive only.

Section 1d - Gyroscope Objects and Variables (Lines 38-40)
In Lines 38 and 39, two objects of the class Adafruit_MPU6050 are created which will allow us to assign each gyroscope to its respective object. These objects will allow us to perform
functions to receive the data from the gyroscope. This is explained in more detail in the Tilt_Sensor_README.txt file found in the github under the Demo-Tutorial folder.
We also create arrays that will store the values received from the gyroscope after the offsets have been accounted for.

Section 1e - PID variables (Lines 45-49)
Lines 45-47 are used to initialize the variables that we will be using in our PID, and line 48 is where we create the PID controller. The PID controller is created with a constructor,
more information on constructors can be found here (https://www.w3schools.com/cpp/cpp_constructors.asp). This page (https://playground.arduino.cc/Code/PIDLibraryConstructor/) 
outlines the parameters that are used in the PID function. The "&" symbol next to the parameters in line 48 means that those parameters are a reference and not a copy, which is 
useful for saving data. More in depth explanations on references can be found online, I think this video (https://www.youtube.com/watch?v=OCL7mSFCIx0) does a great job at explaining the concept.



Section 2a - Baud rate (Line 65)
In this line, we set the baud rate to 9600, which is the standard baud rate for Arduino boards. The baud rate is another way of saying how many bits are being sent per second for serial data
transmission. We set the rate at 9600 because anything more can start causing problems.

Section 2b -  Pin Modes (Lines 69-77)
In this section, we set all the I/O pins we use on the Arduino Due to either input or output. We also set pin 22 to be HIGH right away, this is changed later 
depending on the state of the PID system (This system is still a work in progress).

Section 2c - Interrupts (Lines 80-81)
In these two lines, the interrupts are created that will trigger each time the speed or position of the linear actuator changes. More specifically, they will trigger any time the pin 
assigned to the first parameter changes from high to low or low to high. We use the enableInterrupt function in place of the attachInterrupt function because enableInterrupt is faster.

Section 2d - Soft stop PID (Lines 84-86)
### WIP ###

Section 2e - Load compensator PID (Lines 88-92)
### WIP ###

Section 2f - Hip or knee geometry (Line 94)
This is the location where we decide which actuator calculations we will be using, either HIP or KNEE. These calculations are done in the GyroToVelocity library we imported earlier.
The main concept of the calculations is that we can confidently predict what the lower leg should be doing based on the angular velocity and location of the upper leg. 
This is why we use gyroscopes.

Section 2g - Gyroscope settings and initializations (Lines 96-124)
The if statements in this section of the program are used to catch whether the gyroscopes are connected and working well. We first use the mpu.begin with the gyroscope address as a parameter,
this will catch if there is an error returning data from the gyroscope. If all works well then the LED on the board won't light up. Next, we set up some settings for the gyroscope. These can be
changed depending on what you need the gyroscope to do, but the way we have it now should be working. Lines 106-122 are the same except it tests and sets the other gyroscope. The offsetSwitch is 
where we tell the program which unique gyroscope is being used so that we can adjust for it's specific offset values.



Section 3a