8/3/2022 - Benjamin Crimmins

This README file is designed to go over and explain each major section of the LinearActuatorControlSystem.ino file. Any major new additions
to the .ino file should also be elaborated in this file. 


Section 0 - Libraries (Lines 1-7)
The first 7 lines of the file are devoted to importing libraries that will be used later in the program. The only library that is not imported
from the internet is GyroToVelocity.h, which was written by Bruno H. when he was team lead in 2022. GyroToVelocity.h handles all the suit 
calculations that are specific to our design. I believe the formulas were originally written by Katharine Walters who was main engineer in 2022.


Section 1a - Variable definitions (Lines 8-18)
This section is where the simple Arduino variables such as PIN numbers and PWM values are defined. These variables are distinct from the others because variables 
defined with the #define macro are constant across all files and are not restricted by scope. This means that if LinearActuatorControlSystem.ino were to be imported 
to another file, these variables could be referenced and used in the new file. Our program does not necessarily have to define these variables this way because we aren't 
importing this .ino to anything else, but it's safer to define these variables this way so there are absolutely no scope issues later. 

Section 1b - Array definitions (Lines 21-36)
The arrays defined in the section differ from the variables defined above because these were assigned under the const definition. This type of definition
was used because we are defining arrays that contain a specific type, in this case the double type. These arrays are used later in the program to differentiate 
between which gyro we are using. This is important because each MPU-6050 gyroscope has its own unique offset that must be accounted for. The values were obtained
with an example file included in the Adafruit Sensor Lab library.

Section 1c - Interrupt Arrays (Lines 32-34)
