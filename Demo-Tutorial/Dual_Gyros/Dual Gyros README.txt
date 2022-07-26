7/24/2022 - Benjamin Crimmins

Dual_Gyros.ino is a sketch that is supposed to take two MPU-6050 gryoscope inputs at the same time and return data from both.

This file was added during the testing phases of the dual gyroscope system and was useful for learning how the Arduino Due 
could take in multiple sources of data "at once". The code was borrowed from GitHub user "kapil goswami" (https://github.com/goswamikapil/Arduino-Codes-Youtube)
after seeing a YouTube video showing two MPU-6050 gyroscopes connected to one Arduino Board.

This program switches which gyro it is recieving data from in the void loop by calling a function assigned to 
each gyro. The way the functions distinguish between gryos is by having different addresses inputted in function parameter.
This can be seen in the second line, where MPU2 is assigned to 0x69 and MPU1 is assigned to 0x68. These values
are then used when the function is called in the void loop.

0x68 and 0x69 are the only addresses possible for the MPU-6050, and they are distinguished by a soder bridge on the back of the gyro.
The default address for the MPU-6050 is 0x68, but when the AD0 connection is bridged on the back of the board, the gyro turns to 0x69.
This means that at most you can have two unique gyros connected to the Arduino Due at once, but luckily we only need two. 
There are more complicated ways you could hypothetically have more than two, but at this time we don't need it.

