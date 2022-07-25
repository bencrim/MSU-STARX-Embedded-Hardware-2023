7/24/2022 - Benjamin Crimmins

I2C_Address_test.ino is a sketch that initializes two MPU-6050 gyroscopes and returns data from both. 

This file is very similar to the Dual_Gyros.ino, in that they both attempt to show how to use two MPU-6050 gyroscopes at the same time. 
This file, however, is much simpler than Dual_Gyros so I would recommend looking to this one for guidance on using the gyroscopes. I wasn't able to find 
where this file came from, but I don't believe it was anyone on the team who wrote it.

This program switches which gyro it is receiving data from in the void loop by calling the get.Event function with a different gyroscope object. The objects are assigned 
before void setup and are distingushed by each gyroscopes unique address 0x68 and 0x69.

0x68 and 0x69 are the only addresses possible for the MPU-6050, and they are distinguished by a soder bridge on the back of the gyro.
The default address for the MPU-6050 is 0x68, but when the AD0 connection is bridged on the back of the board, the gyro turns to 0x69.
This means that at most you can have two unique gyros connected to the Arduino Due at once, but luckily we only need two. 
There are more complicated ways you could hypothetically have more than two, but at this time we don't need it.
