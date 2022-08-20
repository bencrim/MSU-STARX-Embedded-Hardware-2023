7/26/2022 - Benjamin Crimmins

Tilt_sensor_gyro.ino is a sketch that prints data received from two MPU-6050 gyroscopes.

This file gives a good look into how we use the Adafruit_MPU6050.h and Adafruit_Sensor.h libraries. 

The first thing we do is create two objects of class Adafruit_MPU6050 that will be used to access the data coming from each gyro. These objects are necessary because the functions
we use later in the file can only be done onto an object from the Adafruit_MPU6050 class.

Line 10 is where 3 sensor_event_t objects are created. These objects are standard to Adafruit sensors and we can get the gyro data we need from within the object.
The sensors and types of data we can receive from this object can be seen in lines 116-139 in the Adafruit_Sensor.h file. We only need to get the data from line 129, 
which is a sensors_vec_t object assigned to our gyros. This is why later in Tilt_sensor_gyro.ino (Lines 65-67), we call the data using a1.gyro.x. The x value from the gyro is a value
in an object in an object. (This took me a very long time to finally understand)

The next major part of this file is in the void tilt_setup() function. This function attempts to initialize each gyro using the .begin function with 
the address as a parameter. If the gyro is initialized, then the function continues to that gyroscopes settings, which you can adjust depending on what you need. 
This process is done for both gyros and then the file continues on to the void loop(). 

In the main loop, the function tilt() is called and repeated. The first thing done in the tilt() function is the .getEvent() function. 
This function returns the most recent data received from the gyroscope sensors_event_t object that we created earlier. The parameters in .getEvent() must be pointers to 
the sensors_event_t objects. This function is explained in line 685-709 in the Adafruit_MPU6050.cpp file.