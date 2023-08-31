//Libraries
#include <PID_v1.h>     //PID library
#include <EnableInterrupt.h>  //library that enables to read pwm values
#include <Adafruit_LSM6DSOX.h> //gyro library
#include <Adafruit_Sensor.h>  //library to run adafruit sensors
#include <Wire.h>   //library that enables serail communications
#include <GyroToVelocity.h>   //geometry functions for HIP and KNEE actuators

//Serial port baud and Arduino pins
#define ANV 5 //analog voltage speed control pin (RED)
#define IN1 2 //direction of actuation (BLUE)
#define IN2 3 //direction of actuation (GRAY)
//GROUND COLORS (BLACK, PINK/YELLOW, TAN)

#define SERIAL_PORT_SPEED 9600 
#define TO_STANDING 53
#define MAX_SPEED_ANGLE 15 //The maximum angle difference value. Used to determine the slope the system calculates actuator speed with.
#define MAX_MOTOR_SPEED_KNEE 100 //The maximum actuator speed allowed. Used to determine the slope the system claculates actuator speed with.
#define MAX_MOTOR_SPEED_HIP 120

#define WINDOW_SIZE 5

int INDEX = 0;
int VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

//zero rate offset in radians/s
//recalibrate gyros periodically, probably once a month or before testing

Adafruit_LSM6DSOX suit_sensor; //gyro sensor
Adafruit_LSM6DSOX pilot_sensor;
//double corrected_X[2], corrected_Y[2], corrected_Z[2]; //variables that store the angular speed after the offset has been applied

float AcX, AcY, AcZ, AcX1, AcY1, AcZ1, pilot_angle, suit_angle, angle_difference, actuatorSpeed;
float ANGLE_DEAD_ZONE; //Threshold of angle difference that will not send signal to the actuator to move. Absolute value is used so the value is +- 0 difference.
String option = "HIP"; //Change based on HIP or KNEE

void setup() 
{
  Serial.print("Setup start\n");
  //begin serial monitor
  Serial.begin(SERIAL_PORT_SPEED); //from 9600 to 115200

  //establish all pins I/O
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  //Checks for gyroscope
  if (!suit_sensor.begin_I2C(0x6A))//0x6A or 0x6B, (&Wire or &Wire1 is used for SDA/SCL and SDA1/SCL1)
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  Serial.println("I2C 0x6A Done");
  //range settings
  suit_sensor.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  suit_sensor.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);

  
  if (!pilot_sensor.begin_I2C(0x6B))//0x6A or 0x6B, &Wire or &Wire1
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  Serial.println("I2C 0x6B Done");
  //range settings
  pilot_sensor.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  pilot_sensor.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  
  Serial.print("Setup end\n");
}

void loop() 
{
  //sensors_event_t a, g, temp; //not needed since we only care about accelerometer data
  suit_sensor.readAcceleration(AcX, AcY, AcZ);
  
  //sensors_event_t a1, g1, temp1; //not needed since we only care about accelerometer data
  pilot_sensor.readAcceleration(AcX1, AcY1, AcZ1);

  //calculate the angles of the sensors in degrees. 
  suit_angle = RAD_TO_DEG * atan(sqrt((AcX*AcX)+(AcY*AcY))/AcZ);
  pilot_angle = RAD_TO_DEG * atan(sqrt((AcX1*AcX1)+(AcY1*AcY1))/AcZ1);
  
  //flips the range of values to make it so it goes from -90 to 90, with 0 being perfectly vertical
  if (suit_angle > 0){
    suit_angle = 90 - suit_angle;
  }
  else if (suit_angle < 0){
    suit_angle = -90 - suit_angle;
  }
  if (pilot_angle > 0){
    pilot_angle = 90 - pilot_angle;
  }
  else if (pilot_angle < 0){
    pilot_angle = -90 - pilot_angle;
  }

  //calculates the difference in angles between the pilot sensor and suit sensor
  if (option == "HIP"){
    angle_difference = (pilot_angle - suit_angle)+4;
  }
  else {
    angle_difference = pilot_angle - suit_angle;
  }
  
  //determine which direction and what speed to output to the actuator
  if (option == "HIP"){
       
    SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
    VALUE = angle_difference;        // Read the next sensor value
    READINGS[INDEX] = VALUE;           // Add the newest reading to the window
    SUM = SUM + VALUE;                 // Add the newest reading to the sum
    INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  
    angle_difference = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
    //if the pilots leg is above the suit leg
    if (angle_difference < 0){
      ANGLE_DEAD_ZONE = 1.5;
      //extend
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      //Check how large the angle difference is to determine actuator speed
      //Check if the angle difference is in the dead zone
      if (abs(angle_difference) < ANGLE_DEAD_ZONE){
        //do nothing
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
      }
      
      //determine speed 
      else if (abs(angle_difference) < MAX_SPEED_ANGLE){
        actuatorSpeed = (MAX_MOTOR_SPEED_HIP/MAX_SPEED_ANGLE)*abs(angle_difference);
        analogWrite(ANV, actuatorSpeed);
      }

      
      else {
        actuatorSpeed = MAX_MOTOR_SPEED_HIP;
        analogWrite(ANV, actuatorSpeed);
      }
    }
    else if (angle_difference > 0){
      //retract
      ANGLE_DEAD_ZONE = 1.5;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      if (abs(angle_difference) < ANGLE_DEAD_ZONE){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
      }
      if (abs(angle_difference) < MAX_SPEED_ANGLE){
        actuatorSpeed = (MAX_MOTOR_SPEED_HIP/MAX_SPEED_ANGLE)*abs(angle_difference);
        analogWrite(ANV, actuatorSpeed);
      }
      else {
        actuatorSpeed = MAX_MOTOR_SPEED_HIP;
        analogWrite(ANV, actuatorSpeed);
      }
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ANV, 0);
    }
  }
  else { //KNEE option
       
    SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
    VALUE = angle_difference;        // Read the next sensor value
    READINGS[INDEX] = VALUE;           // Add the newest reading to the window
    SUM = SUM + VALUE;                 // Add the newest reading to the sum
    INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
  
    angle_difference = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
    //if the pilots leg is above the suit leg
    ANGLE_DEAD_ZONE = 4;
    if (angle_difference < 0){
      //retract
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      //Check how large the angle difference is to determine actuator speed
      //Check if the angle difference is in the dead zone
      if (abs(angle_difference) < ANGLE_DEAD_ZONE){
        //do nothing
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
      }
      
      //determine speed 
      else if (abs(angle_difference) < MAX_SPEED_ANGLE){
        actuatorSpeed = (MAX_MOTOR_SPEED_KNEE/MAX_SPEED_ANGLE)*abs(angle_difference);
        analogWrite(ANV, actuatorSpeed);
      }

      
      else {
        actuatorSpeed = MAX_MOTOR_SPEED_KNEE;
        analogWrite(ANV, actuatorSpeed);
      }
    }
    else if (angle_difference > 0){
      //extend
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      if (abs(angle_difference) < ANGLE_DEAD_ZONE){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
      }
      if (abs(angle_difference) < MAX_SPEED_ANGLE){
        actuatorSpeed = (MAX_MOTOR_SPEED_KNEE/MAX_SPEED_ANGLE)*abs(angle_difference);
        analogWrite(ANV, actuatorSpeed);
      }
      else {
        actuatorSpeed = MAX_MOTOR_SPEED_KNEE;
        analogWrite(ANV, actuatorSpeed);
      }
    }
    else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ANV, 0);
    }
  }
  
  Serial.print("   Suit Angle:");Serial.print(suit_angle);Serial.print("   Pilot Angle:");Serial.print(pilot_angle);Serial.print("   Angle Difference:");Serial.println(angle_difference);

}
