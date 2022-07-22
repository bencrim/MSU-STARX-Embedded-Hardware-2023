#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
float x_val1, y_val1, z_val1, result1;
float x_square1, y_square1, z_square1;
float accel_angle_x1, accel_angle_y1;
sensors_event_t a1, g1, temp1;

float x_val2, y_val2, z_val2, result2;
float x_square2, y_square2, z_square2;
float accel_angle_x2, accel_angle_y2;
sensors_event_t a2, g2, temp2;

float accel_center_x = 0.66;
float accel_center_y = -0.12;
float accel_center_z = 8.60;

void setup() 
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  tilt_setup();
}

void loop() 
{
  tilt();
}
void tilt_setup()
{
  if (!mpu1.begin(0x68)) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  //range settings
  mpu1.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu1.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  if (!mpu2.begin(0x69)) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  //range settings
  mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu2.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);
}
void tilt()
{
  mpu1.getEvent(&a1, &g1, &temp1);
  x_val1 = a1.gyro.x - accel_center_x;
  y_val1 = a1.gyro.y - accel_center_y;
  z_val1 = a1.gyro.z - accel_center_z;
  x_square1 = (x_val1*x_val1);
  y_square1 = (y_val1*y_val1);
  z_square1 = (z_val1*z_val1);
  result1 = sqrt(y_square1 + z_square1);
  result1 = x_val1/result1;
  accel_angle_x1 = atan(result1);
  //Serial.print(accel_angle_x1); Serial.print("   ");
  result1 = sqrt(x_square1 + z_square1);
  result1 = y_val1/result1;
  accel_angle_y1 = atan(result1);
  //Serial.println(accel_angle_y1);
  
  mpu2.getEvent(&a2, &g2, &temp2);
  x_val2 = a2.gyro.x - accel_center_x;
  y_val2 = a2.gyro.y - accel_center_y;
  z_val2 = a2.gyro.z - accel_center_z;
  x_square2 = (x_val2*x_val2);
  y_square2 = (y_val2*y_val2);
  z_square2 = (z_val2*z_val2);
  result2 = sqrt(y_square2 + z_square2);
  result2 = x_val2/result2;
  accel_angle_x2 = atan(result2);
  Serial.print(accel_angle_x2); Serial.print("   ");
  result2 = sqrt(x_square2 + z_square2);
  result2 = y_val2/result2;
  accel_angle_y2 = atan(result2);
  Serial.println(accel_angle_y2);
}
