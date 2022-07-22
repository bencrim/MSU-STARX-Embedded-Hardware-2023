// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

void setup(void) {
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  if (!mpu1.begin(0x68)) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    Serial.println("MPU1 not found");
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  //range settings
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
    if (!mpu2.begin(0x69)) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    Serial.println("MPU2 not found");
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  //range settings
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() 
{

  /* Get new sensor events with the readings */
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);
  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);
  Serial.print(g1.gyro.y); Serial.print("  "); Serial.println(g2.gyro.y);
}
