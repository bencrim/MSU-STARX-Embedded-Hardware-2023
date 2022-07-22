#include <LiquidCrystal.h>

/*
  HC-SR04 NewPing Iteration Demonstration
  HC-SR04-NewPing-Iteration.ino
  Demonstrates using Iteration function of NewPing Library for HC-SR04 Ultrasonic Range Finder
  Displays results on Serial Monitor

  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/

// This uses Serial Monitor to display Range Finder distance readings

// Include NewPing Library
#include "NewPing.h"

// Hook up HC-SR04 with Trig to Arduino Pin 10, Echo to Arduino pin 13
// Maximum Distance is 400 cm

#define TRIGGER_PIN  52
#define ECHO_PIN     53
#define MAX_DISTANCE 400
 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float duration, distance;

int iterations = 5;
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);
void setup() 
{
  Serial.begin (9600);
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Distance:");
}

void loop() 
{
  lcd.setCursor(0,1);
   
  duration = sonar.ping_median(iterations);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  Serial.println(duration);
  distance = (duration / 2) * 0.0343;
  
  // Send results to Serial Monitor
    lcd.print(distance);
    lcd.print(" cm");
    delay(500);
}
