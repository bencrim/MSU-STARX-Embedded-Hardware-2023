#include <PID_v1.h>           //PID library
#include <EnableInterrupt.h>    //interrupt library
#include <Adafruit_MPU6050.h>   //MPU Gyro library
#include <Adafruit_Sensor.h>    
#include <Wire.h>               //I2C communication library
#include <GyroToVelocity.h>
#define SERIAL_PORT_SPEED 9600
#define PWM_NUM  2

#define PWMS  0
#define PWMP  1

#define PWMS_INPUT  A0
#define PWMP_INPUT  A1

uint16_t pwm_values[PWM_NUM];
uint32_t pwm_start[PWM_NUM];
volatile uint16_t pwm_shared[PWM_NUM];
const byte ANV = 5;
const byte IN1 = 2;
const byte IN2 = 3;

const int NUMBER_OF_FIELDS = 1;   // how many comma separated fields we expect
int fieldIndex = 0;               // the current field being received
int dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
int serial_values[NUMBER_OF_FIELDS];  // array holding each final value of the serial monitor inputs
int sign[NUMBER_OF_FIELDS];

bool singleLoop = false;
bool condition = true;
double d1, d2, displacement,velocity;
void setup() 
{
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }
  Serial.begin(9600); 
  pinMode(PWMS_INPUT, INPUT);
  pinMode(PWMP_INPUT, INPUT);
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(22,OUTPUT);
  digitalWrite(22,HIGH);
  enableInterrupt(PWMS_INPUT, calc_speed, CHANGE);
  enableInterrupt(PWMP_INPUT, calc_position, CHANGE);  
}

void loop() 
{
  while(singleLoop)
  {
    pwm_read_values();
    d1 = pwm_values[PWMP];
    Serial.println(serial_values[0]);
    Serial.print("Position start: "); Serial.print(pwm_values[PWMP]);
    Mdirection(serial_values[0]);
    analogWrite(ANV,abs(serial_values[0]));
    delay(500);
    pwm_read_values();
    velocity = pwm_values[PWMS] - 510;
    delay(500);
    //Mdirection(0);
    delay(500);
    pwm_read_values();
    d2 = pwm_values[PWMP];
    Serial.print("   Position end: "); Serial.print(pwm_values[PWMP]);
    displacement = d1 - d2;
    Serial.print("   Displacement: "); Serial.println(displacement);
    Serial.print("Speed: "); Serial.println(velocity);
//      Mdirection(-values[0]);
//      analogWrite(ANV,abs(values[0]));
//      delay(1500);
//      digitalWrite(IN1,LOW);
//      digitalWrite(IN2,LOW);
     //analogWrite(ANV,0);
//      Serial.print("Position: "); Serial.println(PWMP);
     singleLoop = false;
  }
  if(condition)
  {
    analogWrite(ANV,abs(serial_values[0]));
    pwm_read_values();
    velocity = pwm_values[PWMS] -510;
    Serial.print(velocity); Serial.print(","); Serial.println(displacement);
    singleLoop = false;
  }
}

void serialEvent()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    //Serial.println(ch);
    if(ch >= '0' && ch <= '9')
    {
      dummy[fieldIndex] = (dummy[fieldIndex] * 10) + (ch - '0');
      //Serial.println(dummy[fieldIndex]);
    }
    else if(ch == ',')
    {
      if(fieldIndex < (NUMBER_OF_FIELDS - 1))
        fieldIndex++;
    }
    else if(ch == '-')
      sign[fieldIndex] = -1;
    else if(ch == 's')
    {
      condition = !condition;
      Serial.println(condition);
    } 
    else
    {
      for(int i = 0; i < NUMBER_OF_FIELDS; i++)
      {
        serial_values[i] = dummy[i]*sign[i];
        dummy[i] = 0;
        sign[i] = 1;
      }
      fieldIndex = 0;   
      singleLoop = true;
    } 
    
  }
}
void Mdirection(int dir)
{
  if(dir > 0)
  {
    //extend
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if(dir < 0)
  {
    //retract
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
void pwm_read_values() 
{
  noInterrupts();
  memcpy(pwm_values, (const void *)pwm_shared, sizeof(pwm_shared));
  interrupts();
  velocity = (1/26)*(pwm_values[PWMS] - 510) + 1/13;
  displacement = 7.5 / 990 * pwm_values[PWMP] - 7.5;
}

void calc_input(uint8_t channel, uint8_t input_pin) 
{
  if (digitalRead(input_pin) == HIGH) 
  {
    pwm_start[channel] = micros();
  } 
  else 
  {
    uint16_t pwm_compare = (uint16_t)(micros() - pwm_start[channel]);
    if(pwm_compare > 1002)
    {
      pwm_values[PWMP] = 0;
    }
    pwm_shared[channel] = pwm_compare;
  }
}
void calc_speed() { calc_input(PWMS, PWMS_INPUT); }
void calc_position() { calc_input(PWMP, PWMP_INPUT); }
