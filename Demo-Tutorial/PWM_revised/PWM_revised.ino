#include <EnableInterrupt.h>

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

void setup() 
{
  Serial.begin(SERIAL_PORT_SPEED);
  pinMode(PWMS_INPUT, INPUT);
  pinMode(PWMP_INPUT, INPUT);
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ANV,1);
  
  enableInterrupt(PWMS_INPUT, calc_speed, CHANGE);
  enableInterrupt(PWMP_INPUT, calc_position, CHANGE);
  
}

void loop() 
{
  pwm_read_values();

  Serial.print(pwm_values[PWMS]); Serial.print(","); Serial.println(pwm_values[PWMP]);
}
void pwm_read_values() 
{
  noInterrupts();
  memcpy(pwm_values, (const void *)pwm_shared, sizeof(pwm_shared));
  interrupts();
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
