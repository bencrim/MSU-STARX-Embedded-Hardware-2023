#include <PID_v1.h>
#include <EnableInterrupt.h>
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

//Excel data
const int NUMBER_OF_FIELDS = 1; // how many comma separated fields we expect
int fieldIndex = 0;            // the current field being received
double dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
double serial_values[NUMBER_OF_FIELDS];
int sign[NUMBER_OF_FIELDS];
bool singleLoop = false;
bool condition = false;

//PID
double currentSpeed, outputSpeed ,desiredSpeed;
double HP = 100, HI = 500, HD = 0;
PID loadCompensator(&currentSpeed, &outputSpeed ,&desiredSpeed, HP, HI, HD,P_ON_M, DIRECT);
int mode = 1;
float displacement, velocity;
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
  
  //PI Controller
  pwm_read_values();
  desiredSpeed = 50;
  loadCompensator.SetMode(AUTOMATIC);
  loadCompensator.SetOutputLimits(0,255);
}

void loop() 
{
  while(singleLoop)
  {
    
    Serial.println("Setting...");
    delay(1000);
    desiredSpeed = abs(serial_values[0]);
    Mdirection(serial_values[0]);
    loadCompensator.SetTunings(HP,HI,0);
    singleLoop = false;
    //loadCompensator.SetMode(AUTOMATIC);
  }
  pwm_read_values();
  PIDtoggle(pwm_values[PWMS]);
  loadCompensator.Compute();
  analogWrite(ANV, outputSpeed);
  Serial.println(currentSpeed);
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
  currentSpeed = abs(((double)(pwm_values[PWMS] - 510)/26+1/13));
  displacement = abs(7.5 / 990 * pwm_values[PWMP] - 7.5);
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
      pwm_compare = 0;
    }
    pwm_shared[channel] = pwm_compare;
  }
}
void calc_speed() { calc_input(PWMS, PWMS_INPUT); }
void calc_position() { calc_input(PWMP, PWMP_INPUT); }

void PIDtoggle(int hardstop)
{
  if(hardstop > 500 && hardstop < 515)
  {
    loadCompensator.SetMode(MANUAL);
    mode = MANUAL;
  }
  else
  {
    loadCompensator.SetMode(AUTOMATIC);
    mode = AUTOMATIC;
  }
}
