//Libraries
#include <PID_v1.h>     //PID library
#include <EnableInterrupt.h>  //library that enables to read pwm values
#include <Adafruit_MPU6050.h> //gyro library
#include <Adafruit_Sensor.h>  //library to run adafruit sensors
#include <Wire.h>   //library that enables serail communications
#include <GyroToVelocity.h>   //geometry functions for HIP and KNEE actuators
//Serial port baud and Arduino pins
#define SERIAL_PORT_SPEED 9600 
#define PWM_NUM  2 //number of PWM signals
#define PWMS  0 //speed is stored in the 0 position of the array
#define PWMP  1 //position is stored in the 1 position of the array
#define PWMS_INPUT  A0 //analog pin for the speed reading
#define PWMP_INPUT  A1 //analog pin for the position reading
#define ANV 5 //analog voltage speed control pin
#define IN1 2 //direction of actuation
#define IN2 3 //direction of actuation
#define TO_STANDING 53
//zero rate offset in radians/s
//recalibrate gyros periodically, probably once a month or before testing
const double gyro1[] = {-0.0213, -0.0192, -0.03};
const double gyro2[] = {-0.0919, 0.052, -0.019};
const double gyro3[] = {-0.0511, -0.0076, -0.0019};
const double gyro4[] = {-0.0588, -0.0185, -0.0298};
const double gyro5[] = {-0.0231, -0.0526, -0.0486};
const double gyro6[] = {-0.0254, -0.0077, -0.0045};
const double gyro7[] = {-0.1003, -0.0366, -0.0014};
const double gyro8[] = {-0.0791, -0.027, -0.0346};
//variable that holds the specific gyro offset
double X_OFFSET[2];
double Y_OFFSET[2];
double Z_OFFSET[2];
//interrupt arrays
uint32_t pwm_start[PWM_NUM]; // stores the time when PWM square wave begins
uint16_t pwm_values[PWM_NUM]; //stores the width of the PWM pulse in microseconds
volatile uint16_t pwm_shared[PWM_NUM]; //array used in the interrupt routine to hold the pulse width values

Adafruit_MPU6050 mpu; //gyro sensor
Adafruit_MPU6050 mpu1;
double corrected_X[2], corrected_Y[2], corrected_Z[2]; //variables that store the angular speed after the offset has been applied

bool lock_in_place = false;

//PID_runtime
double currentSpeed, currentSpeed_abs, outputSpeed ,desiredSpeed, desiredSpeed2;
double displacement; //position variable
double HP = 50, HI = 20, HD = 0;
PID loadCompensator(&currentSpeed_abs, &outputSpeed ,&desiredSpeed, HP, HI, HD,P_ON_M, DIRECT);
bool runtime_status = true; //keeps track if PID is on or off

/*
PID_stop

This PID is needed as this PID goes by displacment instead of speed
*/
double to_standing_error;
double setpoint = 6.125;   //the point that is considered to standing postion in inches
double HP_stop = 50, HI_stop = 20, HD_stop = 0;  //these are the parameters to the PID
PID StoppingPID(&displacement, &outputSpeed ,&setpoint, HP_stop, HI_stop, HD_stop,P_ON_M, DIRECT);
bool to_standing_status = false; //keeps track if PID is on or off

union Onion
{
    uint8_t     fBytes[sizeof( float )];
    float       fValue;
};
Onion flt;

void setup() 
{
  //begin serial monitor
  Serial.begin(SERIAL_PORT_SPEED); //from 9600 to 115200


  //establish all pins I/O
  pinMode(PWMS_INPUT, INPUT);
  pinMode(PWMP_INPUT, INPUT);
  pinMode(TO_STANDING,INPUT);
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(22,OUTPUT); //dummy pin
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(22,HIGH); //always HIGH

  //interrupts used to read the linear actuator feedback
  enableInterrupt(PWMS_INPUT, calc_speed, CHANGE);
  enableInterrupt(PWMP_INPUT, calc_position, CHANGE); 
  //PI Controller for sending to soft stop
  //placed here as I will toggle the High Pin when turning off this PID
  StoppingPID.SetMode(MANUAL);
  StoppingPID.SetOutputLimits(0,255);
  StopPID_status(true); // turn off this PID
  
  //PI Controller for load compensator
  pwm_read_values(); 
  desiredSpeed = 0;
  loadCompensator.SetMode(AUTOMATIC);
  loadCompensator.SetOutputLimits(0,255);

  HorK(HIP); //Hip or Knee geometry
  //Checks for gyroscope
  if (!mpu.begin(0x69)) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  //range settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
  offsetSwitch(0,1); //Check the gyro number connected to the arduino
  offsetSwitch(1,5);
}

void loop() 
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  
  sensors_event_t a1, g1, temp1;
  mpu.getEvent(&a1, &g1, &temp1);
  
  //Sets the gyro value to 0 if it is below the threshold
  if(abs(g1.gyro.x - X_OFFSET[1]) <= 0.3)  corrected_X[1] = 0;
  else  corrected_X[1] = g1.gyro.x - X_OFFSET[1];   
  pwm_read_values();
  desiredSpeed = abs(geometry(corrected_X[1], displacement));
  desiredSpeed2 = geometry(corrected_X[1], displacement);
  Mdirection(corrected_X[1]);
  loadCompensator.Compute();
  analogWrite(ANV,outputSpeed);
  Serial.print(corrected_X[1]);Serial.print("   ");Serial.print(currentSpeed);Serial.print("   ");Serial.println(desiredSpeed);
  
  flt.fValue = corrected_X[1];
  Sender();

//////////////////////// TO standing implementation
  if((digitalRead(TO_STANDING))) toStanding();
////////////////////////
}
void toStanding()
{
  loadCompensator.SetMode(MANUAL);
  StoppingPID.SetMode(AUTOMATIC);
  pwm_read_values();
  
  Serial.println(".....Begin Move to Standing.....");
  String message = "Understood to be at distance " + String(displacement) +" Setpoint is : " + String(setpoint);
  Serial.println(message);
  to_standing_error = setpoint-displacement;
  //given 0.2 inch error within the setpoint
  while(abs(to_standing_error)>0.2)
  {
    //collect the current actuator speed and displacement
    pwm_read_values();  // this will update current speed and the displacment
    //compute the displacement with the PID
    to_standing_error = setpoint - displacement;
    Serial.println(to_standing_error);
    Mdirection(to_standing_error);
    StoppingPID.Compute(); 
    //write the PID result to the actuator
    analogWrite(ANV, outputSpeed);
   }
   Mdirection(0);
   Serial.println("Pause for durration");
   delay(500);
   Serial.println(".....resuming System.....");
   loadCompensator.SetMode(AUTOMATIC);
   StoppingPID.SetMode(MANUAL);
   lock_in_place = true;
   while(lock_in_place)
   {
    if(digitalRead(TO_STANDING))lock_in_place = false;
   }
   delay(500);
}
void Mdirection(float dir)
{
  if(dir > 0)
  {
    //extend
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if(dir < 0)
  {
    //retract
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
void pwm_read_values() 
{
  noInterrupts(); //Disables interrupts to make sure it doesnt mess up the memory copy below
  memcpy(pwm_values, (const void *)pwm_shared, sizeof(pwm_shared)); //memcpy handles memory to memory copy, copies from pwm_shared to pwm_values
  interrupts(); //Enables interrupts
  currentSpeed = ((double)(pwm_values[PWMS] - 510)/26+1/13); //calculates the real value [inch/sec] for the actuator speed
  currentSpeed_abs = abs(currentSpeed);
  displacement = abs(7.5 / 990 * pwm_values[PWMP] - 7.5); //calculates the real value [inch] for the actuator position
  if(displacement > 7.5){displacement = 7.5;}
}

void calc_input(uint8_t channel, uint8_t input_pin) //Does the math for the PWM data
{
  if (digitalRead(input_pin) == HIGH) //Measures how long the PWM is that its peak
  {
    pwm_start[channel] = micros(); //Micros returns the number of microseconds since the board began running
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

/*
 * This function is used to turn the stopping PID on and off
 *  IMPORTANT NOTE: This will also toggle the other PID on/off
 */

void StopPID_status(bool PID_off)
{
  
  if(PID_off)
  {
    StoppingPID.SetMode(MANUAL); //Manual - PI is deactivated
    to_standing_status = MANUAL;
    
    //turn other PID on
    loadCompensator.SetMode(AUTOMATIC);//Automatic - PI is activated
    runtime_status = AUTOMATIC;
  }
  else
  {
    StoppingPID.SetMode(AUTOMATIC);//Automatic - PI is activated
    to_standing_status = AUTOMATIC;

    //turn other PID off
    loadCompensator.SetMode(MANUAL); //Manual - PI is deactivated
    runtime_status = MANUAL;
  }
}


//the code below is going through testing
void PIDtoggle(bool hardstop) 
{
  if(hardstop == true || displacement >= 7.5 || displacement <= 0.05)
  {
    loadCompensator.SetMode(MANUAL); //Manual - PI is deactivated
    runtime_status = MANUAL;
    digitalWrite(22,HIGH);
  }
  else
  {
    loadCompensator.SetMode(AUTOMATIC);//Automatic - PI is activated
    runtime_status = AUTOMATIC;
    digitalWrite(22,LOW);
  }
}
void offsetSwitch(int gyro, int choice)
{
    switch(choice)
    {
        case 1:
            X_OFFSET[gyro] = gyro1[1];
            Y_OFFSET[gyro] = gyro1[2];
            Z_OFFSET[gyro] = gyro1[3];
        break;

        case 2:
            X_OFFSET[gyro] = gyro2[1];
            Y_OFFSET[gyro] = gyro2[2];
            Z_OFFSET[gyro] = gyro2[3];
        break;

        case 3:
            X_OFFSET[gyro] = gyro3[1];
            Y_OFFSET[gyro] = gyro3[2];
            Z_OFFSET[gyro] = gyro3[3];
        break;
       
        case 4:
            X_OFFSET[gyro] = gyro4[1];
            Y_OFFSET[gyro] = gyro4[2];
            Z_OFFSET[gyro] = gyro4[3];
        break;

        case 5:
            X_OFFSET[gyro] = gyro5[1];
            Y_OFFSET[gyro] = gyro5[2];
            Z_OFFSET[gyro] = gyro5[3];
        break;
        
        case 6:
            X_OFFSET[gyro] = gyro6[1];
            Y_OFFSET[gyro] = gyro6[2];
            Z_OFFSET[gyro] = gyro6[3];
        break;
        
        case 7:
            X_OFFSET[gyro] = gyro7[1];
            Y_OFFSET[gyro] = gyro7[2];
            Z_OFFSET[gyro] = gyro7[3];
        break;
        
        case 8:
            X_OFFSET[gyro] = gyro8[1];
            Y_OFFSET[gyro] = gyro8[2];
            Z_OFFSET[gyro] = gyro8[3];
        break;
    }
}
void Sender()
{
  Serial.write( '>' );
  Serial.write( flt.fBytes, sizeof( float ) );
  Serial.write( '<' );
}
