//PWM reading variables
volatile unsigned long timer[2]; //timer that holds the rising and falling edge of PIN 24/DIO1->reads position waveform
volatile double PWMin; //holds Duty cycle value
volatile long Duty;
double error;
//pinout constants
const byte ANV = 2;
const byte DIO = 24;
const byte IN1 = 26;
const byte IN2 = 27;

void setup() 
{
  Serial.begin(9600);
  pinMode(DIO, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ANV,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(24),ISR1, CHANGE); //attaches interrupt service routine to PIN 24 and triggers anytime there is a change in input  
  analogWrite(ANV,255);
  digitalWrite(IN1,HIGH);
  //digitalWrite(IN2,HIGH);
}
void loop() 
{
  //PIDCompute(10,50,60);
  Serial.println((PWMin));
}
void ISR1() 
{
  switch(digitalRead(24))
  {
    case 0:
    timer[0] = micros();
    break;
    case 1:
    timer[1] = micros();
    Duty = timer[1] - timer[0];
    PWMin = Duty - 502;
    break;
  }
}
void Mdirection(double error)
{
  if(error < 0)
  {
    //extend
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    //M_dir = 0;
  }
  if(error > 0)
  {
    //contract
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    //M_dir = 1;
  }
}
