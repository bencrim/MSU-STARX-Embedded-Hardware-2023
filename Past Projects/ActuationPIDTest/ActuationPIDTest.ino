int Input = A0;
int Output = 4;
char inputText;
int inputNumber;
int difference;

// Set setpoint you want to go to. Currently its at 3 inches.
int ePOS = 255;

void setup() {
  Serial.begin(115200);  // initialize serial communication at 115200 bits per second

  pinMode(Input, INPUT);
  pinMode(Output, OUTPUT);
  
  //retract(); // retracts the stroke on startup
  delay(500); // Delay half a second.
}

void extend() // this function enables the motor to run
{
  analogWrite(Output, 255);
}

void retract() // this function reverses the direction of motor
{
  analogWrite(Output, 0);
}

void run_stop() // this function disables the motor
{
  
}

void loop() {
  if ((Serial.available() > 0))
  {
    inputText = Serial.read();
    inputNumber = (int)(inputText);
  }

  int Setpoint = (inputNumber);

  if (Setpoint >= ePOS || Setpoint <= 0)
  {
    Setpoint = 1;
  }
  
  int Input_Current = analogRead(Input);     // reads the current input value between 0 - 1023.

  difference = Setpoint - Input_Current;

  if (difference > Input_Current) // Setpoint is greater length than current length.
  {
    retract(); // retract the stroke
  }
 
  else if (difference < Input_Current) // Setpoint is less than length of current length.
  {
    extend(); // extend the stroke
  }
  else // Setpoint is eqivalent length than current length.
  {
    
  }
  // 5ms delay.
  delay(5);
}
