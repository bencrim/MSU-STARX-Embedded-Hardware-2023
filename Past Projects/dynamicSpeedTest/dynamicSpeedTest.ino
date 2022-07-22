#define ANV_PIN 7
#define IN1 3
#define IN2 4
#define IO_PIN 5

double difference;

// Length of linear actuator from ranges 0-255.
double Setpoint = 0;

// Rate of Change for Position to find velocity of linear actuator.
unsigned long lastTime = 0;   // Last time you connected to the server, in milliseconds.
const unsigned long updateDelay = 1000L;    // Delay between updates, in milliseconds.
float previousDistance = 0;
float deltaSpeed = 0;   // Speed value calculated from change of position.

void setup() {
  // put your setup code here, to run once:
  pinMode(ANV_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IO_PIN, INPUT);

  Serial.begin(115200);
}

void loop() {
  int Input_Current = map(analogRead(IO_PIN), 0, 1023, 0 , 255);     // reads the current input value between 0 - 255.
  Serial.print("Input_Current: ");
  Serial.println(Input_Current);
 
  difference = Setpoint - Input_Current;

  // if 1000 milliseconds have passed since your last connection, then update delta for velocity.
  if (millis() - lastTime > updateDelay) {
    updateDelta(difference, previousDistance);
    Serial.print("DeltaSpeed: ");
    Serial.println(deltaSpeed);
  }

  analogWrite(ANV_PIN, deltaSpeed); // Changes speed on values 0-255.
//  Serial.println(deltaSpeed);

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

  /*
  // put your main code here, to run repeatedly:
  extend();
  delay(3000);
  retract();
  delay(3000);
  */
}

void extend() {
  Serial.println("Extending...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void retract() {
  Serial.println("Retracting...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void updateDelta(float currentDistance, float previousDistance) {
  deltaSpeed = abs((currentDistance - previousDistance) / (updateDelay / 1000));   // Delta speed is in analog value (0-255) / updateDelay (1 sec)
  lastTime = millis();
}
