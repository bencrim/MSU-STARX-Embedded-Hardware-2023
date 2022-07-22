
//actuator values
float desired_actuator_length = 0;

void setup() {
     Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

}


void serialEvent()
{
  //check if there is something in the input
  if(Serial.available() > 0)
  {
    //get the input
    String input = Serial.readString();
    //update with new length goal
    desired_actuator_length = input.toFloat();
    Serial.println(desired_actuator_length);
  }
}
