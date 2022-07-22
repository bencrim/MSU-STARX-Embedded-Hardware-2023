const int NUMBER_OF_FIELDS = 5;   // how many comma separated fields we expect
int fieldIndex = 0;               // the current field being received
int dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
int values[NUMBER_OF_FIELDS];  // array holding each final value of the serial monitor inputs
int sign[NUMBER_OF_FIELDS];
int singleLoop = false;

void setup() 
{
  // Serial and pin mode setup
  Serial.begin(9600);
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }
}

void loop() 
{
  while(singleLoop)
  {
    for(int i = 0; i < NUMBER_OF_FIELDS; i++)
      Serial.println(values[i]);
    singleLoop = false;
  }
}
void serialEvent()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    if(ch >= '0' && ch <= '9')
    {
      dummy[fieldIndex] = (dummy[fieldIndex] * 10) + (ch - '0');
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
        values[i] = dummy[i]*sign[i];
        dummy[i] = 0;
        sign[i] = 1;
      }
      fieldIndex = 0;   
      singleLoop = true;
    } 
  }
}
