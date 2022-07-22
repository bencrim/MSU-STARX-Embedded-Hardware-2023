#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

union Onion
{
    uint8_t     fBytes[sizeof( float )];
    float       fValue;
};
Onion flt;
float hip_ang_vel_x;
void setup( void )
{
    Serial.begin(9600);
    while( !Serial );
    
}//setup

void loop( void )
{
  hip_ang_vel_x = Receive();
  Serial.println(hip_ang_vel_x);
    
    
}//loop
float Receive()
{
  byte ch, ch1, idx;
  bool done;
  float dummy;
        
    if( Serial.available() > 0 )
    {
        if( Serial.read() == '>' )
        {
          done = false;
          idx = 0;
          while( !done )
          {
            if( Serial.available() > 0 )
            {
              ch = Serial.read();
              if( ch == '<' )
              done = true;
              else
              {
                if( idx < sizeof( float ) )
                            flt.fBytes[idx++] = ch;
              }//else
            }//if
                   
          }//while
          //Serial.print( "Float value received: " ); Serial.println( flt.fValue, 4 );
            
        }//if
        
    }//if
    if(flt.fValue > -9 && flt.fValue <9)
    {
      dummy = flt.fValue; 
      return flt.fValue;
    }
    else
    {
      return dummy;
    }
}
