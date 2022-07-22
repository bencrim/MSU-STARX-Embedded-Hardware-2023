/*
  Repeating WiFi Web Client

 This sketch connects to a a web server and makes a request
 using a WiFi equipped Arduino board. Sends the value of the analog input pins
 to the host.

 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoLowPower.h>

//#include "login_credentials.h"

/* Please enter your sensitive data in the Secret tab/login_credentials.h */
char ssid[] = "starxnetwork";      // Your network SSID (name)
// Password has to be >= 8 length
char pass[] = "starxtext";  // Your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;         // Your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Initialise the WiFi client library
WiFiClient client;

// Server address:
//char server[] = "http://192.168.4.1/";
byte server[] = {192, 168, 4, 1};
//IPAddress server(192,168,0,1);

unsigned long lastConnectionTime = 0;   // Last time you connected to the server, in milliseconds
const unsigned long postingInterval = 5L;  // Delay between updates, in milliseconds

// Buffer of HTML.
char c;

// Buffer of EMG array
int maxMatrixSize = 90;

// Message being sent to host.
String idEmg = "A: ";
String clientMessage = String() + idEmg;

// Second to send the clientMessage on:
unsigned long secondWait = 3;
unsigned long realTimeClock = 0;


void setup()
{
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
      Serial.println("Communication with WiFi module failed!");
      // Don't continue
      while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
      Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
      WiFi.lowPowerMode();  // Enable WiFi Low Power Mode
      // Wait 5 seconds for connection:
      delay(5000);
  }
  // You're connected now, so print out the status:
  printWifiStatus();
}

void loop()
{
  /*
  If there's incoming data from the net connection.
  send it out the serial port.  This is for debugging
  purposes only:
  an HTTP request ends with a blank line
  */
  while (client.available())
  {
      c = client.read();
      Serial.write(c);
  }

  /*
  If ten milliseconds have passed since your last connection,
  then connect again and send data:
  */
  if (millis() - lastConnectionTime > postingInterval)
  {
      httpRequest();
  }

}

/*
Updates Real Time Clock (RTC) of Arduino MKR1010 Wifi.
@return epoch Integer of the time in seconds since January 1st, 1970 on success. 0 on failure.
*/
int updateRTC()
{
    unsigned long epoch;
    int numberOfTries = 0, maxTries = 6;
    do 
    {
        epoch = WiFi.getTime();
        numberOfTries++;
    }
    while ((epoch == 0) && (numberOfTries < maxTries));
    if (numberOfTries == maxTries) 
    {
        Serial.print("NTP unreachable!!");
        while (true);
    }
    else 
    {
        Serial.print("Epoch received: ");
        Serial.println(epoch);
        Serial.println();
        return epoch;
    }
}

// This method makes a HTTP connection to the server:
void httpRequest()
{
  // Close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();
  // If there's a successful connection:
  if (client.connect(server, 80))
  {
      // Reset message being sent to host.
      clientMessage = String() + idEmg;
      for (int pointerEmg = 0; pointerEmg <= maxMatrixSize; pointerEmg++)
      {
          // Prepare to send clientMessage to host.
          int sensorValue0 = analogRead(A0);
          if (pointerEmg == maxMatrixSize)
          {
              // Remove last ", " from clientMessage.
              clientMessage.remove(clientMessage.length() - 1);
              clientMessage.remove(clientMessage.length() - 1);
              // Check the Serial output is correct for client.
              Serial.print("Finished Raw EMG Message: " + clientMessage);
              Serial.print("\n");
              // Wait until we reach a secondWait th second before sending message.
              do
              {
                realTimeClock = updateRTC();
              }
              while (realTimeClock % secondWait != 0);
              // Send clientMessage to host.
              client.print(clientMessage);
              client.println();
              break;
          }
          clientMessage = clientMessage + sensorValue0 + ", ";
      }
      // Note the time that the connection was made:
      lastConnectionTime = millis();
  }
  else
  {
      // If you couldn't make a connection:
      Serial.println("connection failed");
  }
}


void printWifiStatus()
{
  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
