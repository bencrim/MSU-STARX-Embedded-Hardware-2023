/*
  WiFi Web Server

 A AP web server that reads analog values from clients.

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the WiFi.begin() call accordingly.

 Circuit:
 * Analog inputs attached to pins A0 through A5 (optional)
 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoLowPower.h>
#include <RTCZero.h>

//#include "login_credentials.h"

/* Please enter your sensitive data in the Secret tab/login_credentials.h */
char ssid[] = "starxnetwork";      // Your network SSID (name)
// Password has to be >= 8 length
char pass[] = "starxtext";  // Your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;               // Your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(80);

// User input control
int lineCounter = 0;
bool readyUserInput = false;

// Message that is Arduino string and contains one line of data from client emg.
String serverMessage = String();
bool userPromptSubmitted = false;
char c = ' ';

void setup()
{
  // Initialize serial and wait for port to open:
  Serial.begin(115200);

  Serial.println("Access Point Web Server");

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

  /*
  By default the local IP address will be 192.168.4.1
  you can override it with the following: E.g.
  WiFi.config(IPAddress(10, 0, 0, 1));
  */

  // Print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  WiFi.lowPowerMode();  // Enable WiFi Low Power Mode
  if (status != WL_AP_LISTENING)
  {
      Serial.println("Creating access point failed");
      // Don't continue
      while (true);
  }

  // Wait 5 seconds for connection:
  delay(5000);

  // Start the web server on port 80
  server.begin();

  // You're connected now, so print out the status
  printWifiStatus();
}


void loop() {
  // Compare the previous status to the current status
  if (status != WiFi.status())
  {
      // It has changed update the variable
      status = WiFi.status();

      if (status == WL_AP_CONNECTED)
      {
      // A device has connected to the AP
      Serial.println("Device connected to AP");
      } else
      {
      // A device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
      }
  }
    
  // Listen for incoming clients
  WiFiClient client = server.available();
  if (client)
  {
      //Serial.println("new client");
      while (client.connected())
      {
          if (client.available())
          {
             
              char c = client.read();
              if (c == '\t')
              {
                  // Once newline is found in message from client, restart message.
                  Serial.println(serverMessage);
                  serverMessage = String();
                  lineCounter += 1;
                  break;
              }
              else
              {
                  // Append each chracter to string. This is the same as Serial.readString();
                  serverMessage = serverMessage + String(c);
              }
             


          }
      }
      // Give the web browser time to receive the data
      delay(1);

      // Close the connection
      client.stop();
      //Serial.println("client disconnected");
  }
}


void printWifiStatus()
{
  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
