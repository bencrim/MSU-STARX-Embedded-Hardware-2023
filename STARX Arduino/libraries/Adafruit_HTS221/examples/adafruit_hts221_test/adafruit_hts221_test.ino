// Basic demo for reading Humidity and Temperature
#include <Wire.h>
#include <Adafruit_HTS221.h>
#include <Adafruit_Sensor.h>

// For SPI mode, we need a CS pin
#define HTS_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define HTS_SCK 13
#define HTS_MISO 12
#define HTS_MOSI 11

Adafruit_HTS221 hts;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit HTS221 test!");

  // Try to initialize!
  if (!hts.begin_I2C()) {
//  if (!hts.begin_SPI(HTS_CS)) {
//  if (!hts.begin_SPI(HTS_CS, HTS_SCK, HTS_MISO, HTS_MOSI)) {
    Serial.println("Failed to find HTS221 chip");
    while (1) { delay(10); }
  }
  Serial.println("HTS221 Found!");

//  hts.setDataRate(HTS221_RATE_1_HZ);
  Serial.print("Data rate set to: ");
  switch (hts.getDataRate()) {
   case HTS221_RATE_ONE_SHOT: Serial.println("One Shot"); break;
   case HTS221_RATE_1_HZ: Serial.println("1 Hz"); break;
   case HTS221_RATE_7_HZ: Serial.println("7 Hz"); break;
   case HTS221_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
  }

}

void loop() {

  sensors_event_t temp;
  sensors_event_t humidity;
  hts.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  delay(500);
}
