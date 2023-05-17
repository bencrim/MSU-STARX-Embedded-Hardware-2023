// Demo for getting individual unified sensor data from the HTS221 Humidity and Temperature sensor

#include <Adafruit_HTS221.h>

Adafruit_HTS221 hts;
Adafruit_Sensor *hts_humidity, *hts_temp;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit HTS221 test!");

  if (!hts.begin_I2C()) {
    Serial.println("Failed to find HTS221 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("HTS221 Found!");
  hts_temp = hts.getTemperatureSensor();
  hts_temp->printSensorDetails();

  hts_humidity = hts.getHumiditySensor();
  hts_humidity->printSensorDetails();
}

void loop() {
  //  /* Get a new normalized sensor event */
  sensors_event_t humidity;
  sensors_event_t temp;
  hts_humidity->getEvent(&humidity);
  hts_temp->getEvent(&temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (humidity is measured in % relative humidity (% rH) */
  Serial.print("\t\tHumidity: ");Serial.print(humidity.relative_humidity);Serial.println(" % rH");
  Serial.print("\t\tTemperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");


  delay(100);

  /*//   serial plotter friendly format
  Serial.print(temp.temperature);
  Serial.print(",");

  Serial.print(humidity.relative_humidity);

  Serial.println();
  delay(10);
  */
}
