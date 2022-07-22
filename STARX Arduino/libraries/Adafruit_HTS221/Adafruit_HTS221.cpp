
/*!
 *  @file Adafruit_HTS221.cpp
 *
 *  @mainpage Adafruit HTS221 Humidity and Temperature Sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit HTS221 Humidity and Temperature Sensor
 * library
 *
 * 	This is a library for the Adafruit HTS221 breakout:
 * 	https://www.adafruit.com/product/453X
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_HTS221.h"

/*!
 *    @brief  Instantiates a new HTS221 class
 */
Adafruit_HTS221::Adafruit_HTS221(void) {}
Adafruit_HTS221::~Adafruit_HTS221(void) {
  if (temp_sensor) {
    delete temp_sensor;
  }
  if (humidity_sensor) {
    delete humidity_sensor;
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_HTS221::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                int32_t sensor_id) {

  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_HTS221::begin_SPI(uint8_t cs_pin, SPIClass *theSPI,
                                int32_t sensor_id) {
  i2c_dev = NULL;

  if (spi_dev) {
    delete spi_dev; // remove old interface
  }
  spi_dev = new Adafruit_SPIDevice(cs_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0,             // data mode
                                   theSPI);
  if (!spi_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes software SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  sck_pin The arduino pin # connected to SPI clock
 *    @param  miso_pin The arduino pin # connected to SPI MISO
 *    @param  mosi_pin The arduino pin # connected to SPI MOSI
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_HTS221::begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                                int8_t mosi_pin, int32_t sensor_id) {
  i2c_dev = NULL;

  if (spi_dev) {
    delete spi_dev; // remove old interface
  }
  spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0);            // data mode
  if (!spi_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_HTS221::_init(int32_t sensor_id) {
  if (spi_dev) {
    multi_byte_address_mask = 0x40;
  }
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_WHOAMI, 1);

  // make sure we're talking to the right chip
  if (chip_id.read() != HTS221_CHIP_ID) {
    return false;
  }

  _sensorid_humidity = sensor_id;
  _sensorid_temp = sensor_id + 1;
  boot();
  setActive(true); // arise!
  setDataRate(
      HTS221_RATE_12_5_HZ); // set to max data rate (default is one shot)

  _fetchTempCalibrationValues();
  _fetchHumidityCalibrationValues();

  humidity_sensor = new Adafruit_HTS221_Humidity(this);
  temp_sensor = new Adafruit_HTS221_Temp(this);
  return true;
}

/**
 * @brief Restores the trimming function values into registers from flash
 *
 */
void Adafruit_HTS221::boot(void) {
  Adafruit_BusIO_Register ctrl_2 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_CTRL_REG_2, 1);
  Adafruit_BusIO_RegisterBits boot = Adafruit_BusIO_RegisterBits(&ctrl_2, 1, 7);

  boot.write(1);
  while (boot.read()) {
    delay(1);
  }
}

/**
 * @brief Sets the sensor to active or inactive
 *
 * @param active Set to true to enable the sensor, false to disable
 */
void Adafruit_HTS221::setActive(bool active) {
  Adafruit_BusIO_Register ctrl_1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_CTRL_REG_1, 1);
  Adafruit_BusIO_RegisterBits pd_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_1, 1, 7);

  pd_bit.write(active);
}

/**
 * @brief Sets the polarity of the DRDY pin when active
 *
 * @param active_low Set to true to make the DRDY pin active low, false for
 * active low
 */
void Adafruit_HTS221::drdyActiveLow(bool active_low) {
  Adafruit_BusIO_Register ctrl_3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_CTRL_REG_3, 1);
  Adafruit_BusIO_RegisterBits drdy_active_low_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_3, 1, 7);

  drdy_active_low_bit.write(active_low);
}

/**
 * @brief Enables or disables the Data Ready (DRDY) interrupt on the DRDY pin
 *
 * @param drdy_int_enabled Set to true to enable the DRDY interrupt, false to
 * disable
 */
void Adafruit_HTS221::drdyIntEnabled(bool drdy_int_enabled) {
  Adafruit_BusIO_Register ctrl_3 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_CTRL_REG_3, 1);
  Adafruit_BusIO_RegisterBits drdy_int_enabled_bit =
      Adafruit_BusIO_RegisterBits(&ctrl_3, 1, 2);

  drdy_int_enabled_bit.write(drdy_int_enabled);
}

/**
 * @brief Returns the current measurement rate
 *
 * @return hts221_rate_t the current measurement rate
 */
hts221_rate_t Adafruit_HTS221::getDataRate(void) {
  Adafruit_BusIO_Register ctrl_1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_CTRL_REG_1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&ctrl_1, 2, 0);

  return (hts221_rate_t)data_rate_bits.read();
}

/**
 * @brief Sets the rate at which measurements are taken
 *
 * @param data_rate The new measurement rate. Must be a `hts221_rate_t`
 */
void Adafruit_HTS221::setDataRate(hts221_rate_t data_rate) {
  Adafruit_BusIO_Register ctrl_1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, HTS221_CTRL_REG_1, 1);
  Adafruit_BusIO_RegisterBits data_rate_bits =
      Adafruit_BusIO_RegisterBits(&ctrl_1, 2, 0);

  data_rate_bits.write(data_rate);
}

/**************************************************************************/
/*!
    @brief  Gets the humidity sensor and temperature values as sensor events
    @param  humidity Sensor event object that will be populated with humidity
   data
    @param  temp Sensor event object that will be populated with temp data
    @returns true if the event data was read successfully
*/
/**************************************************************************/
bool Adafruit_HTS221::getEvent(sensors_event_t *humidity,
                               sensors_event_t *temp) {
  uint32_t t = millis();
  if (_read() != true) {
    return false;
  };

  // use helpers to fill in the events
  fillTempEvent(temp, t);
  fillHumidityEvent(humidity, t);
  return true;
}

void Adafruit_HTS221::fillTempEvent(sensors_event_t *temp, uint32_t timestamp) {
  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = timestamp;
  temp->temperature = corrected_temp;
}

void Adafruit_HTS221::fillHumidityEvent(sensors_event_t *humidity,
                                        uint32_t timestamp) {
  memset(humidity, 0, sizeof(sensors_event_t));
  humidity->version = sizeof(sensors_event_t);
  humidity->sensor_id = _sensorid_humidity;
  humidity->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  humidity->timestamp = timestamp;
  humidity->relative_humidity = corrected_humidity;
}
/******************* Adafruit_Sensor functions *****************/
/*!
 *  @brief  Updates the measurement data for all sensors simultaneously
 *
    @returns true if the event data was read successfully
 */
/**************************************************************************/
bool Adafruit_HTS221::_read(void) {

  Adafruit_BusIO_Register temp_data =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_TEMP_OUT_L | multi_byte_address_mask), 2);
  Adafruit_BusIO_Register humidity_data = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
      (HTS221_HUMIDITY_OUT | multi_byte_address_mask), 2);

  raw_humidity = 0;
  humidity_data.read(&raw_humidity);

  uint8_t buffer[2];
  if (!temp_data.read(buffer, 2)) {
    return false;
  }
  raw_temperature = 0;

  raw_temperature = buffer[1];
  raw_temperature <<= 8;
  raw_temperature |= buffer[0];

  if (raw_temperature & 0x8000) {
    raw_temperature = raw_temperature - 0xFFFF;
  }

  _applyTemperatureCorrection();
  _applyHumidityCorrection();
  return true;
}
void Adafruit_HTS221::_fetchTempCalibrationValues(void) {
  Adafruit_BusIO_Register t0_degc_x8_l =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_T0_DEGC_X8 | multi_byte_address_mask), 2);
  Adafruit_BusIO_Register t1_t0_msb =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_T1_T0_MSB | multi_byte_address_mask), 1);
  Adafruit_BusIO_Register to_out =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_T0_OUT | multi_byte_address_mask), 2);
  Adafruit_BusIO_Register t1_out =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_T1_OUT | multi_byte_address_mask), 2);
  // From page 26 of https://www.st.com/resource/en/datasheet/hts221.pdf
  uint8_t buffer[4];
  // Get bytes for and assemble T0 and T1
  t1_t0_msb.read(buffer, 1);
  // mask out the MSBs for each value and shift up to T1[8:9]
  T0 = 0;
  T1 = 0;
  T1 = (buffer[0] & 0b1100);
  T1 <<= 6;
  T0 = (buffer[0] & 0b0011);
  T0 <<= 8;

  t0_degc_x8_l.read(buffer, 2);
  //  Or T1[0:7] on to the above to make a full 10 bits
  T0 |= buffer[0];
  T0 >>= 3; // divide by 8 (as documented)
  T1 |= buffer[1];
  T1 >>= 3;

  to_out.read(&T0_OUT);
  t1_out.read(&T1_OUT);
}

void Adafruit_HTS221::_fetchHumidityCalibrationValues(void) {
  Adafruit_BusIO_Register h0_rh_x2 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_H0_RH_X2 | multi_byte_address_mask), 1);
  Adafruit_BusIO_Register h1_rh_x2 =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_H1_RH_X2 | multi_byte_address_mask), 1);

  Adafruit_BusIO_Register h0_t0_out =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_H0_T0 | multi_byte_address_mask), 2);

  Adafruit_BusIO_Register h1_t0_out =
      Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
                              (HTS221_H0_T1 | multi_byte_address_mask), 2);

  h0_rh_x2.read(&H0);
  h1_rh_x2.read(&H1);

  h0_t0_out.read(&H0_T0_OUT);
  h1_t0_out.read(&H1_T0_OUT);
}

/**
 * @brief Use the temperature calibration values to correct the raw value
 *
 */
void Adafruit_HTS221::_applyTemperatureCorrection(void) {

  // info from
  // https://www.st.com/resource/en/datasheet/hts221.pdf
  // Derived from
  // https://github.com/stm32duino/HTS221/blob/b645af37c51c40b0161ea045e11f9f1bc28b8517/src/HTS221_Driver.c#L396

  // I think this can have the consts that are loaded on init factored out to
  // make a simple LSB value
  corrected_temp =
      (float)
          // measured temp(LSB) - offset(LSB) * (calibration measurement delta)
          (float)((int16_t)raw_temperature - (int16_t)T0_OUT) *
          (float)((int16_t)T1 - (int16_t)T0) / // divided by..
          // Calibration LSB delta + Calibration offset?
          (float)((int16_t)T1_OUT - (int16_t)T0_OUT) +
      (int16_t)T0;
}

/**
 * @brief Use the humidity calibration values to correct the raw value
 *
 */
void Adafruit_HTS221::_applyHumidityCorrection(void) {

  // Derived from
  // https://github.com/ameltech/sme-hts221-library/blob/2fe7528f4d42b4b36b39d9f6db76aae25ebe300b/src/Humidity/HTS221.cpp#L185

  uint8_t data = 0;
  uint16_t h_out = 0;
  float h_temp = 0.0;
  float hum = 0.0;

  // Decode Humidity
  hum = ((int16_t)(H1) - (int16_t)(H0)) / 2.0; // remove x2 multiple

  // Calculate humidity in decimal of grade centigrades i.e. 15.0 = 150.
  h_temp = (float)(((int16_t)raw_humidity - (int16_t)H0_T0_OUT) * hum) /
           (float)((int16_t)H1_T0_OUT - (int16_t)H0_T0_OUT);
  hum = (float)((int16_t)H0) / 2.0;    // remove x2 multiple
  corrected_humidity = (hum + h_temp); // provide signed % measurement unit
}

/**
 * @brief Gets the Adafruit_Sensor object for the HTS221's humidity sensor
 *
 * @return Adafruit_Sensor*
 */
Adafruit_Sensor *Adafruit_HTS221::getHumiditySensor(void) {
  return humidity_sensor;
}

/**
 * @brief Gets the Adafruit_Sensor object for the HTS221's humidity sensor
 *
 * @return Adafruit_Sensor*
 */
Adafruit_Sensor *Adafruit_HTS221::getTemperatureSensor(void) {
  return temp_sensor;
}
/**
 * @brief  Gets the sensor_t object describing the HTS221's humidity sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_HTS221_Humidity::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "HTS221_P", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->min_delay = 0;
  sensor->min_value = 0;  // MATH
  sensor->max_value = 1;  // MATH
  sensor->resolution = 0; // depends on calibration values?
}
/**
    @brief  Gets the humidity as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
 */
bool Adafruit_HTS221_Humidity::getEvent(sensors_event_t *event) {
  _theHTS221->_read();
  _theHTS221->fillHumidityEvent(event, millis());

  return true;
}
/**
 * @brief  Gets the sensor_t object describing the HTS221's tenperature sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_HTS221_Temp::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "HTS221_H T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40; // CHECK
  sensor->max_value = 100; // CHECK
  sensor->resolution = 0;  // depends on calibration data?
}
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns true
*/
bool Adafruit_HTS221_Temp::getEvent(sensors_event_t *event) {
  _theHTS221->_read();
  _theHTS221->fillTempEvent(event, millis());

  return true;
}