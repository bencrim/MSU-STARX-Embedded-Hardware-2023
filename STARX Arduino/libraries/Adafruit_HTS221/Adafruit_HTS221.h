/*!
 *  @file Adafruit_HTS221.h
 *
 * 	I2C Driver for the Adafruit HTS221 Humidity and Temperature Sensor
 *library
 *
 * 	This is a library for the Adafruit HTS221 breakout:
 * 	https://www.adafruit.com/products/453X
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_HTS221_H
#define _ADAFRUIT_HTS221_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define HTS221_I2CADDR_DEFAULT 0x5F ///< HTS221 default i2c address
#define HTS221_CHIP_ID 0xBC         ///< HTS221 default device id from WHOAMI
#define HTS221_CTRL_REG_1 0x20      ///< First control regsiter; PD, OBDU, ODR
#define HTS221_CTRL_REG_2                                                      \
  0x21 ///< Second control regsiter; BOOT, Heater, ONE_SHOT
#define HTS221_CTRL_REG_3 0x22   ///< Third control regsiter; DRDY_H_L, DRDY
#define HTS221_HUMIDITY_OUT 0x28 ///< Humidity output register (LSByte)
#define HTS221_TEMP_OUT_L 0x2A   ///< Temperature output register (LSByte)
#define HTS221_H0_RH_X2 0x30     ///< Humididy calibration LSB values
#define HTS221_H1_RH_X2 0x31     ///< Humididy calibration LSB values
#define HTS221_T0_DEGC_X8 0x32   ///< First byte of T0, T1 calibration values
#define HTS221_T1_T0_MSB 0x35    ///< Top 2 bits of T0 and T1 (each are 10 bits)
#define HTS221_H0_T0 0x36        ///< Humididy calibration Time 0 value
#define HTS221_H0_T1 0x3A        ///< Humididy calibration Time 1 value
#define HTS221_T0_OUT 0x3C       ///< T0_OUT LSByte
#define HTS221_T1_OUT 0x3E       ///< T1_OUT LSByte

#define HTS221_WHOAMI 0x0F ///< Chip ID register
/**
 * @brief
 *
 * Allowed values for `setDataRate`.
 */
typedef enum {
  HTS221_RATE_ONE_SHOT,
  HTS221_RATE_1_HZ,
  HTS221_RATE_7_HZ,
  HTS221_RATE_12_5_HZ,
} hts221_rate_t;

class Adafruit_HTS221;

/**
 * @brief  Adafruit Unified Sensor interface for the humidity sensor component
 * of HTS221
 *
 */
class Adafruit_HTS221_Humidity : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the humidity sensor
    @param parent A pointer to the HTS221 class */
  Adafruit_HTS221_Humidity(Adafruit_HTS221 *parent) { _theHTS221 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x221;
  Adafruit_HTS221 *_theHTS221 = NULL;
};

/**
 * @brief Adafruit Unified Sensor interface for the temperature sensor component
 * of HTS221
 *
 */
class Adafruit_HTS221_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the HTS221 class */
  Adafruit_HTS221_Temp(Adafruit_HTS221 *parent) { _theHTS221 = parent; }

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x222;
  Adafruit_HTS221 *_theHTS221 = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the HTS221 I2C Digital Potentiometer
 */
class Adafruit_HTS221 {
public:
  Adafruit_HTS221();
  ~Adafruit_HTS221();

  bool begin_I2C(uint8_t i2c_addr = HTS221_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);

  void boot(void);

  void setActive(bool active);
  hts221_rate_t getDataRate(void);
  void setDataRate(hts221_rate_t data_rate);

  void drdyActiveLow(bool active_low);
  void drdyIntEnabled(bool drdy_int_enabled);

  bool getEvent(sensors_event_t *humidity, sensors_event_t *temp);
  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getHumiditySensor(void);

protected:
  bool _read(void);
  virtual bool _init(int32_t sensor_id);

  float corrected_temp,   ///< Last reading's temperature (C) before scaling
      corrected_humidity; ///< Last reading's humidity (percent) before scaling

  uint16_t _sensorid_humidity; ///< ID number for humidity
  uint16_t _sensorid_temp;     ///< ID number for temperature

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to I2C bus interface

  Adafruit_HTS221_Temp *temp_sensor = NULL; ///< Temp sensor data object
  Adafruit_HTS221_Humidity *humidity_sensor =
      NULL; ///< Humidity sensor data object

private:
  void _fetchTempCalibrationValues(void);
  void _fetchHumidityCalibrationValues(void);
  friend class Adafruit_HTS221_Temp;     ///< Gives access to private members to
                                         ///< Temp data object
  friend class Adafruit_HTS221_Humidity; ///< Gives access to private members to
                                         ///< Humidity data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillHumidityEvent(sensors_event_t *humidity, uint32_t timestamp);

  void _applyTemperatureCorrection(void);
  void _applyHumidityCorrection(void);
  uint16_t T0, T1, T0_OUT, T1_OUT; ///< Temperature calibration values
  uint8_t H0, H1;                  ///< Humidity calibration values
  uint16_t H0_T0_OUT, H1_T0_OUT;   ///< Humidity calibration values
  uint16_t raw_temperature; ///< The raw unscaled, uncorrected temperature value
  uint16_t raw_humidity;    ///< The raw unscaled, uncorrected humidity value

  uint8_t multi_byte_address_mask = 0x80; // default to I2C
};

#endif
