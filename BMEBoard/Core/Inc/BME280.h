/*
 * BME280.h
 *
 *      Author: reedt
 *
 *      Derived from: https://github.com/adafruit/Adafruit_BME280_Library
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include <cstdint>
#include "stm32l4xx_hal.h"


class BME280
{
private:
	typedef struct {
	  uint16_t dig_T1; ///< temperature compensation value
	  int16_t dig_T2;  ///< temperature compensation value
	  int16_t dig_T3;  ///< temperature compensation value

	  uint16_t dig_P1; ///< pressure compensation value
	  int16_t dig_P2;  ///< pressure compensation value
	  int16_t dig_P3;  ///< pressure compensation value
	  int16_t dig_P4;  ///< pressure compensation value
	  int16_t dig_P5;  ///< pressure compensation value
	  int16_t dig_P6;  ///< pressure compensation value
	  int16_t dig_P7;  ///< pressure compensation value
	  int16_t dig_P8;  ///< pressure compensation value
	  int16_t dig_P9;  ///< pressure compensation value

	  uint8_t dig_H1; ///< humidity compensation value
	  int16_t dig_H2; ///< humidity compensation value
	  uint8_t dig_H3; ///< humidity compensation value
	  int16_t dig_H4; ///< humidity compensation value
	  int16_t dig_H5; ///< humidity compensation value
	  int8_t dig_H6;  ///< humidity compensation value
	} bme280_calib_data;

	enum REGISTERS : uint8_t {
	  BME280_REGISTER_DIG_T1 = 0x88,
	  BME280_REGISTER_DIG_T2 = 0x8A,
	  BME280_REGISTER_DIG_T3 = 0x8C,

	  BME280_REGISTER_DIG_P1 = 0x8E,
	  BME280_REGISTER_DIG_P2 = 0x90,
	  BME280_REGISTER_DIG_P3 = 0x92,
	  BME280_REGISTER_DIG_P4 = 0x94,
	  BME280_REGISTER_DIG_P5 = 0x96,
	  BME280_REGISTER_DIG_P6 = 0x98,
	  BME280_REGISTER_DIG_P7 = 0x9A,
	  BME280_REGISTER_DIG_P8 = 0x9C,
	  BME280_REGISTER_DIG_P9 = 0x9E,

	  BME280_REGISTER_DIG_H1 = 0xA1,
	  BME280_REGISTER_DIG_H2 = 0xE1,
	  BME280_REGISTER_DIG_H3 = 0xE3,
	  BME280_REGISTER_DIG_H4 = 0xE4,
	  BME280_REGISTER_DIG_H5 = 0xE5,
	  BME280_REGISTER_DIG_H6 = 0xE7,

	  BME280_REGISTER_CHIPID = 0xD0,
	  BME280_REGISTER_VERSION = 0xD1,
	  BME280_REGISTER_SOFTRESET = 0xE0,

	  BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

	  BME280_REGISTER_CONTROLHUMID = 0xF2,
	  BME280_REGISTER_STATUS = 0XF3,
	  BME280_REGISTER_CONTROL = 0xF4,
	  BME280_REGISTER_CONFIG = 0xF5,
	  BME280_REGISTER_PRESSUREDATA = 0xF7,
	  BME280_REGISTER_TEMPDATA = 0xFA,
	  BME280_REGISTER_HUMIDDATA = 0xFD
	};

	enum sensor_sampling {
	SAMPLING_NONE = 0b000,
	SAMPLING_X1 = 0b001,
	SAMPLING_X2 = 0b010,
	SAMPLING_X4 = 0b011,
	SAMPLING_X8 = 0b100,
	SAMPLING_X16 = 0b101
	};

	enum sensor_mode {
	MODE_SLEEP = 0b00,
	MODE_FORCED = 0b01,
	MODE_NORMAL = 0b11
	};

	enum sensor_filter {
	FILTER_OFF = 0b000,
	FILTER_X2 = 0b001,
	FILTER_X4 = 0b010,
	FILTER_X8 = 0b011,
	FILTER_X16 = 0b100
	};

	enum standby_duration {
	    STANDBY_MS_0_5 = 0b000,
	    STANDBY_MS_10 = 0b110,
	    STANDBY_MS_20 = 0b111,
	    STANDBY_MS_62_5 = 0b001,
	    STANDBY_MS_125 = 0b010,
	    STANDBY_MS_250 = 0b011,
	    STANDBY_MS_500 = 0b100,
	    STANDBY_MS_1000 = 0b101
	  };

	SPI_HandleTypeDef * hspi;
	GPIO_TypeDef * csPort;
	uint16_t csPin;


public:
	BME280(SPI_HandleTypeDef * SPI, GPIO_TypeDef * csPort, uint16_t csPin);
	virtual ~BME280();
	BME280(const BME280 &other) = default;
	BME280(BME280 &&other) = default;
	BME280& operator=(const BME280 &other) = default;
	BME280& operator=(BME280 &&other) = default;

	/**
	 *   @brief  Initialize sensor with given parameters / settings
	 *   @returns true on success, false otherwise
	 */
	bool init();

	/**
	 *   @brief  setup sensor with given parameters / settings
	 *
	 *   This is simply a overload to the normal begin()-function, so SPI users
	 *   don't get confused about the library requiring an address.
	 *   @param mode the power mode to use for the sensor
	 *   @param tempSampling the temp samping rate to use
	 *   @param pressSampling the pressure sampling rate to use
	 *   @param humSampling the humidity sampling rate to use
	 *   @param filter the filter mode to use
	 *   @param duration the standby duration to use
	 */
	void setSampling(sensor_mode mode = MODE_NORMAL,
	                   sensor_sampling tempSampling = SAMPLING_X16,
	                   sensor_sampling pressSampling = SAMPLING_X16,
	                   sensor_sampling humSampling = SAMPLING_X16,
	                   sensor_filter filter = FILTER_X16,
	                   standby_duration duration = STANDBY_MS_1000);

	/*!
	 *  @brief  Take a new measurement (only possible in forced mode)
	    @returns true in case of success else false
	 */
	bool takeForcedMeasurement(void);

	/*!
	 *   @brief  Returns the temperature from the sensor
	 *   @returns the temperature read from the device
	 */
	double readTemperature(void);

	/*!
	 *   @brief  Returns the pressure from the sensor
	 *   @returns the pressure value (in Pascal) read from the device
	 */
	double readPressure(void);

	/*!
	 *  @brief  Returns the humidity from the sensor
	 *  @returns the humidity value read from the device
	 */
	double readHumidity(void);

	/*!
	 *   Calculates the altitude (in meters) from the specified atmospheric
	 *   pressure (in hPa), and sea-level pressure (in hPa).
	 *   @param  seaLevel      Sea-level pressure in hPa
	 *   @returns the altitude value read from the device
	 */
	double readAltitude(double seaLevel);

	/*!
	 *   Calculates the pressure at sea level (in hPa) from the specified
	 * altitude (in meters), and atmospheric pressure (in hPa).
	 *   @param  altitude      Altitude in meters
	 *   @param  atmospheric   Atmospheric pressure in hPa
	 *   @returns the pressure at sea level (in hPa) from the specified altitude
	 */
	double seaLevelForAltitude(double altitude, double pressure);

	/*!
	 *   Returns Sensor ID found by init() for diagnostics
	 *   @returns Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
	 */
	uint32_t sensorID(void);

	/*!
	 *   Returns the current temperature compensation value in degrees Celcius
	 *   @returns the current temperature compensation value in degrees Celcius
	 */
	double getTemperatureCompensation(void);

	/*!
	 *  Sets a value to be added to each temperature reading. This adjusted
	 *  temperature is used in pressure and humidity readings.
	 *  @param  adjustment  Value to be added to each tempature reading in Celcius
	 */
	void setTemperatureCompensation(double);

protected:

	void write8(uint8_t reg, uint8_t value);
	uint8_t read8(uint8_t reg);
	uint16_t read16(uint8_t reg);
	uint32_t read24(uint8_t reg);
	int16_t readS16(uint8_t reg);
	uint16_t read16_LE(uint8_t reg); // little endian
	int16_t readS16_LE(uint8_t reg); // little endian

	/*!
	 *   @brief  Reads the factory-set coefficients
	 */
	void readCoefficients(void);

	/*!
	 *   @brief return true if chip is busy reading cal data
	 *   @returns true if reading calibration, false otherwise
	 */
	bool isReadingCalibration(void);

	uint8_t txBuff[16];
	uint8_t rxBuff[16];

	int32_t _sensorID; 	//!< ID of the BME Sensor
	int32_t t_fine; 	//!< temperature with high resolution, stored as an attribute
	                  	//!< as this is used for temperature compensation reading
	                  	//!< humidity and pressure

	int32_t t_fine_adjust = 0; 	 //!< add to compensate temp readings and in turn
	                             //!< to pressure and humidity readings

	bme280_calib_data _bme280_calib; //!< here calibration data is stored

	struct config {
	    // inactive duration (standby time) in normal mode
	    // 000 = 0.5 ms
	    // 001 = 62.5 ms
	    // 010 = 125 ms
	    // 011 = 250 ms
	    // 100 = 500 ms
	    // 101 = 1000 ms
	    // 110 = 10 ms
	    // 111 = 20 ms
	    unsigned int t_sb : 3; ///< inactive duration (standby time) in normal mode

	    // filter settings
	    // 000 = filter off
	    // 001 = 2x filter
	    // 010 = 4x filter
	    // 011 = 8x filter
	    // 100 and above = 16x filter
	    unsigned int filter : 3; ///< filter settings

	    // unused - don't set
	    unsigned int none : 1;     ///< unused - don't set
	    unsigned int spi3w_en : 1; ///< unused - don't set

	    /// @return combined config register
	    unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
	  };
	  config _configReg; //!< config register object

	  struct ctrl_meas {
	    // temperature oversampling
	    // 000 = skipped
	    // 001 = x1
	    // 010 = x2
	    // 011 = x4
	    // 100 = x8
	    // 101 and above = x16
	    unsigned int osrs_t : 3; ///< temperature oversampling

	    // pressure oversampling
	    // 000 = skipped
	    // 001 = x1
	    // 010 = x2
	    // 011 = x4
	    // 100 = x8
	    // 101 and above = x16
	    unsigned int osrs_p : 3; ///< pressure oversampling

	    // device mode
	    // 00       = sleep
	    // 01 or 10 = forced
	    // 11       = normal
	    unsigned int mode : 2; ///< device mode

	    /// @return combined ctrl register
	    unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
	  };
	  ctrl_meas _measReg; //!< measurement register object


	  struct ctrl_hum {
	    /// unused - don't set
	    unsigned int none : 5;

	    // pressure oversampling
	    // 000 = skipped
	    // 001 = x1
	    // 010 = x2
	    // 011 = x4
	    // 100 = x8
	    // 101 and above = x16
	    unsigned int osrs_h : 3; ///< pressure oversampling

	    /// @return combined ctrl hum register
	    unsigned int get() { return (osrs_h); }
	  };
	  ctrl_hum _humReg; //!< hum register object
};

#endif /* INC_BME280_H_ */
