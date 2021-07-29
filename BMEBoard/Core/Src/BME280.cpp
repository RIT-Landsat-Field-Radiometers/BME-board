/*
 * BME280.cpp
 *
 *      Author: reedt
 */

#include "BME280.h"
#include <limits>
#include <cmath>

#define READ(x) (0x80 | x)
#define WRITE(x) (~0x80 & x)

BME280::BME280(SPI_HandleTypeDef *SPI, GPIO_TypeDef *csPort, uint16_t csPin) :
		hspi(SPI), csPort(csPort), csPin(csPin)
{
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

BME280::~BME280()
{
	// TODO Auto-generated destructor stub
}

bool BME280::init()
{
	_sensorID = read8(BME280_REGISTER_CHIPID);
	if (_sensorID != 0x60)
		return false;

//	write8(BME280_REGISTER_SOFTRESET, 0xB6);

	HAL_Delay(10);

	while (isReadingCalibration())
		HAL_Delay(10);

	readCoefficients(); // read trimming parameters, see DS 4.2.2

	setSampling(); // use defaults

	HAL_Delay(100);
	return true;
}

void BME280::setSampling(sensor_mode mode, sensor_sampling tempSampling,
		sensor_sampling pressSampling, sensor_sampling humSampling,
		sensor_filter filter, standby_duration duration)
{
	_measReg.mode = mode;
	_measReg.osrs_t = tempSampling;
	_measReg.osrs_p = pressSampling;

	_humReg.osrs_h = humSampling;
	_configReg.filter = filter;
	_configReg.t_sb = duration;
	_configReg.none = 0;
	_configReg.spi3w_en = 0;

	write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
	write8(BME280_REGISTER_CONFIG, _configReg.get());
	write8(BME280_REGISTER_CONTROL, _measReg.get());
}

bool BME280::takeForcedMeasurement(void)
{
	bool return_value = false;
	if (_measReg.mode == MODE_FORCED)
	{
		return_value = true;
		write8(BME280_REGISTER_CONTROL, _measReg.get());
		uint32_t timeout_start = HAL_GetTick();
		while (read8(BME280_REGISTER_STATUS) & 0x08)
		{
			if ((HAL_GetTick() - timeout_start) > 2000)
			{
				return_value = false;
				break;
			}
			HAL_Delay(1);
		}
	}
	return return_value;
}

double BME280::readTemperature(void)
{
	int32_t var1, var2;

	int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
	if (adc_T == 0x800000)
		return NAN;
	adc_T >>= 4;

	var1 = ((((adc_T >> 3) - ((int32_t) _bme280_calib.dig_T1 << 1)))
			* ((int32_t) _bme280_calib.dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t) _bme280_calib.dig_T1))
			* ((adc_T >> 4) - ((int32_t) _bme280_calib.dig_T1))) >> 12)
			* ((int32_t) _bme280_calib.dig_T3)) >> 14;

	t_fine = var1 + var2 + t_fine_adjust;

	double T = (t_fine * 5 + 128) >> 8;
	return T / 100;
}

double BME280::readPressure(void)
{
	int64_t var1, var2, p;

	readTemperature(); // must be done first to get t_fine

	int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
	if (adc_P == 0x800000) // value in case pressure measurement was disabled
		return NAN;
	adc_P >>= 4;

	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) _bme280_calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t) _bme280_calib.dig_P5) << 17);
	var2 = var2 + (((int64_t) _bme280_calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) _bme280_calib.dig_P3) >> 8)
			+ ((var1 * (int64_t) _bme280_calib.dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) _bme280_calib.dig_P1)
			>> 33;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) _bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) _bme280_calib.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t) _bme280_calib.dig_P7) << 4);
	double ptemp = (double) p / 256.0;
	return ptemp / 6894.7572931783;
}

double BME280::readHumidity(void)
{
	readTemperature(); // must be done first to get t_fine

	int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
	if (adc_H == 0x8000) // value in case humidity measurement was disabled
		return NAN;

	int32_t v_x1_u32r;

	v_x1_u32r = (t_fine - ((int32_t) 76800));

	v_x1_u32r = (((((adc_H << 14) - (((int32_t) _bme280_calib.dig_H4) << 20)
			- (((int32_t) _bme280_calib.dig_H5) * v_x1_u32r))
			+ ((int32_t) 16384)) >> 15)
			* (((((((v_x1_u32r * ((int32_t) _bme280_calib.dig_H6)) >> 10)
					* (((v_x1_u32r * ((int32_t) _bme280_calib.dig_H3)) >> 11)
							+ ((int32_t) 32768))) >> 10) + ((int32_t) 2097152))
					* ((int32_t) _bme280_calib.dig_H2) + 8192) >> 14));

	v_x1_u32r = (v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* ((int32_t) _bme280_calib.dig_H1)) >> 4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	double h = (v_x1_u32r >> 12);
	return h / 1024.0;
}

double BME280::readAltitude(double seaLevel)
{
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude. See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	double atmospheric = readPressure() / 100.0F;
	return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

double BME280::seaLevelForAltitude(double altitude, double atmospheric)
{
	// Equation taken from BMP180 datasheet (page 17):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude. See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

uint32_t BME280::sensorID(void)
{
	return _sensorID;
}

double BME280::getTemperatureCompensation(void)
{
	return double(((t_fine_adjust * 5) >> 8) / 100);
}

void BME280::setTemperatureCompensation(double adjustment)
{
	// convert the value in C into and adjustment to t_fine
	t_fine_adjust = ((int32_t(adjustment * 100) << 8)) / 5;
}

void BME280::readCoefficients(void)
{
	_bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
	_bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
	_bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

	_bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
	_bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
	_bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
	_bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
	_bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
	_bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
	_bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
	_bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
	_bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

	_bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
	_bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
	_bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
	_bme280_calib.dig_H4 = ((int8_t) read8(BME280_REGISTER_DIG_H4) << 4)
			| (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
	_bme280_calib.dig_H5 = ((int8_t) read8(BME280_REGISTER_DIG_H5 + 1) << 4)
			| (read8(BME280_REGISTER_DIG_H5) >> 4);
	_bme280_calib.dig_H6 = (int8_t) read8(BME280_REGISTER_DIG_H6);
}

bool BME280::isReadingCalibration(void)
{
	uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

	return (rStatus & (1 << 0)) != 0;
}

void BME280::write8(uint8_t reg, uint8_t value)
{
	txBuff[0] = WRITE(reg);
	txBuff[1] = value;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspi, txBuff, 2, 100);

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

uint8_t BME280::read8(uint8_t reg)
{
	txBuff[0] = READ(reg);
	txBuff[1] = 0x00;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);

//	HAL_SPI_Transmit(hspi, txBuff, 1, 100);
//	rxBuff[0] = 0x00;
//	HAL_SPI_Receive(hspi, rxBuff, 1, 100);
	HAL_SPI_TransmitReceive(hspi, txBuff, rxBuff, 2, -1);

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
	return rxBuff[1];
}

uint16_t BME280::read16(uint8_t reg)
{
	txBuff[0] = READ(reg);
	txBuff[1] = 0x00;
	txBuff[2] = 0x00;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);


//	HAL_SPI_Transmit(hspi, txBuff, 1, 100);
//	rxBuff[0] = 0x00;
//	rxBuff[1] = 0x00;
//	HAL_SPI_Receive(hspi, rxBuff, 2, 100);

	HAL_SPI_TransmitReceive(hspi, txBuff, rxBuff, 3, -1);

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
	return (rxBuff[1] << 8) | rxBuff[2];
}

uint32_t BME280::read24(uint8_t reg)
{
	txBuff[0] = READ(reg);
	txBuff[1] = 0x00;
	txBuff[2] = 0x00;
	txBuff[3] = 0x00;
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);


//	HAL_SPI_Transmit(hspi, txBuff, 1, 100);
//	rxBuff[0] = 0x00;
//	rxBuff[1] = 0x00;
//	rxBuff[2] = 0x00;
//	HAL_SPI_Receive(hspi, rxBuff, 3, 100);

	HAL_SPI_TransmitReceive(hspi, txBuff, rxBuff, 4, -1);

	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
	return (rxBuff[1] << 16) | (rxBuff[2] << 8) | rxBuff[3];
}

int16_t BME280::readS16(uint8_t reg)
{
	return (int16_t) read16(reg);
}

uint16_t BME280::read16_LE(uint8_t reg)
{
	uint16_t retval = read16(reg);
	return (retval >> 8) | (retval << 8);
}

int16_t BME280::readS16_LE(uint8_t reg)
{
	return (int16_t) read16_LE(reg);
}

