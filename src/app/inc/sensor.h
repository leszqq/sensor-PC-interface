/*
 * sensor.h
 *
 *  Created on: May 4, 2021
 *      Author: Administrator
 */
/* === includes === */
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

#ifndef APP_INC_SENSOR_H_
#define APP_INC_SENSOR_H_

/* === exported defines === */

/* === exported types === */
enum sensor_OutputType {
	SENSOR_OUT_ACC_DATA,
	SENSOR_OUT_CLICK_DETECTION,
	SENSOR_OUT_FALL_DETECTION
};

struct sensor_XyzData {
	int16_t x, y, z;
};
struct sensor_Output {
	enum sensor_OutputType type;
	union {
		struct sensor_XyzData xyzData;
	};
};
/**
 * Sensor accelerometer data read rate. Parameter for @ref sensor_setAccRate().
 * Assigned values are compliant with accelerometer CTRL1 register content.
 */
enum sensor_AccRate {
	SENSOR_ACC_RATE_3HZ125 = 0x1 << 4,
	SENSOR_ACC_RATE_6HZ25 = 0x2 << 4,
	SENSOR_ACC_RATE_12HZ5 = 0x3 << 4,
	SENSOR_ACC_RATE_25HZ = 0x4 << 4,
	SENSOR_ACC_RATE_50HZ = 0x5 << 4,
	SENSOR_ACC_RATE_100HZ = 0x6 << 4,
	SENSOR_ACC_RATE_200HZ = 0x7 << 4,
	SENSOR_ACC_RATE_400HZ = 0x8 << 4,
	SENSOR_ACC_RATE_800HZ = 0x9 << 4,
	SENSOR_ACC_RATE_1600HZ = 0xA << 4
};

/**
 * Sensor accelerometer anti-alias filter bandwith. Parameter for @ref sensor_setAccAAFilterBW().
 * Assigned values are compliant with accelerometer CTRL2 register content.
 */
enum sensor_AccAAFilterBW {
	SENSOR_ACC_AAFILT_BW_773HZ = 0x0 << 6,/**< SENSOR_ACC_AAFILT_BW_773HZ */
	SENSOR_ACC_AAFILT_BW_194HZ = 0x1 << 6,/**< SENSOR_ACC_AAFILT_BW_194HZ */
	SENSOR_ACC_AAFILT_BW_362HZ = 0x2 << 6,/**< SENSOR_ACC_AAFILT_BW_362HZ */
	SENSOR_ACC_AAFILT_BW_50HZ = 0x3 << 6  /**< SENSOR_ACC_AAFILT_BW_50HZ */

};

/**
 * Sensor accelerometer full scale. Parameter for @ref sensor_setAccFullScale().
 * Assigned values are compliant with accelerometer CTRL2 register content.
 */
enum sensor_AccFullScale {
	SENSOR_ACC_FULL_SCALE_2G = 0x0 << 3,
	SENSOR_ACC_FULL_SCALE_4G = 0x1 << 3,
	SENSOR_ACC_FULL_SCALE_6G = 0x2 << 3,
	SENSOR_ACC_FULL_SCALE_8G = 0x3 << 3,
	SENSOR_ACC_FULL_SCALE_16G = 0x4 << 3
};



/* === tasks === */
/**
 * @brief This task is responsible for reading data from sensor and pushing it to queue.
 * @param params unused
 */
void sensor_task(void * params);


/* === exported functions === */
/**
 * @brief Initialise sensor to work in default mode and create tasks.d
 * @param sensorOutputQueue uninitialised freeRTOS queue which will contain accelerometer output data.
 */
void sensor_init(QueueHandle_t sensorOutputQueue);

/**
 * @brief Start sensor operation.
 */
void sensor_start();

/**
 * @brief Stop sensor operation after one more data read or event detection.
 */
void sensor_stop();


// TODO: temp dla testów
void sensor_process();

/* accelerometer setters */
/**
 * @brief Set accelerometer range.
 * @param accelerometer range
 * @return void
 */
void sensor_setAccFullScale(enum sensor_AccFullScale fullScale);


/**
 * @brief Set accelerometer data read rate
 * @param rate data read rate
 * @return void
 */
void sensor_setAccRate(enum sensor_AccRate rate);


/**
 * @brief Set accelerometer anti alias filter bandwidth.
 * @param bandwith
 */
void sensor_setAccAAFiletrBW(enum sensor_AccAAFilterBW bandwidth);

/* accelerometer getters */
void sensor_getAcc(uint16_t data[3]); // TODO

/**
 * @brief Get accelerometer range.
 * @return accelerometer range
 */
enum sensor_AccFullScale sensor_getAccFullScale();

/**
 * @brief Get accelerometer data read rate.
 * @return accelerometer data read rate
 */
enum sensor_AccRate sensor_getAccRate();

/**
 * @brief Get numbers of samples averaged for accelerometer data readings.
 * @return number of samples used for averaging accelerometer data.
 */
uint16_t sensor_accGetNumOfAveragedSamples();

#endif /* APP_INC_SENSOR_H_ */
