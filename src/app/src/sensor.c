/*
 * sensor.c
 *
 *  Created on: May 4, 2021
 *      Author: Wiktor Lechowicz
 */
#include "sensor.h"
#include "stm32f3xx_hal.h"
#include "main.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "stdbool.h"

#define SENSOR_ADDR                     0x3A

/* registers and register bit patterns */

#define STATUS_M                    0x07
#define OUT_X_L_M                   0x08
#define OUT_X_H_M                   0x09
#define OUT_Y_L_M                   0x0A
#define OUT_Y_H_M                   0x0B
#define OUT_Z_L_M                   0x0C
#define OUT_Z_H_M                   0x0D
#define WHO_AM_I                    0x0F
#define INT_CTRL_M                  0x12
#define INT_SRC_M                   0x13
#define INT_THS_L_M                 0x14
#define INT_THS_H_M                 0x15
#define OFFSET_X_L_M                0x16
#define OFFSET_X_H_M                0x17
#define OFFSET_Y_L_M                0x18
#define OFFSET_Y_H_M                0x19
#define OFFSET_Z_L_M                0x1A
#define OFFSET_Z_H_M                0x1B
#define REFERENCE_X                 0x1C
#define REFERENCE_Y                 0x1D
#define REFERENCE_Z                 0x1E

#define CTRL0                       0x1F
#define     CTRL0_BOOT                  0x80
#define     CTRL0_FIFO_EN               0x40
#define     CTRL0_FTH_EN                0x20
#define     CTRL0_HP_CLICK              0x04
#define     CTRL0_HPIS1                 0x02
#define     CTRL0_HPIS2                 0x01

#define CTRL1                       0x20
#define     CTRL1_AODR3                 0x80
#define     CTRL1_AODR2                 0x40
#define     CTRL1_AODR1                 0x20
#define     CTRL1_AODR0                 0x10
#define     CTRL1_BDU                   0x08
#define     CTRL1_AZEN                  0x04
#define     CTRL1_AYEN                  0x02
#define     CTRL1_AXEN                  0x01
#define     CTRL1_ACC_RATE_25HZ         CTRL1_AODR2

#define CTRL2                       0x21
#define     CTRL2_ABW1                  0x80
#define     CTRL2_ABW0                  0x40
#define     CTRL2_AFS2                  0x20
#define     CTRL2_AFS1                  0x10
#define     CTRL2_AFS0                  0x08
#define     CTRL2_AST                   0x02
#define     CTRL2_SIM                   0x01
#define     CTRL2_ANTI_ALIAS_50HZ       0xC0
#define     CTRL2_FULL_SCALE_4G         0x08

#define CTRL3                       0x22
#define     CTRL3_INT1_BOOT             0x80
#define     CTRL3_INT1_CLICK            0x40
#define     CTRL3_INT1_IG1              0x20
#define     CTRL3_INT1_IG2              0x10
#define     CTRL3_INT1_IGM              0x08
#define     CTRL3_INT1_DRDY_A           0x04
#define     CTRL3_INT1_DRDY_M           0x02
#define     CTRL3_INT1_EMPTY            0x01

#define CTRL4                       0x23
#define     CTRL4_INT2_CLICK            0x80
#define     CTRL4_INT2_IG1              0x40
#define     CTRL4_INT2_IG2              0x20
#define     CTRL4_INT2_IGM              0x10
#define     CTRL4_INT2_DRDY_A           0x08
#define     CTRL4_INT2_DRDY_M           0x04
#define     CTRL4_INT2_OVR              0x02
#define     CTRL4_INT2_TH               0x01

#define CTRL5                       0x24
#define     CTRL5_TEMP_EN               0x80
#define     CTRL5_M_RES1                0x40
#define     CTRL5_M_RES0                0x20
#define     CTRL5_M_ODR2                0x10
#define     CTRL5_M_ODR1                0x08
#define     CTRL5_M_ODR0                0x04
#define     CTRL5_LIR2                  0x02
#define     CTRL5_LIR1                  0x01
#define     CTRL5_M_RES_HIGH            0x60
#define     CTRL5_M_RATE_25HZ           0x0A

#define CTRL6                       0x25
#define     CTRL6_M_FS_4GAUSSS          0x20

#define CTRL7                       0x26
#define CTRL7_M_CONT_CONV           0x00

#define STATUS_A                    0x27
#define     STATUS_A_ZYXADA             0x08    // acc x, y, z new data available

#define OUT_X_L_A                   0x28
#define OUT_X_H_A                   0x29
#define OUT_Y_L_A                   0x2A
#define OUT_Y_H_A                   0x2B
#define OUT_Z_L_A                   0x2C
#define OUT_Z_H_A                   0x2D

#define IG_CFG1						0x30
#define		IG_CFG1_EN_ALL				0x3F
#define IG_SRC1
#define IG_THS1				0x32
#define IG_DUR1						0x33
#define IG_CFG2						0x34
#define IG_CFG2_EN_ALL					0x3F

#define IG_SRC2						0x35
#define IG_THS2						0x36
#define	IG_DUR2						0x37

#define CLICK_CFG					0x38
#define 	CLICK_CFG_ENABLE_DBL_CLICK	0x2A
#define 	CLICK_CFG_ENABLE_SGL_CLICK	0x15

#define CLICK_SRC					0x39
#define 	CLICK_SRC_DBL_CLICK_ENABLE	0x20
#define 	CLICK_SRC_SGL_CLICK_ENABLE	0x10
#define		CLICK_SRC_Z					0x04

#define CLICK_THS					0x3A

#define TIME_LIMIT					0x3B

#define TIME_LATENCY				0x3C
#define TIME_WINDOW					0x3D
#define ACT_THS						0x3E
#define ACT_DUR						0x3F

#define ACC_XYZ_DATA_SIZE           6
#define CLICK_THS_VAL				0x02
#define TIME_LIMIT_VAL				0x2F

#define AUTO_ADDR_INC                   0x80

#define INT1_GPIO_PORT					GPIOC
#define INT1_GPIO_PIN					GPIO_PIN_0
#define INT2_GPIO_PORT					GPIOC
#define INT2_GPIO_PIN					GPIO_PIN_1

#define EVT_NOTIFICATION_QUEUE_LEN		3
#define AUX_TAB_LEN						10
#define MAX_INT16_VAL					32767

/* === private functions === */
/* Init gpio EXTI lines for sensor INT1 and INT2 lines */
inline void initExtiLines() {
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* init GPIOs */
	GPIO_InitTypeDef gpioInit = { 0 };
	gpioInit.Pin = INT1_GPIO_PIN;
	gpioInit.Mode = GPIO_MODE_IT_RISING;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT1_GPIO_PORT, &gpioInit);
	gpioInit.Pin = INT2_GPIO_PIN;
	gpioInit.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(INT2_GPIO_PORT, &gpioInit);

	/* init EXTI */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 10);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 8, 8);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

static void writeSensorRegisters(uint8_t startingReg, uint8_t *data,
		uint8_t numOfRegisters) {
	I2C_writeByteStream(SENSOR_ADDR, startingReg | AUTO_ADDR_INC, data,
			numOfRegisters);
}

static void writeSensorRegister(uint8_t reg, uint8_t data) {
	I2C_writeByteStream(SENSOR_ADDR, reg, &data, 1);
}

static void readSensorRegister(uint8_t reg, uint8_t *data) {
	I2C_readByteStream(SENSOR_ADDR, reg, data, 1);
}

static void readSensorRegisters(uint8_t startingReg, uint8_t *data,
		uint8_t numOfRegisters) {
	I2C_readByteStream(SENSOR_ADDR, startingReg | AUTO_ADDR_INC, data,
			numOfRegisters);
}

/* === private types === */

enum State {
	STATE_IDLE, STATE_ACTIVE
};

enum EventNotification {
	NEW_DATA, NEW_DETECTION
};

struct Acc {
	enum sensor_AccFullScale fullScale;
	enum sensor_AccRate rate;
	enum sensor_AccAAFilterBW AAFilterBW;
};

/* === private variables === */
static struct Base {
	struct Acc acc;						// accelerometer setup
	enum State state;						// sensor state
	SemaphoreHandle_t goActiveSemph;// semaphore given by sensor_start() to make
									// sensor_task start reading data from sensor.
	QueueHandle_t sensorOutputQueue,		// public queue with sensor output.
			evtQueue;	// private queue for handling sensor evt notifications.
						// Queue contain objects of type enum EventNotification
	uint8_t auxTab[AUX_TAB_LEN];
} base;

/* === exported functions === */
void sensor_init(QueueHandle_t sensorOutputQueue) {
	/* init base struct */
	base.state = STATE_IDLE;
	base.sensorOutputQueue = sensorOutputQueue;

	/* init RTOS objects */
	base.evtQueue = xQueueCreate(EVT_NOTIFICATION_QUEUE_LEN,
			sizeof(enum EventNotification));
	CHECK(base.evtQueue);

	base.goActiveSemph = xSemaphoreCreateBinary();

	/* Initialise GPIOs and EXIT for sensor INT1 and INT2 lines */
	initExtiLines();

	/* init I2C */
	I2C_init();

	/* reboot sensor */
	base.auxTab[0] = CTRL0_BOOT;
	writeSensorRegister(CTRL0, base.auxTab[0]);
	for (int i = 0; i < 8000; i++) {
	};

	/* enable high pass filters for click detection and interrupt generators */
	base.auxTab[0] = CTRL0_HP_CLICK;			// | CTRL0_HPIS1 | CTRL0_HPIS2;
	writeSensorRegister(CTRL0, base.auxTab[0]);

	/* enable int1 generation on new data available and int2 generation on click*/
	base.auxTab[0] = CTRL3_INT1_DRDY_A;					// | CTRL3_INT1_DRDY_M;
	base.auxTab[1] = CTRL4_INT2_CLICK | CTRL4_INT2_IG1 | CTRL4_INT2_IG2;
	writeSensorRegisters(CTRL3, base.auxTab, 2);

	/* initial user setups */
	sensor_setAccRate(SENSOR_ACC_RATE_400HZ);
	sensor_setAccAAFiletrBW(SENSOR_ACC_AAFILT_BW_773HZ);
	sensor_setAccFullScale(SENSOR_ACC_FULL_SCALE_2G);

	/* click detection setup */
	writeSensorRegister(IG_CFG2, 0x10);
	writeSensorRegister(IG_THS2, 0x0A);
	writeSensorRegister(IG_DUR2, 0x02);
	writeSensorRegister(CLICK_CFG, 0x10);
	writeSensorRegister(CLICK_SRC, 0x30);
	writeSensorRegister(CLICK_THS, 0x03);
	writeSensorRegister(TIME_LIMIT, 0x0B);
	writeSensorRegister(TIME_LATENCY, 0x0A);
	writeSensorRegister(TIME_WINDOW, 0x0A);
	writeSensorRegister(ACT_THS, 0x00);
	writeSensorRegister(ACT_DUR, 0xF0);
}

void sensor_start() {
	xSemaphoreGive(base.goActiveSemph);
}

void sensor_stop() {
	base.state = STATE_IDLE;
}

void sensor_setAccFullScale(enum sensor_AccFullScale fullScale) {
	base.acc.fullScale = fullScale;
	writeSensorRegister(CTRL2, fullScale | base.acc.AAFilterBW);
}

void sensor_setAccRate(enum sensor_AccRate rate) {
	base.acc.rate = rate;
	writeSensorRegister(CTRL1, rate | CTRL1_AZEN | CTRL1_AYEN | CTRL1_AXEN);// all axis data read enabled by default.
}

void sensor_setAccAAFiletrBW(enum sensor_AccAAFilterBW bandwidth) {
	base.acc.AAFilterBW = bandwidth;
	writeSensorRegister(CTRL2, bandwidth | base.acc.fullScale);
}

enum sensor_AccFullScale sensor_getAccFullScale() {
	return base.acc.fullScale;
}

uint8_t sensor_getAccFullScaleInt() {
	switch (base.acc.fullScale) {
	case SENSOR_ACC_FULL_SCALE_2G:
		return 2;
		break;
	case SENSOR_ACC_FULL_SCALE_4G:
		return 4;
		break;
	case SENSOR_ACC_FULL_SCALE_6G:
		return 6;
		break;
	case SENSOR_ACC_FULL_SCALE_8G:
		return 8;
		break;
	case SENSOR_ACC_FULL_SCALE_16G:
	default:
		return 16;
		break;
	}
}

enum sensor_AccRate sensor_getAccRate() {
	return base.acc.rate;
}

uint32_t sensor_getAccRateInt() {
	switch (base.acc.rate) {
	case SENSOR_ACC_RATE_3HZ125:
		return 3125;
		break;
	case SENSOR_ACC_RATE_6HZ25:
		return 6250;
		break;
	case SENSOR_ACC_RATE_12HZ5:
		return 12500;
		break;
	case SENSOR_ACC_RATE_25HZ:
		return 25000;
		break;
	case SENSOR_ACC_RATE_50HZ:
		return 50000;
		break;
	case SENSOR_ACC_RATE_100HZ:
		return 100000;
		break;
	case SENSOR_ACC_RATE_200HZ:
		return 200000;
		break;
	case SENSOR_ACC_RATE_400HZ:
		return 400000;
		break;
	case SENSOR_ACC_RATE_800HZ:
		return 800000;
		break;
	case SENSOR_ACC_RATE_1600HZ:
	default:
		return 1600000;
		break;
	}
}

void sensor_task(void *params) {
	UNUSED(params);

	enum EventNotification evtNotification;
	struct sensor_Output output;
	while (1) {
		switch (base.state) {
		case STATE_IDLE:
			/* block until active state requested by sensor_start() function. */
			xSemaphoreTake(base.goActiveSemph, portMAX_DELAY);
			/* make initial data read to unblock interrupts */
			readSensorRegisters(OUT_X_L_A, base.auxTab, 6);

			base.state = STATE_ACTIVE;
			break;
		case STATE_ACTIVE:
			/* Block in waiting for new data or event detection interrupt*/
			xQueueReceive(base.evtQueue, &evtNotification, portMAX_DELAY);
			/* Check if new data is available or event occured */
			switch (evtNotification) {
			case NEW_DATA:
				/* specify new data type, read it and put into sensor output queue */
				readSensorRegister(STATUS_A, base.auxTab);

				/* if accelerometer data ready */
				if (base.auxTab[0] && STATUS_A_ZYXADA) {

					output.type = SENSOR_OUT_ACC_DATA;
					/* read and decode accelerometer data */
					readSensorRegisters(OUT_X_L_A, base.auxTab, 6);
					output.xyzData.x =
							((base.auxTab[0] | (base.auxTab[1] << 8)));
					output.xyzData.x = ((double) (output.xyzData.x) / INT16_MAX)
							* 1000.0 * sensor_getAccFullScaleInt();
					output.xyzData.y =
							((base.auxTab[2] | (base.auxTab[3] << 8)));
					output.xyzData.y = ((double) (output.xyzData.y) / INT16_MAX)
							* 1000.0 * sensor_getAccFullScaleInt();
					output.xyzData.z =
							((base.auxTab[4] | (base.auxTab[5] << 8)));
					output.xyzData.z = ((double) (output.xyzData.z) / INT16_MAX)
							* 1000.0 * sensor_getAccFullScaleInt();

					xQueueSendToBack(base.sensorOutputQueue, &output,
							portMAX_DELAY);
				}
				/* Add magnetometer and temperature read here in future */
				break;
			case NEW_DETECTION:
				/* specify new detection type and put it into sensor output queue*/
				readSensorRegister(CLICK_SRC, base.auxTab);
				if (base.auxTab[0] & CLICK_SRC_Z) {
					output.type = SENSOR_OUT_CLICK_DETECTION;
					xQueueSendToBack(base.sensorOutputQueue, &output,
							portMAX_DELAY);
				}
				break;
			}
			break;
		}
	}
}

/* Interrupts on sensor data ready or event detection signals */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	BaseType_t higherPriorityTaskWoken = pdFALSE;
	enum EventNotification notificationToSend;
	if (GPIO_Pin == INT1_GPIO_PIN) {
		notificationToSend = NEW_DATA;
	} else if (GPIO_Pin == INT2_GPIO_PIN) {
		notificationToSend = NEW_DETECTION;
	}
	xQueueSendFromISR(base.evtQueue, &notificationToSend,
			&higherPriorityTaskWoken);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void EXTI0_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(INT1_GPIO_PIN);
}

void EXTI1_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
