#include <cli.h>
#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>
#include "rtc.h"
#include "i2c.h"
#include "sensor.h"
#include "semphr.h"
#include "stdbool.h"

#define CLI_RX_QUEUE_LEN				10
#define CLI_TX_QUEUE_LEN				10
#define SENSOR_OUT_QUEUE_LEN			4

#define DBG_LED_HALF_PERIOD		500
#define DBG_LED_PORT			GPIOB
#define DBG_LED_PIN				GPIO_PIN_13



void CLK_init(void);

/* === private macros === */
#define PRINT_TO_CLI(S, ...)  do { \
        						snprintf((char*)base.auxTab, CLI_MAX_LINE_LEN, S, ##__VA_ARGS__); \
								xQueueSendToBack(base.cliTxQueue, base.auxTab, portMAX_DELAY); \
							  }while(0)

#define CLEAR_CLI()				PRINT_TO_CLI("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\r>>")

#define FORMAT_ACC_DATA(data)	data >= 0 ? " %d.%.3d g" : "-%d.%.3.d g"
/* === private types === */
enum SystemState {
	SYSTEM_IDLE,					// NO-OP, system can be configured
	SYSTEM_ACC_DATA_PROCESSING 		// Accelerometer data reading and processing
};

/* === private variables === */


static struct Base {
	QueueHandle_t 				cliTxQueue,
								cliRxQueue,
								sensorOutputQueue;
	UART_HandleTypeDef 			huart2;
	enum SystemState			state;
	uint8_t 					auxTab[CLI_MAX_LINE_LEN];
	uint16_t					accNumOfAvgSamples;
	bool						accClickDetecionEnabled,
								accFallDetectionEnabled;
	struct sensor_XyzData		accData;				// TODO: make buffer of accData
} base;

#define SEND_NOT_RECOGNISED() 	do{ \
									xQueueSendToBack(CLI_TransmitQueue, "Command not recognised. Type \"help\" for help.\n\r>>", portMAX_DELAY); \
						  	  	}while(0)


/* === private functions === */
static void printAccSetup(){
	/* print full scale */

	uint32_t tempInt = 0;
	switch(sensor_getAccFullScale()){
	case SENSOR_ACC_FULL_SCALE_2G:
		tempInt = 2;
		break;
	case SENSOR_ACC_FULL_SCALE_4G:
		tempInt = 4;
		break;
	case SENSOR_ACC_FULL_SCALE_6G:
		tempInt = 6;
		break;
	case SENSOR_ACC_FULL_SCALE_8G:
		tempInt = 8;
		break;
	case SENSOR_ACC_FULL_SCALE_16G:
		tempInt = 16;
		break;
	}
	PRINT_TO_CLI("full scale range +/- %lu g\n\r", tempInt);

	/* print data read rate */

	switch(sensor_getAccRate()){
	case SENSOR_ACC_RATE_3HZ125:
		tempInt = 3125;
		break;
	case SENSOR_ACC_RATE_6HZ25:
		tempInt = 6250;
		break;
	case SENSOR_ACC_RATE_12HZ5:
		tempInt = 12500;
		break;
	case SENSOR_ACC_RATE_25HZ:
		tempInt = 25000;
		break;
	case SENSOR_ACC_RATE_50HZ:
		tempInt = 50000;
		break;
	case SENSOR_ACC_RATE_100HZ:
		tempInt = 100000;
		break;
	case SENSOR_ACC_RATE_200HZ:
		tempInt = 200000;
		break;
	case SENSOR_ACC_RATE_400HZ:
		tempInt = 400000;
		break;
	case SENSOR_ACC_RATE_800HZ:
		tempInt = 800000;
		break;
	case SENSOR_ACC_RATE_1600HZ:
		tempInt = 1600000;
		break;
	}

	/* print data read rate */
	PRINT_TO_CLI("data read rate: %lu.%lu Hz\n\r",  tempInt / 1000, tempInt % 1000);

	/* print number of averaged samples */
	PRINT_TO_CLI("number of averaged samples: %d\n\r", base.accNumOfAvgSamples);

	/* print number state of fall and click detection */
	char tempStr[5];
	if(base.accFallDetectionEnabled){
		snprintf((char*) tempStr, CLI_MAX_LINE_LEN, "ON");
	} else {
		snprintf((char*) tempStr, CLI_MAX_LINE_LEN, "OFF");
	}
	PRINT_TO_CLI("fall detection %s\n\r", tempStr);

	if(base.accClickDetecionEnabled){
		snprintf((char*) tempStr, CLI_MAX_LINE_LEN, "ON");
	} else {
		snprintf((char*) tempStr, CLI_MAX_LINE_LEN, "OFF");
	}
	PRINT_TO_CLI("click detection %s\n\r", tempStr);
}


static void printHelp(){
	PRINT_TO_CLI("List of available commands:\n\racc get setup\n\r");
	PRINT_TO_CLI("acc set range [2g|4g|6g|8g|16g]\n\racc set rate ");
	PRINT_TO_CLI("[3.125Hz|6.25Hz|12.5Hz|25Hz|50Hz|100Hz|200Hz|400");
	PRINT_TO_CLI("Hz|800Hz|1600Hz\n\racc set avg number [1-1000]");
	PRINT_TO_CLI("\n\racc set click det [on|off]\n\r");
	PRINT_TO_CLI("acc set fall det [on|off]start\n\r");
}


#define ANY_CLI_ACTIVITY_DETECTED (pdTRUE == xQueueReceive(base.cliRxQueue, base.auxTab, 0))

void main_task(void * params){
	UNUSED(params);

	/* main system loop */
	while(1){
		switch(base.state){
		case SYSTEM_IDLE:
			/* Block task until new command available */
			xQueueReceive(base.cliRxQueue, base.auxTab, portMAX_DELAY);
			/* Execute command TODO */
			if(0 == strncmp((char *) base.auxTab, "help", CLI_MAX_LINE_LEN)){
				printHelp();
			} else if(0 == strncmp((char *) base.auxTab, "acc get setup", CLI_MAX_LINE_LEN)){
				printAccSetup();
			} else if(0 == strncmp((char*) base.auxTab, "start", CLI_MAX_LINE_LEN)){
				PRINT_TO_CLI("acc x:    acc y:    acc z:");
				sensor_start();
				base.state = SYSTEM_ACC_DATA_PROCESSING;
			}
			// TODO reszta komand
			break;
		case SYSTEM_ACC_DATA_PROCESSING:
			if(ANY_CLI_ACTIVITY_DETECTED){
				/* in case of anything received on CLI, go to IDLE state */
				base.state = SYSTEM_IDLE;
				CLEAR_CLI();

			} else{
				struct sensor_Output sensOut = {0};
				/* Block in waiting for next data or event from accelerometer */
				if(pdTRUE == xQueueReceive(base.sensorOutputQueue, &sensOut, portMAX_DELAY)){
					switch(sensOut.type){
					case SENSOR_OUT_ACC_DATA:
						/* temp TODO */
						base.accData = sensOut.xyzData;
						PRINT_TO_CLI(FORMAT_ACC_DATA(base.accData.x) + FORMAT_ACC_DATA(base.accData.y) + FORMAT_ACC_DATA(base.accData.z),
							base.accData.x / 1000, base.accData, base.accData.y / 1000, base.accData.y, base.accData.z / 1000, base.accData.z);

						break;
					case SENSOR_OUT_CLICK_DETECTION:
						/* send notification and time of click detection * to CLI TODO*/
						break;
					case SENSOR_OUT_FALL_DETECTION:
						/* send notification and time of fall detection * to CLI TODO*/
						break;
					}
				}
			}
			break;
		}

		/* temp dbg LED */
		HAL_GPIO_TogglePin(DBG_LED_PORT, DBG_LED_PIN);
	}
}
int main()
{
	/* initialize hardware */
	HAL_Init();
	CLK_init();

	/* init global RTOS variables (queues, semaphores) */
	base.cliTxQueue = xQueueCreate(CLI_TX_QUEUE_LEN, sizeof(uint8_t) * CLI_MAX_LINE_LEN);
	CHECK(base.cliTxQueue);

	base.cliRxQueue = xQueueCreate(CLI_RX_QUEUE_LEN, sizeof(uint8_t) * CLI_MAX_LINE_LEN);
	CHECK(base.cliRxQueue);

	base.sensorOutputQueue = xQueueCreate(SENSOR_OUT_QUEUE_LEN, sizeof(struct sensor_Output));
	CHECK(base.sensorOutputQueue);

	base.accNumOfAvgSamples = 1;
	base.accClickDetecionEnabled = false;
	base.accClickDetecionEnabled = false;


	RTC_init();
	CLI_init(base.cliTxQueue, base.cliRxQueue, &base.huart2);

	sensor_init(base.sensorOutputQueue);

	if(!(pdTRUE == xTaskCreate(main_task, "main task", configMINIMAL_STACK_SIZE, NULL, 1, NULL))){
		errorHandler();
	}
	if(!(pdTRUE == xTaskCreate(CLI_task, "CLI task", configMINIMAL_STACK_SIZE, NULL, 2, NULL))){
		errorHandler();
	}
	if(!(pdTRUE == xTaskCreate(sensor_task, "sensor task", configMINIMAL_STACK_SIZE, NULL, 3, NULL))){
		errorHandler();
	}

	/* start sheduler */
	vTaskStartScheduler();

	while(1)
	{
		__NOP();
	}
}

void errorHandler(void)
{
	__disable_irq();
	while(1){
		__NOP();
	}
}

void CLK_init(void){
	/* Reset and clock control initialisation. */
	RCC_OscInitTypeDef RCC_OscInit = {0};
	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInit.HSEState = RCC_HSE_BYPASS;
	RCC_OscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInit.LSEState = RCC_LSE_ON;
	RCC_OscInit.HSIState = RCC_HSI_OFF;
	RCC_OscInit.LSIState = RCC_LSI_OFF;
	RCC_OscInit.PLL.PLLState = RCC_PLL_OFF;
	if(HAL_OK != HAL_RCC_OscConfig(&RCC_OscInit)){
		errorHandler();
	}

	/* SYSCLK, AHB and APB init */
	RCC_ClkInitTypeDef RCC_ClkInit = {0};
	RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	if(HAL_OK != HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_2)){
		errorHandler();
	}
}
