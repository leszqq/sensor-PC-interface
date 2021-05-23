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
#include <stdlib.h>

#define CLI_RX_QUEUE_LEN				10										/// length of queue containing command received on CLI
#define CLI_TX_QUEUE_LEN				10										/// length of queue containing commands to send via CLI
#define SENSOR_OUT_QUEUE_LEN			4										/// length of queue containing sensor output

#define MAIN_TASK_SACK_SIZE				512

#define ACC_SET_RATE_VALUE_POS_IN_CLI	13
#define ACC_RATE_STRING_MAX_LEN			7

/** Available rates of reading data samples from accelerometer */
#define ACC_RATE_25HZ_STRING			"25Hz"
#define ACC_RATE_50HZ_STRING			"50Hz"
#define ACC_RATE_100HZ_STRING			"100Hz"
#define ACC_RATE_200HZ_STRING			"200Hz"
#define ACC_RATE_400HZ_STRING			"400Hz"
#define ACC_RATE_800HZ_STRING			"800Hz"
#define ACC_RATE_1600HZ_STRING			"160Hz"

/** Available sample average range */
#define ACC_MIN_AVG_NUMBER				1
#define ACC_MAX_AVG_NUMBER				1000

/** Clock initialisation */
void CLK_init(void);

/* === private macros === */
#define PRINT_TO_CLI(S, ...)  				do { \
        										snprintf((char*)base.auxTab , CLI_MAX_LINE_LEN, S, ##__VA_ARGS__); \
        										xQueueSendToBack(base.cliTxQueue, base.auxTab, portMAX_DELAY); \
											}while(0)




#define CLEAR_CLI()							PRINT_TO_CLI("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\r>>")

#define FORMAT_ACC_DATA(data)				(data >= 0 ? "   %d.%.3d g" : "  -%d.%.3d g")

#define PRINT_COMMAND_NOT_RECOGNISED()		PRINT_TO_CLI("Wrong command.Type in \"help\" for command list."); \
											PRINT_TO_CLI("\n\r>>");


#define ANY_CLI_ACTIVITY_DETECTED			 (pdTRUE == xQueueReceive(base.cliRxQueue, base.auxTab, 0))

#define SEND_NOT_RECOGNISED() 				do{ \
												xQueueSendToBack(CLI_TransmitQueue, "Command not recognised. Type \"help\" for help.\n\r>>", portMAX_DELAY); \
						  	  				}while(0)

/* === private types === */
enum SystemState {
	SYSTEM_IDLE,																/// Data is not read or processed, system can be configured
	SYSTEM_ACC_DATA_PROCESSING 													/// Accelerometer data reading and processing
};

/* === private variables === */

struct AveragedData{
	int16_t						xDataBuff[ACC_MAX_AVG_NUMBER],					/// buffer for x axis data
								yDataBuff[ACC_MAX_AVG_NUMBER],					/// buffer for y axis data
								zDataBuff[ACC_MAX_AVG_NUMBER],					/// buffer for z axis data
								numOfAveragedSamples,							/// number of averaged samples
								head;											/// next index in buffer to be written
	int32_t						xNumerator,										/// sum of @ref numOfAveragedSamples most recent x axis samples
	 	 	 	 	 	 	 	yNumerator,										/// sum of @ref numOfAveragedSamples most recent y axis samples
								zNumerator;										/// sum of @ref numOfAveragedSamples most recent z axis samples
};

static struct Base {
	QueueHandle_t 				cliTxQueue,										/// CLI transfer queue
								cliRxQueue,										/// CLI receive queue
								sensorOutputQueue;								/// queue with data received from sensor
	UART_HandleTypeDef 			huart2;
	enum SystemState			state;											/// fsm state
	uint8_t 					auxTab[CLI_MAX_LINE_LEN];						/// general purpose array
	struct AveragedData			accData;										/// struct for data averaging purposes
	bool						clickDetecionEnabled;							/// click detection enabled flag
} base;




/* === private functions === */

static void printAccSetup(){

	/* print full scale */
	PRINT_TO_CLI("\n\rfull scale range +/- %u g\n\r", sensor_getAccFullScaleInt());
	/* print data read rate */
	PRINT_TO_CLI("data read rate: %lu.%lu Hz\n\r",  sensor_getAccRateInt() / 1000, sensor_getAccRateInt() % 1000);

	/* print number of averaged samples */
	PRINT_TO_CLI("number of averaged samples: %d\n\r", base.accData.numOfAveragedSamples);

	/* print state of  click detection */
	char tempStr[4];

	if(base.clickDetecionEnabled){
		snprintf((char*) tempStr, CLI_MAX_LINE_LEN, "ON");
	} else {
		snprintf((char*) tempStr, CLI_MAX_LINE_LEN, "OFF");
	}
	PRINT_TO_CLI("click detection %s\n\r", tempStr);
}

static void printHelp(){
	PRINT_TO_CLI("\n\rList of available commands:\n\racc get setup");
	PRINT_TO_CLI("\n\racc set range [2g|4g|6g|8g|16g]\n\racc set ra");
	PRINT_TO_CLI("te [25Hz|50Hz|100Hz|200Hz|400Hz|800Hz|1600Hz]\n\r");
	PRINT_TO_CLI("acc set avg number [1-1000]\n\racc set click det ");
	PRINT_TO_CLI("[on|off]\n\rstart\n\n\r>>");
}

static void setAccFullScale(uint8_t fullScaleVal){
	switch(fullScaleVal){
	case 2:
		sensor_setAccFullScale(SENSOR_ACC_FULL_SCALE_2G);
		break;
	case 4:
		sensor_setAccFullScale(SENSOR_ACC_FULL_SCALE_4G);
		break;
	case 6:
		sensor_setAccFullScale(SENSOR_ACC_FULL_SCALE_6G);
		break;
	case 8:
		sensor_setAccFullScale(SENSOR_ACC_FULL_SCALE_8G);
		break;
	case 16:
		sensor_setAccFullScale(SENSOR_ACC_FULL_SCALE_16G);
		break;
	default:
		PRINT_TO_CLI("Wrong full scale value\n\r");
		break;
	}
}

static void setAccRate(uint16_t rateVal){
	switch(rateVal){
	case 25:
		sensor_setAccRate(SENSOR_ACC_RATE_25HZ);
		break;
	case 50:
		sensor_setAccRate(SENSOR_ACC_RATE_50HZ);
		break;
	case 100:
		sensor_setAccRate(SENSOR_ACC_RATE_100HZ);
		break;
	case 200:
		sensor_setAccRate(SENSOR_ACC_RATE_200HZ);
		break;
	case 400:
		sensor_setAccRate(SENSOR_ACC_RATE_400HZ);
		break;
	case 800:
		sensor_setAccRate(SENSOR_ACC_RATE_800HZ);
		break;
	case 1600:
		sensor_setAccRate(SENSOR_ACC_RATE_1600HZ);
		break;
	default:
		PRINT_TO_CLI("Wrong rate value\n\r");
		break;
	}
}

static void setAccAvgNumber(uint16_t avgNumebr){
	if(avgNumebr > ACC_MIN_AVG_NUMBER && avgNumebr < ACC_MAX_AVG_NUMBER){
		base.accData.numOfAveragedSamples = avgNumebr;
	} else {
		PRINT_TO_CLI("wrong number of averaged samples");
	}
}

void main_task(void * params){
	UNUSED(params);

	/* temp variables */
	uint16_t tempInt = 0;
	RTC_TimeTypeDef rtcTime = {0};
	RTC_DateTypeDef rtcDate = {0};

	PRINT_TO_CLI("Type in \"help\" for command list\n\r>>");
	/* main system loop */
	while(1){
		switch(base.state){
		case SYSTEM_IDLE:
			/* Block task until new command available */
			xQueueReceive(base.cliRxQueue, base.auxTab, portMAX_DELAY);
			/* Execute commands */
			if(base.auxTab[0] == 0){
				PRINT_TO_CLI("\n\r>>");

			} else if(0 == strncmp((char *) base.auxTab, "help", CLI_MAX_LINE_LEN)){
				printHelp();

			} else if(0 == strncmp((char *) base.auxTab, "acc get setup", CLI_MAX_LINE_LEN)){
				printAccSetup();

			} else if(1 == sscanf((char*) base.auxTab, "acc set range %hhug", (uint8_t*)&tempInt)) {
				setAccFullScale(tempInt);

			} else if(1 == sscanf((char*) base.auxTab, "acc set rate %huHz", &tempInt)){
				setAccRate(tempInt);

			} else if(1 == sscanf((char*) base.auxTab, "acc set avg number %hu", &tempInt)){
				setAccAvgNumber(tempInt);

			} else if(0 == strncmp((char*) base.auxTab, "acc set click det on", CLI_MAX_LINE_LEN)){
				base.clickDetecionEnabled = true;

			} else if(0 == strncmp((char*) base.auxTab, "acc set click det off", CLI_MAX_LINE_LEN)){
				base.clickDetecionEnabled = false;

			} else if(0 == strncmp((char*) base.auxTab, "start", CLI_MAX_LINE_LEN)){
				PRINT_TO_CLI("   acc x:    acc y:    acc z:    ");
				PRINT_TO_CLI("last click time: \n\r");
				sensor_start();
				base.state = SYSTEM_ACC_DATA_PROCESSING;

			} else{
				PRINT_COMMAND_NOT_RECOGNISED();
			}
			break;
		case SYSTEM_ACC_DATA_PROCESSING:
			if(ANY_CLI_ACTIVITY_DETECTED){
				/* in case of anything received on CLI, go to IDLE state */
				base.state = SYSTEM_IDLE;
				CLEAR_CLI();

			} else{
				struct sensor_Output sensOut = {0};
				int16_t averagedVal;
				/* Block in waiting for next data or event from accelerometer */
				if(pdTRUE == xQueueReceive(base.sensorOutputQueue, &sensOut, portMAX_DELAY)){
					switch(sensOut.type){
					case SENSOR_OUT_ACC_DATA:
						/* Calculate average value and print it to CLI */
						base.accData.xDataBuff[base.accData.head] = sensOut.xyzData.x;
						base.accData.yDataBuff[base.accData.head] = sensOut.xyzData.y;
						base.accData.zDataBuff[base.accData.head] = sensOut.xyzData.z;
						int16_t substrSampleIndex = base.accData.head - base.accData.numOfAveragedSamples;
						if(substrSampleIndex < 0){
							substrSampleIndex += ACC_MAX_AVG_NUMBER;			// modulo could be used here, but this way it is more effective
						}

						base.accData.xNumerator = base.accData.xNumerator + base.accData.xDataBuff[base.accData.head] - base.accData.xDataBuff[substrSampleIndex];
						base.accData.yNumerator = base.accData.yNumerator + base.accData.yDataBuff[base.accData.head] - base.accData.yDataBuff[substrSampleIndex];
						base.accData.zNumerator = base.accData.zNumerator + base.accData.zDataBuff[base.accData.head] - base.accData.zDataBuff[substrSampleIndex];

						base.accData.head++;
							if(base.accData.head >= ACC_MAX_AVG_NUMBER){
								base.accData.head = 0;
							}

						/* print new data on CLI */
						if(4 <= uxQueueSpacesAvailable(base.cliTxQueue)){
							PRINT_TO_CLI("\r");
							averagedVal = base.accData.xNumerator / base.accData.numOfAveragedSamples;
							PRINT_TO_CLI(FORMAT_ACC_DATA(averagedVal), abs(averagedVal) / 1000, abs(averagedVal) % 1000);

							averagedVal = base.accData.yNumerator / base.accData.numOfAveragedSamples;
							PRINT_TO_CLI(FORMAT_ACC_DATA(averagedVal), abs(averagedVal) / 1000, abs(averagedVal) % 1000);

							averagedVal = base.accData.zNumerator / base.accData.numOfAveragedSamples;
							PRINT_TO_CLI(FORMAT_ACC_DATA(averagedVal), abs(averagedVal) / 1000, abs(averagedVal) % 1000);
						}
						break;
					case SENSOR_OUT_CLICK_DETECTION:
						/* send notification and time of click detection to CLI */
						if(base.clickDetecionEnabled){
							HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
							HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
							PRINT_TO_CLI("   %02d:%02d:%02d\b\b\b\b\b\b\b\b", rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);
						}
						break;
					}
				}
			}
			break;
		}
	}
}
int main()
{
	/* initialise hardware */
	HAL_Init();
	CLK_init();

	/* init global RTOS variables (queues, semaphores) */
	base.cliTxQueue = xQueueCreate(CLI_TX_QUEUE_LEN, sizeof(uint8_t) * CLI_MAX_LINE_LEN);
	CHECK(base.cliTxQueue);

	base.cliRxQueue = xQueueCreate(CLI_RX_QUEUE_LEN, sizeof(uint8_t) * CLI_MAX_LINE_LEN);
	CHECK(base.cliRxQueue);

	base.sensorOutputQueue = xQueueCreate(SENSOR_OUT_QUEUE_LEN, sizeof(struct sensor_Output));
	CHECK(base.sensorOutputQueue);

	/* initial app setups */
	base.accData.numOfAveragedSamples = 1;
	base.clickDetecionEnabled = false;

	/* initialise modules and start tasks */
	RTC_init();

	CLI_init(base.cliTxQueue, base.cliRxQueue, &base.huart2);

	sensor_init(base.sensorOutputQueue);

	if(!(pdTRUE == xTaskCreate(main_task, "main task", MAIN_TASK_SACK_SIZE, NULL, 1, NULL))){
		errorHandler();
	}
	if(!(pdTRUE == xTaskCreate(CLI_task, "CLI task", configMINIMAL_STACK_SIZE, NULL, 2, NULL))){
		errorHandler();
	}
	if(!(pdTRUE == xTaskCreate(sensor_task, "sensor task", configMINIMAL_STACK_SIZE, NULL, 3, NULL))){
		errorHandler();
	}

	/* start scheduler */
	vTaskStartScheduler();

	/* debug trap */
	while(1)
	{
		__NOP();
	}
}

/** debug purpose function for catching assertions */
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
