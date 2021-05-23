/*
 * uart_cli.c
 *
 *  Created on: Mar 25, 2021
 *      Author: Wiktor Lechowicz
 */
#define BACKSPACE 127

#define DELETE    0x7F

/* private includes */
#include "cli.h"
#include "uart.h"
#include <stdint.h>
#include "string.h"
#include "stm32f3xx.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* === private macros === */
#define PRINT(S, ...) do { \
                            snprintf(base.transmitBuff, CLI_MAX_LINE_LEN, S, ##__VA_ARGS__); \
                            xQueueSend(base.txQueue, base.transmitBuff, portMAX_DELAY); \
                         }while(0)

/* === private defines === */
#define RECEIVED_BUFF_LEN                   30

/* === private variables === */
static struct cli {
	UART_HandleTypeDef *huart;
	QueueHandle_t txQueue;
	QueueHandle_t rxQueue;
	char transmitBuff[CLI_MAX_LINE_LEN];
	char receivedBuff[CLI_MAX_LINE_LEN];
	uint8_t receivedIndex;
} base;

/* === exported functions === */
void CLI_init(QueueHandle_t txQueue, QueueHandle_t rxQueue,
		UART_HandleTypeDef *huart) {
	assert_param(txQueue);
	assert_param(rxQueue);
	assert_param(huart);

	base.huart = huart;
	UART_Init(base.huart);

	base.txQueue = txQueue;
	base.rxQueue = rxQueue;
	base.receivedIndex = 0;

	/* Start character receiving using IT. */
	HAL_UART_Receive_IT(base.huart, (uint8_t*) &base.receivedBuff[0], 1);
}

void CLI_task(void *params) {
	UNUSED(params);

	while (1) {
		/* wait for uart ready */
		while (HAL_UART_STATE_READY != base.huart->gState) {
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		xQueueReceive(base.txQueue, base.transmitBuff, portMAX_DELAY);
		HAL_UART_Transmit_DMA(base.huart, base.transmitBuff,
				strnlen((char*) base.transmitBuff, CLI_MAX_LINE_LEN));
	}
}

/* === callbacks === */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	BaseType_t higherPriorityTaskWoken = pdFALSE;
	if (huart == base.huart) {
		/* On BACKSPACE key, put "\b \b" in transmit queue*/
		if (base.receivedBuff[base.receivedIndex] == BACKSPACE) {
			if (base.receivedIndex > 0) {

				xQueueSendToBackFromISR(base.txQueue, "\b \b",
						&higherPriorityTaskWoken);
				base.receivedIndex--;
			}
			/* If ENTER key press, send buffer to controller to decode*/
		} else if (base.receivedBuff[base.receivedIndex] == CLI_ENTER) {
			xQueueSendToBackFromISR(base.txQueue, "\n\r",
					&higherPriorityTaskWoken);
			base.receivedBuff[base.receivedIndex] = '\0';
			xQueueSendToBackFromISR(base.rxQueue, base.receivedBuff,
					&higherPriorityTaskWoken);

			base.receivedIndex = 0;
			/* Print received character and store it in the buffer. */
		} else {
			HAL_UART_Transmit_IT(base.huart,
					(uint8_t*) &base.receivedBuff[base.receivedIndex], 1);
			if (base.receivedIndex < CLI_MAX_LINE_LEN) {
				base.receivedIndex++;
			} else {
				/* If message too long */
				base.receivedIndex = 0;
				xQueueSendToBackFromISR(base.txQueue,
						"\n\rCommand too long.\n\r>>",
						&higherPriorityTaskWoken);
			}
		}
		HAL_UART_Receive_IT(base.huart,
				(uint8_t*) &base.receivedBuff[base.receivedIndex], 1);
		portEND_SWITCHING_ISR(higherPriorityTaskWoken);
	}
}
