/*
 * cli.h
 *
 *  Created on: Mar 25, 2021
 *      Author: Wiktor Lechowicz
 *
 *      This code contain reusable code for usart based command line interface targeted for stm32 MCUs based on HAL drivers.
 *
 *          * HOW TO USE *
 *          Create struct CLI_field_destrictor array to describe content of memory with tag strings and type specifiers CLI_field_type.
 *
 *          Initialize CLI (CLI_Init()) with just created memory descriptor.
 *          Example in main.c
 */

/* === exported includes === */

#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* === exported defines === */
#define CLI_MAX_LINE_LEN	50


/* === exported variables === */
/* === exported variables === */

/* === exported functions === */
void CLI_init(QueueHandle_t txQueue, QueueHandle_t rxQueue, UART_HandleTypeDef* huart);

void CLI_task(void * params);

#ifndef APP_INC_UART_CLI_C_
#define APP_INC_UART_CLI_C_

#endif /* APP_INC_UART_CLI_C_ */
