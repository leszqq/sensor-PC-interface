/*
 * check_macros.h
 *
 *  Created on: 26 mar 2021
 *      Author: Wiktor Lechowicz
 */

#ifndef APP_INC_CHECK_MACROS_H_
#define APP_INC_CHECK_MACROS_H_

#include <stdio.h>
#include <string.h>
#include "usart.h"

/* debug and error check macros */


#ifdef DEBUG
#define DBG_STRING_LEN             120
uint8_t dbg_string[DBG_STRING_LEN];

/* warning in debug build */
#define WARN(C, format, ...) if(!(C)) { \
                            snprintf((char*) dbg_string, DBG_STRING_LEN, "[WRN]t[ms]:%ld:%s:%u:%s: " format "\n\r",  HAL_GetTick(), __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
                                    HAL_UART_Transmit(&huart2, dbg_string, strlen((char*)dbg_string), HAL_MAX_DELAY); } \

/* log in debug build */
#define DBG_LOG(format, ...)    snprintf((char*) dbg_string, DBG_STRING_LEN, "[LOG]t[ms]:%ld:%s:%u:%s: " format "\n\r",  HAL_GetTick(), __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
                                  HAL_UART_Transmit(&huart2, dbg_string, strlen((char*)dbg_string), HAL_MAX_DELAY);

/* assert in debug build */
#define CHECK(C, format, ...) if(!(C)) { \
                            snprintf((char*) dbg_string, DBG_STRING_LEN, "[ERR]t[ms]:%ld:%s:%u:%s: " format "\n\r",  HAL_GetTick(), __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
                                    HAL_UART_Transmit(&huart2, dbg_string, strlen((char*)dbg_string), HAL_MAX_DELAY); \
                            goto error; } \

#else
/* macros in release build */
#define WARN(C, format, ...)
#define DBG_LOG(format, ...)
#define CHECK(C, format, ...) if(!(C)) goto error
#endif

#endif /* APP_INC_CHECK_MACROS_H_ */
