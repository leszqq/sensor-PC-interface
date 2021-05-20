#ifndef MAIN_H
#define MAIN_H

#include "stm32f3xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

//#include "task.h"

/* === exported defines === */
#define CLI_MAX_LINE_LEN				50

#define CHECK(C) if(!(C)) errorHandler()

/**
 * @brief This function is executed in case of error.
 * @retval None
 */
void errorHandler(void);
#endif
