/*
 * rtc.h
 *
 *  Created on: 11 May 2021
 *      Author: Administrator
 */

#ifndef APP_INC_RTC_H_
#define APP_INC_RTC_H_
#include "stm32f3xx_hal.h"
/* === exported variables === */
extern RTC_HandleTypeDef hrtc;

/* === exported functions === */

/** Real Time Clock initialisation function */
void
RTC_init (void);

#endif /* APP_INC_RTC_H_ */
