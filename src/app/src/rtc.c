/*
 * rtc.c
 *
 *  Created on: 11 May 2021
 *      Author: Administrator
 */
#include "rtc.h"
#include "stm32f3xx_hal.h"
#include "main.h"

RTC_HandleTypeDef hrtc;
/* === exported functions === */
void RTC_init(void){

	__HAL_RCC_PWR_CLK_ENABLE();													// Enable power controller interface clock.
	HAL_PWR_EnableBkUpAccess();													// Enable accces to RTC domain.
	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);									// Select RTC clock source.
	__HAL_RCC_RTC_ENABLE();														// Enable RTC clock.

	/* RTC parameters init.*/
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	if(HAL_OK != HAL_RTC_Init(&hrtc)){
		errorHandler();
	};
}



