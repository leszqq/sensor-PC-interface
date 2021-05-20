/**
  ******************************************************************************
  * @file    stm32f3xx_hal_msp_template.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied 
  *          to the user folder.
  *         
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/** @addtogroup STM32F3xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL MSP module
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Exported_Functions HAL MSP Exported Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @retval None
  */
void HAL_MspInit(void)
{
	/* LED for blinking */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpioInit;
	gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
	gpioInit.Pin = GPIO_PIN_13;
	gpioInit.Pull = GPIO_NOPULL;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioInit);

	__HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief  DeInitializes the Global MSP.
  * @retval None
  */
void HAL_MspDeInit(void)
{
  
}

/**
  * @brief  Initializes the PPP MSP.
  * @retval None
  */
void HAL_PPP_MspInit(void)
{
   
}

/**
  * @brief  DeInitializes the PPP MSP.
  * @retval None
  */
void HAL_PPP_MspDeInit(void)
{
  
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
