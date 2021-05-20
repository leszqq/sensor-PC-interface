/*
 * uart.c
 *
 *  Created on: 1 May 2021
 *      Author: Administrator
 */
#include "uart.h"
#include "main.h"

DMA_HandleTypeDef hdma_cli_tx;
static struct Base {
	UART_HandleTypeDef* huart;
} base;

void UART_Init(UART_HandleTypeDef* huart){
	/* enable DMA channel 7 */
	__HAL_RCC_DMA1_CLK_ENABLE();
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 10, 10);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	/* configure uart2 */
	huart->Instance = USART2;
	huart->Init.BaudRate = 115200;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;
	huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;

	base.huart = huart;
	if(HAL_OK != HAL_UART_Init(huart)){
		errorHandler();
	}
}


/**
  * @brief Initialize the UART MSP.
  * @param huart UART handle.
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{

	if(USART2 == huart->Instance){
		/* enable USART2 CLK */
		__HAL_RCC_USART2_CLK_ENABLE();

		/* enable and configure PA2(Tx) and PA3(Rx) pins */
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitTypeDef gpioInit;
		gpioInit.Mode = GPIO_MODE_AF_PP;
		gpioInit.Alternate = GPIO_AF7_USART1;
		gpioInit.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		gpioInit.Pull = GPIO_NOPULL;
		gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &gpioInit);

		/* Init USART2 DMA */
		hdma_cli_tx.Instance = DMA1_Channel7;
		hdma_cli_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_cli_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_cli_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_cli_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_cli_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_cli_tx.Init.Mode = DMA_NORMAL;
		hdma_cli_tx.Init.Priority = DMA_PRIORITY_LOW;
		if(HAL_OK != HAL_DMA_Init(&hdma_cli_tx)){
			errorHandler();
		}

		__HAL_LINKDMA(huart, hdmatx, hdma_cli_tx);

		/* USART2 interrupt init */
		HAL_NVIC_SetPriority(USART2_IRQn, 10, 10);
		HAL_NVIC_EnableIRQ(USART2_IRQn);

	}
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_cli_tx);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(base.huart);
}
