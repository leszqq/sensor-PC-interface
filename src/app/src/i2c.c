/*
 * myI2C.c
 *
 *  Created on: May 8, 2021
 *      Author: Wiktor Lechowicz
 */

#define I2Cx				I2C2												// used I2C
#include "I2C.h"
#include "stm32f302x8.h"														// device registers

#define TIMINGR_CONTENT		0x10E8122C											// calculation based on reference manual

void I2C_init() {
	/* GPIOA CLK enable */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= (0x2UL << GPIO_MODER_MODER9_Pos)
			| (0x2UL << GPIO_MODER_MODER10_Pos);	// alternate function mode
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;		// open drain
	GPIOA->PUPDR |= (0x1UL << GPIO_PUPDR_PUPDR9_Pos)
			| (0x1UL << GPIO_PUPDR_PUPDR10_Pos);				// pull ups
	GPIOA->AFR[1] |= (GPIO_AF4_I2C2 << GPIO_AFRH_AFRH1_Pos)
			| (GPIO_AF4_I2C2 << GPIO_AFRH_AFRH2_Pos);

	/* configure and enable I2C clock */
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;						// reset I2C2
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;						// reset I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;	// enable I2C2 register control clock
	RCC->CFGR3 |= RCC_CFGR3_I2C2SW_SYSCLK;		// select I2C kernel clk source

	/* configure and enable I2C2 */
	I2C2->CR2 &= ~I2C_CR1_PE;									// disable I2C2
	I2C2->TIMINGR = TIMINGR_CONTENT;							// set timings
	I2C2->CR1 &= ~I2C_CR1_NOSTRETCH;				// enable clock stretching
	I2C2->CR1 |= I2C_CR1_PE;									// enable I2C2
}

enum I2C_Status I2C_writeByteStream(uint8_t slaveAddr, uint8_t memAddr,
		uint8_t *pData, uint8_t dataLen) {
	assert_param(I2Cx);
	if (dataLen != 0) {
		assert_param(pData);
	}

	while (I2Cx->ISR & I2C_ISR_BUSY) {			// wait for I2Cx bus not busy
	}
	taskENTER_CRITICAL();
	I2Cx->CR2 &= ~I2C_CR2_ADD10;				// select 7-bit addressing mode
	I2Cx->CR2 = I2C_CR2_SADD & slaveAddr;					// set slave address
	I2Cx->CR2 &= ~I2C_CR2_RD_WRN;							// set write mode

	I2Cx->CR2 &= ~I2C_CR2_NBYTES_Msk;
	I2Cx->CR2 |= (dataLen + 1) << I2C_CR2_NBYTES_Pos;

	I2Cx->CR2 |= I2C_CR2_AUTOEND;// STOP will be generated after NBYTES send;\	// write data to TXDR
	I2Cx->TXDR = memAddr;
	I2Cx->CR2 |= I2C_CR2_START;						// generate start condition

	for (uint8_t i = 0; i < dataLen; i++) {
		while (!(I2Cx->ISR & I2C_ISR_TXIS)) {
			__NOP();
		}
		I2Cx->TXDR = pData[i];
	}
	taskEXIT_CRITICAL();
	return I2C_SUCCES;
}

enum I2C_Status I2C_readByteStream(uint8_t slaveAddr, uint8_t memAddr,
		uint8_t *pData, uint8_t dataLen) {
	assert_param(I2Cx);
	assert_param(pData);
	taskENTER_CRITICAL();
	/* set device memory address to read from. */
	I2C_writeByteStream(slaveAddr, memAddr, pData, 0);

	while (I2Cx->ISR & I2C_ISR_BUSY) {			// wait for I2Cx bus not busy
	}
	/* read */
	I2Cx->CR2 |= I2C_CR2_RD_WRN;
	I2Cx->CR2 &= ~I2C_CR2_NBYTES_Msk;
	I2Cx->CR2 |= (dataLen) << I2C_CR2_NBYTES_Pos;// set number of bytes to read
	I2Cx->CR2 |= I2C_CR2_AUTOEND;
	I2Cx->CR2 |= I2C_CR2_START;

	for (uint8_t i = 0; i < dataLen; i++) {
		/* wait for data in RXDR */
		while (!(I2Cx->ISR & I2C_ISR_RXNE)) {
		}
		pData[i] = I2Cx->RXDR;
	}
	taskEXIT_CRITICAL();

	return I2C_SUCCES;
}

