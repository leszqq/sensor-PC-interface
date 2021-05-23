/*
 * myI2C.h
 *
 *  Created on: May 1, 2021
 *      Author: Wiktor Lechowicz
 *      Description:
 *      This file contains functions for initialization and handling of I2C peripherial for stm32f3xx devices.
 *      Desired initialization values have been hardcoded in myI2C_Init function.
 */
#ifndef INC_MYI2C_H_
#define INC_MYI2C_H_

#include "main.h"
#include "stm32f302x8.h"


/** error codes for detecting transmission failure(NACK, arbitration lost or bus error) */
enum I2C_Status {
	I2C_FAILURE	= 0x00,
	I2C_SUCCES	= 0x01,
	I2C_BUSY	= 0x02
};

#define myI2C_SUCCESS    0x01
#define myI2C_FAILURE    0x00
/**
 * @brief This function initializes the I2C2 peripherial.
 */
void I2C_init();

/**
 * @brief write a byte stream to given memory location of a slave in blocking mode
 * @param slaveAdrr -       slave address
 * @param memAddr   -       memory address
 * @param pData     -       pointer to data to send
 * @param dataLen   -       length of byte stream
 * @retval I2C_status
 */
enum I2C_Status I2C_writeByteStream(uint8_t slaveAddr, uint8_t memAddr, uint8_t * pData, uint8_t dataLen);


/**
 * @brief read byte stream from given slave memory location in polling mode.
 * @param slaveAddr -       slave address
 * @param memAddr   -       memory address
 * @param pData     -       address at which the received byte stream will be stored
 * @param dataLen   -       length of byte stream
 * @retval myI2C status
 */
enum I2C_Status I2C_readByteStream(uint8_t slaveAddr, uint8_t memAddr, uint8_t * pData, uint8_t dataLen);


#endif /* INC_MYI2C_H_ */
