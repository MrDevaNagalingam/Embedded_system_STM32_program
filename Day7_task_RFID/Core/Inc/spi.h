/*
 * spi.h
 *
 *  Created on: april 23, 2023
 *      Author: chaman
 */

#ifndef SPI_H_
#define SPI_H_
#include "stdint.h"
void SPI_Init(void);
int8_t SPI_Transmit(uint8_t *data, uint32_t size);
int8_t SPI_Receive(uint8_t *data, uint32_t size);

#endif /* SPI_H_ */
