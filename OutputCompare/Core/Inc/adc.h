/*
 * adc.h
 *
 *  Created on: May 14, 2021
 *      Author: Israel
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void pa1_adc_init(void);
uint32_t adc_read(void);
void start_converstion(void);


#endif /* ADC_H_ */
