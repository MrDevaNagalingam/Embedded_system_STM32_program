#include "spi.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "delay.h"
#include "stdio.h"

void SPI_Init(void)
{
	#define AF5 0x05
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //enable clock forn gpio a
	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN; //enable clock for spi1
	GPIOA->MODER|=GPIO_MODER_MODE5_1|GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1;
	GPIOA->MODER&=~(GPIO_MODER_MODE5_0|GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0);
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR5|GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7;

	GPIOA->AFR[0]|=(AF5<<20)|(AF5<<24)|(AF5<<28);
	SPI1->CR2=0;
	SPI1->CR1=SPI_CR1_SSM|SPI_CR1_MSTR|SPI_CR1_BR_2|SPI_CR1_SSI|SPI_CR1_SPE;



}


int8_t SPI_Transmit(uint8_t *data, uint32_t size)
{


	uint32_t i		=0;
	uint8_t  temp	=0;
	uint32_t start=millis();
	temp =SPI1->DR;
	temp=SPI1->SR;
	while(i<size)
		{
		while(!((SPI1->SR)&SPI_SR_TXE)){if(millis()-start>1000){
			printf("TXE timed out\r\n");
			return -1;}} // wait to transmision buffer to be emplty
		SPI1->DR= data[i];
		while(!(SPI1->SR&SPI_SR_BSY)){if(millis()-start>1000){printf("BSY timed out\r\n");return -1;}}
		i++;
		}
while(!((SPI1->SR)&SPI_SR_TXE)){if(millis()-start>1000){printf("TXE2 time dout\r\n");return -1;}}
while((SPI1->SR)&SPI_SR_BSY){if(millis()-start>1000){printf("BSY2 timed out\r\n"); return -1;}}
temp =SPI1->DR;
temp=SPI1->SR;
return 0;
}


int8_t SPI_Receive(uint8_t *data, uint32_t size)
{
while(size)
		{
	uint32_t start=millis();
		SPI1->DR=0;
		while(!(SPI1->SR&SPI_SR_RXNE)){if(millis()-start>200){return -1;}}
		*data++=(SPI1->DR);
			size--;
		}
return 0;
}

