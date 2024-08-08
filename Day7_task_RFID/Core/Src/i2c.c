


#include "i2c.h"
#include "stm32f4xx.h"                  // Device header
#include "stdio.h"

void i2c_init(void){
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //enable gpiob clock
RCC->APB1ENR|=RCC_APB1ENR_I2C1EN; //enable i2c1 clock
GPIOB->MODER|=0xA0000; //set pb8and9 to alternative function
GPIOB->AFR[1]|=0x44;
GPIOB->OTYPER|=GPIO_OTYPER_OT8|GPIO_OTYPER_OT9; //set pb8 and pb9 as open drain

I2C1->CR1=0x8000;
I2C1->CR1&=~0x8000;
I2C1->CR2|=0x10;
I2C1->CCR=80; //standard 100KHz
I2C1->TRISE=17;
I2C1->CR1|=0x1;
	
}

int8_t i2c_readByte(uint8_t saddr,uint8_t maddr, uint8_t *data)
{
uint64_t start=millis();
volatile int tmp=0;
while(I2C1->SR2&I2C_SR2_BUSY){if(millis()-start>500){return -1;}}
I2C1->CR1|=I2C_CR1_START;
start=millis();
while(!(I2C1->SR1&I2C_SR1_SB)){if(millis()-start>500){return -1;}}
I2C1->DR=saddr<<1;
while(!(I2C1->SR1&I2C_SR1_ADDR)){if(millis()-start>500){return -1;}}
tmp=I2C1->SR2;
while(!(I2C1->SR1&I2C_SR1_TXE)){if(millis()-start>500){return -1;}}
I2C1->DR=maddr;
while(!(I2C1->SR1&I2C_SR1_TXE)){if(millis()-start>500){return -1;}}
I2C1->CR1|=I2C_CR1_START;
while(!(I2C1->SR1&I2C_SR1_SB)){if(millis()-start>500){return -1;}}	
I2C1->DR=saddr<<1|1;
while(!(I2C1->SR1&I2C_SR1_ADDR)){if(millis()-start>500){return -1;}}
I2C1->CR1&=~I2C_CR1_ACK;
tmp =I2C1->SR2;
I2C1->CR1|=I2C_CR1_STOP;
while(!(I2C1->SR1&I2C_SR1_RXNE)){if(millis()-start>500){return -1;}}
*data++=I2C1->DR;
return 0;
}

int8_t i2c_writeByte(uint8_t saddr,uint8_t maddr,uint8_t data){
uint64_t start=millis();
volatile int Temp=0;
while(I2C1->SR2&I2C_SR2_BUSY){if(millis()-start>500){return -1;}}          /*wait until bus not busy*/
I2C1->CR1|=I2C_CR1_START;                 /*generate start*/
while(!(I2C1->SR1&I2C_SR1_SB)){if(millis()-start>500){return -1;}}         /*wait until start bit is set*/
I2C1->DR = saddr<< 1;                 	 /* Send slave address*/
while(!(I2C1->SR1&I2C_SR1_ADDR)){if(millis()-start>500){return -1;}}      /*wait until address flag is set*/
Temp = I2C1->SR2; 											 /*clear SR2 by reading it */
while(!(I2C1->SR1&I2C_SR1_TXE)){if(millis()-start>500){return -1;}}       /*Wait until Data register empty*/
I2C1->DR = maddr;                        /* send memory address*/
while(!(I2C1->SR1&I2C_SR1_TXE)){if(millis()-start>500){return -1;}}       /*wait until data register empty*/
I2C1->DR = data; 	
while (!(I2C1->SR1 & I2C_SR1_BTF)){if(millis()-start>500){return -1;}}     /*wait until transfer finished*/
I2C1->CR1 |=I2C_CR1_STOP;								 /*Generate Stop*/	
return 0;
}

int8_t i2c_WriteMulti(uint8_t saddr,uint8_t maddr,uint8_t *buffer, uint8_t length){
uint64_t start=millis();
while (I2C1->SR2 & I2C_SR2_BUSY){if(millis()-start>500){return -1;}}           //wait until bus not busy
I2C1->CR1 |= I2C_CR1_START;                   //generate start
while (!(I2C1->SR1 & I2C_SR1_SB)){if(millis()-start>500){return -1;}}					//wait until start is generated
volatile int Temp=0;
I2C1->DR = saddr<< 1;                 	 			// Send slave address
while (!(I2C1->SR1 & I2C_SR1_ADDR)){if(millis()-start>500){return -1;}}        //wait until address flag is set
Temp = I2C1->SR2; 						      //Clear SR2
while (!(I2C1->SR1 & I2C_SR1_TXE)){if(millis()-start>500){return -1;}}           //Wait until Data register empty
I2C1->DR = maddr;                      				// send memory address
while (!(I2C1->SR1 & I2C_SR1_TXE)){if(millis()-start>500){return -1;}}           //wait until data register empty
//sending the data
for (uint8_t i=0;i<length;i++)
 { 
 I2C1->DR=buffer[i]; 													//filling buffer with command or data
	while (!(I2C1->SR1 & I2C_SR1_BTF)){if(millis()-start>500){return -1;}}
 }	
                             
I2C1->CR1 |= I2C_CR1_STOP;	//wait until transfer finished

return 0;
}


int8_t i2c_ReadMulti(uint8_t saddr,uint8_t maddr, uint8_t n, uint8_t* data)
{
	uint64_t start=millis();
	volatile int temp=0;
	while (I2C1->SR2 & I2C_SR2_BUSY){if(millis()-start>500){return -1;}}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){if(millis()-start>500){return -1;}}
	I2C1->DR=saddr<<1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){if(millis()-start>500){return -1;}}
	temp=I2C1->SR2;
	while(!(I2C1->SR1&I2C_SR1_TXE)){if(millis()-start>500){return -1;}}
	I2C1->DR = maddr;
	while(!(I2C1->SR1&I2C_SR1_TXE)){if(millis()-start>500){return -1;}}
	I2C1->CR1|=I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){if(millis()-start>500){return -1;}}
	I2C1->DR=saddr<<1|1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){if(millis()-start>500){return -1;}}
	temp=I2C1->SR2;
	I2C1->CR1|=I2C_CR1_ACK;
	while(n>0U)
		{
		if(n==1U)
				{
				I2C1->CR1&=~I2C_CR1_ACK;
					I2C1->CR1|=I2C_CR1_STOP;
					while(!(I2C1->SR1&I2C_SR1_RXNE)){if(millis()-start>500){return -1;}}
					*data++=I2C1->DR;
						break;
				}
			else
					{
					
					while(!(I2C1->SR1&I2C_SR1_RXNE)){if(millis()-start>500){return -1;}}
						(*data++)=I2C1->DR;
							n--;
					
					}	
				
			
		}	
	return 0;	
}


void i2c_bus_scan(void)
{        
		int a=0; //address variable
         for (uint8_t i=0;i<128;i++) //go through all 127 address
   {
			uint32_t start=millis();
            I2C1->CR1 |= I2C_CR1_START; //generate start 

            while(!(I2C1->SR1 & I2C_SR1_SB)){
            if(millis()-start>100){printf("bus error\r\n"); return;}} // wait to start to be generated
            I2C1->DR=(i<<1|0); // transmit the address
            while(!(I2C1->SR1)|!(I2C1->SR2)){}; //clear status register
            I2C1->CR1 |= I2C_CR1_STOP; //generate stop condition
						//printf("stop generated \r\n");
           // delay(100);//minium wait time is 40 uS, but for sure, leave it 100 uS
						for (volatile int i=0;i<1000;i++){;}
            a=(I2C1->SR1&I2C_SR1_ADDR); //read the status register address set
            if (a==2)//if the address is valid
         {
					 //print the address
					// printf("Found I2C device at adress 0x%X (hexadecimal), or %d (decimal)\r\n",i,i);
           //UART_Write_String(data);
         }
     }
}
