/**
  ******************************************************************************
  * @file    terminal.c 
  * @author  Ali Firat ARI
  * @author  Irfan Bilaloglu
  * @author  Sencer Altintop
  * @version V1.0.1
  * @brief  Manage waveform genarator with USART
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Gebze Technical University</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "terminal.h"
#include "generator.h"


/* Private definitions ---------------------------------------------------------*/
#define MAX_FREQ 500000
#define MIN_FREQ 2000

#define MAX_MAGNITUDE 100
#define MIN_MAGNITUDE 0

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t tDelay;
char rx_buffer[31] = {0};
char rx_temp[3] = {0}; 
int rx_done = 0;


void Systick_Handler(void)
{
    if (tDelay != 0x00)
    {
        tDelay--;
    }
}
void __Clear(void)
{
	for(int i = 0 ; i < 31 ; i++)
		rx_buffer[i] = 0;
}


void UART2_Handler(void)
{
    if (((USART2->SR) >> 5) & 0x01)
    {
        static uint8_t i = 0;
        char data = (uint8_t) (USART2->DR & 0x000000FF);

        if(((data != '\n') && (data != '\n') && (data != 13) && (data != 10) ) && (i < 30))
        {	
			if (data == 8){
				rx_buffer[i] = 0;
				i--;
				UART_SendData("\b");
			}
			else 
			{
				rx_buffer[i++] = data;
				rx_temp[0] = data;
				UART_SendData(rx_temp);
				
			}

        } else {
            rx_buffer[i] = '\0';
            USART2->SR &= 0xFFFFFFDF;
			
            if (strcmp(rx_buffer,"") == 0)
            {

            }

            i = 0;
			UART_SendData("\r\n");
			if ((strcmp(rx_buffer,"")==0) || (strcmp(rx_buffer,'\r')==0) || 
				(strcmp(rx_buffer,'\n')==0) || (strcmp(rx_buffer,'\n\r')==0) || 
				(strcmp(rx_buffer,'\r\n')==0) || (strcmp(rx_buffer, 13)==0) || (strcmp(rx_buffer, 10)==0) )
				{
				
				}
			////////////////////////CREDITS AND HELP//////////////////////////////
			else if ((strcmp(rx_buffer,"help")==0) || (strcmp(rx_buffer,"h")==0)) {
				Send_Help();
				}
			else if ((strcmp(rx_buffer,"credits")==0) || (strcmp(rx_buffer,"c")==0)) {
				UART_SendData("Credits");
				Send_Info();
				}
			else if ((strcmp(rx_buffer,"led group")==0) || (strcmp(rx_buffer,"l")==0)) {
				UART_SendData("Showing group id with leds");
				Send_GroupId(3);
				}
			////////////////////////SET WAVEFORM//////////////////////////////
			else if ((strcmp(rx_buffer,"set wave sine")==0) || (strcmp(rx_buffer,"s w sin")==0)) {
                set_wave_type(SIN_WAVE);
				UART_SendData("Cofiguration has been setting sinus waveform");
				}
			else if ((strcmp(rx_buffer,"set wave square")==0) || (strcmp(rx_buffer,"s w squ")==0)) {
                set_wave_type(SQUARE_WAVE);
				UART_SendData("Cofiguration has been setting square waveform");
				}
			else if ((strcmp(rx_buffer,"set wave triangular")==0) || (strcmp(rx_buffer,"s w tri")==0)) {
                set_wave_type(TRIANGLE_WAVE);
				UART_SendData("Cofiguration has been setting triangular waveform");
				}
			else if ((strcmp(rx_buffer,"set wave sawtooth")==0) || (strcmp(rx_buffer,"s w saw")==0)) {
                set_wave_type(SAWTOOTH_WAVE);
				UART_SendData("Cofiguration has been setting sawtooth waveform");
				}
			else if ((strcmp(rx_buffer,"set wave whitenoise")==0) || (strcmp(rx_buffer,"s w noi")==0)) {
				UART_SendData("Cofiguration has been setting whitenoise waveform");	
			} 

			////////////////////////SET MAGNITUDE//////////////////////////////
			else if ((strncmp(rx_buffer,"set magnitude", 13 )==0) || (strncmp(rx_buffer,"s m", 3)==0)) {
				
				if (((strlen(rx_buffer) == 3 || strlen(rx_buffer) == 4) && (strncmp(rx_buffer,"s m", 3)==0) ) ||
				(((strlen(rx_buffer) == 13 || strlen(rx_buffer) == 14) && (strncmp(rx_buffer,"set magnitude", 13)==0) ))
				)
				{
					UART_SendData("Please give a magnitude level.");
				}
				else if 
				(!(	((strlen(rx_buffer) > 14 && (strlen(rx_buffer) < 18)) && (strncmp(rx_buffer,"set magnitude", 13)==0)) ||
						((strlen(rx_buffer) > 4 && (strlen(rx_buffer) < 8)) && (strncmp(rx_buffer,"s m", 3)==0))
					))
				{
					UART_SendData("Please give a correct level");	
				}

				else if (
						((strlen(rx_buffer) > 14 && (strlen(rx_buffer) < 18)) && (strncmp(rx_buffer,"set magnitude", 13)==0)) ||
						((strlen(rx_buffer) > 4 && (strlen(rx_buffer) < 8)) && (strncmp(rx_buffer,"s m", 3)==0))
						)
				{
					
					if (strncmp(rx_buffer,"s m", 3)==0)
					{
						char *c_magnitude = NULL;
						c_magnitude = strtok(rx_buffer, "s m");
						int magnitude = parseInt(c_magnitude);

						if ( ( (MAX_MAGNITUDE + 1) > magnitude ) && ( (MIN_MAGNITUDE) < magnitude) )
						{
							UART_SendData("Correct magnitude level value. ");
							UART_SendData(c_magnitude);
                            calculate_amp((uint16_t)(magnitude));
						}
						else
						{
							UART_SendData("Not correct magnitude level value. ");
							UART_SendData(c_magnitude);
						}
					}

					else if (strncmp(rx_buffer,"set magnitude", 13)==0)
					{
						char *c_magnitude = NULL;
						c_magnitude = strtok(rx_buffer, "set magnitude");
						int magnitude = parseInt(c_magnitude);

						if ( ( (MAX_MAGNITUDE + 1) > magnitude ) && ( (MIN_MAGNITUDE) < magnitude) )
						{
							UART_SendData("Correct magnitude level value. ");
							UART_SendData(c_magnitude);
							calculate_amp((uint16_t)(magnitude));
						}
						else
						{
							UART_SendData("Not correct magnitude level value. ");
							UART_SendData(c_magnitude);
						}
					}

				}
				else {
					UART_SendData("Unknown error!");
				}
				
			}
			////////////////////////SET FREQ//////////////////////////////
			else if ((strncmp(rx_buffer,"set frequency", 13 )==0) || (strncmp(rx_buffer,"s f", 3)==0)) {
				
				if (((strlen(rx_buffer) == 3 || strlen(rx_buffer) == 4) && (strncmp(rx_buffer,"s f", 3)==0) ) ||
				(((strlen(rx_buffer) == 13 || strlen(rx_buffer) == 14) && (strncmp(rx_buffer,"set frequency", 13)==0) ))
				)
				{
					UART_SendData("Please give a frequency.");
				}
				else if 
				(!(	((strlen(rx_buffer) > 14 && (strlen(rx_buffer) < 22)) && (strncmp(rx_buffer,"set frequency", 13)==0)) ||
						((strlen(rx_buffer) > 4 && (strlen(rx_buffer) < 12)) && (strncmp(rx_buffer,"s f", 3)==0))
					))
				{
					UART_SendData("Please give a correct frequency");	
				}

				else if (
						((strlen(rx_buffer) > 14 && (strlen(rx_buffer) < 22)) && (strncmp(rx_buffer,"set frequency", 13)==0)) ||
						((strlen(rx_buffer) > 4 && (strlen(rx_buffer) < 12)) && (strncmp(rx_buffer,"s f", 3)==0))
						)
				{
					
					if (strncmp(rx_buffer,"s f", 3)==0)
					{
						char *c_frequency = NULL;
						c_frequency = strtok(rx_buffer, "s f");
						int frequency = parseInt(c_frequency);

						if ( ( (MAX_FREQ + 1) > frequency ) && ( (MIN_FREQ - 1) < frequency) )
						{
                            calculate_freq(frequency);
							UART_SendData("Correct frequency value. ");
							UART_SendData(c_frequency);
						}
						else
						{
							UART_SendData("Not correct frequency value. ");
							UART_SendData(c_frequency);
						}
						
					}
					else if (strncmp(rx_buffer,"set frequency", 13)==0)
					{
						char *c_frequency = NULL;
						c_frequency = strtok(rx_buffer, "set frequency");
						int frequency = parseInt(c_frequency);

						if ( ( (MAX_FREQ + 1) > frequency ) && ( (MIN_FREQ - 1) < frequency) )
						{
                            calculate_freq(frequency);
							UART_SendData("Correct frequency value. ");
							UART_SendData(c_frequency);
						}
						else
						{
							UART_SendData("Not correct frequency value. ");
							UART_SendData(c_frequency);
						}
					}

				}
				else {
					UART_SendData("Unknown error!");
				}
				
			}
			////////////////////////NEW COMMAND///////////////////
			//else if ((strncmp(rx_buffer,"new command", 13 )==0) || (strncmp(rx_buffer,"s f", 3)==0)) {
			//
			//}
			//////////////////////////////////////////////////////
			else {
				UART_SendData("Unknown command");
			}
			UART_SendData("\r\n");
        }
    }
}


void Init_Systick(uint32_t s,uint8_t cen)
{
    //Clear CTRL register
    SysTick->CTRL = 0x00000;
    /*Main clock source is runnig with HSI by default which is at 8 Mhz.
     *Systick clock sorunce can be sat with CTRL register's second bit
     * 0: Processor clock/8 (AHB/8)
     * 1: Processor clock
    */
   SysTick->CTRL |= (0 << 2);
   //Enable syscallback 
   SysTick->CTRL |= ((uint32_t)cen << 1);
   //Load value
   SysTick->LOAD = s;
   //Set the Current Value to 0 
   SysTick->VAL  = 0;
   //Enable SysTick bit0
   SysTick->CTRL |= (1 << 0);
}

void Init_Uart(void)
{
    RCC->APB1ENR |= (1 << 17); //Enable clock for USART2
    RCC->AHB1ENR |= (1 << 0);  //USART2 is connected to GPIOA, enable GPIOA clock

    //set pins as alternate func (2nd and 3rd pins)
    GPIOA->MODER &= 0xFFFFFF0F; // Reset bits 10-15 to clear old values
    //GPIOA->MODER |= 0x000000A0; // Set 2nd and 3rd pins as alternate func mode.
	GPIOA->MODER |= (2 << 4); // Set 2nd and 3rd pins as alternate func mode.
	GPIOA->MODER |= (2 << 6); // Set 2nd and 3rd pins as alternate func mode.
    //USART pins speed are high
    //GPIOD->OSPEEDR |= 0x000000A0;
	GPIOA->OSPEEDR |= (3 << 4); // Set pin2 to very high speed
	GPIOA->OSPEEDR |= (3 << 6); // Set pin3 to very high speed

    //AF7 for USART2 in alternate func register
    GPIOA->AFR[0] |= (0x7 << 8);  // for pin 2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin 3

    /*
     *USART2 word length M,bit 12
     *USART2->CR |= (0 << 12); // 0 - 1,8,n
    */

   USART2->CR1 |= (1 << 3);    //USART2_Tx enable, bit 3
   USART2->CR1 |= (1 << 2);    //USART2_Rx enable bit 2

   //set Rx Not Enable İnterrupt Enable (RXNEIE)
   USART2->CR1 |=(1 << 5);

   NVIC_SetPriority(USART2_IRQn,1);
   NVIC_EnableIRQ(USART2_IRQn);
   /*
    * Baund_rate = fCk / (8 * (2 - OVER8) * USARTDIV)
    * Forc fCk = 42 Mhz, Baund = 115200, OVER8 = 0
    * USARTDIV = 42 Mhz / 115200 / 16 = 22.7865
    * We can alsa look at the table and 115.2 KBps baud
    * we need to set 22.8125
    * Fraction :16*0.8125 = 13
    * Mantissa : 22
    * 12-bit mantissa and 4-bit fraction
    */ 
   USART2->BRR |= (22 << 4);
   USART2->BRR |= 13;

   //Enable USART2
   USART2->CR1 |= (1 << 13);
}

void UART_SendData(const char* str)
{
    for (uint32_t i=0; i<strlen((const char*)str); i++)
    {
        //Send Data
        USART2->DR = str[i];
        //wait for transmi complate ,sixth bit of SR, TC
        while(!(USART2->SR & (1 << 6)));
    }
}

void Send_Info(void)
{
    const char* str = "\n\n\r\
    *************Gebze Technical University****************\n\n\r\
	Electronics Engineering ELEC458 \n\n\r\ 
	Embedded System Design\n\n\r\
    				Ali Firat ARI 131024074\n\n\r\
				Irfan Bilaloglu 151024095\n\n\r\
				Sencer Altintop 141024065\n\n\r\
     ************Wave Generator ***********   \n\n\r";
    UART_SendData(str);
}

void Send_Help(void)
{
    const char* help_str = 
	"\n\r\
	help                               : h                        : Shows help page \n\r\
	credits                            : c                        : Shows credits page \n\r\
	led group                          : l                        : Shows group id with leds \n\r\
	set wave sine                      : s w sin                  : Change waveform to sinus shape \n\r\
	set wave square                    : s w squ                  : Change waveform to square shape \n\r\
	set wave triangular                : s w tri                  : Change waveform to triangular shape \n\r\
	set wave sawtooth                  : s w saw                  : Change waveform to sawtooth shape \n\r\
	set wave whitenoise                : s w noi                  : Change waveform to whitenoise shape \n\r\
	set magnitude  <0-100>             : s f <0-100>              : Set to magnitude level  \n\r\
	set frequency  <0-100000000>       : s f <0-100000000>        : Set to frequency \n\r\
	";
    UART_SendData(help_str);
}


void Send_GroupId(uint8_t id)
{
	if ((1 <= id) && (id <= 15))
	{
		RCC->AHB1ENR |= (1 << 3);
		//GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
		GPIOD->MODER |= 0x55000000;
		GPIOD->ODR  = (uint16_t)(id << 12);
		delay_ms(2000);
		GPIOD->ODR  = (uint16_t)(0 << 12);
		delay_ms(2000);
}
	else
		printf("Error! \n The Group ID is out of range (1 to 15)");

}

void delay_ms(volatile uint32_t s)
{
	tDelay = s;
	while(tDelay != 0);
}

int parseInt(char* chars)
{
    int sum = 0;
    int len = strlen(chars);
    for (int x = 0; x < len; x++)
    {
        int n = chars[len - (x + 1)] - '0';
        sum = sum + powInt(n, x);
    }
    return sum;
}

int powInt(int x, int y)
{
    for (int i = 0; i < y; i++)
    {
        x *= 10;
    }
    return x;
}
