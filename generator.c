/**
  ******************************************************************************
  * @file    generator.c 
  * @author  Ali Firat ARI
  * @author  Irfan Bilaloglu
  * @author  Sencer Altintop
  * @version V1.0.1
  * @brief  Waveform genarator
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Gebze Technical University</center></h2>
  *
  ******************************************************************************
  */


#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>
#include "generator.h"
#include "terminal.h"


/*************************************************
* Vector Table
*************************************************/
// get the stack pointer location from linker
typedef void (* const intfunc)(void);
extern unsigned long __stack;

// attribute puts table in beginning of .vectors section
//   which is the beginning of .text section in the linker script
// Add other vectors -in order- here
// Vector table can be found on page 372 in RM0090
__attribute__ ((section(".vectors")))
void (* const vector_table[])(void) = {
	(intfunc)((unsigned long)&__stack), /* 0x000 Stack Pointer */
	Reset_Handler,                      /* 0x004 Reset         */
	Default_Handler,                    /* 0x008 NMI           */
	Default_Handler,                    /* 0x00C HardFault     */
	Default_Handler,                    /* 0x010 MemManage     */
	Default_Handler,                    /* 0x014 BusFault      */
	Default_Handler,                    /* 0x018 UsageFault    */
	0,                                  /* 0x01C Reserved      */
	0,                                  /* 0x020 Reserved      */
	0,                                  /* 0x024 Reserved      */
	0,                                  /* 0x028 Reserved      */
	Default_Handler,                    /* 0x02C SVCall        */
	Default_Handler,                    /* 0x030 Debug Monitor */
	0,                                  /* 0x034 Reserved      */
	Default_Handler,                    /* 0x038 PendSV        */
	Systick_Handler,                    /* 0x03C SysTick       */
	0,                                  /* 0x040 Window WatchDog Interrupt                                         */
	0,                                  /* 0x044 PVD through EXTI Line detection Interrupt                         */
	0,                                  /* 0x048 Tamper and TimeStamp interrupts through the EXTI line             */
	0,                                  /* 0x04C RTC Wakeup interrupt through the EXTI line                        */
	0,                                  /* 0x050 FLASH global Interrupt                                            */
	0,                                  /* 0x054 RCC global Interrupt                                              */
	0,			                        /* 0x058 EXTI Line0 Interrupt                                              */
	0,                                  /* 0x05C EXTI Line1 Interrupt                                              */
	0,                                  /* 0x060 EXTI Line2 Interrupt                                              */
	0,                                  /* 0x064 EXTI Line3 Interrupt                                              */
	0,                                  /* 0x068 EXTI Line4 Interrupt                                              */
	0,                                  /* 0x06C DMA1 Stream 0 global Interrupt                                    */
	0,                                  /* 0x070 DMA1 Stream 1 global Interrupt                                    */
	0,                                  /* 0x074 DMA1 Stream 2 global Interrupt                                    */
	0,                                  /* 0x078 DMA1 Stream 3 global Interrupt                                    */
	0,                                  /* 0x07C DMA1 Stream 4 global Interrupt                                    */
	0,                                  /* 0x080 DMA1 Stream 5 global Interrupt                                    */
	0,                                  /* 0x084 DMA1 Stream 6 global Interrupt                                    */
	0,                                  /* 0x088 ADC1, ADC2 and ADC3 global Interrupts                             */
	0,                                  /* 0x08C CAN1 TX Interrupt                                                 */
	0,                                  /* 0x090 CAN1 RX0 Interrupt                                                */
	0,                                  /* 0x094 CAN1 RX1 Interrupt                                                */
	0,                                  /* 0x098 CAN1 SCE Interrupt                                                */
	0,                                  /* 0x09C External Line[9:5] Interrupts                                     */
	0,                                  /* 0x0A0 TIM1 Break interrupt and TIM9 global interrupt                    */
	tim1_UI,                            /* 0x0A4 TIM1 Update Interrupt and TIM10 global interrupt                  */
	0,                                  /* 0x0A8 TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	0,		                            /* 0x0AC TIM1 Capture Compare Interrupt                                    */
	0,           			            /* 0x0B0 TIM2 global Interrupt                                             */
	0,                                  /* 0x0B4 TIM3 global Interrupt                                             */
	0,			                        /* 0x0B8 TIM4 global Interrupt                                             */
	0,                                  /* 0x0BC I2C1 Event Interrupt                                              */
	0,                                  /* 0x0C0 I2C1 Error Interrupt                                              */
	0,                                  /* 0x0C4 I2C2 Event Interrupt                                              */
	0,                                  /* 0x0C8 I2C2 Error Interrupt                                              */
	0,                                  /* 0x0CC SPI1 global Interrupt                                             */
	0,                                  /* 0x0D0 SPI2 global Interrupt                                             */
	0,                                  /* 0x0D4 USART1 global Interrupt                                           */
	UART2_Handler,                      /* 0x0D8 USART2 global Interrupt                                           */
	0,                                  /* 0x0DC USART3 global Interrupt                                           */
	0,                                  /* 0x0E0 External Line[15:10] Interrupts                                   */
	0,                                  /* 0x0E4 RTC Alarm (A and B) through EXTI Line Interrupt                   */
	0,                                  /* 0x0E8 USB OTG FS Wakeup through EXTI line interrupt                     */
	0,                                  /* 0x0EC TIM8 Break Interrupt and TIM12 global interrupt                   */
	0,                                  /* 0x0F0 TIM8 Update Interrupt and TIM13 global interrupt                  */
	0,                                  /* 0x0F4 TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
	0,                                  /* 0x0F8 TIM8 Capture Compare global interrupt                             */
	0,                                  /* 0x0FC DMA1 Stream7 Interrupt                                            */
	0,                                  /* 0x100 FSMC global Interrupt                                             */
	0,                                  /* 0x104 SDIO global Interrupt                                             */
	0,                                  /* 0x108 TIM5 global Interrupt                                             */
	0,                                  /* 0x10C SPI3 global Interrupt                                             */
	0,                                  /* 0x110 UART4 global Interrupt                                            */
	0,                                  /* 0x114 UART5 global Interrupt                                            */
	0,                                  /* 0x118 TIM6 global and DAC1&2 underrun error  interrupts                 */
	0,                                  /* 0x11C TIM7 global interrupt                                             */
	0,                                  /* 0x120 DMA2 Stream 0 global Interrupt                                    */
	0,                                  /* 0x124 DMA2 Stream 1 global Interrupt                                    */
	0,                                  /* 0x128 DMA2 Stream 2 global Interrupt                                    */
	0,                                  /* 0x12C DMA2 Stream 3 global Interrupt                                    */
	0,                                  /* 0x130 DMA2 Stream 4 global Interrupt                                    */
	0,                                  /* 0x134 Ethernet global Interrupt                                         */
	0,                                  /* 0x138 Ethernet Wakeup through EXTI line Interrupt                       */
	0,                                  /* 0x13C CAN2 TX Interrupt                                                 */
	0,                                  /* 0x140 CAN2 RX0 Interrupt                                                */
	0,                                  /* 0x144 CAN2 RX1 Interrupt                                                */
	0,                                  /* 0x148 CAN2 SCE Interrupt                                                */
	0,                                  /* 0x14C USB OTG FS global Interrupt                                       */
	0,                                  /* 0x150 DMA2 Stream 5 global interrupt                                    */
	0,                                  /* 0x154 DMA2 Stream 6 global interrupt                                    */
	0,                                  /* 0x158 DMA2 Stream 7 global interrupt                                    */
	0,                                  /* 0x15C USART6 global interrupt                                           */
	0,                                  /* 0x160 I2C3 event interrupt                                              */
	0,                                  /* 0x164 I2C3 error interrupt                                              */
	0,                                  /* 0x168 USB OTG HS End Point 1 Out global interrupt                       */
	0,                                  /* 0x16C USB OTG HS End Point 1 In global interrupt                        */
	0,                                  /* 0x170 USB OTG HS Wakeup through EXTI interrupt                          */
	0,                                  /* 0x174 USB OTG HS global interrupt                                       */
	0,                                  /* 0x178 DCMI global interrupt                                             */
	0,                                  /* 0x17C RNG global Interrupt                                              */
	0                                   /* 0x180 FPU global interrupt                                              */
};



/*************************************************
* Global Variables
*************************************************/


uint32_t period = 84-1; // Amplitude of created signal
uint32_t pwm_period = 84-1; // Period of pwm signal
//uint32_t pwm_period = 168-1; // Period of pwm signal 186 MHz/ 168 Sample = 1 MHz
uint32_t wave_period = 2000; // Period of created signal
uint16_t amplitude = 99;

const uint_fast8_t max_lut = 10000;
volatile uint_fast8_t sin_lut[10000];


uint8_t wave_type = SAWTOOTH_WAVE; // Default


/*************************************************
* Interrupt Handlers
*************************************************/

void Default_Handler(void)
{
	//TIM1->CR1 |= (0 << 1);
	//for (;;);  // Wait forever
	NMI_Handler();
}

void NMI_Handler(void)
{
	// Disable Timer 1 module (CEN, bit0)
	TIM1->CR1 &= (1 << 0);

	/* Enable GPIOD clock (AHB1ENR: bit 3) */
	// AHB1ENR: XXXX XXXX XXXX XXXX XXXX XXXX XXXX 1XXX
	RCC->AHB1ENR |= 0x00000008;

	GPIOD->MODER &= ~(0x3 << 28); // Reset bits 25:24 to clear old values
	GPIOD->MODER |=  (0x1 << 28); // Make PD14 Red Led output

	// Open led
	//GPIOD->ODR |= (1 << 12);

	//Blink led PD12
	while(1)
	{
		basic_delay(LEDDELAY);
		GPIOD->ODR ^= (1 << 14);  // Toggle LED
	}

}


/*************************************************
* TIM1 Update Interrupt and TIM10 global interrupt 
*************************************************/
void tim1_UI(void)
{

	//Reset timer interrupt
	TIM1->SR = (uint16_t)(0x0000);

	switch(wave_type) {
		case SIN_WAVE:
			sin_generate();
			break;			
		case SQUARE_WAVE:
			sqr_generate();
			break;
		case TRIANGLE_WAVE:
			triangle_generate();
			break;
		case SAWTOOTH_WAVE:
			sawtooth_generate();
			break; 

		//default: // Go to error function

	}	

}


/*************************************************
* On the fly calculator functions
*************************************************/

void make_luts(int _wave_period) {

    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
    //TIM1->DIER &= (1 << 0); //Channel 1 update interrupt
    TIM1->SR = (uint16_t)(0x0000);
    TIM1->CR1 &= (0 << 0);
    

	for(int i=0; i<_wave_period; i++) {
		sin_lut[i] = (_wave_period/2) * ( sin(2 * M_PI * i / _wave_period) + 1);
	}

    TIM1->CR1 |= (1 << 0);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    

}

void calculate_freq(uint32_t freq){
	// Base PWM period = 84 => 168/84 => 2MHz
	// Base wave_period = 100
	// Base freq = 20 Khz;
	const uint32_t base = 2000000; 
	//const uint32_t base = 1000000; 

	int _wave_period = base / freq;
	make_luts(_wave_period);
	wave_period = _wave_period;

}

void calculate_amp(uint16_t amp) {
	const uint16_t max_amp = 100;
	period = pwm_period * amp / max_amp;
}


void set_wave_type(int w_type)
{
    wave_type = w_type;
}

// A simple and not accurate delay function
// that will change the speed based on the optimization settings
void basic_delay(volatile uint32_t s)
{
	for(s; s>0; s--){
		__asm__("NOP");
	}
}


void init_signal_pin(void)
{
	// enable GPIOA clock
	RCC->AHB1ENR |= (1 << 0);

	GPIOA->MODER |= (0x2 << 16); // Set PA8 to alternate mode TIM1_CH1
	GPIOA->AFR[1] |= (0x1 << 0); // Choose Timer1 as Alternative Function for PA8 AF1
	GPIOA->OSPEEDR |= (0x3 << 16); //  PA8 to very high speed

}

void init_timer1(void)
{
	uint32_t duty = pwm_period/2;

	calculate_freq((uint32_t)(2000)); // 2Khz
	calculate_amp(100); // 100 / 100

	// enable TIM1 clock (bit0)
	RCC->APB2ENR |= (1 << 0);

	// Timer clock runs at ABP1 * 2
	//   since ABP1 is set to /4 of fCLK
	//   thus 168M/4 * 2 = 84Mhz
	// set prescaler to 83999
	//   it will increment counter every prescalar cycles
	// fCK_PSC / (PSC[15:0] + 1)
	// 84 Mhz / 8399 + 1 = 10 khz timer clock speed
	// 84 Mhz / 83 + 1 = 1 Mhz timer clock speed
	//TIM4->PSC = 83;

	// set prescaler to 167
	//   it will increment counter every prescalar cycles
	// fCK_PSC / (PSC[15:0] + 1)
	// 168 Mhz / 167 + 1 = 1 Mhz timer clock speed
	TIM1->PSC = 0;

	// set period
	TIM1->ARR = pwm_period;

	// set duty cycle on channel 1
	TIM1->CCR1 = duty;



	// enable channel 1 in capture/compare register
	// set oc1 mode as pwm (0b110 or 0x6 in bits 6-4)
	TIM1->CCMR1 |= (0x6 << 4);
	// enable oc1 preload bit 3
	TIM1->CCMR1 |= (1 << 3);

	TIM1->CCER |= (1 << 0); // enable capture/compare ch1 output

	TIM1->BDTR |= (1 << 15); // enable main output

	//TIM1->DIER |= (1 << 1); //Channel 1 capture compare
	TIM1->DIER |= (1 << 0); //Channel 1 update interrupt

	// priority TIM1 IRQ from NVIC
	//  TIM1 Update Interrupt and TIM10 global interrupt  
	NVIC->IP[TIM1_UP_TIM10_IRQn] = 0x13; // Priority level 

	// TIM1_CC_IRQn
	// TIM1 Capture Compare Interrupt  
	//NVIC->IP[TIM1_CC_IRQn] = 0x12; // Priority level 

	// enable TIM1 IRQ from NVIC
	// TIM1 Update Interrupt and TIM10 global interrupt  
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	// TIM1 Capture Compare Interrupt 
	// NVIC_EnableIRQ(TIM1_CC_IRQn);

	// Enable Timer 1 module (CEN, bit0)
	TIM1->CR1 |= (1 << 0);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	/* set system clock to 168 Mhz */
	set_sysclk_to_168();


	// For white noise enable Random Number Generator
	//RCC->AHB2ENR |= 0x40;     // Enable Clock of RNG 
    //RNG->CR |= (1 << 2);	  // RNG enable no interrupt Page 769

	//NVI_Handler();

    init_signal_pin();

    Init_Systick(21000, 1);
    //Send_GroupId(3);
    Init_Uart();
    Send_Help();

    init_timer1();
   
 

	while(1)
	{
		// Do nothing. let timer handler do its magic
	}

	return 0;
}


/*************************************************
* Function Generators
*************************************************/


/*************************************************
* Sin wave Generator
*************************************************/
void sin_generate(void)
{
	static uint32_t t = 0;
	uint32_t f = wave_period-1;

	if (t >= f)
	{
		t = 0;
	} else
	{
		++t;
	}


	// set new duty cycle
	TIM1->CCR1 = (uint16_t)( (sin_lut[t] * period) / f );
}

/*************************************************
* Square wave Generator
*************************************************/
void sqr_generate(void)
{
	static uint32_t t = 0;
	uint32_t f = wave_period-1;
	
	if (t >= f)
	{
		t = 0;
	} else
	{
		++t;
	}

	// Generate 1 with pwm
	if (t <= f/2) {
		TIM1->CCR1 = (uint16_t)((period + 1));
	}
	// Generate 0 with pwm
	else {
		TIM1->CCR1 = (uint16_t)(0);		
	}
}

/*************************************************
* Sawtooth Generator
*************************************************/
void sawtooth_generate(void)
{
	static uint32_t t = 0;
	
	if (t >= wave_period)
	{
		t = 0;
	} else
	{
		++t;
	}

	TIM1->CCR1 = (uint16_t)(t * period / (wave_period) );
	
}


/*************************************************
* Triangle Generator
*************************************************/
void triangle_generate(void)
{
	static uint32_t t = 0;
	uint32_t f = wave_period-1;
	

	if (t <= f/2) {
		TIM1->CCR1 = (uint16_t)(t* 2 * period / wave_period );
		++t;
	}
	else {
		TIM1->CCR1 = (uint16_t)(abs(f - t) * 2 * period / (wave_period) );
		++t;
		if(t >= f) t=0;
	}

	
}

/*************************************************
* White Noise Generator
*************************************************/
void white_noise_generate(void)
{

	TIM4->CCR2 = (uint16_t)( period / 2);
	return;

	uint32_t f;
	uint16_t random_number;
	float_t big_num = (uint16_t)(0xFFFF);

	f = period;
	
	while (!(RNG->SR&1));    // Wait for usable Random Number

	// Scale 32bit random number to our period
	// Get our period to max number ratio
	// random_number = (RNG->DR >> 16) * (period / 0xFFFF);
	random_number =  (RNG->DR >> 16) * (float)( f / big_num );

	TIM4->CCR2 = (uint16_t)( random_number / 2);
	
}