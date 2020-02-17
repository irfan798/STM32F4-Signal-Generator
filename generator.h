/**
  ******************************************************************************
  * @file    generator.h 
  * @author  Ali Firat ARI
  * @author  Irfan Bilaloglu
  * @author  Sencer Altintop
  * @version V1.0.1
  * @brief   Header for generator.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Gebze Technical University</center></h2>
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GENERATOR_H
#define __GENERATOR_H

/*************************************************
* Constants
*************************************************/
#define SIN_WAVE 1
#define SQUARE_WAVE 2
#define TRIANGLE_WAVE 3
#define SAWTOOTH_WAVE 4
#define NOISE_WAVE 5

#define LEDDELAY	1000000

/*************************************************
* function declarations
*************************************************/
void Default_Handler(void);
void make_luts(int _wave_period);
void set_wave_type(int w_type);
void calculate_freq(uint32_t freq);
void sin_generate(void);
void sqr_generate(void);
void sawtooth_generate(void);
void triangle_generate(void);
void white_noise_generate(void);
void tim1_CCI(void);
void tim1_UI(void);
void basic_delay(volatile uint32_t s);
void NMI_Handler(void);

#endif