/**
  ******************************************************************************
  * @file    terminal.h 
  * @author  Ali Firat ARI
  * @author  Irfan Bilaloglu
  * @author  Sencer Altintop
  * @version V1.0.1
  * @brief   Header for terminal.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Gebze Technical University</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TERMINAL_H
#define __TERMINAL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "math.h"

/* Exported functions ------------------------------------------------------- */
void Systick_Handler(void);

void Init_Systick(uint32_t s,uint8_t cen);
void delay_ms(volatile uint32_t s);

void UART2_Handler(void);
void __Clear(void);

void Init_Uart(void);
void UART_SendData(const char* str);
void Send_Info(void);
void Send_Help(void);

void Send_GroupId(uint8_t id);

int powInt(int x, int y);
int parseInt(char* chars);

#endif /* __TERMINAL_H */