// * ===============================================================================
// * File         : debug.c
// * Author       : Wang Hairun
// * Created Date : 07/08/2015 | 12:35:21 PM | Wednesday,July
// * Description  : 
// * ===============================================================================
#include "stm32f30x_conf.h"
#include "stm32f30x.h"
#include <stdarg.h>
#include <stdio.h>


// *******************************************************************
// Desc: 
//
// *******************************************************************
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// *******************************************************************
// Desc: Prototype
//
// *******************************************************************

// *******************************************************************
// Desc: Definitions
//
// *******************************************************************
void DebugInit(void)
{
    // ------------------------------------------------------------------ 
    // Desc: UART for Debug 
    //       PA2   TX
    //       PA3   RX
    // ------------------------------------------------------------------ 
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);  

    USART_InitStructure.USART_BaudRate = 921600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void d_printf(const char *fmt, ...)
{
    va_list	arguments;
    va_start(arguments, fmt);
	vprintf(fmt, arguments);

    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {}
	va_end(arguments);

}


PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
  {}

  return ch;
}

void delay_1us(void)
{
   __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
   __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
   __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
   __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
   __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
   __nop();__nop();__nop();__nop();__nop();__nop();__nop();
}

void delay_1ms(void)
{
    uint32_t n = 12000;
    while(n--);
}

void delay_nus(uint32_t n)
{
    while(n--)
    {
        delay_1us();
    }
}

void delay_nms(uint32_t n)
{
    while(n--)
    {
        delay_1ms();
    }
}

