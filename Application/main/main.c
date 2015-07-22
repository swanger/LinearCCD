// * ===============================================================================
// * File         : main.c
// * Author       : Wang Hairun
// * Created Date : 07/08/2015 | 08:59:07 AM | Wednesday,July
// * Description  : 
// * ===============================================================================
#include "stm32f30x_conf.h"
#include "stm32f30x.h"
#include "debug.h"
#include "ILX511.h"

#include "usb_hwinit.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

__IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;

uint32_t packet_sent=1;
uint32_t packet_receive=1;

extern __IO uint32_t bDeviceState; /* USB device status */

extern uint16_t data[2086];

int main()
{
     __disable_irq();


    // ------------------------------------------------------------------ 
    // Desc: System Clk Config
    //
    //  HCLK = PCLK2 = SYSCLK = 72M 
    //  PCLK1 = 36M
    // ------------------------------------------------------------------ 
     SystemInit();
     // SysTick_Config(SystemCoreClock / 1000);

#if 0 
     // ------------------------------------------------------------------ 
     // Desc: CLK OUTPUT for measure
     // ------------------------------------------------------------------ 

     //PA8 alternate mode
     GPIO_InitTypeDef        GPIO_InitStructure;

     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
     GPIO_Init(GPIOA, &GPIO_InitStructure);

     GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_0);    

     RCC_MCOConfig(RCC_MCOSource_PLLCLK,RCC_MCOPrescaler_1);
     // while(1);
#endif


     // ------------------------------------------------------------------ 
     // Desc: Debug 
     // ------------------------------------------------------------------ 
     DebugInit();
     // d_printf("UART debug Init OK\r\n");

     // ------------------------------------------------------------------ 
     // Desc: ILX511 CLK ON
     // ------------------------------------------------------------------ 
#if 1
     {
         GPIO_InitTypeDef        GPIO_InitStructure;
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
         GPIO_Init(GPIOB, &GPIO_InitStructure);
     }
#endif

#if 0
     ILX_Init();
     __enable_irq();
     ILX_CLKOn();
     delay_nms(50);//CCD power up

     ConvertOn();
#endif


#if 0

     d_printf("\r\n-------------------\r\n");
     {
         uint32_t i;
         for(i=32;i<2048+32;i++)
             d_printf("0x%04X ",data[i]);

     }
#endif


#if 0
    USBInit();
#endif

#if 0
    while (1)
    {
        if (bDeviceState == CONFIGURED)
        {
            CDC_Receive_DATA();
            if (Receive_length  != 0)
            {
                if (packet_sent == 1)
                    CDC_Send_DATA ((unsigned char*)Receive_Buffer,Receive_length);
                Receive_length = 0;
            }
        }
    }
#endif

     while(1);

    return 0;
}






