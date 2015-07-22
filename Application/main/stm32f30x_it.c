/**
  ******************************************************************************
  * @file    ADC/ADC_AnalogWatchdog/stm32f30x_it.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    31-October-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_it.h"
#include "debug.h"

/** @addtogroup STM32F30x_StdPeriph_Examples
  * @{
  */

/** @addtogroup AWDG_Mode
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    // GPIOB->ODR ^= 0x0200;
}

/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/******************************************************************************/
/*                 STM32F30x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f30x.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles ADC1 global interrupts requests.
  * @param  None
  * @retval None
  */
void ADC1_IRQHandler(void)
{
#if 0
    int i;
    if(ADC_GetITStatus(ADC1,ADC_IT_EOC) != RESET)
    {
        ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
        GPIOB->ODR |= 0x0200;
        delay_nus(1);
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
    }
#endif
}

void DMA1_Channel1_IRQHandler(void)
{

#if 0
    int i;
    if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
        GPIOB->ODR |= 0x0200;
        delay_nus(1);
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);

    }
#endif
}

void TIM2_IRQHandler(void)
{
#if 0
    //CC1 Interrupt
    if(TIM_GetITStatus(TIM2,TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
        // clk_cnt++;
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
        GPIOB->ODR |= 0x0200;
        delay_nus(1);
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
    }
#endif
#if 0
    //CC1 Interrupt
    if(TIM_GetITStatus(TIM2,TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM2,TIM_IT_CC2);
        // clk_cnt++;
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
        GPIOB->ODR |= 0x0200;
        delay_nus(1);
        GPIOB->ODR &= ~0x0200;
        delay_nus(1);
    }
#endif


#if 0

    if(TIM2->SR & TIM_IT_CC1)
    {
        TIM2->SR = (uint16_t)~TIM_IT_CC1;
        GPIOB->ODR |= 0x0200;
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        GPIOB->ODR &= ~0x0200;
    }
#endif
}



void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}

void USBWakeUp_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
