// * ===============================================================================
// * File         : ILX511.c
// * Author       : Wang Hairun
// * Created Date : 07/08/2015 | 14:13:17 PM | Wednesday,July
// * Description  : 
// * ===============================================================================
#include "ILX511.h"
#include "stm32f30x_conf.h"
#include "stm32f30x.h"
#include "debug.h"


#define ADC1_JDR_ADDRESS     0x50000080
#define ADC1_DR_ADDRESS      0x50000040


uint16_t data[2086]={0x0555,0x0555,0x0555};

void ILX_Init(void)
{
    GPIO_InitTypeDef            GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    DMA_InitTypeDef             DMA_InitStructure;

    uint32_t TimerPeriod = 0;
    uint32_t Channel1Pulse = 0;


    ADC_InitTypeDef             ADC_InitStructure;
    ADC_InjectedInitTypeDef     ADC_InjectedInitStructure;
    ADC_CommonInitTypeDef       ADC_CommonInitStructure;
    __IO uint32_t calibration_value = 0;
    
    // NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


     // ------------------------------------------------------------------ 
    // Desc: ROG Config
    // ------------------------------------------------------------------ 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_2);
                           
   // ------------------------------------------------------------------ 
    // Desc: TIM2 PWM 1MHz
    // ------------------------------------------------------------------ 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);    

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

    TimerPeriod = 72*2-1;
    Channel1Pulse = (uint32_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ForcedOC1Config(TIM2,TIM_ForcedAction_InActive);

    // TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);


    // ------------------------------------------------------------------ 
    // Desc: ADC12 Config
    // ------------------------------------------------------------------ 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

    ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  
    delay_nms(10);
  
    ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) != RESET );
    calibration_value = ADC_GetCalibrationValue(ADC1);
     
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;             
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;                  
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
    ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_3;
#if 0
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
#else
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_FallingEdge;
#endif
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
    ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
    ADC_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5);

    // ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
    ADC_DMACmd(ADC1,ENABLE);
    ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);

    // ------------------------------------------------------------------ 
    // Desc: DMA Config
    // ------------------------------------------------------------------ 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 2086;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    // DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
}

void ILX_CLKOn(void)
{
    TIM_OCInitTypeDef           TIM_OCInitStructure;

    uint32_t TimerPeriod = 0;
    uint32_t Channel1Pulse = 0;


    TimerPeriod = 72*2-1;
    Channel1Pulse = (uint32_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);


#if 0
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
#else
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
#endif
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    // TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
}


void ILX_CLKOff(void)
{
    TIM_CtrlPWMOutputs(TIM2, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
    // TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);

    TIM_ForcedOC1Config(TIM2,TIM_ForcedAction_InActive);
}

void ConvertOn(void)
{
    ILX_CLKOff();
    delay_nus(4);
    ROGDown();
    delay_nus(6);
    ROGUp();
    delay_nus(4);
    ILX_CLKOn();

    delay_nms(12);//integration time

    ILX_CLKOff();
    delay_nus(4);
    ROGDown();
    delay_nus(6);
    ROGUp();
    delay_nus(4);
    ILX_CLKOn();

    ADC_Cmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));

    ADC_StartConversion(ADC1);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
    DMA_ClearFlag(DMA1_FLAG_TC1);


    ADC_StopConversion(ADC1);
    ILX_CLKOff();



#if 0
    while (1)
    {
        uint32_t ADC1ConvertedValue = 0;
        double ADC1ConvertedVoltage = 0;
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET);

        ADC1ConvertedValue = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedSequence_1);

        ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
        d_printf("%fmV\r\n",ADC1ConvertedVoltage);
    }
#endif

#if 0
    {
        uint32_t i = 0;
        while (i<2086)
        {
            uint32_t ADC1ConvertedValue = 0;
            double ADC1ConvertedVoltage = 0;
            while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

            // ADC1ConvertedValue = ADC_GetConversionValue(ADC1);
            data[i] = ADC_GetConversionValue(ADC1);

            // ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
            // d_printf("%fmV\r\n",ADC1ConvertedVoltage);
            i++;
        }
    }

#endif
#if 0
    while (1)
    {
        uint32_t ADC1ConvertedValue = 0;
        double ADC1ConvertedVoltage = 0;
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

        ADC1ConvertedValue = ADC_GetConversionValue(ADC1);

        ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
        d_printf("%fmV\r\n",ADC1ConvertedVoltage);
    }

#endif
}

void ConvertOff(void)
{
#if 0
    ADC_StopInjectedConversion(ADC1);
#else
    ADC_StopConversion(ADC1);
#endif
    ADC_Cmd(ADC1, DISABLE);
}



void ROGDown(void)
{
    GPIO_ResetBits(GPIOB,GPIO_Pin_2);
}

void ROGUp(void)
{
    GPIO_SetBits(GPIOB,GPIO_Pin_2);
}


