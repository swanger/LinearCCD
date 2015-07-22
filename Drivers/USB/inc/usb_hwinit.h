// * ===============================================================================
// * File         : usb_hwinit.h
// * Author       : Wang Hairun
// * Created Date : 07/13/2015 | 10:22:20 AM | Monday,July
// * Description  : 
// * ===============================================================================
#ifndef USB_HWINIT_H
#define USB_HWINIT_H

#include "stm32f30x_conf.h"
#include "usb_type.h"

extern void USBInit(void);

extern void Get_SerialNum(void);
extern void USB_Cable_Config (FunctionalState NewState);
extern void Enter_LowPowerMode(void);
extern void Leave_LowPowerMode(void);
extern uint32_t CDC_Send_DATA (uint8_t *ptrBuffer, uint8_t Send_length);
extern uint32_t CDC_Receive_DATA(void);



#endif

