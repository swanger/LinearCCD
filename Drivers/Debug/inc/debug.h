// * ===============================================================================
// * File         : debug.h
// * Author       : Wang Hairun
// * Created Date : 07/08/2015 | 12:42:09 PM | Wednesday,July
// * Description  : 
// * ===============================================================================
#ifndef     DEBUG_H
#define     DEBUG_H

extern void DebugInit(void);
extern void d_printf(const char *fmt, ...);
extern void delay_nus(uint32_t n);
extern void delay_nms(uint32_t n);
extern void delay_1us(void);
extern void delay_1ms(void);



#endif
