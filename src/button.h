#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x_gpio.h"

void buttonInit(void);
int buttonPressed(void);
#ifdef HW_DISCOVERY
    #define button GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
    #define button GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#endif

#endif /* __BUTTON_H */
