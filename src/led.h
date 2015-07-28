#ifndef __LED_H
#define __LED_H

#include "stm32f10x_gpio.h"

void ledsInit(void);

#ifdef USE_STM32_DISCOVERY
    #define pinPC6On() GPIO_WriteBit(GPIOC, GPIO_Pin_6, Bit_SET)
    #define pinPC6Off() GPIO_WriteBit(GPIOC, GPIO_Pin_6, Bit_RESET)
    #define pinPC6Toggle() GPIO_WriteBit(GPIOC, GPIO_Pin_6, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_6)))

    #define pinPC7On() GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_SET)
    #define pinPC7Off() GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit_RESET)
    #define pinPC7Toggle() GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)))

	/* on discovery hardware the red LED is a blue one */
	#define ledRedOff ledBlueOff
	#define ledRedOn ledBlueOn
	#define ledRedToggle ledBlueToggle
    #define ledBlueOn() GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET)
    #define ledBlueOff() GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET)
    #define ledBlueToggle() GPIO_WriteBit(GPIOC, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8)))

    #define ledGreenOn() GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET)
    #define ledGreenOff() GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET)
    #define ledGreenToggle() GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9)))
#elif defined HW_LINSEN_V0_1
	/* on linSen hardware the blue LED is a red one */
	#define ledBlueOff ledRedOff
	#define ledBlueOn ledRedOn
	#define ledBlueToggle ledRedToggle
    #define ledRedOn() GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET)
    #define ledRedOff() GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET)
    #define ledRedToggle() GPIO_WriteBit(GPIOA, GPIO_Pin_3, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3)))

    #define ledGreenOn() GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET)
    #define ledGreenOff() GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET)
    #define ledGreenToggle() GPIO_WriteBit(GPIOA, GPIO_Pin_2, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2)))
#elif defined HW_LINSEN_V0_2
	/* on linSen hardware the blue LED is a red one */
	#define ledBlueOff ledRedOff
	#define ledBlueOn ledRedOn
	#define ledBlueToggle ledRedToggle
    #define ledRedOn() GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET)
    #define ledRedOff() GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET)
    #define ledRedToggle() GPIO_WriteBit(GPIOA, GPIO_Pin_2, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2)))

    #define ledGreenOn() GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET)
    #define ledGreenOff() GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET)
    #define ledGreenToggle() GPIO_WriteBit(GPIOA, GPIO_Pin_1, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1)))
#endif

#endif /* __LED_H */
