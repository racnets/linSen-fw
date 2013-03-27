#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "led.h"

void ledsInit(void) {
    #ifdef HW_DISCOVERY
    	/* GPIOC clock enable */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	    /* GPIOC Configuration:Pin6, Pin7, Pin8 and 9 as alternate function push-pull */
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	    GPIO_Init(GPIOC, &GPIO_InitStructure);
    #elif HW_LINSEN_V0_1
    	/* GPIOA clock enable */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	    /* GPIOC Configuration:Pin2, Pin3 as alternate function push-pull */
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	    GPIO_Init(GPIOA, &GPIO_InitStructure);
    #elif HW_LINSEN_V0_2
    	/* GPIOA clock enable */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	    /* GPIOC Configuration:Pin1, Pin2 as alternate function push-pull */
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	    GPIO_Init(GPIOA, &GPIO_InitStructure);
    #endif
}
