#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "button.h"

void buttonInit(void) {
    #ifdef HW_DISCOVERY
	    /* GPIOA clock enable */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	    /* GPIOA Configuration:Pin0 as input floating */
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	    GPIO_Init(GPIOA, &GPIO_InitStructure);
    #elif ( HW_LINSEN_V0_1 || HW_LINSEN_V0_2 )
	    /* GPIOB clock enable */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	    /* GPIOB Configuration:Pin8 as input floating */
	    GPIO_InitTypeDef GPIO_InitStructure;
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	    GPIO_Init(GPIOB, &GPIO_InitStructure);
    #endif
}

int buttonPressed(void) {
	static int pressed = 0;

	if (button) {
		if (pressed) return 0;
		else {
			pressed = 1;
			return 1;
		}
	} else {
		pressed = 0;
		return 0;
	}
}

