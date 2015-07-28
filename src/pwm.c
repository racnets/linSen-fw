#include <stdint.h>

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "pwm.h"

#ifdef USE_USE_STM32_DISCOVERY
#define TIMx					TIM1
#define RCC_APByPeriph_TIMx			RCC_APB2Periph_TIM3
#define RCC_APByPeriphClockCmd		RCC_APB2PeriphClockCmd
#define RCC_APB2Periph_GPIOz		RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO
#define GPIOz						GPIOC
#define GPIO_Pin					GPIO_Pin_8 | GPIO_Pin_9
#define GPIO_PinRemap()				GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE)
#define PCLK_Frequency				PCLK1_Frequency
#define TIM_OCmInit(a,b)			TIM_OC3Init(a,b)
#define	TIM_OCmPreloadConfig(a,b)	TIM_OC3PreloadConfig(a, b);
#define TIMx_IRQn					TIM3_IRQn
#define CCR							CCR3
#define TIMx_IRQHandler				TIM3_IRQHandler
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
/* TIM1-C4 via AF PA11(USBDM) */
/* later maybe also TIM1_2N via Remap AF PB0(SL1) */
#define TIMx						TIM1
#define RCC_APByPeriph_TIMx			RCC_APB2Periph_TIM1
#define RCC_APByPeriphClockCmd		RCC_APB2PeriphClockCmd
#define RCC_APByPeriph_GPIOz		RCC_APB2Periph_GPIOA
#define GPIOz						GPIOA
#define GPIO_PinN					GPIO_Pin_11
#define	GPIO_PinRemap()
#define PCLK_Frequency				PCLK2_Frequency
#define TIM_OCmInit(a,b)			TIM_OC4Init(a,b)
#define	TIM_OCmPreloadConfig(a,b)	TIM_OC4PreloadConfig(a, b);
#define TIMx_IRQn					TIM1_UP_IRQn
#define CCR							CCR4
#define TIMx_IRQHandler				TIM1_UP_IRQHandler
#endif

void (*updateFunction)(void);
/*
 * returns realizable frequency
 */
int pwmInit(uint16_t frequency, uint16_t resolution) {
	static int running = 0;

	/* stop running modules */
	if ( running ) {
		/* stop timer */
		TIM_Cmd(TIMx, DISABLE);
	}
	running = 1;

    /***************************************************/
	/* clocks configuration */
	/* TIMER clock enable */
	RCC_APByPeriphClockCmd(RCC_APByPeriph_TIMx, ENABLE);
	/* enable GPIO (maybe AF also */
	RCC_APByPeriphClockCmd(RCC_APByPeriph_GPIOz, ENABLE);

    /***************************************************/
	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_PinN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOz, &GPIO_InitStructure);
	/* potential remap  */
	GPIO_PinRemap();

    RCC_ClocksTypeDef  rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
    uint32_t pclk = rcc_clocks.PCLK_Frequency;
    uint16_t prescaler = (pclk / frequency) >> resolution;
    uint16_t period = (1<<resolution) - 1;

   	/* TIMER configuration */
	/* pwm clock timer configuration */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	/* Output Compare Timing Mode configuration: Channel4 */
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCmInit(TIMx, &TIM_OCInitStructure);

	/* OC preload enable */
	TIM_OCmPreloadConfig(TIMx, TIM_OCPreload_Enable);

	/* TIM auto-reload preload enable */
	TIM_ARRPreloadConfig(TIMx, ENABLE);

	/* TIM enable counter */
	TIM_SetCounter(TIMx, 0);
	TIM_Cmd(TIMx, ENABLE);

	return (SystemCoreClock >> resolution) / prescaler;
}

void pwmInitIRQ(void (*uf)(void)) {
	updateFunction = uf;

	TIM_Cmd(TIMx, DISABLE);

    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    /* enable the TIM global interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIMx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* enable TIM update interrupt */
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIMx, ENABLE);
}
/*
 * interrupt service routines
 */
void TIMx_IRQHandler(void) {
	if (TIM_GetITStatus(TIMx, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
		(*updateFunction)();
	}
}

void setPwm(uint16_t channel, uint16_t value) {
	if (channel & 0x0001) TIMx->CCR = value;
}
