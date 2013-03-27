/* startup file for STM32 */
#include "stm32f10x.h"

/* processor specific values provided by stm32.ld */
extern unsigned long _estack;
extern unsigned long _sidata;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;

/* functions provided by user code */
extern int main(void);

/* define WEAK so that other code can override interupt routines */
#define WEAK __attribute__ ((weak))

/* functions defined in this file */
void default_Handler();
void Reset_Handler(void);
void WEAK NMI_Handler(void);												
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK UsageFault_Handler(void);
void WEAK SVC_Handler(void);
void WEAK DebugMon_Handler(void);
void WEAK PendSV_Handler(void);
void WEAK SysTick_Handler(void);

/* External Interrupts */
void WEAK WWDG_IRQHandler(void);
void WEAK PVD_IRQHandler(void);
void WEAK TAMPER_IRQHandler(void);
void WEAK RTC_IRQHandler(void);
void WEAK FLASH_IRQHandler(void);
void WEAK RCC_IRQHandler(void);
void WEAK EXTI0_IRQHandler(void);
void WEAK EXTI1_IRQHandler(void);
void WEAK EXTI2_IRQHandler(void);
void WEAK EXTI3_IRQHandler(void);
void WEAK EXTI4_IRQHandler(void);
void WEAK DMA1_Channel1_IRQHandler(void);
void WEAK DMA1_Channel2_IRQHandler(void);
void WEAK DMA1_Channel3_IRQHandler(void);
void WEAK DMA1_Channel4_IRQHandler(void);
void WEAK DMA1_Channel5_IRQHandler(void);
void WEAK DMA1_Channel6_IRQHandler(void);
void WEAK DMA1_Channel7_IRQHandler(void);
void WEAK ADC1_IRQHandler(void);
void WEAK EXTI9_5_IRQHandler(void);
void WEAK TIM1_BRK_TIM15_IRQHandler(void);
void WEAK TIM1_UP_TIM16_IRQHandler(void);
void WEAK TIM1_TRG_COM_TIM17_IRQHandler(void);
void WEAK TIM1_CC_IRQHandler(void);
void WEAK TIM2_IRQHandler(void);
void WEAK TIM3_IRQHandler(void);
void WEAK TIM4_IRQHandler(void);
void WEAK I2C1_EV_IRQHandler(void);
void WEAK I2C1_ER_IRQHandler(void);
void WEAK I2C2_EV_IRQHandler(void);
void WEAK I2C2_ER_IRQHandler(void);
void WEAK SPI1_IRQHandler(void);
void WEAK SPI2_IRQHandler(void);
void WEAK USART1_IRQHandler(void);
void WEAK USART2_IRQHandler(void);
void WEAK USART3_IRQHandler(void);
void WEAK EXTI15_10_IRQHandler(void);
void WEAK RTCAlarm_IRQHandler(void);
void WEAK CEC_IRQHandler(void);
void WEAK TIM6_DAC_IRQHandler(void);
void WEAK TIM7_IRQHandler(void);


/* interupt service routine vectors */
__attribute__ ((section(".isr_vector")))
const void * isrVector[] = {
	(void *const)&_estack,			/* Top of Stack */
	Reset_Handler,					/* Reset Handler */
	NMI_Handler,					/* NMI Handler */
	HardFault_Handler,				/* Hard Fault Handler */
	MemManage_Handler,				/* MPU Fault Handler */
	BusFault_Handler,				/* Bus Fault Handler */
	UsageFault_Handler,				/* Usage Fault Handler */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	SVC_Handler,					/* SVCall Handler */
	DebugMon_Handler,				/* Debug Monitor Handler */
	0,								/* Reserved */
	PendSV_Handler,					/* PendSV Handler */
	SysTick_Handler,				/* SysTick Handler */

	/* External Interrupts */
	WWDG_IRQHandler,				/* Window Watchdog */
	PVD_IRQHandler,					/* PVD through EXTI Line detect */
	TAMPER_IRQHandler,				/* Tamper */
	RTC_IRQHandler,					/* RTC */
	FLASH_IRQHandler,				/* Flash */
	RCC_IRQHandler,					/* RCC */
	EXTI0_IRQHandler,				/* EXTI Line 0 */
	EXTI1_IRQHandler,				/* EXTI Line 1 */
	EXTI2_IRQHandler,				/* EXTI Line 2 */
	EXTI3_IRQHandler,				/* EXTI Line 3 */
	EXTI4_IRQHandler,				/* EXTI Line 4 */
	DMA1_Channel1_IRQHandler,		/* DMA1 Channel 1 */
	DMA1_Channel2_IRQHandler,		/* DMA1 Channel 2 */
	DMA1_Channel3_IRQHandler,		/* DMA1 Channel 3 */
	DMA1_Channel4_IRQHandler,		/* DMA1 Channel 4 */
	DMA1_Channel5_IRQHandler,		/* DMA1 Channel 5 */
	DMA1_Channel6_IRQHandler,		/* DMA1 Channel 6 */
	DMA1_Channel7_IRQHandler,		/* DMA1 Channel 7 */
	ADC1_IRQHandler,				/* ADC1 */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	EXTI9_5_IRQHandler,				/* EXTI Line 9..5 */
	TIM1_BRK_TIM15_IRQHandler,		/* TIM1 Break and TIM15 */
	TIM1_UP_TIM16_IRQHandler,		/* TIM1 Update and TIM16 */
	TIM1_TRG_COM_TIM17_IRQHandler,	/* TIM1 Trigger and Commutation and TIM17 */
	TIM1_CC_IRQHandler,				/* TIM1 Capture Compare */
	TIM2_IRQHandler,				/* TIM2 */
	TIM3_IRQHandler,				/* TIM3 */
	TIM4_IRQHandler,				/* TIM4 */
	I2C1_EV_IRQHandler,				/* I2C1 Event */
	I2C1_ER_IRQHandler,				/* I2C1 Error */
	I2C2_EV_IRQHandler,				/* I2C2 Event */
	I2C2_ER_IRQHandler,				/* I2C2 Error */
	SPI1_IRQHandler,				/* SPI1 */
	SPI2_IRQHandler,				/* SPI2 */
	USART1_IRQHandler,				/* USART1 */
	USART2_IRQHandler,				/* USART2 */
	USART3_IRQHandler,				/* USART3 */
	EXTI15_10_IRQHandler,			/* EXTI Line 15..10 */
	RTCAlarm_IRQHandler,			/* RTC Alarm through EXTI Line */
	CEC_IRQHandler,					/* HDMI-CEC */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	0,								/* Reserved */
	TIM6_DAC_IRQHandler,			/* TIM6 and DAC underrun */
	TIM7_IRQHandler					/* TIM7 */
};

void _init_data() {
	/* copy the data segment into ram */
	unsigned long *src = &_sidata;
	unsigned long *dst = &_sdata;

	if (src != dst)
		while(dst < &_edata)
			*(dst++) = *(src++);

	/* zero the bss segment */
	dst = &_sbss;
	while(dst < &_ebss)
		*(dst++) = 0;
}

void Reset_Handler(void) {
	/* copy the data segment into ram */
	_init_data();
	/* inits system - mainly clocks */
	/* for more infos: libs/CMSIS/CM3/DeviceSupport/ST/STM3/STM32F10x/system_stm32f10x.c */
	SystemInit();
	/* call main() as starting point for user code */
	main();
}

void default_Handler(void) {
	return ;
}

#pragma weak MMI_Handler = default_Handler
#pragma weak HardFault_Handler = default_Handler
#pragma weak MemManage_Handler = default_Handler
#pragma weak BusFault_Handler = default_Handler
#pragma weak UsageFault_Handler = default_Handler
#pragma weak SVC_Handler = default_Handler
#pragma weak DebugMon_Handler = default_Handler
#pragma weak PendSV_Handler = default_Handler
//#pragma weak SysTick_Handler = default_Handler

/* External Interrupts */
#pragma weak WWDG_IRQHandler = default_Handler
#pragma weak PVD_IRQHandler = default_Handler
#pragma weak TAMPER_IRQHandler = default_Handler
#pragma weak RTC_IRQHandler = default_Handler
#pragma weak FLASH_IRQHandler = default_Handler
#pragma weak RCC_IRQHandler = default_Handler
#pragma weak EXTI0_IRQHandler = default_Handler
#pragma weak EXTI1_IRQHandler = default_Handler
#pragma weak EXTI2_IRQHandler = default_Handler
#pragma weak EXTI3_IRQHandler = default_Handler
#pragma weak EXTI4_IRQHandler = default_Handler
#pragma weak DMA1_Channel1_IRQHandler = default_Handler
#pragma weak DMA1_Channel2_IRQHandler = default_Handler
#pragma weak DMA1_Channel3_IRQHandler = default_Handler
#pragma weak DMA1_Channel4_IRQHandler = default_Handler
#pragma weak DMA1_Channel5_IRQHandler = default_Handler
#pragma weak DMA1_Channel6_IRQHandler = default_Handler
#pragma weak DMA1_Channel7_IRQHandler = default_Handler
#pragma weak ADC1_IRQHandler = default_Handler
#pragma weak EXTI9_5_IRQHandler = default_Handler
#pragma weak TIM1_BRK_TIM15_IRQHandler = default_Handler
#pragma weak TIM1_UP_TIM16_IRQHandler = default_Handler
#pragma weak TIM1_TRG_COM_TIM17_IRQHandler = default_Handler
#pragma weak TIM1_CC_IRQHandler = default_Handler
#pragma weak TIM2_IRQHandler = default_Handler
#pragma weak TIM3_IRQHandler = default_Handler
#pragma weak TIM4_IRQHandler = default_Handler
#pragma weak I2C1_EV_IRQHandler = default_Handler
#pragma weak I2C1_ER_IRQHandler = default_Handler
#pragma weak I2C2_EV_IRQHandler = default_Handler
#pragma weak I2C2_ER_IRQHandler = default_Handler
#pragma weak SPI1_IRQHandler = default_Handler
#pragma weak SPI2_IRQHandler = default_Handler
#pragma weak USART1_IRQHandler = default_Handler
#pragma weak USART2_IRQHandler = default_Handler
#pragma weak USART3_IRQHandler = default_Handler
#pragma weak EXTI15_10_IRQHandler = default_Handler
#pragma weak RTCAlarm_IRQHandler = default_Handler
#pragma weak CEC_IRQHandler = default_Handler
#pragma weak TIM6_DAC_IRQHandler = default_Handler
#pragma weak TIM7_IRQHandler = default_Handler

void SysTick_Handler() {
	GPIOC->BSRR = 1<<9; 
}
