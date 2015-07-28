#include "stm32f10x.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#include "time.h"

void time_init() {
	/* setup RTC */
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);
	/* Reset Backup Domain */
	BKP_DeInit();
#ifdef HW_DISCOVERY
	/* Enable LSE */
	RCC_LSEConfig(RCC_LSE_ON);
	/* Wait till LSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{}
	/* Select LSE as RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);
	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* Set RTC prescaler: set RTC period to 1msec */
	RTC_SetPrescaler(33); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/RTC_Prescaler */
	/* Wait until last write operation on RTC registers has finished */
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
	/* Select HSE(8Mhz)/128 as RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);
	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);
	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* Set RTC prescaler: set RTC period to 1msec */
	RTC_SetPrescaler(62); /* RTC period = RTCCLK/RTC_PR = (8Mhz / 128)/RTC_Prescaler */
#endif
	RTC_WaitForLastTask();

	/* setup systick */
	//SysTick->LOAD = SystemCoreClock / 8;		/* set reload register to 1 sec */
	SysTick->LOAD = SysTick_LOAD_RELOAD;		/* set reload register to maximum */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE |
					SysTick_CTRL_ENABLE;       	/* Enable SysTick Timer */
 }

/* return time in us */
uint32_t get_time() {
	return RTC_GetCounter();
}

/* return sys ticks */
uint32_t get_systick() {
//	return 8 * SysTick->VAL;
	return SysTick->VAL;
}
