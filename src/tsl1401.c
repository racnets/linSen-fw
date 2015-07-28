#include <math.h>

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#include "main.h"
#include "tsl1401.h"
#include "usart1.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define DMAADCBUFFERSIZE	TSL1401PIXELCOUNT
/* maximum supported adc sample rate:
 * 14Mhz >= ADC_clock @72Mhz APB2_clock 12Mhz
 * trigger latency: 2(regular) + 1/f_PCLK2(external)
 * sample time: 1.5 cycles - 239.5 cycles
 * successive approximation time: 12.5 cycles
 * conversion time: 17-255 cycles
 * -> 705 kHz sample rate
 */
#define PIX_CLK_MAX_1_5		705
#define PIX_CLK_MAX_7_5		521
#define PIX_CLK_MAX_13_5	413
#define PIX_CLK_MAX_28_5	272
#define PIX_CLK_MAX_41_5	210
#define PIX_CLK_MAX_55_5	169
#define PIX_CLK_MAX_71_5	137
#define PIX_CLK_MAX_239_5	47

/* amount of adc buffer datasets */
#define DMANBOFADCBUFFER 3
/* buffer for the 3 datasets: current, latest, last */
volatile uint16_t _ADC_value[DMANBOFADCBUFFER][DMAADCBUFFERSIZE];
/* index for latest dataset in buffer */
volatile int actBuffer = 0;
/* data pointer to latest dataset */
volatile uint16_t *tsl1401_dataset;	
/* data pointer to last dataset */
volatile uint16_t *tsl1401_dataset_old;

/* dma buffer to hold/get adc values */
volatile uint16_t dmaAdcBuffer[(DMAADCBUFFERSIZE << 1)];

/* variables */
volatile int tsl1401_data_transfer_complete = 0;
volatile int dmac2getNext = 1;

/* pixel clock(kHz) and period */
int pixClk;
int pixPeriod;

/* exposure in us */
int exposure;
exposure_range_t exposure_range;

/* period for the start signal */
int expCount;

/* helper functions */
void DMAC2CopyBuffer(int htc);
void getParameters(int _exposure, int _pixClk);
void timers_init(int _clock);

/*
 * initialization 
 */
 
/* 
 * setup all hardware corresponding to tsl1401 sensor
 * important to start the timer after adc and dma
 * otherwise the data will be misaligned
 * TODO: add functional description
 */
void tsl1401_init(int _exposure, int _pixel_clock) {
	static int running = 0;

	/* stop running modules */
	if ( running ) {
		/* stop dma */
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel1, DISABLE);
		
		/* stop adc */
		ADC_Cmd(ADC1, DISABLE);
		
		/* stop timers */
		TIM_Cmd(TIM2, DISABLE);
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
	}
	running = 1;


	/* calculate timer parameters */
	getParameters(_exposure, _pixel_clock);

	/* initialize some local variables */
	tsl1401_dataset = _ADC_value[actBuffer];
	
	
    /***************************************************/
	/* clocks configuration */
	/* TIM2-4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	/* enable ADC1, GPIOA, GPIOB, AF */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	/* set adc clock */
#ifdef USE_STM32_DISCOVERY
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
#endif
	/* enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


    /***************************************************/
	/* GPIO configuration */
#ifdef USE_STM32_DISCOVERY
	/* Configure PA.07 (ADC Channel7) as analog input */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure PA.01 & PA.06 as alternate function push-pull - RAW CLK, SI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure PB.09 as alternate function push-pull - CLK*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif defined HW_LINSEN_V0_1
	/* Configure PB.01 (ADC Channel9) as analog input */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure PB.00 & PB.09 as alternate function push-pull - SI, CLK */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif defined HW_LINSEN_V0_2
	/* Configure PB.01 (ADC Channel9) as analog input */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure PB.00, PB.05 & PB.09 as alternate function push-pull - SI0, SI1, CLK */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
#endif


    /***************************************************/
	/* NVIC Configuration */
	NVIC_InitTypeDef NVIC_InitStructure;
	/* enable the DMA1 global interrupt on channel 1*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* enable the DMA1 global interrupt on channel 2*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_Init(&NVIC_InitStructure);


    /***************************************************/
	/* ADC1 configuration */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular configuration */ 
	uint8_t adc_sampleTime;
	if (pixClk < PIX_CLK_MAX_239_5) adc_sampleTime = ADC_SampleTime_239Cycles5;
	else if (pixClk < PIX_CLK_MAX_71_5) adc_sampleTime = ADC_SampleTime_71Cycles5;
	else if (pixClk < PIX_CLK_MAX_55_5) adc_sampleTime = ADC_SampleTime_55Cycles5;
	else if (pixClk < PIX_CLK_MAX_41_5) adc_sampleTime = ADC_SampleTime_41Cycles5;
	else if (pixClk < PIX_CLK_MAX_28_5) adc_sampleTime = ADC_SampleTime_28Cycles5;
	else if (pixClk < PIX_CLK_MAX_13_5) adc_sampleTime = ADC_SampleTime_13Cycles5;
	else if (pixClk < PIX_CLK_MAX_7_5) adc_sampleTime = ADC_SampleTime_7Cycles5;
	else adc_sampleTime = ADC_SampleTime_1Cycles5;
#ifdef USE_STM32_DISCOVERY
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, adc_sampleTime);
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, adc_sampleTime);
#endif

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));


    /***************************************************/
	/* DMA1 channel1 configuration */
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dmaAdcBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = DMAADCBUFFERSIZE << 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* enable transfer complete interrupt */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);


    /***************************************************/
	/* DMA1 channel2 configuration */
	//DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)dmaAdcBuffer;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)_ADC_value[actBuffer];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = DMAADCBUFFERSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* enable transfer complete interrupt */
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	

    /***************************************************/
	/* 1st TIMER configuration */
	/* pixel clock timer configuration */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = pixPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* Output Compare Timing Mode configuration: Channel2 */
	/* pixel clock */	
	TIM_OCInitStructure.TIM_Pulse = pixPeriod / 2;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	/* master mode configuration - needed for 2nd timer */
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);


    /***************************************************/
	/* 2nd TIMER configuration */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_Period = expCount;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Output Compare Timing Mode configuration: Channel4 */
	/* ADC enable pulse */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    
	TIM_OCInitStructure.TIM_Pulse = DMAADCBUFFERSIZE;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

#ifdef USE_STM32_DISCOVERY
	/* Output Compare Timing Mode configuration: Channel1 */
	/* SI1 pulse */	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
#elif defined HW_LINSEN_V0_1
	/* Output Compare Timing Mode configuration: Channel3 */
	/* SI pulse */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
#elif defined HW_LINSEN_V0_2
	/* Output Compare Timing Mode configuration: Channel2 */
	/* SI0 pulse */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	/* Output Compare Timing Mode configuration: Channel3 */
	/* SI1 pulse */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
#endif

	/* slave mode configuration */
	/* needed - timer 2 & 4 are (only) synchron due to identical setup */
	/* timer 3 is only synchron, if it depends on timer 2 as clock source */
	/* otherwise timer 3 is asynchron to timer 2 & 4 */
	/* TIM3 and ITR3 -> TIM4 as trigger input */
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_External1);
	TIM_SelectInputTrigger(TIM3, TIM_TS_ITR3);   	

	/* master mode configuration - needed for timer 4 */
	/* OC4Ref-signal is send to timer 3 trigger output */
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC4Ref);


    /***************************************************/
	/* 3rd TIMER configuration */
	/* ADC clock timer configuration */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_Period = pixPeriod;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Output Compare Timing Mode configuration: Channel4 */
	/* ADC clock */	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    
	TIM_OCInitStructure.TIM_Pulse = pixPeriod / 2;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	/* slave mode configuration */
	/* gated: counter clock is enabled when trigger input is high */
	/* TIM2 and ITR2 -> TIM3 as trigger input */
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);


    /****************************************************/
	/* start the modules */
	/* enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	/* enable ADC1 conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	/* TIM2 enable counter */
	TIM_SetCounter(TIM2, 0);
	TIM_Cmd(TIM2, ENABLE);
	/* TIM3 enable counter */
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
	/* TIM4 enable counter */
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, ENABLE);
}



/*
 * interrupt service routines
 */
 
/* interrupt service routine for DMA1 channel 1 */
/* TODO: functional description */
void DMA1_Channel1_IRQHandler() {
	/* test on channel1 transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC1)) {
		/* clear channel1 transfer complete interrupt */
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		/* copy data from buffer to user memory */
		DMAC2CopyBuffer(0);
	} else 
	/* test on channel1 half transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_HT1)) {
		/* clear channel1 half transfer complete interrupt */
		DMA_ClearITPendingBit(DMA1_IT_HT1);
		/* copy data from buffer to user memory */
		DMAC2CopyBuffer(1);
	}
}

/* interrupt service routine for DMA1 channel 2 */
/* stops dma1 channel 2 and sets flag for transfer complete */
void DMA1_Channel2_IRQHandler() {
	/* test on channel2 transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC2)) {
		/* clear channel2 transfer complete interrupt */
		DMA_ClearITPendingBit(DMA1_IT_TC2);
		/* disable DMA1 channel2 */
		DMA_Cmd(DMA1_Channel2, DISABLE);
		/* set flags */
		tsl1401_data_transfer_complete = 1;
	}
}



/* 
 * helper functions
 */
 
/* switches DMA buffer so that user can access a complete data set safely */
void tsl1401_toggle_buffer() {
	if (!dmac2getNext) {
		tsl1401_dataset_old = tsl1401_dataset;
		tsl1401_dataset = _ADC_value[actBuffer];
		actBuffer++;
		if (actBuffer == DMANBOFADCBUFFER) actBuffer = 0;
		/* activate buffer copy */
		dmac2getNext = 1;
	}
}

/* copies dataset from dma-buffer to userspace */
void DMAC2CopyBuffer(int htc) {
	if (dmac2getNext) {
		dmac2getNext = 0;
		/* DMA1 channel2 reconfiguration */
		/* change DMA memory base address */
		if (htc) DMA1_Channel2->CPAR = (uint32_t)dmaAdcBuffer;
		else DMA1_Channel2->CPAR = (uint32_t)(&dmaAdcBuffer[DMAADCBUFFERSIZE]);
		/* must be reset */
		DMA1_Channel2->CMAR = (uint32_t)_ADC_value[actBuffer];
		DMA1_Channel2->CNDTR = DMAADCBUFFERSIZE;
		/* start channel 2 in MEM2MEM mode */
		DMA_Cmd(DMA1_Channel2, ENABLE);
	}
}


/* in kHz */
void setPixelClock(int value) {
	tsl1401_init(exposure, value);
}

/* in kHz */
int getPixelClock() {
    return pixClk;
}


/* sets the exposure value - argument in µs */
void setExposureValue(int value) {
	tsl1401_init(value, pixClk);
}


/* returns the exposure value in µs */
int getExposureValue() {
	return exposure;
}


/*
 * set the exposure and pixel clock parameter
 * the exposure valid range depends on the chosen pixel clock
*/
inline void getParameters(int _exposure, int _pixClk) {
    int _minExposure;
    int _maxExposure;
	
    /* pixel clock */
	/* limit pixel clock to valid values 5kHz - 8000kHz */
	/* adc only supports PIX_CLK_MAX */
    if (_pixClk < 5) pixClk = 5;
    else if (_pixClk > PIX_CLK_MAX_1_5) pixClk = PIX_CLK_MAX_1_5;
    else pixClk = _pixClk;
    
    /* pixPeriod <= 14400(72M/5k)*/
    RCC_ClocksTypeDef  rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
    uint32_t pclk1 = rcc_clocks.PCLK1_Frequency;
    pixPeriod = (pclk1 / 1000) / pixClk;
    pixClk = ((pclk1 / 1000) / pixPeriod);
    
    /* minimal possible exposure: 129clk + 20µs */
    /* minimal value: 36µs @ 8Mhz */
    _minExposure = 129 * 1000 / pixClk + 20;

    /* maximal possible exposure: 2^16-1 * 1 / pixel clock but maximal 100ms */
    _maxExposure = pow(2,16)-1 * 1000 / pixClk;
    if (_maxExposure > 100000) _maxExposure = 100000;
	exposure_range.max_exposure = _maxExposure;
	exposure_range.min_exposure = _minExposure;	
	
    /* limit exposure to valid values 36µs - 100ms*/
    if (_exposure < _minExposure) exposure = _minExposure;
    else if (_exposure > _maxExposure) exposure = _maxExposure;
    else exposure = _exposure;

    expCount = exposure * pixClk / 1000; 
}
