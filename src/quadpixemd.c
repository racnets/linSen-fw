#include <math.h>

#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "main.h"
#include "quadpixemd.h"

quad_pix_result_t quad_pix_result;
quad_pix_result_t quad_pix_filter_result;
quad_pix_debug_t quad_pix_filter_debug;

/* maximum supported adc sample rate:
 * 14Mhz >= ADC_clock @72Mhz APB2_clock 12Mhz
 * trigger latency: 3(injected) cycles
 * sample time: 1.5 cycles - 239.5 cycles
 * conversion time: 12.5 cycles
 * -> 857 kHz sample rate
 */
#define UR_MAX_1_5      175000
#define UR_MAX_7_5      130000
#define UR_MAX_13_5     100000
#define UR_MAX_28_5     65000
#define UR_MAX_41_5     50000
#define UR_MAX_55_5     40000
#define UR_MAX_71_5     30000
#define UR_MAX_239_5    10000

/* ADC unit
 * ADC on VL else ADC2
 */
#ifdef USE_STM32_DISCOVERY
#define ADC ADC1
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
#define ADC ADC2
#endif

/*
 * initialization 
 */
 
/* 
 * setup all hardware corresponding to quad pix sensor
 * the filters are designed corresponding to 1000Hz sampling frequency
 * so update must fit too sampling frequency
 * TODO: add functional description
 */
void quad_pix_emd_init(int _update_rate) {
    static int running = 0;

    /* stop running modules */
    if (running) {
        /* stop adc */
        ADC_Cmd(ADC, DISABLE);

        /* stop timers */
        TIM_Cmd(TIM1, DISABLE);
    }
    running = 1;

    /* calculate timer parameters */
    RCC_ClocksTypeDef  rcc_clocks;
    RCC_GetClocksFreq(&rcc_clocks);
    uint32_t pclk2 = rcc_clocks.PCLK2_Frequency;
    uint32_t period = pclk2 / _update_rate;
    uint32_t prescaler = 1;
    while (period > 0xFFFF) {
        period /= 2;
        prescaler *= 2;
    }
    prescaler--;


    /***************************************************/
    /* clocks configuration */
    /* enable ADC, GPIOA with alternate function and timer 1 */
#ifdef USE_STM32_DISCOVERY
    /* set adc clock */
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
    /* set adc clock */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC2 | RCC_APB2Periph_TIM1, ENABLE);
#endif


    /***************************************************/
    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    /* enable the ADC interrupt */
#ifdef STM32F10X_MD_VL
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
#else
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
#endif
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    /***************************************************/
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef USE_STM32_DISCOVERY
    /* Configure PA.01, PA.02, PA.03 & PA.04 as analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#elif defined HW_LINSEN_V0_1
    /* later */
#elif defined HW_LINSEN_V0_2
    /* Configure PA.04, PA.05, PA.06 & PA.07 as analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif


    /***************************************************/
    /* TIMER configuration - timer 1 */
    /* pixel clock timer configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_Pulse = period / 2;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);


    /***************************************************/
    /* ADC configuration */
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    ADC_Init(ADC, &ADC_InitStructure);

    /* ADC injected configuration */
    ADC_InjectedSequencerLengthConfig(ADC, 4);

    uint8_t adc_sampleTime;
    if (_update_rate < UR_MAX_239_5) adc_sampleTime = ADC_SampleTime_239Cycles5;
    else if (_update_rate < UR_MAX_71_5) adc_sampleTime = ADC_SampleTime_71Cycles5;
    else if (_update_rate < UR_MAX_55_5) adc_sampleTime = ADC_SampleTime_55Cycles5;
    else if (_update_rate < UR_MAX_41_5) adc_sampleTime = ADC_SampleTime_41Cycles5;
    else if (_update_rate < UR_MAX_28_5) adc_sampleTime = ADC_SampleTime_28Cycles5;
    else if (_update_rate < UR_MAX_13_5) adc_sampleTime = ADC_SampleTime_13Cycles5;
    else if (_update_rate < UR_MAX_7_5) adc_sampleTime = ADC_SampleTime_7Cycles5;
    else adc_sampleTime = ADC_SampleTime_1Cycles5;
#ifdef USE_STM32_DISCOVERY
    ADC_InjectedChannelConfig(ADC, ADC_Channel_1, 1, adc_sampleTime);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_2, 2, adc_sampleTime);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_16, 3, adc_sampleTime);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_17, 4, adc_sampleTime);
#elif defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
    ADC_InjectedChannelConfig(ADC, ADC_Channel_4, 1, adc_sampleTime);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_5, 2, adc_sampleTime);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_6, 3, adc_sampleTime);
    ADC_InjectedChannelConfig(ADC, ADC_Channel_7, 4, adc_sampleTime);
#endif

    /* adc trigger setup */
//	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
//    ADC_ExternalTrigInjectedConvConfig(ADC, ADC_ExternalTrigInjecConv_T1_TRGO);
    /* alternatively  */
    ADC_ExternalTrigInjectedConvConfig(ADC, ADC_ExternalTrigInjecConv_T1_CC4);
    TIM_CtrlPWMOutputs(TIM1, ENABLE); 	/* enables timer outputs */

    ADC_ExternalTrigInjectedConvCmd(ADC, ENABLE);
    /* enable end of conversion interrupt */
    ADC_ITConfig(ADC, ADC_IT_JEOC, ENABLE);

    ADC_TempSensorVrefintCmd(ENABLE);


    /****************************************************/
    /* start the modules */
    /* enable ADC conversion */
    ADC_Cmd(ADC, ENABLE);

    /* Enable ADC reset calibration register */
    ADC_ResetCalibration(ADC);
    /* Check the end of ADC reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC));

    /* Start ADC calibration */
    ADC_StartCalibration(ADC);
    /* Check the end of ADC calibration */
    while(ADC_GetCalibrationStatus(ADC));

    ADC_SoftwareStartConvCmd(ADC, ENABLE);

    /* enable timer */
    TIM_Cmd(TIM1, ENABLE);
}



/*
 * interrupt service routines
 */
#ifdef STM32F10X_MD_VL
void ADC1_IRQHandler(void) {
#else
void ADC1_2_IRQHandler(void) {
#endif
    if (ADC_GetFlagStatus(ADC, ADC_FLAG_JEOC)) {
        quad_pix_result.value[0] = ADC_GetInjectedConversionValue(ADC, ADC_InjectedChannel_1);
        quad_pix_result.value[1] = ADC_GetInjectedConversionValue(ADC, ADC_InjectedChannel_2);
        quad_pix_result.value[2] = ADC_GetInjectedConversionValue(ADC, ADC_InjectedChannel_3);
        quad_pix_result.value[3] = ADC_GetInjectedConversionValue(ADC, ADC_InjectedChannel_4);

        quad_pix_result.update = 1;

        /* Clear ADC JEOC pending interrupt bit */
        ADC_ClearITPendingBit(ADC, ADC_IT_JEOC);
    }
}

/*
 * elementary motion detector as per Reichardt
 */
void quad_pix_emd_update() {
    int fo;
    int i;

    /* WARNING
     * has negativ effect on the adc-values
     */
//    static int count = 1000;
//    if (!count--) {
//        count = 1000;
//        ledRedToggle();
//    }

    /* input filters */
    int fi[4];
    int dfo[4];
    int hfo[4];
    static filter_t input_filter_stage_01[4] = {};
    static filter_t input_filter_stage_02[4] = {};
    static filter_t input_filter_stage_03[4] = {};
    static filter_t high_pass_filter_stage_01[4] = {};
    static filter_t delay_filter_stage_01[4] = {};

    for (i = 0; i < 4; ++i) {
        /* input low pass filter */
        fo = update_filter(fpam*quad_pix_result.value[i], &input_filter_stage_01[i], input_filter_stage_01_biquad, fpam_exp);

        fo = update_filter(fo, &input_filter_stage_02[i], input_filter_stage_02_biquad, fpam_exp);

        fo = update_filter(fo, &input_filter_stage_03[i], input_filter_stage_03_biquad, fpam_exp);
        fi[i] = fo;

        /* high pass filter */
        fo = update_filter(fo, &high_pass_filter_stage_01[i], high_pass_filter_stage_01_biquad, fpam_exp);
        hfo[i] = fo;

        /* delay low pass filter */
        fo = update_filter(fo, &delay_filter_stage_01[i], delay_filter_stage_01_biquad, fpam_exp);
        dfo[i] = fo;

    }

    /* correlation */
    int c[4][4];
    c[0][1] = correlation(dfo[0], hfo[1], fpam_exp);
    c[0][2] = correlation(dfo[0], hfo[2], fpam_exp);
    c[0][3] = correlation(dfo[0], hfo[3], fpam_exp);
    c[1][0] = correlation(dfo[1], hfo[0], fpam_exp);
    c[1][2] = correlation(dfo[1], hfo[2], fpam_exp);
    c[1][3] = correlation(dfo[1], hfo[3], fpam_exp);
    c[2][0] = correlation(dfo[2], hfo[0], fpam_exp);
    c[2][1] = correlation(dfo[2], hfo[1], fpam_exp);
    c[2][3] = correlation(dfo[2], hfo[3], fpam_exp);
    c[3][0] = correlation(dfo[3], hfo[0], fpam_exp);
    c[3][1] = correlation(dfo[3], hfo[1], fpam_exp);
    c[3][2] = correlation(dfo[3], hfo[2], fpam_exp);

    /* substraction */
    static int s[6];
    /* y-direction */
    s[0] = c[0][1] - c[1][0];
    s[1] = c[2][3] - c[3][2];
    /* x-direction */
    s[2] = c[1][2] - c[2][1];
    s[3] = c[0][3] - c[3][0];
    /* diag */
    s[4] = c[1][3] - c[3][1];
    s[5] = c[0][2] - c[2][0];

    /* output */
    quad_pix_filter_result.value[0] = s[2] + s[3];
    quad_pix_filter_result.value[1] = s[0] + s[1];
    quad_pix_filter_result.value[2] = s[4];
    quad_pix_filter_result.value[3] = s[5];
    quad_pix_filter_result.update = 1;

    static const int debug_channels[2] = {1, 2};
    quad_pix_filter_debug.value[0] = fi[debug_channels[0]];
    quad_pix_filter_debug.value[1] = hfo[debug_channels[0]];
    quad_pix_filter_debug.value[2] = dfo[debug_channels[0]];
    quad_pix_filter_debug.value[3] = c[debug_channels[0]][debug_channels[1]];
    quad_pix_filter_debug.value[4] = fi[debug_channels[1]];
    quad_pix_filter_debug.value[5] = hfo[debug_channels[1]];
    quad_pix_filter_debug.value[6] = dfo[debug_channels[1]];
    quad_pix_filter_debug.value[7] = c[debug_channels[1]][debug_channels[0]];
    quad_pix_filter_debug.value[8] = s[3];
    quad_pix_filter_debug.value[9] = s[2];
    quad_pix_filter_debug.update = 1;
}
