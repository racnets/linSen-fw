#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

#include "main.h"

#include "button.h"
#include "eeprom.h"
#include "i2c.h"
#include "led.h"
#include "quadpixemd.h"
#include "servo.h"
#include "time.h"
#include "tsl1401.h"
#include "usart1.h"

#ifdef USE_MAVLINK
#include "mavlink_bridge.h"
#endif


void RCC_Configuration(void);

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar] = {I2C_SLAVE_ADDRESS_ADDR};

int blockMatchResultInt = 0;

config_u_t config = {};
result_t linSen_result = {0};

int main(void) {
    /* configure clocks configuration */
    RCC_Configuration();

    /* initialize configure structure */
    config.s.sys_id = LIN_SEN_SYSID;
    config.s.id = LIN_SEN_ID;
    config.s.usart_modus = OPEN;
    config.s.oflow_algo = BM_1;
    config.s.oflow_algo_param = 0;
    config.s.result_output = OFF;

    /* initialize modules */
    buttonInit();
    ledsInit();
    time_init();

#ifdef USE_I2C
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();
    /* EEPROM Init */
    EE_Init();
    /* get i2c address from flash */
    uint16_t i2c_addr = 0x30;
    if (EE_ReadVariable(I2C_SLAVE_ADDRESS_ADDR, &i2c_addr) != 0) EE_WriteVariable(I2C_SLAVE_ADDRESS_ADDR, i2c_addr);

    i2c_init(i2c_addr);
#endif
#ifdef USE_USART
    USART1Init();
#endif

#if defined USE_STM32_DISCOVERY && defined USE_TSL && defined USE_EMD
    /* only one function is supported, because they share one ADC unit */
    #error only one of TSL or EMD is supported on this platform
#elif defined USE_STM32_DISCOVERY && defined USE_TSL && !defined USE_EMD
    tsl1401_init(50000, 200);
#elif defined USE_STM32_DISCOVERY && !defined USE_TSL && defined USE_EMD
    quad_pix_emd_init(1000);
#endif

#if defined HW_LINSEN_V0_1 || defined HW_LINSEN_V0_2
#ifdef USE_TSL
    tsl1401_init(50000, 200);
#endif
#ifdef USE_EMD
    quad_pix_emd_init(1000);
#endif
#endif

#ifdef USE_SERVO
    servoInit();
#endif


	ledGreenOn();

    /* run programm */
    while (1) {
        /* round-trip measurement */
        static uint32_t _systick_then, _systick_now;

        _systick_now = get_systick();
        uint32_t round_trip = _systick_then - _systick_now;
        _systick_then = _systick_now;

        /* process button status */
        if (buttonPressed()) {
            ledBlueToggle();
#ifdef USE_SERVO
			toggleServoSpeed();
#endif
        }


#ifdef USE_TSL
        /******************************/
        /* TSL1401                    */
        /* dma1 transfer complete     */
        if (tsl1401_data_transfer_complete) {
            tsl1401_data_transfer_complete = 0;

            /* toggle buffer to hold new data */
            /* prepare DMA for new cycle */
            tsl1401_toggle_buffer();
            /* do something with the data */
            /* only safe within these context */

            /* benchmark */
            ledGreenToggle();

            /* calculate average of line buffer */
            if ((config.s.exposure_calib != OFF) || (config.s.oflow_algo & BINARY) || (config.s.calc_average_brightness != OFF)) {
                config.s.average_brightness = crossSection(tsl1401_dataset, TSL1401PIXELCOUNT);

                if (config.s.calc_average_brightness & ONCE) config.s.calc_average_brightness &= ~ONCE;
            }

            /* auto calibration */
            if (config.s.exposure_calib != OFF) {
                exposure_calibration();
            }
            if (config.s.exposure_calib_status == RUNNING) ledBlueOn();
            //else ledBlueOff();

            /* compute & print block match result */
            if (config.s.oflow_algo != OFF && config.s.result_output != OFF) {
                if (config.s.oflow_algo & BINARY) makeBinary(tsl1401_dataset, TSL1401PIXELCOUNT, config.s.average_brightness);
                /* block Match with block size max */
                if (config.s.oflow_algo & BM_1)    blockMatch(tsl1401_dataset, tsl1401_dataset_old, TSL1401PIXELCOUNT, 8, 8, &linSen_result);

                blockMatchResultInt += linSen_result.global;
                if (config.s.oflow_algo & DEBUG) {
                    printChar('#');
                    printInt32(blockMatchResultInt);
                    printChar('\n');
                }
            } else {
                blockMatchResultInt = 0;
                linSen_result.global = 0;
            }

            /* visualize optical flow */
            if (config.s.result_output & LEDS) {
                if (linSen_result.global != 0) ledRedToggle();
                else ledRedOff();
            }

#ifdef USE_I2C
            /* process i2c output */
            if (config.s.result_output & I2C) {
                /* if data was requested by i2c the integral is flushed */
                if (config.s.result_output & I2C_REQUEST) {
                    config.s.result_output &= ~I2C_REQUEST;
                    i2c_init_result(&linSen_result);
                } else
                /* the data not being send is being integrated */
                 ;//i2c_add_result(&linSen_result);
            }
#endif
#if defined USE_USART && defined USE_MAVLINK
            /* output sensor debug data via mavlink */
            if (config.s.debug_modus & MAVLINK) mavlinkSendLinRaw((uint16_t*)tsl1401_dataset);

            /* output sensor data via mavlink */
            if (config.s.result_output & MAVLINK) mavlinkSendOpticalFlow(linSen_result.global, linSen_result.size /*0*/, 0);
#endif
        }
#endif // USE_TSL

#ifdef USE_EMD
        /******************************/
        /* quadPix emd                */
        /* conversion complete        */
        if (quad_pix_result.update) {
            quad_pix_result.update = 0;

            quad_pix_emd_update();

#if defined USE_USART && defined USE_MAVLINK
            /* output sensor debug data via mavlink */
//            if (config.s.debug_modus & MAVLINK) mavlinkSendQuadRaw((uint32_t*)quad_pix_result.value);
#endif
        }
        /* filter complete */
        if (quad_pix_filter_result.update) {
            quad_pix_filter_result.update = 0;

#if defined USE_USART && defined USE_MAVLINK
            /* output sensor data via mavlink */
            if (config.s.result_output & MAVLINK) mavlinkSendOpticalFlow(quad_pix_filter_result.value[0], quad_pix_filter_result.value[1], 0);

            /* output sensor debug data via mavlink */
            if (config.s.debug_modus & MAVLINK) mavlinkSendQuadRaw((uint32_t*)quad_pix_filter_debug.value);
#endif
        }
#endif
    }
}

void RCC_Configuration(void) {
    /* Configures the Low Speed APB clock (APB1-domain) */
	/* not more then 36Mhz are allowed */
    if (SystemCoreClock > 36E6) RCC_PCLK1Config(RCC_HCLK_Div2);
    else RCC_PCLK1Config(RCC_HCLK_Div1);
}
