#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_flash.h"

#include "main.h"

#include "eeprom.h"
#include "led.h"
#include "usart1.h"
#include "tsl1401.h"
#include "blockmatch.h"
#include "calibration.h"
#include "button.h"
#include "mavlink_bridge.h"
#include "time.h"
#include "i2c.h"

void RCC_Configuration(void);

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NumbOfVar] = {I2C_SLAVE_ADDRESS_ADDR};

int blockMatchResultInt = 0;

config_u_t config = {0};
result_t linSen_result = {0};

int main(void) {
	uint16_t i2c_addr = 0x30;
	int res;

	/* configure clocks configuration */
	RCC_Configuration();

	/* initialize configure structure */
	config.s.sys_id = LIN_SEN_SYSID;
	config.s.id = LIN_SEN_ID;
	config.s.usart_modus = OPEN;
	config.s.oflow_algo = BM_1;
	config.s.oflow_algo_param = 0;
	config.s.result_output |= LEDS;

	/* initialize modules */
	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();
	/* EEPROM Init */
	EE_Init();
	/* get i2c address from flash */
	if (EE_ReadVariable(I2C_SLAVE_ADDRESS_ADDR, &i2c_addr) != 0) EE_WriteVariable(I2C_SLAVE_ADDRESS_ADDR, i2c_addr);

	ledsInit();
	buttonInit();
	tsl1401_init(50000, 200);
	USART1Init();
	time_init();
	i2c_init(i2c_addr);

	/* run programm */
	while (1) {
		/* process usart1 rx data */
		if (usart1_rx_data_available() && config.s.usart_modus != OFF) {
			//if (config.s.usart_modus == TERMINAL || config.s.usart_modus == OPEN) parseRxBuffer();
			if (config.s.usart_modus == MAVLINK || config.s.usart_modus == OPEN) processMavlink();
		}

		/* handle mavlink data */
		if (config.s.usart_modus == MAVLINK) handleMavlink();
		
		/* process button status */
		if (buttonPressed()) {
    		ledBlueToggle();
		}

		/* dma1 transfer complete */
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
				if (config.s.oflow_algo & BM_1)	blockMatch(tsl1401_dataset, tsl1401_dataset_old, TSL1401PIXELCOUNT, 8, 8, &linSen_result);

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
			
			/* process i2c output */
			if (config.s.result_output & I2C) {
				/* if data was requestet by i2c the integral is flushed */
				if (config.s.result_output & I2C_REQUEST) {
					config.s.result_output &= ~I2C_REQUEST;
					i2c_init_result(&linSen_result);
				} else
				/* the data not beeing send is beeing integrated */
				 ;//i2c_add_result(&linSen_result);
			}

			/* output sensor debug data via mavlink */
			if (config.s.debug_modus & MAVLINK) mavlinkSendRaw((uint16_t*)tsl1401_dataset);

			/* output sensor data via mavlink */
			if (config.s.result_output & MAVLINK) mavlinkSendOpticalFlow(linSen_result.global, linSen_result.size, 0);				
		}
	}
}

void RCC_Configuration(void) {
	/* set peripheral clock 1 to PCLK1 = HCLK */
	RCC_PCLK1Config(RCC_HCLK_Div1);
}
