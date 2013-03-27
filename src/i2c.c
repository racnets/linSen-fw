#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "main.h"
#include "i2c.h"
#include "led.h"
//#include "usart1.h"
#include "tsl1401.h"
#include "calibration.h"

result_t i2c_result;

uint8_t state;
uint8_t addr;

void processReceivedData(uint8_t data);
uint8_t getRequestedData(void);

void i2c_init(uint8_t address) {
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* peripheral clock enable */		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	/* configure i2c-2 pins: SCL, SDA */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* setup the i2c-2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	/* i2c-2 configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = address;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Init(I2C2, &I2C_InitStructure);

	/* enable the i2c-2 Interrupt */
	I2C_ITConfig(I2C2, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);

	/* enable i2c-2 */
	I2C_Cmd(I2C2, ENABLE);	
}

void I2C2_EV_IRQHandler(void) {
    uint8_t flag;
    uint8_t data;

    if (I2C_GetITStatus(I2C2, I2C_IT_ADDR)) {
        /* address matched - cleared by reading SR1 before SR2 */
        flag = I2C2->SR2;
        /* reset state machine */
        state = 0;

//        printChar('a');
    }

    if (I2C_GetITStatus(I2C2, I2C_IT_RXNE)) {
        /* data register not empty - cleared by reading DR */
        data = I2C_ReceiveData(I2C2);
        /* process received data */
        processReceivedData(data);
        
//        printChar('r');
//        printInt32(data);
    }

    if (I2C_GetITStatus(I2C2, I2C_IT_STOPF)) {
        /* stop condition - cleared by reading SR1 and writing to CR1 */
        I2C_GenerateSTOP(I2C2, DISABLE);

//        printChar('s');
    }

    if (I2C_GetITStatus(I2C2, I2C_IT_TXE)) {
        /* load data */
        data = getRequestedData();
        /* data register empty - cleared by writing data to DR */
        I2C_SendData(I2C2, data);

//        printChar('t');
//        printInt32(data);
    }

//    printChar('\n');
}

void I2C2_ER_IRQHandler(void) {
/*    I2C_Cmd(I2C2, RESET);   
	I2C_Cmd(I2C2, ENABLE);
*/
    if (I2C_GetITStatus(I2C2, I2C_IT_AF)) {
        /* acknowledge failed - cleared by clearing corresponding bit */
        I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
//        printChar('n');
    }
//    printChar('\n');
}

void processReceivedData(uint8_t data) {
/*	printChar('p');
	printInt32(state);
	printChar('/');
	printInt32(data);
*/
    /* process data */
    switch (state) {
        case 0: {
            /* after address match - register address follows */
            addr = data;
            /* next state */
            state = 1;
            break;
        }
        case 1: {
            /* write to register */
            switch (addr) {
                case 0: {
                    /* command register */
                    switch (data) {
						case 0x2A: {
							/* change i2c address */
							break;
						}
                        case 0x42: {
                            /* change config struct */
                            state = 2;
                            break;
                        }
                        default:;
                    }
                    break;
                }
                case 0x21: {
					/* set exposure */
					setExposureValue(100*data);
					break;
				}
                case 0x22: {
					/* set exposure */
					setPixelClock(10*data);
					break;
				}
                case 0x23: {
					/* set brightness setpoint */
					if (config.s.exposure_calib == OFF) config.s.exposure_calib = ONCE;
					exposure_calib_param.setpoint = 16*data;
					break;
				}
                default:;
            }
            break;
        }
        case 2: {
            /* get inner write address */
            addr = data;
            state = 3;
            break;
		}
        case 3: {
            /* write to config struct */
            if (addr < sizeof(config_u_t)) config.a[addr++] = data;
            break;
        }
        default:;
    }   
}

uint8_t getRequestedData(void) {
    uint8_t data = 0;

//	printChar('g');
//	printInt32(addr);
	
    switch (addr++) {
        case 0 ... 0x20: {
            /* config struct */
            if (addr < sizeof(config_u_t)) data = config.a[addr];
            break;
        }
        case 0x21: {
			/* exposure */
			data = getExposureValue() / 100;
			break;
		}
		case 0x22: {
			/* pixel clock */
			data = getPixelClock() / 10;
			break;
		}
		case 0x23: {
			/* average brightness */
			config.s.calc_average_brightness |= ONCE;
			data = config.s.average_brightness / 16;
			break;
		}
		case 0x24 ... 0x29:
			/* reserved */
			break;
		case 0x30:
			/* global result vector */
			data = i2c_result.global;
			break;
		case 0x31:
			/* global result vector */
			data = i2c_result.global >> 8;
			if (i2c_result.overflow) ledRedToggle();
			/* set result output to i2c */
			config.s.result_output |= I2C | I2C_REQUEST;
			break;
        default:;
			addr = 0x30;
			/* result vector */
    }    
    
    return data;
}

void i2c_init_result(result_t* result){
	i2c_result = *result;
}

void i2c_add_result(result_t* result){
	int tmp;
	int i;
	
	tmp = i2c_result.global + result->global;
	i2c_result.global = tmp;
//	if (tmp > 32768 || tmp < -32766) i2c_result.overflow++;
	
	for (i = 0; i < result->size; ++i) {
		tmp = i2c_result.vector[i] + result->vector[i];
		i2c_result.vector[i] = tmp;
//		if (tmp > 32768 || tmp < -32766) i2c_result.overflow++; 
	}
}
