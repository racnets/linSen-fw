#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "main.h"
#include "i2c.h"

#include "calibration.h"
#include "eeprom.h"
#include "led.h"

#ifdef USE_EMD
#include "quadpixemd.h"
#endif
#ifdef USE_SERVO
#include "servo.h"
#endif
#ifdef USE_TSL
#include "tsl1401.h"
#endif

result_t i2c_result;

uint8_t state;
uint8_t addr;

void processReceivedData(uint8_t data);
uint8_t getRequestedData(void);

void i2c_init(uint8_t address) {

	/* peripheral clock enable */		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	/* configure i2c-2 pins: SCL, SDA */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* setup the i2c-2 Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	/* i2c-2 configuration */
	I2C_DeInit(I2C2);
	I2C_InitTypeDef  I2C_InitStructure;
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
	I2C_ITConfig(I2C2, I2C_IT_BUF, ENABLE);

	/* enable i2c-2 */
	I2C_Cmd(I2C2, ENABLE);
}

void I2C2_EV_IRQHandler(void) {
    static uint8_t data = 0;

   if (I2C_GetITStatus(I2C2, I2C_IT_ADDR)) {
        /* address matched and acknowledged - cleared by reading SR1 before SR2 */
		/* reset state machine */
	   (__IO void)(I2C2->SR2);
	   state = 0;
    }

   if (I2C_GetITStatus(I2C2, I2C_IT_TXE)) {
        /* load data */
   		data = getRequestedData();
   		/* data register empty - cleared by writing data to DR */
   		I2C_SendData(I2C2, data);
    }

    if (I2C_GetITStatus(I2C2, I2C_IT_RXNE)) {
        /* data register not empty - cleared by reading DR */
        data = I2C_ReceiveData(I2C2);
        /* process received data */
        processReceivedData(data);
    }

    if (I2C_GetITStatus(I2C2, I2C_IT_STOPF)) {
        /* stop condition - cleared by reading SR1 and writing to CR1 */
        I2C_Cmd(I2C2, ENABLE);
    }
}

void I2C2_ER_IRQHandler(void) {

	if (I2C_GetITStatus(I2C2, I2C_IT_AF)) {
        /* acknowledge failed - cleared by clearing corresponding bit */
        I2C_ClearITPendingBit(I2C2, I2C_IT_AF);
    }
}

/*
 * addr	size	write
 * 0x00	b		command register
 * 				0x2A	change i2c address
 * 				0x42	change config struct
 * ...			not available
 * servo stuff
 * 0x31	b		servo on/off
 * 0x32	w		servo value
 * 0x34	w		servo min value
 * 0x36 w		servo max value
 * 0x38 b		servo movement
 * 0x39	b		servo speed
 * TSL stuff
 * 0x81	b		exposure
 * 0x82	b		pixel clock
 * 0x83	b		brightness set point
 */
void processReceivedData(uint8_t data) {
	static uint16_t tmpValue = 0;

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
							state = 4;
							break;
						}
#ifdef USE_TSL
                        case 0x42: {
                            /* change config struct */
                            state = 2;
                            break;
                        }
#endif
                        default:;
                    }
                    break;
                }
#ifdef USE_SERVO
                case 0x31: {
                	addr++;
                	/* set servo 0 on/off */
					break;
				}
                case 0x32:;
                case 0x34:;
                case 0x36: {
                	addr++;
                	tmpValue = data;
					break;
				}
                case 0x33: {
                	addr++;
                	/* set servo 0 value */
                	setServoPosition((data << 8) | tmpValue);
                	break;
                }
                case 0x35: {
                	addr++;
                	/* set servo 0 min value */
                	setServoMinPosition((data << 8) | tmpValue);
					break;
				}
                case 0x37: {
                	addr++;
                	/* set servo 0 max value */
                	setServoMaxPosition((data << 8) | tmpValue);
					break;
				}
                case 0x39: {
                	addr++;
                	/* set servo 0 speed */
                	setServoSpeed(data);
					break;
				}
#endif
#ifdef USE_TSL
                case 0x81: {
					/* set exposure */
					setExposureValue(100*data);
					break;
				}
                case 0x82: {
					/* set exposure */
					setPixelClock(10*data);
					break;
				}
                case 0x83: {
					/* set brightness setpoint */
					if (config.s.exposure_calib == OFF) config.s.exposure_calib = ONCE;
					exposure_calib_param.setpoint = 16*data;
					break;
				}
#endif
                default:;
            }
            break;
        }
#ifdef USE_TSL
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
#endif
        case 4: {
			/* write new i2c slave address to flash */
			EE_WriteVariable(I2C_SLAVE_ADDRESS_ADDR, data);
			break;
		}
        default:;
    }   
}

/*
 * addr	size	read
 * 0x00	b		component id
 * 0x01	b		unique id
 * ...
 * 0x20	b		end of system struct
 * servo related
 * 0x31 b		get servo 0 on/off
 * 0x32	w		servo value
 * 0x34 w		servo min value
 * 0x36 w		servo max value
 * 0x39 b		servo speed
 * EMD related
 * 0x70 w		average brightness
 * 0x72 w		quadpix result vector 0
 * 0x74 w		quadpix result vector 1
 * 0x76 w		quadpix result vector 2
 * 0x78 w		quadpix result vector 3

 * TSL related
 * 0x81	b		exposure
 * 0x82	b		pixel clock
 * 0x83	b		average brightness
 * ...
 * 0x90	w		global result scalar
 * 0x92 b		1st local result scalar
 * 0x...		nth local result scalar
 */

uint8_t getRequestedData(void) {
    static uint16_t data = 0;
	static uint32_t tmpValue = 0;
	
    switch (addr++) {
        case 0 ... 0x20: {
            /* config struct */
            if (addr <= sizeof(config_u_t)) data = config.a[addr-1];
            break;
        }
#ifdef USE_SERVO
        case 0x31: {
			/* get servo on/off */
			data = 1;
			break;
		}
		case 0x32: {
			 /* get servo 0 value */
			data = getServoPosition();
			break;
		}
		case 0x34: {
			 /* get servo 0 value */
			data = getServoMinPosition();
			break;
		}
		case 0x36: {
			 /* get servo 0 value */
			data = getServoMaxPosition();
			break;
		}
		case 0x33:;
		case 0x35:;
		case 0x37: {
        	data = data >> 8;
        	break;
		}
		case 0x39: {
			/* get servo 0 speed */
			data = getServoSpeed();
			break;
		}
#endif
#ifdef USE_EMD
		case 0x70:
			/* quadpix global result vector */
			tmpValue = quad_pix_result.value[0];
			tmpValue += quad_pix_result.value[1];
			tmpValue += quad_pix_result.value[2];
			tmpValue += quad_pix_result.value[3];
			data = tmpValue;
			break;
		case 0x72:
			/* quadpix result vector 0 */
			tmpValue = quad_pix_result.value[0];
			data = tmpValue;
			break;
		case 0x74:
			/* quadpix result vector 1 */
			tmpValue = quad_pix_result.value[1];
			data = tmpValue;
			break;
		case 0x76:
			/* quadpix result vector 2 */
			tmpValue = quad_pix_result.value[2];
			data = tmpValue;
			break;
		case 0x78:
			/* quadpix result vector 3 */
			tmpValue = quad_pix_result.value[3];
			data = tmpValue;
			break;
		case 0x77:
			/* set address to first result */
			addr = 0x72;
		case 0x71:;
		case 0x73:;
		case 0x75:;
			case 0x79:
			data = tmpValue >> 8;
			break;
#endif
#ifdef USE_TSL
        case 0x81: {
			/* exposure */
			data = getExposureValue() / 100;
			break;
		}
		case 0x82: {
			/* pixel clock */
			data = getPixelClock() / 10;
			break;
		}
		case 0x83: {
			/* average brightness */
			config.s.calc_average_brightness |= ONCE;
			data = config.s.average_brightness / 16;
			break;
		}
		case 0x90:
			/* tsl global result */
			data = i2c_result.global;
			break;
		case 0x91:
			/* tsl global result scalar */
			data = i2c_result.global >> 8;
			if (i2c_result.overflow) ledRedToggle();
			/* set result output to i2c */
			config.s.result_output |= I2C | I2C_REQUEST;
			break;
		case 0x92 ... 0x92+MAX_RESULT_SCALARS:
			/* tsl global result scalar */
			data = ((uint8_t *)i2c_result.scalar)[addr - 0x92];
			/* set result output to i2c */
			config.s.result_output |= I2C | I2C_REQUEST;
			break;
#endif
        default:;
			addr = 0x00;
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
		tmp = i2c_result.scalar[i] + result->scalar[i];
		i2c_result.scalar[i] = tmp;
//		if (tmp > 32768 || tmp < -32766) i2c_result.overflow++; 
	}
}