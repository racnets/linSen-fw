#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"

#include "main.h"
#include "led.h"
#include "usart1.h"
#include "tsl1401.h"

#define TXBUFFERSIZE 0xFF
#define RXBUFFERSIZE 0xFF
#define TEXT "\n\rLinear Sensor Array test\n\r"
uint8_t txBuffer[TXBUFFERSIZE] = TEXT;
uint8_t rxBuffer[RXBUFFERSIZE];
volatile uint8_t ptrTxEnd = sizeof(TEXT);
volatile uint8_t ptrTx = 0; 
volatile uint8_t ptrRx = 0; 
volatile uint8_t ptrRxEnd = 0;

volatile uint8_t newRxData = 0;

volatile int printBufferFlag = 0;
volatile int bufferCrossSection = 0;
int bufferFullCounter = 0;
int nbBufferOutput = 0;
int unlimitedBufferOutput = 0;

void printChar(char c);
void printInt32(int32_t value);
void DMA_Configuration(void);

volatile FlagStatus USART1DMASendingStatus;

uint16_t debug;

void USART1Init() {
	
	/* setup NVIC */
	  NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	/* setup USART */
	/* enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200; //MAX 1500000
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	
	/* setup GPIO */
	/* enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Enable the EVAL_COM1 Transmit interrupt: this interrupt is generated when the 
	   EVAL_COM1 transmit data register is empty */  
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	/* Enable the EVAL_COM1 Receive interrupt: this interrupt is generated when the 
	   EVAL_COM1 receive data register is not empty */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
	
	/* setup DMA */
	DMA_Configuration();
}

void USART1_IRQHandler(void)
{
	uint8_t _rx;	

	/* receive */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		/* Read one byte from the receive data register */
		/* automatically clears pending bit */
		_rx = USART_ReceiveData(USART1);
		/* fill rx buffer */
		if ((ptrRxEnd + 1) != ptrRx) rxBuffer[ptrRxEnd++] = _rx;
	}
	
	/* transmit */
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{   
		/* if fifo not empty - else DISABLE interrupt*/
		if (ptrTx != ptrTxEnd)
		{
			/* write one byte to the transmit data register */
			/* automatically clears pending bit */
			USART_SendData(USART1, txBuffer[ptrTx++]);
		} else USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}

int usart1_rx_data_available() {
	return (ptrRxEnd != ptrRx);
}
 
int usart1_rx_get_data_available(uint8_t* c) {
	if (ptrRxEnd != ptrRx) {
		*c = rxBuffer[ptrRx++];
		return 1;
	} else return 0;
}

int usart1_tx_buffer_used() {
	return ptrTxEnd - ptrTx;
}

void printInt32(int32_t value) {
	int _value;

	_value = value / 10;
	if (_value) printInt32(_value);
	else if (value < 0) USART1Putchar('-');
	if (value < 0) value = -value;
	USART1Putchar((char)(value%10 + '0'));
}

void printChar(char c) {
//	if ((c == '\n') || (c == '\r')) {
	if (c == '\n') {
		USART1Putchar('\n');
		USART1Putchar('\r');
	} else USART1Putchar(c);
}

void USART1Putchar(uint8_t c) {
	int _bufferFullCounter = 0;
	/* while fifo full idle until it is empty */
	if ((ptrTxEnd + 1) == ptrTx) {
		while (ptrTxEnd != ptrTx) {
			__asm("nop");
			if (!_bufferFullCounter++) bufferFullCounter++;
		}
	}

	/* fill fifo */
	txBuffer[ptrTxEnd++] = c;
/*	if (ptrTx == (ptrTxEnd - 1)) {
		ledGreenToggle();
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}
*/	if (ptrTx != ptrTxEnd) {
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		if (USART_GetFlagStatus(USART1, USART_IT_TXE) != SET) {
			USART_SendData(USART1, txBuffer[ptrTx++]);
		}
	}
}

void parseRxBuffer(void) {
	char _rx = rxBuffer[ptrRx++];

	switch (_rx) {
		/* set mode */
		case 's': {
			if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
			else break;
			switch (_rx) {
				/* led, GPIO */
				case 'l': {
					if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
					else break;
					switch (_rx) {
						case '2':
						case 'g': {
							ledGreenOn();
							break;
						}
						default:;
					}	
					break;						
				}
				/* parameter */
				case 'p': {
					if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
					else break;
					switch (_rx) {
						/* pixelclock */
						case 'p': {
							int _pxlClock = 0;
							if ((ptrRxEnd - ptrRx) > 1) _rx = rxBuffer[ptrRx++];
							else break;
							if (_rx != ' ')	break;
							for (ptrRx; ptrRx <= ptrRxEnd; ptrRx++) {
								_rx = rxBuffer[ptrRx];
								if ((_rx >= '0') && (_rx <= '9')) {
									_pxlClock *= 10;
									_pxlClock += _rx - '0';
								}
							}
							if (_pxlClock) setPixelClock(_pxlClock);
							break;
						}
						/* size of buffer output */
						case 'b': {
							nbBufferOutput = 0;
							if ((ptrRxEnd - ptrRx) > 1) _rx = rxBuffer[ptrRx++];
							else break;
							if (_rx != ' ')	break;
							for (ptrRx; ptrRx <= ptrRxEnd; ptrRx++) {
								_rx = rxBuffer[ptrRx];
								if ((_rx >= '0') && (_rx <= '9')) {
									nbBufferOutput *= 10;
									nbBufferOutput += _rx - '0';
								}
							}
							break;
						}
						default:;
					}
					break;							
				}
				/* block match */
				case 'b': {
					config.s.oflow_algo |= BM_1;		
					config.s.oflow_algo &= ~ BINARY;
					if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
					else break;
					switch (_rx) {
						/* binaryBlockMatch */
						case 'b': {
							config.s.oflow_algo |= BINARY;
							break;
						}
						/* debug */
						case 'd': {
							config.s.oflow_algo |= DEBUG;
							break;
						}
						default: {
						}
					}
					break;
				}
				default:;
			}
			break;
		}
		/* clear mode */
		case 'c': {
			if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
			else break;
			switch (_rx) {
				/* led, GPIO */
				case 'l': {
					if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
					else break;
					switch (_rx) {
						case '2':
						case 'g': {
							ledGreenOff();
							break;
						}
						default:;
					}							
					break;
				}
				/* block match */
				case 'b': {
					config.s.oflow_algo &= ~BM_1;
					if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
					else break;
					switch (_rx) {
						case 'd': {
							config.s.oflow_algo &= ~DEBUG;
							break;
						}
						default:;
					}							
					break;
				}
				default:;
			}
			break;
		}
		/* print mode */
		case 'p': {
			if (ptrRx != ptrRxEnd) _rx = rxBuffer[ptrRx++];
			else break;
			switch (_rx) {
				/* buffer */
				case 'b': {
					printBufferFlag = 1;
					break;
				}
				/* cross section */
				case 's': {
					bufferCrossSection = 1;
					break;
				}
				/* unlimited */
				case 'u': {
					unlimitedBufferOutput = 1;
					break;
				}
				default:;
			}
			break;
		}
		default:;
	}
	ptrRx = ptrRxEnd;
}

int USART1DMAReady() {
	return (USART1DMASendingStatus == RESET);
}

int sendUSART1DMA(uint32_t addr, uint16_t size) {	
	if (USART1DMASendingStatus == RESET) {
		/* mark USART1 activ */
		USART1DMASendingStatus = SET;
		
		/* set usart1 tx data register address */
		DMA1_Channel4->CPAR = 0x40013804;
		/* set source data address */
		DMA1_Channel4->CMAR = addr;
		/* set source data size(in byte) */
		DMA1_Channel4->CNDTR = size;
		
		/* TODO: eigentlich nicht nötig - geht ohne aber nicht */
		DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

		/* restart dma */
		DMA_Cmd(DMA1_Channel4, ENABLE);
	} else {
		return 0;
	}
	return 1;
}

void DMA_Configuration(void) {
	/* NVIC Configuration */
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the DMA1 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* mark USART1 inactiv */
	USART1DMASendingStatus = RESET;

	/* enable USART1 DMA */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  
  	/* enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* configure DMA */
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40013804;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)txBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	
	/* enable transfer complete interrupt */
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

	/* enable DMA */
	//DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1_Channel4_IRQHandler() {
	/* test on channel4 transfer complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC4)) {
		/* clear channel1 transfer complete interrupt */
		DMA_ClearITPendingBit(DMA1_IT_TC4);
		/* mark USART1 inactiv */
		USART1DMASendingStatus = RESET;
		/* stop DMA */
		DMA_Cmd(DMA1_Channel4, DISABLE);
	}
}
