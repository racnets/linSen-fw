#ifndef __USART1_H
#define __USART1_H

void USART1Init(void);

/* never print too much in interrupt handler routines!!! -> dead lock */
void printChar(char c);
void printInt32(int32_t value);
void parseRxBuffer(void);
void USART1Putchar(uint8_t c);
int usart1_rx_get_data_available(uint8_t* c);
int usart1_rx_data_available(void);
int usart1_tx_buffer_used(void);
int USART1DMAReady(void);
int sendUSART1DMA(uint32_t adr, uint16_t size);



/* flags */
extern volatile int printBufferFlag;
extern volatile int bufferCrossSection;
extern volatile uint8_t newRxData;
/* variables */
extern int nbBufferOutput;
extern int unlimitedBufferOutput;
extern int bufferFullCounter;
extern uint16_t debug;

#endif /* __TIMER2_H */
