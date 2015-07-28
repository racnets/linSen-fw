#ifndef PWM_H_
#define PWM_H_

int pwmInit(uint16_t frequency, uint16_t resolution);
void pwmInitIRQ(void (*uf)(void));
void setPwm(uint16_t channel, uint16_t value);

#endif /* PWM_H_ */
