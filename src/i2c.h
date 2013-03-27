#ifndef __I2C_H
#define __I2C_H

#include "main.h"

void i2c_init(uint8_t address);

void i2c_add_result(result_t* result);
void i2c_init_result(result_t* result);

#endif // __I2C_H
