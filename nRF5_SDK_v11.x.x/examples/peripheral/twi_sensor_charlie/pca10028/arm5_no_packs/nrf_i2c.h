#ifndef __NRF_I2C_H__
#define __NRF_I2C_H__

#include <stdio.h>
#include "app_error.h" 
#include "nrf.h"

typedef enum {
    I2C_0,
    I2C_1,
    I2C_2
} I2C_ID;

int i2c_init(char id, char addr);
int i2c_write_byte(char id, char value, bool pending); 
int i2c_read_byte(char id, char *value); 
int i2c_write(char id, char *buf, int len);
int i2c_read(char id, char *buf, int len);
int i2c_write_reg(char id, char reg, char value); 
int i2c_read_reg(char id, char reg, char *value);
int i2c_write_reg_word(char id, char reg, uint16_t value); 
int i2c_read_reg_word(char id, char reg, uint16_t *value, bool LSM);

#endif
