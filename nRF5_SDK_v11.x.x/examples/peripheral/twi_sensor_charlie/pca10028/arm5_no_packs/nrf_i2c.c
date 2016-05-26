#include <stdio.h>

#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_error.h"
#include "pca10028.h"
#include "app_util_platform.h"

#include "nrf_i2c.h"

#define usleep(val)     nrf_delay_us(val)
#define MAX_I2C         1

// TWI0: CLK-> 1, DAT->  2
// TWI1: CLK-> 7, DAT-> 30 
static char _i2c_addr[MAX_I2C];
static nrf_drv_twi_t _p_twi_instance[MAX_I2C] = {
    NRF_DRV_TWI_INSTANCE(0)
};    

const nrf_drv_twi_config_t twi_mma_7660_config = {
	 .scl                = ARDUINO_SCL_PIN,
	 .sda                = ARDUINO_SDA_PIN,
	 .frequency          = NRF_TWI_FREQ_100K,
	 .interrupt_priority = APP_IRQ_PRIORITY_HIGH
};

int i2c_init(char id, char addr)
{
    uint32_t ret_code;  
      
    ret_code = nrf_drv_twi_init(&_p_twi_instance[id], &twi_mma_7660_config, NULL, NULL); 
    APP_ERROR_CHECK(ret_code); 
    
    nrf_drv_twi_enable(&_p_twi_instance[id]); 
    
    _i2c_addr[id] = addr;
    
    return ret_code;
}

// Note: pending menas no stop bit.
int i2c_write_byte(char id, char value, bool pending)
{
    uint8_t data = value;
    uint32_t ret_code;
       
    ret_code = nrf_drv_twi_tx(&_p_twi_instance[id], _i2c_addr[id], &data, 1, pending); 
    APP_ERROR_CHECK(ret_code); 
       
    return ret_code;
}

int i2c_read_byte(char id, char *value)
{
    uint32_t ret_code;
       
    ret_code = nrf_drv_twi_rx(&_p_twi_instance[id], _i2c_addr[id], (uint8_t*) value, 1);
    APP_ERROR_CHECK(ret_code); 
    
    return ret_code;    
}

int i2c_write(char id, char *buf, int len)
{
    uint32_t ret_code;
       
    ret_code = nrf_drv_twi_tx(&_p_twi_instance[id], _i2c_addr[id], (uint8_t*) buf, len, false);
    APP_ERROR_CHECK(ret_code); 

    return ret_code;        
}

int i2c_read(char id, char *buf, int len) 
{
    uint32_t ret_code;
       
    ret_code = nrf_drv_twi_rx(&_p_twi_instance[id], _i2c_addr[id], (uint8_t*) buf, len);
    APP_ERROR_CHECK(ret_code); 
    
    return ret_code;    
}

int i2c_write_reg(char id, char reg, char value)
{
    uint32_t ret_code;
    char buf[2];
    
    buf[0] = reg;
    buf[1] = value;
    
    ret_code = i2c_write(id, (char*) buf, 2);
    
    return ret_code;
}

int i2c_read_reg(char id, char reg, char *value)
{
    uint32_t ret_code;
    
    ret_code = i2c_write_byte(id, reg, true);
    ret_code = i2c_read_byte(id, value);
    
    return ret_code;
}

int i2c_write_reg_word(char id, char reg, uint16_t value)
{
    uint32_t ret_code;
    char buf[3];
    
    buf[0] = reg;
    buf[1] = (value & 0xFF00) >> 8;
    buf[2] = value & 0x00FF;
    
    ret_code = i2c_write(id, buf, 3);
    
    return ret_code;
}

int i2c_read_reg_word(char id, char reg, uint16_t *value, bool LSB)
{
    uint32_t ret_code;
    char buf[2];
    
    ret_code = i2c_write_byte(id, reg, true);  
    ret_code = i2c_read(id, buf, 2);
    
    if(LSB)
    {
        *value  = buf[0] << 8;
        *value += buf[1]; 
    }    
    else
    {
        *value  = buf[1] << 8;
        *value += buf[0]; 
    }
    
    return ret_code;
}
