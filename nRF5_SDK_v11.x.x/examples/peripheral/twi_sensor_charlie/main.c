/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
//#include "nrf_i2c.h"

/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

/*Common addresses definition for accelereomter. */
#define MMA7660_ADDR        (0x98U >> 1)

#define MMA7660_REG_XOUT    0x00U
#define MMA7660_REG_YOUT    0x01U
#define MMA7660_REG_ZOUT    0x02U
#define MMA7660_REG_TILT    0x03U
#define MMA7660_REG_SRST    0x04U
#define MMA7660_REG_SPCNT   0x05U
#define MMA7660_REG_INTSU   0x06U
#define MMA7660_REG_MODE    0x07U
#define MMA7660_REG_SR      0x08U
#define MMA7660_REG_PDET    0x09U
#define MMA7660_REG_PD      0x0AU

/* Mode for MMA7660. */
#define ACTIVE_MODE 1u

/*Failure flag for reading from accelerometer. */
#define MMA7660_FAILURE_FLAG (1u << 6)

/*Tilt specific bits*/
#define TILT_TAP_MASK (1U << 5)
#define TILT_SHAKE_MASK (1U << 7)

// [max 255, otherwise "int16_t" won't be sufficient to hold the sum
//  of accelerometer samples]
#define NUMBER_OF_SAMPLES 20

/* Define version of GCC. */
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)

/**
 * @brief Union to keep raw and converted data from accelerometer samples at one memory space.
 */
typedef union{
    uint8_t raw;
    int8_t  conv;
}elem_t;

/**
 * @brief Enum for selecting accelerometer orientation.
 */
typedef enum{
    LEFT = 1,
    RIGHT = 2,
    DOWN = 5,
    UP = 6
}accelerometer_orientation_t;

/**
 * @brief Structure for holding samples from accelerometer.
 */
typedef struct
{
    elem_t  x;
    elem_t  y;
    elem_t  z;
    uint8_t tilt;
} sample_t;

#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"           // Hack to GCC 4.9.3 bug. Can be deleted after switch on using GCC 5.0.0
#endif
#endif
/* Buffer for samples. */

#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic pop
#endif
#endif
/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;

/**
 * @brief UART events handler.
 */
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Get BMI160 accel data.
 */
void bmi160_get_accel()
{
    uint8_t data1 = 0, data2 = 0;
    int accel_x = 0, accel_y = 0, accel_z = 0;
    
    /*read accel z*/
    i2c_read_reg(0,0x12,&data1);
    i2c_read_reg(0,0x13,&data2);
    accel_x = (data2 << 8) + data1;
    
    if((data2 & 0x80) >> 7 == 1)
    {
        accel_x = 0xFFFF - accel_x;
    }
    else
    {
        accel_x = -accel_x;
    }
    
    /*read accel y*/
    i2c_read_reg(0,0x14,&data1);
    i2c_read_reg(0,0x15,&data2);
    accel_y = (data2 << 8) + data1;
    
    if((data2 & 0x80) >> 7 == 1)
    {
        accel_y = 0xFFFF - accel_y;
    }
    else
    {
        accel_y = -accel_y;
    }
    
    /*read accel z*/
    i2c_read_reg(0,0x16,&data1);
    i2c_read_reg(0,0x17,&data2);
    accel_z = (data2 << 8) + data1;
    
    if((data2 & 0x80) >> 7 == 1)
    {
        accel_z = 0xFFFF - accel_z;
    }
    else
    {
        accel_z = -accel_z;
    }
    
    printf("%f\t%f\t%f\r\n",(float)accel_x/16384,(float)accel_y/16384,(float)accel_z/16384);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uart_config();
    printf("\n\rBMI160 sensor\r\n");
	i2c_init(0,0x69);
	i2c_write_reg(0,0x7E,0x11);
    
    while(true)
    {
        bmi160_get_accel();
        nrf_delay_ms(10);
    }
}

/** @} */
