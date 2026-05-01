
#pragma once

#include "driver/gpio.h"

static const gpio_num_t STATUS_LED_GPIO = GPIO_NUM_16;          // D9 LED

static const gpio_num_t TM1637_CLK_GPIO = GPIO_NUM_17;          // J4 CLK
static const gpio_num_t TM1637_DIO_GPIO = GPIO_NUM_18;          // J4 DIO

static const gpio_num_t DISPLAY_I2C_SDA_GPIO = GPIO_NUM_22;     // J3 SDA
static const gpio_num_t DISPLAY_I2C_SCL_GPIO = GPIO_NUM_23;     // J3 SCL

static const gpio_num_t ADS1115_I2C_SDA_GPIO = GPIO_NUM_14;     // U3 SDA
static const gpio_num_t ADS1115_I2C_SCL_GPIO = GPIO_NUM_13;     // U3 SCL
static const gpio_num_t ADS1115_ALERT_RDY_GPIO = GPIO_NUM_4;    // U3 ALERT/RDY

static const gpio_num_t FLOW_SENSOR_GPIO = GPIO_NUM_33;         // /prutok
static const gpio_num_t TEMPERATURE_SENSOR_GPIO = GPIO_NUM_32;  // /teplomery

static const gpio_num_t RS485_RX_GPIO = GPIO_NUM_27;            // MAX3485 TXD
static const gpio_num_t RS485_TX_GPIO = GPIO_NUM_26;            // MAX3485 RXD
static const gpio_num_t RS485_EN_GPIO = GPIO_NUM_25;            // MAX3485 EN

static const gpio_num_t BOOT_BUTTON_GPIO = GPIO_NUM_0;          // GPIO0
