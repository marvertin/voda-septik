
#pragma once

#include "driver/gpio.h"
#include <esp_adc/adc_oneshot.h>

static const gpio_num_t STATUS_LED_GPIO = GPIO_NUM_16;          // D9 LED

static const gpio_num_t TM1637_CLK_GPIO = GPIO_NUM_17;          // J4 CLK
static const gpio_num_t TM1637_DIO_GPIO = GPIO_NUM_18;          // J4 DIO

static const gpio_num_t DISPLAY_I2C_SDA_GPIO = GPIO_NUM_22;     // J3 SDA
static const gpio_num_t DISPLAY_I2C_SCL_GPIO = GPIO_NUM_23;     // J3 SCL

static const gpio_num_t ADS1115_I2C_SDA_GPIO = GPIO_NUM_14;     // U3 SDA
static const gpio_num_t ADS1115_I2C_SCL_GPIO = GPIO_NUM_13;     // U3 SCL

static const gpio_num_t FLOW_SENSOR_GPIO = GPIO_NUM_33;         // /prutok
static const gpio_num_t TEMPERATURE_SENSOR_GPIO = GPIO_NUM_32;  // /teplomery

static const gpio_num_t RS485_RX_GPIO = GPIO_NUM_27;            // MAX3485 TXD
static const gpio_num_t RS485_TX_GPIO = GPIO_NUM_26;            // MAX3485 RXD
static const gpio_num_t RS485_EN_GPIO = GPIO_NUM_25;            // MAX3485 EN

static const gpio_num_t BOOT_BUTTON_GPIO = GPIO_NUM_0;          // GPIO0

// Interi ADC uz nejsou ve schematu pouzite, ale ponechavame je zatim na volnych kanalech bez kolizi.
static const adc_channel_t LEVEL_SENSOR_ADC_CHANNEL = ADC_CHANNEL_0;                 // GPIO36

static const adc_channel_t PRESSURE_SENSOR_BEFORE_ADC_CHANNEL = ADC_CHANNEL_3;       // GPIO39
static const adc_channel_t PRESSURE_SENSOR_AFTER_ADC_CHANNEL = ADC_CHANNEL_6;         // GPIO34

static const gpio_num_t FLOW_SIMULATOR_GPIO = GPIO_NUM_4;       // Nepouzito ve schematu, ponechano na volnem GPIO
static const gpio_num_t FLOW_SIMULATOR_OUTPUT_GPIO = FLOW_SIMULATOR_GPIO; // Zpetna kompatibilita pro starsi kod
