
#pragma once

#include "driver/gpio.h"
#include <esp_adc/adc_oneshot.h>

static const gpio_num_t STATUS_LED_GPIO = GPIO_NUM_5;
static const gpio_num_t AUX_LED_GPIO = GPIO_NUM_4;

static const gpio_num_t TM1637_CLK_GPIO = GPIO_NUM_18;
static const gpio_num_t TM1637_DIO_GPIO = GPIO_NUM_19;

static const gpio_num_t I2C_SDA_GPIO = GPIO_NUM_21;
static const gpio_num_t I2C_SCL_GPIO = GPIO_NUM_22;

static const gpio_num_t FLOW_SENSOR_GPIO = GPIO_NUM_17;
static const gpio_num_t BOOT_BUTTON_GPIO = GPIO_NUM_0;
static const gpio_num_t TEMPERATURE_SENSOR_GPIO = GPIO_NUM_16;

static const adc_channel_t LEVEL_SENSOR_ADC_CHANNEL = ADC_CHANNEL_6;
static const adc_unit_t LEVEL_SENSOR_ADC_UNIT = ADC_UNIT_1;
static const adc_bitwidth_t LEVEL_SENSOR_ADC_BITWIDTH = ADC_BITWIDTH_12;
static const adc_atten_t LEVEL_SENSOR_ADC_ATTENUATION = ADC_ATTEN_DB_12;

