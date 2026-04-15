
#pragma once

#include "driver/gpio.h"
#include <esp_adc/adc_oneshot.h>

static const gpio_num_t STATUS_LED_GPIO = GPIO_NUM_23;           // GPIO23
static const gpio_num_t AUX_LED_GPIO = GPIO_NUM_4;              // GPIO4

static const gpio_num_t TM1637_CLK_GPIO = GPIO_NUM_18;          // GPIO18
static const gpio_num_t TM1637_DIO_GPIO = GPIO_NUM_19;          // GPIO19

static const gpio_num_t DISPLAY_I2C_SDA_GPIO = GPIO_NUM_22;             // GPIO22
static const gpio_num_t DISPLAY_I2C_SCL_GPIO = GPIO_NUM_21;             // GPIO21

static const gpio_num_t ADS1115_I2C_SDA_GPIO = GPIO_NUM_23;     // Jen nastřeleno, aby tam něco bylo, není fyzicky připojeno. GPIO23 je sdíleno s LED, ale pokud se ADS1115 vůbec nepoužije, nemělo by to vadit. Pokud se ADS1115 použije, bude potřeba přepojit SDA na jiný GPIO pin a aktualizovat tento define. --- IGNORE ---
static const gpio_num_t ADS1115_I2C_SCL_GPIO = GPIO_NUM_25;     // Jen nastřeleno, aby tam něco bylo, není fyzicky připojeno. GPIO25 je sdíleno s RS485 EN, ale pokud se ADS1115 vůbec nepoužije, nemělo by to vadit. Pokud se ADS1115 použije, bude potřeba přepojit SCL na jiný GPIO pin a aktualizovat tento define. --- IGNORE ---

static const gpio_num_t FLOW_SENSOR_GPIO = GPIO_NUM_26;         // GPIO26
static const gpio_num_t FLOW_SIMULATOR_GPIO = GPIO_NUM_25;      // GPIO25
static const gpio_num_t BOOT_BUTTON_GPIO = GPIO_NUM_0;          // GPIO0
static const gpio_num_t TEMPERATURE_SENSOR_GPIO = GPIO_NUM_27;  // GPIO27
static const gpio_num_t RS485_RX_GPIO = GPIO_NUM_17;        // GPIO17 (MAX3485 TXD)
static const gpio_num_t RS485_TX_GPIO = GPIO_NUM_16;        // GPIO16 (MAX3485 RXD)
static const gpio_num_t RS485_EN_GPIO = GPIO_NUM_5;         // GPIO5 (MAX3485 EN)

static const adc_channel_t LEVEL_SENSOR_ADC_CHANNEL = ADC_CHANNEL_0;          // ADC1_CH6 (GPIO34)

static const adc_channel_t PRESSURE_SENSOR_BEFORE_ADC_CHANNEL = ADC_CHANNEL_6; // ADC1_CH4 (GPIO32)
static const adc_channel_t PRESSURE_SENSOR_AFTER_ADC_CHANNEL = ADC_CHANNEL_7;  // ADC1_CH5 (GPIO33)
