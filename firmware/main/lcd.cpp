#include "lcd.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <hd44780.h>
#include <pcf8574.h>
#include <esp_log.h>

#ifdef __cplusplus
}
#endif

#include "pins.h"
#include "app_error_check.h"

#define TAG "lcd"
#define LCD_QUEUE_LENGTH 8

static QueueHandle_t lcd_queue = NULL;
static i2c_dev_t pcf8574;
static hd44780_t lcd;
static bool s_lcd_available = false;
static constexpr i2c_port_t DISPLAY_I2C_PORT = I2C_NUM_0;

static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

static void lcd_task(void *pvParameters)
{
    lcd_msg_t msg;

    while (1) {
        if (xQueueReceive(lcd_queue, &msg, portMAX_DELAY) == pdTRUE) {
            hd44780_gotoxy(&lcd, msg.x, msg.y);
            hd44780_puts(&lcd, msg.text);
        }
    }
}

void lcd_init(void)
{
    s_lcd_available = false;

    esp_err_t result = i2cdev_init();
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "LCD disabled: i2cdev_init failed: %s", esp_err_to_name(result));
        return;
    }

    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    result = pcf8574_init_desc(&pcf8574,
                               0x27,
                               DISPLAY_I2C_PORT,
                               DISPLAY_I2C_SDA_GPIO,
                               DISPLAY_I2C_SCL_GPIO);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "LCD disabled: pcf8574_init_desc failed: %s", esp_err_to_name(result));
        return;
    }

    uint8_t probe_value = 0;
    result = pcf8574_port_read(&pcf8574, &probe_value);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "LCD not detected on I2C address 0x27: %s", esp_err_to_name(result));
        return;
    }

    lcd.write_cb = write_lcd_data;
    lcd.font = HD44780_FONT_5X8;
    lcd.lines = 2;
    lcd.pins.rs = 0;
    lcd.pins.e  = 2;
    lcd.pins.d4 = 4;
    lcd.pins.d5 = 5;
    lcd.pins.d6 = 6;
    lcd.pins.d7 = 7;
    lcd.pins.bl = 3;

    result = hd44780_init(&lcd);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "LCD disabled: hd44780_init failed: %s", esp_err_to_name(result));
        return;
    }
    hd44780_switch_backlight(&lcd, true);

    lcd_queue = xQueueCreate(LCD_QUEUE_LENGTH, sizeof(lcd_msg_t));
    if (lcd_queue == NULL) {
        ESP_LOGW(TAG, "LCD disabled: queue allocation failed");
        return;
    }

    if (xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 4, NULL) != pdPASS) {
        vQueueDelete(lcd_queue);
        lcd_queue = NULL;
        ESP_LOGW(TAG, "LCD disabled: task creation failed");
        return;
    }

    s_lcd_available = true;
    ESP_LOGI(TAG, "LCD initialized and task started");
}

bool lcd_is_available(void)
{
    return s_lcd_available;
}

BaseType_t lcd_print(uint8_t x, uint8_t y, const char *text, bool clear_line, TickType_t timeout)
{
    if (!s_lcd_available || lcd_queue == NULL || text == NULL) {
        return pdFALSE;
    }

    lcd_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.x = x;
    msg.y = y;
    msg.clear_line = clear_line;
    strncpy(msg.text, text, LCD_MAX_TEXT_LEN);
    msg.text[LCD_MAX_TEXT_LEN] = 0;
    return xQueueSend(lcd_queue, &msg, timeout);
}

BaseType_t lcd_send_msg(const lcd_msg_t *msg, TickType_t timeout)
{
    if (!s_lcd_available || lcd_queue == NULL || msg == NULL) {
        return pdFALSE;
    }

    return xQueueSend(lcd_queue, msg, timeout);
}
