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

#define TAG "LCD"
#define LCD_QUEUE_LENGTH 8

static QueueHandle_t lcd_queue = NULL;
static i2c_dev_t pcf8574;
static hd44780_t lcd;

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
    APP_ERROR_CHECK("E400", i2cdev_init());

    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    APP_ERROR_CHECK("E401", pcf8574_init_desc(&pcf8574, 0x27, I2C_NUM_0, I2C_SDA_GPIO, I2C_SCL_GPIO));

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

    APP_ERROR_CHECK("E402", hd44780_init(&lcd));
    hd44780_switch_backlight(&lcd, true);

    lcd_queue = xQueueCreate(LCD_QUEUE_LENGTH, sizeof(lcd_msg_t));
    configASSERT(lcd_queue);

    xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 4, NULL);
}

BaseType_t lcd_print(uint8_t x, uint8_t y, const char *text, bool clear_line, TickType_t timeout)
{
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
    return xQueueSend(lcd_queue, msg, timeout);
}
