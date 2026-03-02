#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdint.h>
#include <stdbool.h>

#define LCD_MAX_TEXT_LEN 16

// Struktura zprávy pro LCD
typedef struct {
    uint8_t x;         // Sloupec (0..15)
    uint8_t y;         // Řádek (0..1)
    char text[LCD_MAX_TEXT_LEN+1]; // Text k zobrazení (null-terminated)
    bool clear_line;   // Pokud true, smaže řádek před zápisem
} lcd_msg_t;

// Inicializace LCD a fronty
void lcd_init(void);

// Pošle zprávu do fronty LCD (z libovolného tasku)
BaseType_t lcd_print(uint8_t x, uint8_t y, const char *text, bool clear_line, TickType_t timeout);

// Alternativně lze poslat přímo lcd_msg_t
BaseType_t lcd_send_msg(const lcd_msg_t *msg, TickType_t timeout);
