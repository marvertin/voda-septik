#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <driver/gpio.h>
#include "lcd.h"

#define TAG "LEVEL_DEMO"

// ADC konfigurace pro senzor hladiny
// Tlakový pouzdový senzor do 2m s 150Ω šunt odporem
// Rozsah ADC: 0-3.3V (0-4095 na 12-bit ADC)

// ADC kanál pro senzor hladiny - GPIO3, ADC1_CHANNEL_2
// Upravte podle vaší skutečné konfigurace
static const adc_channel_t LEVEL_ADC_CHANNEL = ADC_CHANNEL_6;
static const adc_unit_t LEVEL_ADC_UNIT = ADC_UNIT_1;

// Kalibrace senzoru
// Tlakový senzor: 0-2m vody = 0-0.2 bar
// Výstupní napětí: obvykle 0.5-4.5V (některé mají 0-3.3V)
// Kalibrace: Ajustujte tyto hodnoty podle vašeho konkrétního senzoru
#define ADC_RAW_MIN 520        // RAW hodnota pro 0m
#define ADC_RAW_MAX 4095     // RAW hodnota pro 2m
#define HEIGHT_MIN 0.0f      // Výška v metrech pro minimální napětí
#define HEIGHT_MAX 2.0f      // Výška v metrech pro maximální napětí

// Počet vzorků pro průměrování
#define ADC_SAMPLES 100

static adc_oneshot_unit_handle_t adc_handle = NULL;

typedef struct {
    int poradi;
    uint32_t hodnota;
} buf_t;

#define BUF_SIZE 31 // velikost bufferu pro ukládání hodnot
#define POCET_ORIZNUTYCH_HODNOT 5 // Počet oříznutcýh hodnot, které se odstraní z každé strany z bufferu pro výpočet průměru, musí být menší než polovina velikosti bufferu

// Buffer pro ukládání posledních měření, buffer je seřazen podle velkiosti hodnoty, největší hodnota je na indexu 0, nejmenší na indexu BUF_SIZE-1
// Je průběžně aktualizován a slouží pro výpočet oříznutého průmeru, kdy se odstraní několi nejvyšší a nejnižší hodnota a zbytek se zprůměruje.
// Budder má zarážky nba začátku a konci, aby se nemusel začátek a konec hlídat
static buf_t buf[BUF_SIZE+2];
static int poradi = 0;          // pořadí měření, bude se zvyšovat s každým měřením modulo BUF_SIZE

static void init_buffer() {
    for (int i = 0; i < BUF_SIZE+2; i++) {
        buf[i].poradi = i - 1; // jako by ty nuly přišly těsně po sobě
        buf[i].hodnota = 0;
    }
    // První hodnota se nastaví na minimální hodnotu, poslení na maximální hodnotu, aby se oříznutí vždy odstranilo
    buf[0].hodnota = 0;
    buf[0].poradi = -1;
    buf[BUF_SIZE+1].hodnota = UINT32_MAX;  // Maximální hdointa, kterou číslo může nabývatM
    buf[BUF_SIZE+1].poradi = -1;
}

/*
 * Vloží novou hodnotu do bufferu a aktualizuje pořadí všech záznamů.
 * Buffer je seřazen podle velikosti hodnoty, největší hodnota je na indexu 1, nejmenší na indexu BUF_SIZE,
 *  protože index 0 a BUF_SIZE+1 jsou zarážky s extrémními hodnotami, které se nikdy nebudou používat pro ukládání skutečných měření. 
 * Po vložení nové hodnoty se zkontroluje, zda je potřeba ji posunout doleva nebo doprava, aby se udržel správný pořádek.
 */
static void insert_value(uint32_t value) {
    // ZAlogujeme vkládanou hodnotu a aktuální pořadí pro debug
    //ESP_LOGI(TAG, "Vkládám hodnotu %lu, aktuální poradi=%d", value, poradi);

    int index = 0; // index pro vložení nové hodnoty, bude se hledat v bufferu podle pořadí, které se zvyšuje s každým měřením modulo BUF_SIZE
    // Nejprve najdeme index pro vložení nové hodnoty, určitě se najde, protože hodnota byla dříve vložena
    // Hledá se nejstaší hodnota, která má stejné pořadí jako aktuální pořadí, protože hodnoty jdou modulo
    for (int i = 1; i < BUF_SIZE+1; i++) {
        if (poradi == buf[i].poradi) {
            index = i;
            break;
        }
    }

    // Vypíšeme v debugu nalezený index pro vložení nové hodnoty
    //ESP_LOGI(TAG, "Hledam index pro vložení nové hodnoty %lu, poradi=%d, našel index=%d s hodnotou %lu a poradi=%d", value, poradi, index, buf[index].hodnota, buf[index].poradi);

    buf[index].hodnota = value;

    for (;;) {
        // Nemusíme kontrolovat hranice, protože na začátku a konci jsou zarážky s extrémními hodnotami
        if (buf[index].hodnota < buf[index-1].hodnota) {
            // Prohodíme s levým sousedem
            buf_t temp = buf[index];
            buf[index] = buf[index-1];
            buf[index-1] = temp;
            index--; 
        } else if (buf[index].hodnota > buf[index+1].hodnota) {
            // Prohodíme s pravým sousedem
            buf_t temp = buf[index];
            buf[index] = buf[index+1];
            buf[index+1] = temp;
            index++;
        } else {
            break; // Hodnota je na správném místě
        }
    }

    poradi = (poradi + 1) % BUF_SIZE; // Zvyšujeme pořadí pro buňu pro nové měření

}

static uint32_t prumer(void) {
    uint64_t sum = 0;
    // Bufer má první a poslední hodnotu zarážku a vlastní buffer je od 1 do BUF_SIZE - 1, ale pole je o dva větší
    for (int i = 1 + POCET_ORIZNUTYCH_HODNOT; i <= BUF_SIZE - POCET_ORIZNUTYCH_HODNOT; i++) {
        sum += buf[i].hodnota;
    }
    uint32_t average = sum / (BUF_SIZE - 2 * POCET_ORIZNUTYCH_HODNOT);
    return average;
}   

/**
 * Inicializuje ADC pro čtení senzoru hladiny
 */
static esp_err_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = LEVEL_ADC_UNIT,
    };
    
    if (adc_oneshot_new_unit(&init_config, &adc_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba: Nelze inicializovat ADC jednotku");
        return ESP_FAIL;
    }
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    
    if (adc_oneshot_config_channel(adc_handle, LEVEL_ADC_CHANNEL, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba: Nelze nakonfigurovat ADC kanál");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * Čte průměrnou hodnotu z ADC
 * @return průměrná RAW hodnota ADC
 */
static uint32_t adc_read_average(void)
{
    int raw_value = 0;  
    if (adc_oneshot_read(adc_handle, LEVEL_ADC_CHANNEL, &raw_value) != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při čtení ADC");
        return 0;
    }
    insert_value(raw_value); // Vložíme hodnotu do bufferu pro výpočet oříznutého průměru
    vTaskDelay(pdMS_TO_TICKS(10));  // Krátká pauza mezi vzorky
    
    return prumer();
}

/**
 * Převede RAW ADC hodnotu na výšku hladiny v metrech
 * @param raw_value RAW hodnota z ADC
 * @return výška hladiny v metrech
 */
static float adc_raw_to_height(uint32_t raw_value)
{
    // Lineární interpolace
    float height = HEIGHT_MIN + (float)(raw_value - ADC_RAW_MIN) * 
                   (HEIGHT_MAX - HEIGHT_MIN) / (float)(ADC_RAW_MAX - ADC_RAW_MIN);
    
    // Omezení na rozsah
    if (height < HEIGHT_MIN) height = HEIGHT_MIN;
    if (height > HEIGHT_MAX) height = HEIGHT_MAX;
    
    return height;
}

static void level_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Spouštění demá čtení hladiny...");
    
    // Inicializace ADC
    if (adc_init() != ESP_OK) {
        ESP_LOGE(TAG, "Chyba při inicializaci ADC");
        vTaskDelete(NULL);
        return;
    }
    
    uint32_t raw_value;
    float height;
    
    while (1)
    {
        // Čtení průměru z ADC
        raw_value = adc_read_average();
        
        // Převod na výšku
        height = adc_raw_to_height(raw_value);
        
        // Výstup do logu
        ESP_LOGI(TAG, "Surová hodnota: %lu | Výška hladiny: %.3f m", raw_value, height);
        
        // Zobrazení na LCD
        char buf[12];
        snprintf(buf, sizeof(buf), "H:%.2fm", height);
        lcd_print(0, 1, buf, true, 0); // Zobraz na druhý řádek, první sloupec
        
        // Čtení každou sekundu
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void hladina_demo_init(void)
{
    init_buffer(); // Inicializace bufferu pro měření

    // Testování bufferu - vkládáme testovací hodnoty v náhodném pořadí
    uint32_t test_values[] = {500, 2000, 100, 3500, 1500, 4000, 750, 2500, 1000, 3000, 200, 3800, 1200, 2200, 4095};
    int num_test_values = sizeof(test_values) / sizeof(test_values[0]);
    
    ESP_LOGI(TAG, "=== TESTOVÁNÍ BUFFERU ===");
    
    for (int i = 0; i < num_test_values; i++) {
        insert_value(test_values[i]);
        
        ESP_LOGI(TAG, "--- Vložena hodnota %lu (krok %d) ---", test_values[i], i + 1);
        ESP_LOGI(TAG, "Poradi: %d", poradi);
        
        // Vypíšeme celý buffer
        char log_line[256];
        int pos = 0;
        for (int j = 0; j < BUF_SIZE + 2; j++) {
            pos += snprintf(log_line + pos, sizeof(log_line) - pos, 
                          "[%d:%lu] ", buf[j].poradi, buf[j].hodnota);
        }
        ESP_LOGI(TAG, "aBuffer: %s", log_line);
        ESP_LOGI(TAG, "");
        
        prumer(); // Vypočítáme a vypíšeme průměr z oříznutých hodnot
        //ESP_LOGI(TAG, ""); // Prázdný řádek pro lepší čitelnost

        vTaskDelay(pdMS_TO_TICKS(100)); // Krátká pauza mezi vkládáním
    }
    
    ESP_LOGI(TAG, "=== TESTOVÁNÍ DOKONČENO ===");

    xTaskCreate(level_task, TAG, configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}
