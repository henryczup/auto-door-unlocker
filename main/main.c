#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "XE0068DT_CONTROLLER";

#define PIN_RI 4   // Ring Indicate input
#define PIN_OH 5   // Off Hook control
#define PIN_D0 12  // Data bus bit 0
#define PIN_D1 13  // Data bus bit 1
#define PIN_D2 14  // Data bus bit 2
#define PIN_D3 15  // Data bus bit 3
#define PIN_WR 21  // Write control
#define PIN_RD 22  // Read control
#define PIN_CS 23  // Chip select
#define PIN_RS0 25 // Register select

#define DTMF_NINE 0b1001
#define DEBOUNCE_TIME_MS 1000 // 1 second debounce

static TaskHandle_t phone_task_handle = NULL;
static SemaphoreHandle_t operation_mutex = NULL;
static TickType_t last_interrupt_time = 0;

static void configure_pins(void);
static void xe0068dt_init(void);
static void write_xe0068dt_register(uint8_t reg_select, uint8_t data);
static uint8_t read_xe0068dt_register(uint8_t reg_select);
static void IRAM_ATTR gpio_isr_handler(void *arg);
static void phone_control_task(void *arg);

static void configure_pins(void)
{
    gpio_config_t io_conf = {};

    // configure out pins
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << PIN_OH) |
                            (1ULL << PIN_D0) | (1ULL << PIN_D1) |
                            (1ULL << PIN_D2) | (1ULL << PIN_D3) |
                            (1ULL << PIN_WR) | (1ULL << PIN_RD) |
                            (1ULL << PIN_CS) | (1ULL << PIN_RS0));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // configure ring pins with interupt
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Active LOW
    io_conf.pin_bit_mask = (1ULL << PIN_RI);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    // init output signals
    gpio_set_level(PIN_OH, 0);
    gpio_set_level(PIN_WR, 1);
    gpio_set_level(PIN_RD, 1);
    gpio_set_level(PIN_CS, 1);
    gpio_set_level(PIN_RS0, 0);
}

static void xe0068dt_init(void)
{
    if (xSemaphoreTakeRecursive(operation_mutex, portMAX_DELAY))
    {
        read_xe0068dt_register(1);

        // wrote to reg A to enable tone output and select DTMF mode
        uint8_t control_reg_a = 0;
        control_reg_a |= (1 << 0); // TOUT = 1 (enable tone output)
        control_reg_a |= (0 << 1); // CP/DTMF = 0 (select DTMF mode)
        control_reg_a |= (0 << 2); // IRQ = 0 (disable interrupts unless needed)
        control_reg_a |= (1 << 3); // RSEL = 1 (select Control Register B for next write)

        write_xe0068dt_register(1, control_reg_a);

        // write to control reg B to disable burst mode
        uint8_t control_reg_b = 0;
        control_reg_b |= (1 << 0); // Burst = 1 (disable burst mode)
        control_reg_b |= (0 << 1); // Test = 0 (normal operation)
        control_reg_b |= (0 << 2); // S/D = 0 (dual tone generation)
        control_reg_b |= (0 << 3); // C/R = not used unless S/D=1)

        write_xe0068dt_register(1, control_reg_b);

        // re-enable TOUT in control reg A
        control_reg_a = 0;
        control_reg_a |= (1 << 0);
        control_reg_a |= (0 << 1);
        control_reg_a |= (0 << 2);
        control_reg_a |= (0 << 3);

        write_xe0068dt_register(1, control_reg_a);

        ESP_LOGI(TAG, "XE0068DT module initialized");

        xSemaphoreGiveRecursive(operation_mutex);
    }
}

static uint8_t read_xe0068dt_register(uint8_t reg_select)
{
    uint8_t data = 0;

    if (xSemaphoreTakeRecursive(operation_mutex, portMAX_DELAY))
    {

        gpio_set_level(PIN_RS0, reg_select & 0x01);

        gpio_set_direction(PIN_D0, GPIO_MODE_INPUT);
        gpio_set_direction(PIN_D1, GPIO_MODE_INPUT);
        gpio_set_direction(PIN_D2, GPIO_MODE_INPUT);
        gpio_set_direction(PIN_D3, GPIO_MODE_INPUT);

        gpio_set_level(PIN_CS, 0); // chip select active
        gpio_set_level(PIN_WR, 1); // write inactive
        gpio_set_level(PIN_RD, 0); // read active

        esp_rom_delay_us(1);

        data |= gpio_get_level(PIN_D0) << 0;
        data |= gpio_get_level(PIN_D1) << 1;
        data |= gpio_get_level(PIN_D2) << 2;
        data |= gpio_get_level(PIN_D3) << 3;

        // finish read
        gpio_set_level(PIN_RD, 1); // read inactive
        gpio_set_level(PIN_CS, 1); // chip select inactice

        gpio_set_direction(PIN_D0, GPIO_MODE_OUTPUT);
        gpio_set_direction(PIN_D1, GPIO_MODE_OUTPUT);
        gpio_set_direction(PIN_D2, GPIO_MODE_OUTPUT);
        gpio_set_direction(PIN_D3, GPIO_MODE_OUTPUT);

        ESP_LOGI(TAG, "Read from register: RS0=%u, data=0x%02x", reg_select, data);
        xSemaphoreGiveRecursive(operation_mutex);
    }
    return data;
}

static void write_xe0068dt_register(uint8_t reg_select, uint8_t data)
{
    if (xSemaphoreTakeRecursive(operation_mutex, portMAX_DELAY))
    {
        gpio_set_direction(PIN_D0, GPIO_MODE_OUTPUT);
        gpio_set_direction(PIN_D1, GPIO_MODE_OUTPUT);
        gpio_set_direction(PIN_D2, GPIO_MODE_OUTPUT);
        gpio_set_direction(PIN_D3, GPIO_MODE_OUTPUT);

        gpio_set_level(PIN_D0, data & 0x01);
        gpio_set_level(PIN_D1, (data >> 1) & 0x01);
        gpio_set_level(PIN_D2, (data >> 2) & 0x01);
        gpio_set_level(PIN_D3, (data >> 3) & 0x01);

        gpio_set_level(PIN_RS0, reg_select & 0x01);

        gpio_set_level(PIN_CS, 0); // chip select active
        gpio_set_level(PIN_RD, 1); // read inactive
        gpio_set_level(PIN_WR, 0); // write active

        esp_rom_delay_us(1);

        // finish write
        gpio_set_level(PIN_WR, 1); // write inactive
        gpio_set_level(PIN_CS, 1); // chip select inactive

        ESP_LOGI(TAG, "Wrote to register: RS0=%u, data=0x%02x", reg_select, data);
        xSemaphoreGiveRecursive(operation_mutex);
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TickType_t current_time = xTaskGetTickCountFromISR();

    if (gpio_num == PIN_RI && phone_task_handle != NULL)
    {
        // Check if enough time has passed since last interrupt
        if ((current_time - last_interrupt_time) >= pdMS_TO_TICKS(DEBOUNCE_TIME_MS))
        {
            last_interrupt_time = current_time;
            vTaskNotifyGiveFromISR(phone_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

static void phone_control_task(void *arg)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Ring detected");

        if (xSemaphoreTakeRecursive(operation_mutex, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Going off-hook");
            gpio_set_level(PIN_OH, 1);

            vTaskDelay(pdMS_TO_TICKS(100));

            xe0068dt_init();

            ESP_LOGI(TAG, "Starting DTMF tone generation");
            write_xe0068dt_register(0, DTMF_NINE);

            vTaskDelay(pdMS_TO_TICKS(500));

            // stop tone output by clearing TOUT bit in control register A
            uint8_t control_reg_a = 0;
            control_reg_a |= (0 << 0); // disable tone output
            control_reg_a |= (0 << 1);
            control_reg_a |= (0 << 2);
            control_reg_a |= (0 << 3);
            write_xe0068dt_register(1, control_reg_a);

            ESP_LOGI(TAG, "DTMF tone generation complete");

            ESP_LOGI(TAG, "Hanging up");
            gpio_set_level(PIN_OH, 0);

            ESP_LOGI(TAG, "Returned to idle state");
            xSemaphoreGiveRecursive(operation_mutex);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing XE0068DT controller");

    // Create mutex for operation synchronization
    operation_mutex = xSemaphoreCreateRecursiveMutex();

    if (operation_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    configure_pins();

    gpio_install_isr_service(0);

    gpio_isr_handler_add(PIN_RI, gpio_isr_handler, (void *)PIN_RI);

    xTaskCreate(phone_control_task, "phone_control", 8192, NULL, 5, &phone_task_handle);

    ESP_LOGI(TAG, "XE0068DT controller initialized and running");
}
