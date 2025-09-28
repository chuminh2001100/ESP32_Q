#include "gpio_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void sim7600_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << SIM7600_PWRKEY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(SIM7600_PWRKEY_GPIO, 1); // default HIGH
}

// ---------------- Power/Reset ----------------
void sim7600_power_on(void) {
    gpio_set_level(SIM7600_PWRKEY_GPIO, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(SIM7600_PWRKEY_GPIO, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    sim7600_is_on = true;
    ESP_LOGI(TAG, "SIM7600 powered ON");
}

void sim7600_power_off(void) {
    gpio_set_level(SIM7600_PWRKEY_GPIO, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    gpio_set_level(SIM7600_PWRKEY_GPIO, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    sim7600_is_on = false;
    ESP_LOGI(TAG, "SIM7600 powered OFF");
}


