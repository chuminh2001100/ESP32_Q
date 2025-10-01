#include "gpio_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define SIM7600_PWRKEY_GPIO   GPIO_NUM_23   // ví dụ dùng GPIO4 để bật module
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


