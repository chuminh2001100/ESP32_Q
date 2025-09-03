#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "gpio_config.h"
#include "sim7600.h"

#define ALERT_INPUT_GPIO   GPIO_NUM_25   // input trigger
#define RESET_OUTPUT_GPIO  GPIO_NUM_26   // output reset

static const char *TAG = "MAIN";

// UART reader task
void uart_reader_task(void *pvParameters) {
    char buffer[512];
    while (1) {
        int len = sim7600_read_resp(buffer, sizeof(buffer), 1000);
        if (len > 0) {
            ESP_LOGI("UART_READER", "SIM7600 >> %s", buffer);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// MQTT publisher task
void mqtt_task(void *pvParameters) {
    while (1) {
        int level = gpio_get_level(ALERT_INPUT_GPIO);

        if (level == 1) {
            // Đọc thêm 2 lần để lọc nhiễu
            bool confirmed = true;
            for (int i = 0; i < 2; i++) {
                vTaskDelay(100 / portTICK_PERIOD_MS); // delay 100ms
                if (gpio_get_level(ALERT_INPUT_GPIO) != 1) {
                    confirmed = false;
                    break;
                }
            }

            if (confirmed) {
                ESP_LOGW(TAG, "ALERT INPUT CONFIRMED HIGH! Start MQTT session...");

                bool ok = sim7600_mqtt_connect("ESP32Client", "tcp://broker.hivemq.com:1883");
                if (!ok) {
                    sim7600_reset_module();
                    continue;
                }

                // gửi liên tục trong 20s, mỗi 2s 1 lần
                for (int i = 0; i < 10; i++) {
                    if (!sim7600_mqtt_publish("esp32/alert", "ESP32 ALERT!")) {
                        sim7600_reset_module();
                        break;
                    }
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                }

                sim7600_mqtt_disconnect();

                // reset output xuống 0 trong 1s
                gpio_set_level(RESET_OUTPUT_GPIO, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(RESET_OUTPUT_GPIO, 1);
            }
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void gpio_app_init(void) {
    // Input
    gpio_config_t in_conf = {
        .pin_bit_mask = 1ULL << ALERT_INPUT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    // Output
    gpio_config_t out_conf = {
        .pin_bit_mask = 1ULL << RESET_OUTPUT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_set_level(RESET_OUTPUT_GPIO, 1);
}

void app_main(void) {
    ESP_LOGI(TAG, "Init GPIO...");
    sim7600_gpio_init();
    sim7600_power_on();
    gpio_app_init();

    ESP_LOGI(TAG, "Init UART...");
    sim7600_uart_init();

    // start tasks
    xTaskCreate(uart_reader_task, "uart_reader_task", 4096, NULL, 10, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 9, NULL);
}
