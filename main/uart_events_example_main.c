#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "gpio_config.h"
#include "sim7600.h"

#define ALERT_INPUT_GPIO   GPIO_NUM_0   // input trigger
#define RESET_OUTPUT_GPIO  GPIO_NUM_26  // output reset

static const char *TAG = "MAIN";

// Hàm chống dội phím
static bool debounce_gpio(int gpio, int expected_level, int samples, int delay_ms) {
    for (int i = 0; i < samples; i++) {
        if (gpio_get_level(gpio) != expected_level) {
            return false;
        }
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
    return true;
}

// ---------------- MQTT Task ----------------
void mqtt_task(void *pvParameters) {
    while (1) {
        int level = gpio_get_level(ALERT_INPUT_GPIO);

        if (level == 0) {
            ESP_LOGW(TAG, "ALERT LOW detected, checking debounce...");

            if (!debounce_gpio(ALERT_INPUT_GPIO, 0, 5, 10)) {
                ESP_LOGW(TAG, "Debounce failed, ignore!");
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }

            ESP_LOGI(TAG, "ALERT confirmed!");

            // Kiểm tra sóng/mạng trước khi gửi MQTT
           if (!sim7600_ready_for_mqtt_retry(5, 2000)) {
			    ESP_LOGE(TAG, "Module not ready after retries -> Resetting SIM7600");
			    sim7600_reset_module();
			    continue;
		    }

            // MQTT session
            ESP_LOGI(TAG, "Starting MQTT session...");
            if (!sim7600_mqtt_start()) {
                ESP_LOGE(TAG, "MQTT START failed -> Resetting SIM7600");
                sim7600_reset_module();
                continue;
            }

            ESP_LOGI(TAG, "Connecting to broker: mqtt.mcvmindd.cloud:1883");
            if (!sim7600_mqtt_connect("ESP32Client",
                                      "tcp://mqtt.mcvmindd.cloud:1883",
                                      60, 1)) {
                ESP_LOGE(TAG, "MQTT CONNECT failed -> Resetting SIM7600");
                sim7600_reset_module();
                continue;
            }

            ESP_LOGI(TAG, "Publishing to topic: smart/devices/alert");
            for (int i = 0; i < 10; i++) {
                if (!sim7600_mqtt_publish("esp32/minhcv5/alert",
                                          "ESP32 ALERT!", 1)) {
                    ESP_LOGE(TAG, "MQTT PUBLISH failed -> Resetting SIM7600");
                    sim7600_reset_module();
                    break;
                }
                ESP_LOGI(TAG, "MQTT message %d/10 sent OK", i + 1);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }

            ESP_LOGI(TAG, "Disconnecting MQTT...");
            sim7600_mqtt_disconnect();

            // Toggle reset output trong 1s
            ESP_LOGI(TAG, "Toggling RESET_OUTPUT_GPIO LOW for 1s");
            gpio_set_level(RESET_OUTPUT_GPIO, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(RESET_OUTPUT_GPIO, 1);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// ---------------- GPIO Init ----------------
static void gpio_app_init(void) {
    // Input ALERT
    gpio_config_t in_conf = {
        .pin_bit_mask = 1ULL << ALERT_INPUT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    // Output RESET
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

// ---------------- Main Entry ----------------
void app_main(void) {
    ESP_LOGI(TAG, "==== SYSTEM START ====");

    // Init GPIO cho SIM7600 (PWRKEY, v.v…)
    sim7600_gpio_init();

    // Init GPIO ALERT + RESET
    gpio_app_init();

    // Init UART giao tiếp SIM7600
    sim7600_uart_init();

    // Kiểm tra tình trạng SIM7600 trước khi bật
    ESP_LOGI(TAG, "Checking SIM7600 status...");

	int sim_ok = sim7600_basic_check();
	if (!sim_ok) {
	    ESP_LOGW(TAG, "No response -> powering ON SIM7600 (1st try)...");
	    sim7600_power_on();
	    vTaskDelay(5000 / portTICK_PERIOD_MS);
	
	    sim_ok = sim7600_basic_check();
	    if (!sim_ok) {
	        ESP_LOGW(TAG, "Still no response -> powering ON SIM7600 (2nd try)...");
	        sim7600_power_on();
	        vTaskDelay(5000 / portTICK_PERIOD_MS);
	
	        sim_ok = sim7600_basic_check();
	        if (!sim_ok) {
	            ESP_LOGE(TAG, "SIM7600 FAILED after 2 retries -> restarting ESP32...");
	            vTaskDelay(2000 / portTICK_PERIOD_MS);
	            esp_restart();
	        } else {
	            ESP_LOGI(TAG, "SIM7600 OK after 2nd power on.");
	        }
	    } else {
	        ESP_LOGI(TAG, "SIM7600 OK after 1st power on.");
	    }
	} else {
	    ESP_LOGI(TAG, "SIM7600 already ON and responding.");
	}

    // Tạo task MQTT
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 9, NULL);

    ESP_LOGI(TAG, "==== SYSTEM READY ====");
}
