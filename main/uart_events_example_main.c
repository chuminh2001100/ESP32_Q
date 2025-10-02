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

bool mqtt_connected = false;

typedef enum {
    MQTT_STATE_IDLE = 0,        // chưa CMQTTSTART
    MQTT_STATE_STARTED,         // đã CMQTTSTART
    MQTT_STATE_CLIENT_CREATED,  // đã CMQTTACCQ
    MQTT_STATE_CONNECTED        // đã CONNECT
} mqtt_state_t;

static mqtt_state_t mqtt_state = MQTT_STATE_IDLE;

static QueueHandle_t sim7600_event_queue = NULL;
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


void sim7600_event_task(void *param) {
    sim7600_event_t evt;

    while (1) {
        if (xQueueReceive(sim7600_event_queue, &evt, portMAX_DELAY)) {
            switch (evt.type) {
                case SIM7600_EVENT_MQTT_DISCONNECTED:
                    ESP_LOGW(TAG, "[EVENT] MQTT Disconnected! Need reconnect...");
                    mqtt_connected = false;
                    break;

                case SIM7600_EVENT_MQTT_SUBRECV:
                    ESP_LOGI("MQTT_TASK", "Received message: topic=%s payload=%s", evt.topic, evt.payload);
                    // ✅ Xử lý theo topic
                    if (strcmp(evt.topic, "esp32/minhcv5/cmd1") == 0) {
                        ESP_LOGI("MQTT_TASK", "CMD1 -> toggle LED");
                        // xử lý action 1
                    } 
                    else if (strcmp(evt.topic, "esp32/minhcv5/cmd2") == 0) {
                        ESP_LOGI("MQTT_TASK", "CMD2 -> reset SIM");
                        // xử lý action 2
                    } 
                    else if (strcmp(evt.topic, "esp32/minhcv5/cmd3") == 0) {
                        ESP_LOGI("MQTT_TASK", "CMD3 -> custom action");
                        // xử lý action 3
                    }
                    break;
				case SIM7600_EVENT_MQTT_CONNECTED:
				 	ESP_LOGI(TAG, "[EVENT] MQTT CONNECTED");
                    break;
                case SIM7600_EVENT_NETWORK_LOST:
                    ESP_LOGW(TAG, "[EVENT] Network Lost!");
                    break;

                case SIM7600_EVENT_NETWORK_READY:
                    ESP_LOGI(TAG, "[EVENT] Network Ready!");
                    break;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void sim7600_mqtt_reconnect() {
    ESP_LOGW("MQTT", "Reconnecting MQTT...");

    // Nếu đang kết nối → ngắt trước
    if (mqtt_connected && mqtt_state == MQTT_STATE_CONNECTED) {
        ESP_LOGI("MQTT", "Disconnecting old session...");
        sim7600_send_cmd_str("AT+CMQTTDISC=0,60", "OK", 2, 2000);
        mqtt_connected = false;
    }

    // Nếu đã tạo client → giải phóng
    if (mqtt_state >= MQTT_STATE_CLIENT_CREATED) {
        ESP_LOGI("MQTT", "Releasing old client...");
        sim7600_send_cmd_str("AT+CMQTTREL=0", "OK", 2, 2000);
        mqtt_state = MQTT_STATE_STARTED;
    }

    // Nếu đã start → dừng rồi start lại cho chắc
    if (mqtt_state >= MQTT_STATE_STARTED) {
        ESP_LOGI("MQTT", "Stopping old MQTT service...");
        sim7600_send_cmd_str("AT+CMQTTSTOP", "OK", 2, 3000);
        mqtt_state = MQTT_STATE_IDLE;
    }

    if (!sim7600_ready_for_mqtt_retry(5, 2000)) {
        ESP_LOGW(TAG, "Network not ready, retry after 5s...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
    }

    // 🔁 Bắt đầu lại từ đầu
    ESP_LOGI("MQTT", "Starting new MQTT session...");
    if (!sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 3, 3000)) {
        ESP_LOGE("MQTT", "Failed to start MQTT service");
        return;
    }
    mqtt_state = MQTT_STATE_STARTED;

    // Tạo client
    char client_id[32];
    snprintf(client_id, sizeof(client_id), "ESP32_AQ-%lu", (unsigned long)xTaskGetTickCount());
    char accq_cmd[64];
    snprintf(accq_cmd, sizeof(accq_cmd), "AT+CMQTTACCQ=0,\"%s\",1", client_id);

    if (!sim7600_send_cmd_str(accq_cmd, "OK", 2, 2000)) {
        ESP_LOGE("MQTT", "Failed to create MQTT client");
        mqtt_state = MQTT_STATE_STARTED;
        return;
    }
    mqtt_state = MQTT_STATE_CLIENT_CREATED;

    // Connect broker
    ESP_LOGI("MQTT", "Connecting to broker...");
    if (!sim7600_send_cmd_str("AT+CMQTTCONNECT=0,\"tcp://mqtt.mcvmind.cloud:1883\",60,1,mqttuser,Vht@2026", "+CMQTTCONNECT: 0,0", 3, 5000)) {
        ESP_LOGE("MQTT", "Failed to connect to broker");
        mqtt_state = MQTT_STATE_STARTED;
        return;
    }

    mqtt_state = MQTT_STATE_CONNECTED;
    mqtt_connected = true;

    // Subscribe lại topic
    ESP_LOGI("MQTT", "Subscribing topics...");
    sim7600_mqtt_subscribe("esp32/minhcv5/cmd1", 1);
    sim7600_mqtt_subscribe("esp32/minhcv5/cmd2", 1);
    sim7600_mqtt_subscribe("esp32/minhcv5/cmd3", 1);

    ESP_LOGI("MQTT", "Reconnect success!");
}
// ---------------- MQTT Task ----------------
void mqtt_task(void *pvParameters) {
    const TickType_t heartbeat_interval = 300000 / portTICK_PERIOD_MS; // 5 phút
    TickType_t last_heartbeat = 0;
    int count_send_fail = 0;
    int count_send_fail_alert = 0;
    while (1) {
        // 2️⃣ Gửi heartbeat mỗi 5 phút
        if (mqtt_connected && (xTaskGetTickCount() - last_heartbeat >= heartbeat_interval)) {
            ESP_LOGI(TAG, "Sending heartbeat...");
            if (!sim7600_mqtt_publish("esp32/minhcv5/heartbeat", "ESP32 ALIVE", 1)) {
                ESP_LOGE(TAG, "Heartbeat publish failed -> mark disconnected");
                count_send_fail++;
                if (count_send_fail >= 3) {
                    ESP_LOGE(TAG, "3 consecutive send failures -> reconnecting MQTT");
                    count_send_fail = 0;
                    sim7600_mqtt_reconnect();
                } 
            } else {
                count_send_fail = 0;
                last_heartbeat = xTaskGetTickCount();
            }
        }

        // 3️⃣ Kiểm tra GPIO, nếu LOW thì gửi alert
        int level = gpio_get_level(ALERT_INPUT_GPIO);
        if (level == 0 && debounce_gpio(ALERT_INPUT_GPIO, 0, 5, 10)) {
            ESP_LOGI(TAG, "🚨 ALERT GPIO detected! Sending MQTT alert...");

            if (!sim7600_mqtt_publish("esp32/minhcv5/alert", "ESP32 ALERT!", 1)) {
                ESP_LOGE(TAG, "Alert publish failed -> mark disconnected");
                count_send_fail_alert++;
                if (count_send_fail_alert >= 3) {
                    ESP_LOGE(TAG, "3 consecutive send failures -> reconnecting MQTT");
                    count_send_fail_alert = 0;
                    sim7600_mqtt_reconnect();
                }
            }
            else {
                count_send_fail_alert = 0;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (!mqtt_connected) {
			ESP_LOGI(TAG,"Start Connect MQTT");
            sim7600_mqtt_reconnect();
        }

        // 4️⃣ Delay vòng lặp
        vTaskDelay(300 / portTICK_PERIOD_MS);
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
	esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "==== SYSTEM START ====");

    // Init GPIO cho SIM7600 (PWRKEY, v.v…)
    sim7600_gpio_init();

    // Init GPIO ALERT + RESET
    gpio_app_init();

    // Init UART giao tiếp SIM7600
    sim7600_uart_init();

    // Kiểm tra tình trạng SIM7600 trước khi bật
    ESP_LOGI(TAG, "Checking SIM7600 status...");

	bool sim_ok;
	sim_ok = sim7600_basic_check();
	ESP_LOGI(TAG, "After check sim7600 basic");
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

    sim7600_event_queue = xQueueCreate(10, sizeof(sim7600_event_t));
    sim7600_set_event_queue(sim7600_event_queue);
    xTaskCreate(sim7600_event_task, "sim7600_event_task", 4096, NULL, 10, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 9, NULL);

    ESP_LOGI(TAG, "==== SYSTEM READY ====");
}
