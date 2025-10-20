#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

#include "gpio_config.h"
#include "sim7600.h"

#define ALERT_INPUT_GPIO_1   GPIO_NUM_27   // input trigger
#define ALERT_INPUT_GPIO_2   GPIO_NUM_25  // input trigger
#define ALERT_INPUT_GPIO_3   GPIO_NUM_32   // input trigger
#define RESET_OUTPUT_GPIO_1  GPIO_NUM_14  // output reset
#define RESET_OUTPUT_GPIO_2  GPIO_NUM_26
#define RESET_OUTPUT_GPIO_3  GPIO_NUM_33
#define KEY_1               123456
#define KEY_2               123457
#define KEY_3               123458
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
                    char topic_check[64];
                    snprintf(topic_check, sizeof(topic_check), "smart/devices/%d/control", KEY_1);
                    char topic_check_2[64];
                    snprintf(topic_check_2, sizeof(topic_check_2), "smart/devices/%d/control", KEY_2);
                    char topic_check_3[64];
                    snprintf(topic_check_3, sizeof(topic_check_3), "smart/devices/%d/control", KEY_3);
                    if (strcmp(evt.topic, topic_check) == 0) {
                        ESP_LOGI("MQTT_TASK", "CMD1 -> toggle LED");
                        gpio_set_level(RESET_OUTPUT_GPIO_1, 1);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        gpio_set_level(RESET_OUTPUT_GPIO_1, 0);
                        // xử lý action 1
                    } 
                    else if (strcmp(evt.topic, topic_check_2) == 0) {
                        ESP_LOGI("MQTT_TASK", "CMD2 -> reset SIM");
                        gpio_set_level(RESET_OUTPUT_GPIO_2, 1);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        gpio_set_level(RESET_OUTPUT_GPIO_2, 0);
                        // xử lý action 2
                    } 
                    else if (strcmp(evt.topic, topic_check_3) == 0) {
                        ESP_LOGI("MQTT_TASK", "CMD3 -> custom action");
                        gpio_set_level(RESET_OUTPUT_GPIO_3, 1);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        gpio_set_level(RESET_OUTPUT_GPIO_3, 0);
                        // xử lý action 3
                    }
                    break;
				case SIM7600_EVENT_MQTT_CONNECTED:
				 	ESP_LOGI(TAG, "[EVENT] MQTT CONNECTED");
				 	mqtt_connected = true;
                    break;
                case SIM7600_EVENT_NETWORK_LOST:
                    ESP_LOGW(TAG, "[EVENT] Network Lost!");
                    break;
                case SIM7600_EVENT_MQTT_PUB_SUCCESS:
                    ESP_LOGI(TAG, "[EVENT] MQTT Publish Success!");
                    break;
                case SIM7600_EVENT_MQTT_PUB_FAILED:
                    ESP_LOGE(TAG, "[EVENT] MQTT Publish Failed!");
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
        sim7600_send_cmd_str("AT+CMQTTDISC=0,60", "OK", 1, 2000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mqtt_connected = false;
    }

    // Nếu đã tạo client → giải phóng
    if (mqtt_state >= MQTT_STATE_CLIENT_CREATED) {
        ESP_LOGI("MQTT", "Releasing old client...");
        sim7600_send_cmd_str("AT+CMQTTREL=0", "OK", 1, 2000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        mqtt_state = MQTT_STATE_STARTED;
    }

    // Nếu đã start → dừng rồi start lại cho chắc
    if (mqtt_state >= MQTT_STATE_STARTED) {
        ESP_LOGI("MQTT", "Stopping old MQTT service...");
        sim7600_send_cmd_str("AT+CMQTTSTOP", "OK", 1, 3000);
        mqtt_state = MQTT_STATE_IDLE;
    }
    

    if (!sim7600_ready_for_mqtt_retry(2, 2000)) {
        ESP_LOGW(TAG, "Network not ready, retry after 5s...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        return;
    }

    // 🔁 Bắt đầu lại từ đầu
    ESP_LOGI("MQTT", "Starting new MQTT session...");
    if (!sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 1, 3000)) {
        ESP_LOGE("MQTT", "Failed to start MQTT service");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }
    mqtt_state = MQTT_STATE_STARTED;

    // Tạo client
    char client_id[32];
    snprintf(client_id, sizeof(client_id), "ESP32_AQ-%lu", (unsigned long)xTaskGetTickCount());
    char accq_cmd[64];
    snprintf(accq_cmd, sizeof(accq_cmd), "AT+CMQTTACCQ=0,\"%s\"", client_id);
	ESP_LOGI("Main_minhcv", "Check accq_cmd: %s", accq_cmd);
    if (!sim7600_send_cmd_str(accq_cmd, "OK", 1, 2000)) {
        ESP_LOGE("MQTT", "Failed to create MQTT client");
        mqtt_state = MQTT_STATE_STARTED;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }
    mqtt_state = MQTT_STATE_CLIENT_CREATED;

    // Connect broker
    ESP_LOGI("MQTT", "Connecting to broker...");
    if (!sim7600_send_cmd_str("AT+CMQTTCONNECT=0,\"tcp://mqtt.mcvmind.cloud:1883\",60,1,\"mqttuser\",\"Vht@2026\"","OK", 1, 10000)) {
        ESP_LOGE("MQTT", "Failed to connect to broker");
        mqtt_state = MQTT_STATE_STARTED;
        return;
    }
    /*if (!sim7600_send_cmd_str("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,1","+CMQTTCONNECT: 0,0", 2, 5000)){
		ESP_LOGE("MQTT", "Failed to connect to broker");
        mqtt_state = MQTT_STATE_STARTED;
        return;
	}*/

    mqtt_state = MQTT_STATE_CONNECTED;
    mqtt_connected = true;
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	sim7600_send_cmd_str("AT", "OK", 1, 2000);
	vTaskDelay(6000 / portTICK_PERIOD_MS);
    // Subscribe lại topic
    ESP_LOGI("MQTT", "Subscribing topics...");
    char str[64];
    snprintf(str, sizeof(str), "smart/devices/%d/control", KEY_1);
    sim7600_mqtt_subscribe(str, 0);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    str[0] = '\0';
    snprintf(str, sizeof(str), "smart/devices/%d/control", KEY_2);
    sim7600_mqtt_subscribe(str, 0);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    str[0] = '\0';
    snprintf(str, sizeof(str), "smart/devices/%d/control", KEY_3);
    sim7600_mqtt_subscribe(str, 0);
    ESP_LOGI("MQTT", "Reconnect success!");
}
// ---------------- MQTT Task ----------------
void mqtt_task(void *pvParameters) {
    const TickType_t heartbeat_interval = 300000 / portTICK_PERIOD_MS; // 5 phút
    TickType_t last_heartbeat_1 = 0;
    TickType_t last_heartbeat_2 = 0;
    TickType_t last_heartbeat_3 = 0;
    int count_send_fail = 0;
    int count_send_fail_2 = 0;
    int count_send_fail_3 = 0;
    int count_send_fail_alert = 0;
    int count_send_fail_alert_2 = 0;
    int count_send_fail_alert_3 = 0;
    int count_reconnect = 0;
    int count_network_fail = 0;
    TickType_t last_alert_sent_1 = 0;
    TickType_t last_alert_sent_2 = 0;
    TickType_t last_alert_sent_3 = 0;

    // const TickType_t alert_cooldown = 180000 / portTICK_PERIOD_MS; // 3 phút
    const TickType_t alert_cooldown = 60000 / portTICK_PERIOD_MS; // 1 phút
    esp_task_wdt_init(120, true);
    esp_task_wdt_add(NULL);
    esp_task_wdt_status(NULL);
    esp_task_wdt_reset();
    while (1) {
        // 2️⃣ Gửi heartbeat mỗi 5 phút
        if (mqtt_connected && (xTaskGetTickCount() - last_heartbeat_1 >= heartbeat_interval)) {
            ESP_LOGI(TAG, "Sending heartbeat...");
            char topic[64];
            snprintf(topic, sizeof(topic), "smart/devices/%d/lastactive", KEY_1);
            char content[32];
            snprintf(content, sizeof(content), "{\"status\":\"alive\",\"key\":%d}", KEY_1);
            if (!sim7600_mqtt_publish(topic, content, 1)) {
                ESP_LOGE(TAG, "Heartbeat publish failed -> mark disconnected");
                count_send_fail++;
                if (count_send_fail >= 5) {
                    ESP_LOGE(TAG, "5 consecutive send failures -> reconnecting MQTT");
                    count_send_fail = 0;
                    mqtt_connected = false;
                } 
            } else {
                ESP_LOGI(TAG, "Heartbeat publish success for KEY_1");
                count_send_fail = 0;
                last_heartbeat_1 = xTaskGetTickCount();
            }
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            float vbat = read_battery_voltage();
            ESP_LOGI(TAG, "Battery voltage: %.2f V", vbat); 
            float percent = voltageToPercent(vbat, 3.5, 4.2);
            ESP_LOGI(TAG, "Battery percent: %.2f %%", percent);
            char topic_battery[64];
            snprintf(topic_battery, sizeof(topic_battery), "smart/devices/%d/battery", KEY_1);
            char content_battery[128];
            snprintf(content_battery, sizeof(content_battery), "{\"voltage\":%.2f,\"percent\":%.2f,\"key\":%d}", vbat, percent, KEY_1);
            if (!sim7600_mqtt_publish(topic_battery, content_battery, 1 )) {
                ESP_LOGE(TAG, "Battery publish failed -> mark disconnected");
                count_send_fail++;
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                if (count_send_fail >= 5) {
                    ESP_LOGE(TAG, "5 consecutive send failures -> reconnecting MQTT");
                    count_send_fail = 0;
                    mqtt_connected = false;
                } 
            } else {
                ESP_LOGI(TAG, "Battery publish success for KEY_1");
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                count_send_fail = 0;
                last_heartbeat_1 = xTaskGetTickCount();
            }
        }

        // Gửi heartbeat cho key 2
        if (mqtt_connected && (xTaskGetTickCount() - last_heartbeat_2 >= heartbeat_interval && KEY_2 != KEY_1)) {
            ESP_LOGI(TAG, "Sending heartbeat for KEY_2...");
            char topic[64];
            snprintf(topic, sizeof(topic), "smart/devices/%d/lastactive", KEY_2);
            char content[32];
            snprintf(content, sizeof(content), "{\"status\":\"alive\",\"key\":%d}", KEY_2);
            if (!sim7600_mqtt_publish(topic, content, 1)) {
                ESP_LOGE(TAG, "Heartbeat publish failed -> mark disconnected");
                count_send_fail_2++;
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                if (count_send_fail_2 >= 5) {
                    ESP_LOGE(TAG, "5 consecutive send failures -> reconnecting MQTT");
                    count_send_fail_2 = 0;
                    mqtt_connected = false;
                } 
            } else {
                ESP_LOGI(TAG, "Heartbeat publish success for KEY_2");
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                count_send_fail_2 = 0;
                last_heartbeat_2 = xTaskGetTickCount();
            }
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            float vbat = read_battery_voltage();
            ESP_LOGI(TAG, "Battery voltage: %.2f V", vbat);
            float percent = voltageToPercent(vbat, 3.5, 4.2);
            ESP_LOGI(TAG, "Battery percent: %.2f %%", percent);
            char topic_battery[64];
            snprintf(topic_battery, sizeof(topic_battery), "smart/devices/%d/battery", KEY_2);
            char content_battery[128];
            snprintf(content_battery, sizeof(content_battery), "{\"voltage\":%.2f,\"percent\":%.2f,\"key\":%d}", vbat, percent, KEY_2);
            if (!sim7600_mqtt_publish(topic_battery, content_battery, 1)) {
                ESP_LOGE(TAG, "Battery publish failed -> mark disconnected");
                count_send_fail_2++;
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                if (count_send_fail_2 >= 5) {
                    ESP_LOGE(TAG, "5 consecutive send failures -> reconnecting MQTT");
                    count_send_fail_2 = 0;
                    mqtt_connected = false;
                } 
            } else {
                ESP_LOGI(TAG, "Battery publish success for KEY_2");
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                count_send_fail_2 = 0;
                last_heartbeat_2 = xTaskGetTickCount();
            }
        }

        // Gửi heartbeat cho key 3
        if (mqtt_connected && (xTaskGetTickCount() - last_heartbeat_3 >= heartbeat_interval && KEY_3 != KEY_1)) {
            ESP_LOGI(TAG, "Sending heartbeat for KEY_3...");
            char topic[64];
            snprintf(topic, sizeof(topic), "smart/devices/%d/lastactive", KEY_3);
            char content[32];
            snprintf(content, sizeof(content), "{\"status\":\"alive\",\"key\":%d}", KEY_3);
            if (!sim7600_mqtt_publish(topic, content, 1)) {
                ESP_LOGE(TAG, "Heartbeat publish failed -> mark disconnected");
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                count_send_fail_3++;
                if (count_send_fail_3 >= 5) {
                    ESP_LOGE(TAG, "5 consecutive send failures -> reconnecting MQTT");
                    count_send_fail_3 = 0;
                    mqtt_connected = false;
                } 
            } else {
                ESP_LOGI(TAG, "Heartbeat publish success for KEY_3");
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                count_send_fail_3 = 0;
                last_heartbeat_3 = xTaskGetTickCount();
            }
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            float vbat = read_battery_voltage();
            ESP_LOGI(TAG, "Battery voltage: %.2f V", vbat);
            float percent = voltageToPercent(vbat, 3.5, 4.2);
            ESP_LOGI(TAG, "Battery percent: %.2f %%", percent);
            char topic_battery[64];
            snprintf(topic_battery, sizeof(topic_battery), "smart/devices/%d/battery", KEY_3);
            char content_battery[128];
            snprintf(content_battery, sizeof(content_battery), "{\"voltage\":%.2f,\"percent\":%.2f,\"key\":%d}", vbat, percent, KEY_3);
            if (!sim7600_mqtt_publish(topic_battery, content_battery, 1)) {
                ESP_LOGE(TAG, "Battery publish failed -> mark disconnected");
                count_send_fail_3++;
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                if (count_send_fail_3 >= 5) {
                    ESP_LOGE(TAG, "5 consecutive send failures -> reconnecting MQTT");
                    count_send_fail_3 = 0;
                    mqtt_connected = false;
                } 
            } else {
                ESP_LOGI(TAG, "Battery publish success for KEY_3");
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                count_send_fail_3 = 0;
                last_heartbeat_3 = xTaskGetTickCount();
            }
        }

        // 3️⃣ Kiểm tra GPIO, nếu LOW thì gửi alert
        int level = gpio_get_level(ALERT_INPUT_GPIO_1);
        if (level == 0 && debounce_gpio(ALERT_INPUT_GPIO_1, 0, 5, 10)) {
            ESP_LOGI(TAG, "ALERT GPIO detected! Sending MQTT alert from KEY_1...");
            TickType_t now = xTaskGetTickCount();
            if (now - last_alert_sent_1 >= alert_cooldown) {
                ESP_LOGW(TAG, "Alert sent cooldown passed, sending alert...");
                if (mqtt_connected){
                  char topic[64];
                  snprintf(topic, sizeof(topic), "smart/devices/%d/info", KEY_1);
                  char content[128];
                  snprintf(content, sizeof(content), "{\"alert\":\"true\", \"alarm_light\":\"true\",\"key\":%d}", KEY_1);
				  if (!sim7600_mqtt_publish(topic, content, 0)) {
		                ESP_LOGE(TAG, "Alert publish failed -> mark disconnected");
		                count_send_fail_alert++;
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
		                if (count_send_fail_alert >= 3) {
		                    ESP_LOGE(TAG, "3 consecutive send failures -> reconnecting MQTT");
		                    count_send_fail_alert = 0;
                            mqtt_connected = false;
		                }
		          } else {
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
		                count_send_fail_alert = 0;
                        last_alert_sent_1 = now; // update timestamp
		          }
			    }		
                vTaskDelay(1500 / portTICK_PERIOD_MS);
            }
        }
        int level_2 = gpio_get_level(ALERT_INPUT_GPIO_2);
        if (level_2 == 0 && debounce_gpio(ALERT_INPUT_GPIO_2, 0, 5, 10)) {
            ESP_LOGI(TAG, "ALERT GPIO detected! Sending MQTT alert from KEY_2...");
            TickType_t now = xTaskGetTickCount();
            if (now - last_alert_sent_2 >= alert_cooldown) {
                ESP_LOGW(TAG, "Alert sent cooldown passed, sending alert...");
                if (mqtt_connected){
                    char topic[64];
                    snprintf(topic, sizeof(topic), "smart/devices/%d/info", KEY_2);
                    char content[128];
                    snprintf(content, sizeof(content), "{\"alert\":\"true\", \"alarm_light\":\"true\",\"key\":%d}", KEY_2);
                    if (!sim7600_mqtt_publish(topic, content, 0)) {
                        ESP_LOGE(TAG, "Alert publish failed -> mark disconnected");
                        count_send_fail_alert++;
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
                        if (count_send_fail_alert >= 3) {
                            ESP_LOGE(TAG, "3 consecutive send failures -> reconnecting MQTT");
                            count_send_fail_alert = 0;
                            mqtt_connected = false;
                        }
                    } else {
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
                        count_send_fail_alert = 0;
                        last_alert_sent_2 = now; // update timestamp
                    }
			    }
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        int level_3 = gpio_get_level(ALERT_INPUT_GPIO_3);
        if (level_3 == 0 && debounce_gpio(ALERT_INPUT_GPIO_3, 0, 5, 10)) {
            ESP_LOGI(TAG, "ALERT GPIO detected! Sending MQTT alert from KEY_3...");
            TickType_t now = xTaskGetTickCount();
            if (now - last_alert_sent_3 >= alert_cooldown) {
                ESP_LOGW(TAG, "Alert sent cooldown passed, sending alert...");
                if (mqtt_connected){
                    char topic[64];
                    snprintf(topic, sizeof(topic), "smart/devices/%d/info", KEY_3);
                    char content[128];
                    snprintf(content, sizeof(content), "{\"alert\":\"true\", \"alarm_light\":\"true\",\"key\":%d}", KEY_3);
                    if (!sim7600_mqtt_publish(topic, content, 0)) {
                        ESP_LOGE(TAG, "Alert publish failed -> mark disconnected");
                        count_send_fail_alert++;
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
                        if (count_send_fail_alert >= 3) {
                            ESP_LOGE(TAG, "3 consecutive send failures -> reconnecting MQTT");
                            count_send_fail_alert = 0;
                            mqtt_connected = false;
                        }
                    } else {
                        vTaskDelay(1500 / portTICK_PERIOD_MS);
                        count_send_fail_alert = 0;
                        last_alert_sent_3 = now; // update timestamp
                    }
			    }
            }
            vTaskDelay(1500 / portTICK_PERIOD_MS);
        }

        if (!mqtt_connected) {
			ESP_LOGI(TAG,"Start Connect MQTT");
			sim7600_check_signal_expect();
			sim7600_check_sim_ready(5, 2000);
            if (!sim7600_is_network_registered(3, 5000)) {
                ESP_LOGW(TAG, "Network not ready, delay longer before retrying MQTT...");
                vTaskDelay(5000 / portTICK_PERIOD_MS);  // đợi thêm
                count_network_fail++;

                // Nếu quá nhiều lần mà vẫn không có mạng thì reset module
                if (count_network_fail == 6) {
                    ESP_LOGE(TAG, "No network after 6 tries -> reset module");
                    sim7600_reset_module();
                    vTaskDelay(20000 / portTICK_PERIOD_MS);
                    sim7600_basic_check();
                }

                // Nếu vẫn thất bại sau 10 lần thì restart ESP32
                if (count_network_fail >= 10) {
                    ESP_LOGE(TAG, "No network after 10 tries -> restart ESP32");
                    esp_restart();
                }
            } else {
                count_network_fail = 0;
                vTaskDelay(5000 / portTICK_PERIOD_MS);  // đợi thêm
                sim7600_mqtt_reconnect();
                count_reconnect++;
                if (count_reconnect == 3){
                    sim7600_reset_module();
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    sim7600_basic_check();
                }
                if (count_reconnect == 6){
                    sim7600_reset_module();
                    esp_restart();
                }
            }
        } else {
            count_network_fail = 0;
			count_reconnect = 0;
		}

        // 4️⃣ Delay vòng lặp
        vTaskDelay(300 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();
    }
}




// ---------------- GPIO Init ----------------
static void gpio_app_init(void) {
    // Input ALERT
    gpio_config_t in_conf = {
        .pin_bit_mask = 1ULL << ALERT_INPUT_GPIO_1 | 1ULL << ALERT_INPUT_GPIO_2 | 1ULL << ALERT_INPUT_GPIO_3,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);
    
    gpio_set_level(ALERT_INPUT_GPIO_1, 1);
    gpio_set_level(ALERT_INPUT_GPIO_2, 1);
    gpio_set_level(ALERT_INPUT_GPIO_3, 1);
    // Output RESET
    gpio_config_t out_conf = {
        .pin_bit_mask = 1ULL << RESET_OUTPUT_GPIO_1 | 1ULL << RESET_OUTPUT_GPIO_2 | 1ULL << RESET_OUTPUT_GPIO_3,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_set_level(RESET_OUTPUT_GPIO_1, 0);
    gpio_set_level(RESET_OUTPUT_GPIO_2, 0);
    gpio_set_level(RESET_OUTPUT_GPIO_3, 0);
}

// ---------------- Main Entry ----------------
void app_main(void) {
	esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "==== SYSTEM START ====");

    // Init GPIO cho SIM7600 (PWRKEY, v.v…)
    sim7600_gpio_init();
    // Init ADC đọc pin
    init_adc();
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
	    sim7600_check_signal_expect();
	    if (!sim_ok) {
	        ESP_LOGW(TAG, "Still no response -> powering ON SIM7600 (2nd try)...");
	        sim7600_power_on();
	        vTaskDelay(5000 / portTICK_PERIOD_MS);
		
	        sim_ok = sim7600_basic_check();
	        sim7600_check_signal_expect();
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
    xTaskCreate(mqtt_task, "mqtt_task", 8192, NULL, 9, NULL);

    ESP_LOGI(TAG, "==== SYSTEM READY ====");
}
