#include "sim7600.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "SIM7600";
static bool sim7600_is_on = false;

// Queue cho response và URC
static QueueHandle_t sim7600_resp_queue;
static QueueHandle_t sim7600_urc_queue;
static QueueHandle_t uart_event_queue;

typedef struct {
    char data[BUF_SIZE];
} sim7600_msg_t;

// ---------------- UART Event Task ----------------
static void sim7600_uart_event_task(void *arg) {
    uart_event_t event;
    uint8_t buf[BUF_SIZE];

    while (1) {
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA: {
                int len = uart_read_bytes(SIM7600_UART_NUM, buf, sizeof(buf)-1, 20 / portTICK_PERIOD_MS);
                if (len > 0) {
                    buf[len] = 0;
                    ESP_LOGI(TAG, "UART RX: %s", buf);

                    sim7600_msg_t msg;
                    strncpy(msg.data, (char*)buf, sizeof(msg.data));

                    // URC hay response
                    if (strstr((char*)buf, "RDY") ||
					    strstr((char*)buf, "+CPIN:") ||
					    strstr((char*)buf, "Call Ready") ||
					    strstr((char*)buf, "SMS") ||
					    strstr((char*)buf, "SMS DONE") ||
					    strstr((char*)buf, "+CMTI:") ||
					    strstr((char*)buf, "+CREG:")) {
					    // Đây là URC
					    xQueueSend(sim7600_urc_queue, &msg, 0);
					} else {
					    // Mặc định coi là response cho command
					    xQueueSend(sim7600_resp_queue, &msg, 0);
					}
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART overflow");
                uart_flush_input(SIM7600_UART_NUM);
                xQueueReset(uart_event_queue);
                break;
            default:
                break;
            }
        }
    }
}

// ---------------- UART Init ----------------
void sim7600_uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = SIM7600_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk= UART_SCLK_APB,
    };

    uart_driver_install(SIM7600_UART_NUM, BUF_SIZE*2, 0, 20, &uart_event_queue, 0);
    uart_param_config(SIM7600_UART_NUM, &uart_config);
    uart_set_pin(SIM7600_UART_NUM, SIM7600_TXD_PIN, SIM7600_RXD_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Tạo queue
    sim7600_resp_queue = xQueueCreate(5, sizeof(sim7600_msg_t));
    sim7600_urc_queue  = xQueueCreate(5, sizeof(sim7600_msg_t));

    // Task reader
    xTaskCreate(sim7600_uart_event_task, "sim7600_uart_evt", 4096, NULL, 12, NULL);

    ESP_LOGI(TAG, "UART2 init done, baud=%d", SIM7600_BAUDRATE);
}

// ---------------- Helper ----------------
int sim7600_read_resp(char *buffer, int max_len, int timeout_ms) {
    sim7600_msg_t msg;
    if (xQueueReceive(sim7600_resp_queue, &msg, timeout_ms / portTICK_PERIOD_MS)) {
        strncpy(buffer, msg.data, max_len);
        buffer[max_len-1] = '\0';
        ESP_LOGI(TAG, "Recv: %s", buffer);
        return strlen(buffer);
    }
    ESP_LOGW(TAG, "No response within %d ms", timeout_ms);
    return 0;
}

bool sim7600_send_cmd(const char *cmd, const char *expect, int retry, int timeout_ms) {
    char buffer[BUF_SIZE];

    for (int i = 0; i < retry; i++) {
        char send_buf[128];
        snprintf(send_buf, sizeof(send_buf), "%s\r\n", cmd);
        uart_write_bytes(SIM7600_UART_NUM, send_buf, strlen(send_buf));
        ESP_LOGI(TAG, "Send: %s", cmd);

        int attempts = 5; // đọc nhiều lần trong 1 vòng retry
        while (attempts--) {
            int len = sim7600_read_resp(buffer, sizeof(buffer), timeout_ms);
            if (len > 0) {
                // Nếu có chuỗi mong đợi → thành công
                if (strstr(buffer, expect)) {
                    ESP_LOGI(TAG, "CMD OK: %s", cmd);
                    return true;
                }

                // Nếu là URC thì bỏ qua, chờ tiếp
                if (strstr(buffer, "RDY") ||
                    strstr(buffer, "+CPIN:") ||
                    strstr(buffer, "+CREG:") ||
                    strstr(buffer, "+CGATT:") ||
                    strstr(buffer, "+CMQTT")) {
                    ESP_LOGW(TAG, "Skip URC while waiting: %s", buffer);
                    continue;
                }

                // Không phải URC cũng không khớp expect → cảnh báo
                ESP_LOGW(TAG, "Unexpected resp: %s", buffer);
            }
        }
        ESP_LOGW(TAG, "Retry %d/%d failed for %s", i+1, retry, cmd);
    }

    ESP_LOGE(TAG, "CMD FAILED: %s", cmd);
    return false;
}



bool sim7600_basic_check(void) {
    // Check "AT"
    if (!sim7600_send_cmd("AT", "OK", 3,2000)) {
        ESP_LOGE("SIM7600", "No response to AT");
        return false;
    }

    // Disable echo "ATE0"
    if (!sim7600_send_cmd("ATE0", "OK", 3,2000)) {
        ESP_LOGW("SIM7600", "ATE0 failed, but continue...");
    } else {
        ESP_LOGI("SIM7600", "Echo disabled (ATE0 OK)");
    }

    return true;
}

// ---------------- Power Control ----------------
void sim7600_power_on(void) {
    ESP_LOGI(TAG, "Powering ON SIM7600...");
    gpio_set_level(SIM7600_PWRKEY_GPIO, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(SIM7600_PWRKEY_GPIO, 1);
    vTaskDelay(16000 / portTICK_PERIOD_MS);
    sim7600_is_on = true;
    ESP_LOGI(TAG, "SIM7600 powered ON");
}

void sim7600_power_off(void) {
    ESP_LOGI(TAG, "Powering OFF SIM7600...");
    gpio_set_level(SIM7600_PWRKEY_GPIO, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    gpio_set_level(SIM7600_PWRKEY_GPIO, 1);
    vTaskDelay(25000 / portTICK_PERIOD_MS);
    sim7600_is_on = false;
    ESP_LOGI(TAG, "SIM7600 powered OFF");
}

void sim7600_reset_module(void) {
    if (!sim7600_is_on) {
        ESP_LOGW(TAG, "SIM7600 not powered on yet, skip reset!");
        return;
    }
    ESP_LOGW(TAG, "Resetting SIM7600 via power cycle...");
    sim7600_power_off();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    sim7600_power_on();
    ESP_LOGI(TAG, "SIM7600 reset complete");
}

// ---------------- Check Functions ----------------
bool sim7600_check_signal(void) {
    ESP_LOGI(TAG, "Checking signal quality...");
    if (!sim7600_send_cmd("AT+CSQ", "+CSQ:", 3, 2000)) return false;

    // đọc URC từ urc_queue
    sim7600_msg_t msg;
    if (xQueueReceive(sim7600_urc_queue, &msg, 1000 / portTICK_PERIOD_MS)) {
        char *p = strstr(msg.data, "+CSQ:");
        if (p) {
            int rssi;
            sscanf(p, "+CSQ: %d", &rssi);
            ESP_LOGI(TAG, "RSSI=%d", rssi);
            return (rssi >= 10 && rssi <= 31);
        }
    }
    return false;
}

bool sim7600_check_network(void) {
    ESP_LOGI(TAG, "Checking network...");
    if (!sim7600_send_cmd("AT+CREG?", "+CREG:", 3, 2000)) return false;

    sim7600_msg_t msg;
    if (xQueueReceive(sim7600_urc_queue, &msg, 1000 / portTICK_PERIOD_MS)) {
        char *p = strstr(msg.data, "+CREG:");
        if (p) {
            int n, stat;
            sscanf(p, "+CREG: %d,%d", &n, &stat);
            ESP_LOGI(TAG, "CREG stat=%d", stat);
            return (stat == 1 || stat == 5);
        }
    }
    return false;
}

bool sim7600_check_gprs(void) {
    ESP_LOGI(TAG, "Checking GPRS attach...");
    if (!sim7600_send_cmd("AT+CGATT?", "+CGATT:", 3, 2000)) return false;

    sim7600_msg_t msg;
    if (xQueueReceive(sim7600_urc_queue, &msg, 1000 / portTICK_PERIOD_MS)) {
        if (strstr(msg.data, "+CGATT: 1")) return true;
    }
    return false;
}

bool sim7600_ready_for_mqtt(void) {
    // Step 1: Basic check (AT + ATE0)
    if (!sim7600_basic_check()) {
        return false;
    }

    // Step 2: Check CSQ (signal quality)
    if (!sim7600_send_cmd("AT+CSQ", "+CSQ", 3, 2000)) {
    ESP_LOGE("SIM7600", "CSQ failed");
    return false;
	}

	// Step 3: Check CREG (network registration)
	if (!sim7600_send_cmd("AT+CREG?", "+CREG: 0,1", 3, 3000) &&
	    !sim7600_send_cmd("AT+CREG?", "+CREG: 0,5", 3, 3000)) {
	    ESP_LOGE("SIM7600", "CREG not registered");
	    return false;
	}
	
	// Step 4: Check CGATT (GPRS attach)
	if (!sim7600_send_cmd("AT+CGATT?", "+CGATT: 1", 3, 3000)) {
	    ESP_LOGE("SIM7600", "CGATT not attached");
	    return false;
	}

    ESP_LOGI("SIM7600", "SIM7600 is ready for MQTT");
    return true;
}


// ---------------- MQTT Functions ----------------
bool sim7600_mqtt_start(void) {
    return sim7600_send_cmd("AT+CMQTTSTART", "OK", 3, 5000);
}

bool sim7600_mqtt_connect(const char *client_id, const char *broker, int keepalive, int cleansession) {
    if (!sim7600_mqtt_start()) return false;

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"", client_id);
    if (!sim7600_send_cmd(cmd, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",%d,%d,\"\",\"\"", broker, keepalive, cleansession);
    uart_write_bytes(SIM7600_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(SIM7600_UART_NUM, "\r\n", 2);

    sim7600_msg_t msg;
    for (int i = 0; i < 10; i++) {
        if (xQueueReceive(sim7600_urc_queue, &msg, 1000 / portTICK_PERIOD_MS)) {
            if (strstr(msg.data, "+CMQTTCONNECT: 0,0")) return true;
            if (strstr(msg.data, "+CMQTTCONNECT: 0,1")) return false;
        }
    }
    return false;
}

bool sim7600_mqtt_publish(const char *topic, const char *payload, int qos) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", (int)strlen(topic));
    if (!sim7600_send_cmd(cmd, ">", 3, 2000)) return false;
    if (!sim7600_send_cmd(topic, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", (int)strlen(payload));
    if (!sim7600_send_cmd(cmd, ">", 3, 2000)) return false;
    if (!sim7600_send_cmd(payload, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,%d,60", qos);
    return sim7600_send_cmd(cmd, "OK", 3, 5000);
}

bool sim7600_mqtt_disconnect(void) {
    if (!sim7600_send_cmd("AT+CMQTTDISC=0,60", "OK", 3, 3000)) return false;
    if (!sim7600_send_cmd("AT+CMQTTREL=0", "OK", 3, 2000)) return false;
    if (!sim7600_send_cmd("AT+CMQTTSTOP", "OK", 3, 3000)) return false;
    return true;
}


bool sim7600_check_signal_with_retry(int retries, int delay_ms) {
    for (int i = 0; i < retries; i++) {
        if (sim7600_check_signal()) {   // hàm gốc gửi AT+CSQ
            return true;                // thành công
        }
        ESP_LOGW("SIM7600", "CSQ check failed (attempt %d/%d), retrying...", i + 1, retries);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
    return false;   // sau N lần vẫn fail thì coi như lỗi thật
}

bool sim7600_ready_for_mqtt_retry(int retries, int delay_ms) {
    for (int i = 0; i < retries; i++) {
        if (sim7600_ready_for_mqtt()) {
            return true;
        }
        ESP_LOGW("SIM7600", "Module not ready for MQTT yet (try %d/%d)", i+1, retries);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
    return false;
}