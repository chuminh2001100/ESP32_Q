#include "sim7600.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

static const char *TAG = "SIM7600";
bool sim7600_waiting_resp = false;
static char sim7600_rx_buffer[BUF_SIZE];
static SemaphoreHandle_t sim7600_resp_sem;

static QueueHandle_t sim7600_event_queue = NULL;

void sim7600_set_event_queue(QueueHandle_t queue) {
    sim7600_event_queue = queue;
}
// ---------------- Helper ----------------
static inline void trim_response(char *str) {
    char *start = str;
    while (*start == '\r' || *start == '\n' || *start == ' ') start++;
    memmove(str, start, strlen(start) + 1);

    char *end = str + strlen(str) - 1;
    while (end >= str && (*end == '\r' || *end == '\n' || *end == ' ')) {
        *end-- = '\0';
    }
}
// ---------------- UART Init ----------------
void sim7600_uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = SIM7600_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk= UART_SCLK_APB,
    };

    uart_driver_install(SIM7600_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(SIM7600_UART_NUM, &uart_cfg);
    uart_set_pin(SIM7600_UART_NUM, SIM7600_TXD_PIN, SIM7600_RXD_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(sim7600_uart_event_task, "sim7600_uart_evt", 4096, NULL, 12, NULL);
    ESP_LOGI(TAG, "UART init done, baud=%d", SIM7600_BAUDRATE);
}

// ---------------- Command ----------------
bool sim7600_send_cmd_str(const char *cmd, const char *expect, int retry, int timeout_ms) {
    char tx_buf[128];
    size_t len = snprintf(tx_buf, sizeof(tx_buf), "%s\r\n", cmd);

    for (int i = 0; i < retry; i++) {
        uart_flush_input(SIM7600_UART_NUM);
        uart_write_bytes(SIM7600_UART_NUM, tx_buf, len);
        ESP_LOGI(TAG, "Send: %s", cmd);

        sim7600_waiting_resp = true;
        if (xSemaphoreTake(sim7600_resp_sem, timeout_ms / portTICK_PERIOD_MS)) {
            sim7600_waiting_resp = false;
            if (strstr(sim7600_rx_buffer, expect)) {
                ESP_LOGI(TAG, "Recv: %s", sim7600_rx_buffer);
                return true;
            }
        }
        sim7600_waiting_resp = false;
        ESP_LOGW(TAG, "Retry %d/%d", i + 1, retry);
    }

    ESP_LOGE(TAG, "CMD FAILED: %s", cmd);
    return false;
}

bool sim7600_ready_for_mqtt_retry(int retries, int delay_ms) {
    for (int i = 0; i < retries; i++) {
        ESP_LOGI(TAG, "[%d/%d] Checking SIM7600 network...", i + 1, retries);

        bool ok = true;

        // Module phản hồi?
        if (!sim7600_send_cmd_str("AT", "OK", 1, 1000)) {
            ESP_LOGW(TAG, "No response from SIM7600");
            ok = false;
        }

        // SIM sẵn sàng?
        if (ok && !sim7600_send_cmd_str("AT+CPIN?", "READY", 1, 2000)) {
            ESP_LOGW(TAG, "SIM not ready");
            ok = false;
        }

        // Đăng ký mạng GSM?
        if (ok && 
            !sim7600_send_cmd_str("AT+CREG?", "+CREG: 0,1", 1, 2000) &&
            !sim7600_send_cmd_str("AT+CREG?", "+CREG: 0,5", 1, 2000)) {
            ESP_LOGW(TAG, "Network not registered (GSM)");
            ok = false;
        }

        // Đăng ký mạng dữ liệu?
        if (ok && 
            !sim7600_send_cmd_str("AT+CGREG?", "+CGREG: 0,1", 1, 2000) &&
            !sim7600_send_cmd_str("AT+CGREG?", "+CGREG: 0,5", 1, 2000)) {
            ESP_LOGW(TAG, "Network not registered (GPRS)");
            ok = false;
        }

        // GPRS attach?
        if (ok && !sim7600_send_cmd_str("AT+CGATT?", "+CGATT: 1", 1, 2000)) {
            ESP_LOGW(TAG, "GPRS not attached");
            ok = false;
        }

        // Có IP chưa?
        if (ok && !sim7600_send_cmd_str("AT+CGPADDR", "+CGPADDR: 1,", 1, 2000)) {
            ESP_LOGW(TAG, "No IP address");
            ok = false;
        }

        if (ok) {
            ESP_LOGI(TAG, "SIM7600 network is ready!");
            return true;
        }

        // Chưa sẵn sàng, chờ rồi thử lại
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }

    ESP_LOGE(TAG, "❌ SIM7600 network NOT ready after %d retries", retries);
    return false;
}


void sim7600_uart_reader_task(void *arg) {
    uint8_t rx_buf[BUF_SIZE];
    int len;

    while (1) {
        len = uart_read_bytes(SIM7600_UART_NUM, rx_buf, sizeof(rx_buf) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            rx_buf[len] = '\0';
            trim_response((char*)rx_buf);
            ESP_LOGI(TAG, "UART recv: %s", rx_buf);

            if (sim7600_waiting_resp) {
                strncpy(sim7600_rx_buffer, (char*)rx_buf, BUF_SIZE);
                xSemaphoreGive(sim7600_resp_sem); // báo task gửi biết có phản hồi
            } else {
                // xử lý URC trực tiếp
			   if (strstr((char*)rx_buf, "+CMQTTDISC:")) {
                    sim7600_event_t ev = { .type = SIM7600_EVENT_MQTT_DISCONNECTED };
                    if (sim7600_event_queue)
                        xQueueSend(sim7600_event_queue, &ev, 0);
                } 
                else if (strstr((char*)rx_buf, "+CMQTTSUBRECV:")) {
                    sim7600_event_t ev = { .type = SIM7600_EVENT_MQTT_SUBRECV };
                    // ✅ parse theo format:
                    // +CMQTTSUBRECV: 0,<msgid>,"topic",<len>,payload
                    // Ta chỉ cần topic + payload
                    char topic[64] = {0};
                    char payload[256] = {0};

                    // Cách parse an toàn: tìm vị trí thủ công
                    char *start_topic = strchr((char*)rx_buf, '\"');
                    char *end_topic = start_topic ? strchr(start_topic + 1, '\"') : NULL;
                    if (start_topic && end_topic) {
                        size_t topic_len = end_topic - start_topic - 1;
                        strncpy(ev.topic, start_topic + 1, topic_len);
                        ev.topic[topic_len] = '\0';
                    } else {
                        strcpy(ev.topic, "unknown");
                    }

                    // Payload là phần sau dấu phẩy cuối
                    char *last_comma = strrchr((char*)rx_buf, ',');
                    if (last_comma && strlen(last_comma) > 1) {
                        strncpy(ev.payload, last_comma + 1, sizeof(ev.payload) - 1);
                        trim_response(ev.payload);
                    } else {
                        strcpy(ev.payload, "");
                    }

                    ESP_LOGI(TAG, "Parsed MQTT msg: topic='%s', payload='%s'", ev.topic, ev.payload);

                    if (sim7600_event_queue){
                        xQueueSend(sim7600_event_queue, &ev, 0);
                    } 
                }
            }
        }
    }
}

// ---------------- Basic Check ----------------
bool sim7600_basic_check(void) {
    if (!sim7600_send_cmd_str("AT", "OK", 3, 2000)) return false;
    sim7600_send_cmd_str("ATE0", "OK", 3, 2000);
    return true;
}

// ---------------- Power/Reset ----------------
void sim7600_power_on(void) {
    gpio_set_level(SIM7600_PWRKEY_GPIO, 0);
    vTaskDelay(800 / portTICK_PERIOD_MS);
    gpio_set_level(SIM7600_PWRKEY_GPIO, 1);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "SIM7600 powered ON");
}

void sim7600_power_off(void) {
    gpio_set_level(SIM7600_PWRKEY_GPIO, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    gpio_set_level(SIM7600_PWRKEY_GPIO, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "SIM7600 powered OFF");
}

void sim7600_reset_module(void) {
    ESP_LOGW(TAG, "Resetting SIM7600...");
    sim7600_power_off();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    sim7600_power_on();
}

// ---------------- MQTT ----------------
bool sim7600_mqtt_start(void) {
    if (sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 3, 5000)) return true;
    char buf[BUF_SIZE];
    if (strstr(buf, "+CMQTTSTART: 23")) {
        sim7600_send_cmd_str("AT+CMQTTSTOP", "OK", 3, 3000);
        return sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 3, 5000);
    }
    return false;
}

bool sim7600_mqtt_connect(const char *client_id, const char *broker, int keepalive, int cleansession) {
    if (!sim7600_mqtt_start()) return false;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"", client_id);
    if (!sim7600_send_cmd_str(cmd, "OK", 3, 2000)) return false;
    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",%d,%d,\"\",\"\"", broker, keepalive, cleansession);
    if (!sim7600_send_cmd_str(cmd, "OK", 3, 5000)) return false;

    sim7600_msg_t msg;
    for (int i = 0; i < 10; i++) {
        if (xQueueReceive(sim7600_urc_queue, &msg, 1000/portTICK_PERIOD_MS)) {
            if (strstr(msg.data, "+CMQTTCONNECT: 0,0")) return true;
        }
    }
    return false;
}

bool sim7600_mqtt_publish(const char *topic, const char *payload, int qos) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", (int)strlen(topic));
    if (!sim7600_send_cmd_str(cmd, ">", 3, 2000)) return false;
    if (!sim7600_send_cmd_str(topic, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", (int)strlen(payload));
    if (!sim7600_send_cmd_str(cmd, ">", 3, 2000)) return false;
    if (!sim7600_send_cmd_str(payload, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,%d,60", qos);
    return sim7600_send_cmd_str(cmd, "OK", 3, 5000);
}

bool sim7600_mqtt_disconnect(void) {
    sim7600_send_cmd_str("AT+CMQTTDISC=0,60", "OK", 3, 3000);
    sim7600_send_cmd_str("AT+CMQTTREL=0", "OK", 3, 2000);
    sim7600_send_cmd_str("AT+CMQTTSTOP", "OK", 3, 3000);
    return true;
}

bool sim7600_check_signal(int *out_rssi) {
    const char *cmd = "AT+CSQ";
    const char *expect = "+CSQ:";
    int retry = 3;
    int timeout_ms = 2000;

    // Gửi lệnh AT+CSQ
    if (!sim7600_send_cmd_str(cmd, expect, retry, timeout_ms)) {
        ESP_LOGE(TAG, "Không nhận được phản hồi CSQ");
        return false;
    }

    // Đến đây thì sim7600_rx_buffer chứa phản hồi
    // Ví dụ: "\r\n+CSQ: 20,99\r\n\r\nOK\r\n"
    char *p = strstr(sim7600_rx_buffer, "+CSQ:");
    if (!p) {
        ESP_LOGE(TAG, "Không tìm thấy +CSQ trong phản hồi");
        return false;
    }

    int rssi = 0, ber = 0;
    if (sscanf(p, "+CSQ: %d,%d", &rssi, &ber) == 2) {
        ESP_LOGI(TAG, "RSSI = %d, BER = %d", rssi, ber);
        if (out_rssi) *out_rssi = rssi;

        if (rssi == 99) {
            ESP_LOGW(TAG, "RSSI = 99 (không xác định)");
            return false;
        }

        if (rssi >= 10) {
            ESP_LOGI(TAG, "Sóng đủ mạnh (RSSI = %d)", rssi);
            return true;
        } else {
            ESP_LOGW(TAG, "Sóng yếu (RSSI = %d)", rssi);
            return false;
        }
    }

    ESP_LOGE(TAG, "Phân tích +CSQ thất bại");
    return false;
}


bool parse_mqtt_urc(const char *urc, char *topic_out, char *payload_out) {
    const char *p = strstr(urc, "+CMQTTSUBRECV:");
    if (!p) return false;

    // format: +CMQTTSUBRECV: 0,14,"device/ctrl/1",5,"ON"
    int client, topic_len, payload_len;
    char topic[64], payload[128];
    if (sscanf(p, "+CMQTTSUBRECV: %d,%d,\"%[^\"]\",%d,\"%[^\"]\"", 
               &client, &topic_len, topic, &payload_len, payload) == 5) {
        strcpy(topic_out, topic);
        strcpy(payload_out, payload);
        return true;
    }
    return false;
}

void handle_mqtt_command(const char *topic, const char *payload) {
    if (strcmp(topic, "device/ctrl/1") == 0) {
        ESP_LOGI(TAG, "Control 1 -> payload: %s", payload);
        // ví dụ: bật/tắt GPIO
    } else if (strcmp(topic, "device/ctrl/2") == 0) {
        ESP_LOGI(TAG, "Control 2 -> payload: %s", payload);
    } else if (strcmp(topic, "device/ctrl/3") == 0) {
        ESP_LOGI(TAG, "Control 3 -> payload: %s", payload);
    }
}
