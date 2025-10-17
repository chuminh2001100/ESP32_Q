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
    sim7600_resp_sem = xSemaphoreCreateBinary();
    if (sim7600_resp_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create sim7600_resp_sem!");
    }
    xTaskCreate(sim7600_uart_reader_task, "sim7600_uart_evt", 7168, NULL, 12, NULL);
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
		vTaskDelay(1000 / portTICK_PERIOD_MS);
        // SIM sẵn sàng?
        if (ok && !sim7600_send_cmd_str("AT+CPIN?", "READY", 1, 2000)) {
            ESP_LOGW(TAG, "SIM not ready");
            ok = false;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Đăng ký mạng GSM?
        if (ok && 
            !sim7600_send_cmd_str("AT+CREG?", "+CREG: 0,1", 1, 2000) &&
            !sim7600_send_cmd_str("AT+CREG?", "+CREG: 0,5", 1, 2000)) {
            ESP_LOGW(TAG, "Network not registered (GSM)");
            ok = false;
        }
        vTaskDelay(800 / portTICK_PERIOD_MS);

        // Đăng ký mạng dữ liệu?
        if (ok && 
            !sim7600_send_cmd_str("AT+CGREG?", "+CGREG: 0,1", 1, 2000) &&
            !sim7600_send_cmd_str("AT+CGREG?", "+CGREG: 0,5", 1, 2000)) {
            ESP_LOGW(TAG, "Network not registered (GPRS)");
            ok = false;
        }
		vTaskDelay(800 / portTICK_PERIOD_MS);
        // GPRS attach?
        if (ok && !sim7600_send_cmd_str("AT+CGATT?", "+CGATT: 1", 1, 2000)) {
            ESP_LOGW(TAG, "GPRS not attached");
            ok = false;
        }
		vTaskDelay(800 / portTICK_PERIOD_MS);
		
		if (ok && !sim7600_send_cmd_str("AT+CGDCONT=1,\"IP\",\"v-internet\"", "OK", 1, 3000)) {
            ESP_LOGW(TAG, "GPRS not attached");
            ok = false;
        }
		vTaskDelay(800 / portTICK_PERIOD_MS);
		if (!sim7600_send_cmd_str("AT+NETOPEN?", "+NETOPEN: 1", 1, 3000)) {
		    ESP_LOGW(TAG, "Network is closed or unknown, resetting...");
		    // ✅ Đóng mạng cũ (nếu có)
		    sim7600_send_cmd_str("AT+NETCLOSE", "OK", 1, 3000);
		    vTaskDelay(1000 / portTICK_PERIOD_MS);
		    // ✅ Mở lại mạng mới
		    if (!sim7600_send_cmd_str("AT+NETOPEN", "+NETOPEN: 1", 2, 8000)) {
		        ESP_LOGE(TAG, "Failed to open network");
		        ok = false;
		    }
		}
		vTaskDelay(800 / portTICK_PERIOD_MS);
        // Có IP chưa?
        if (ok && !sim7600_send_cmd_str("AT+CGPADDR", "+CGPADDR: 1,", 1, 2000)) {
            ESP_LOGW(TAG, "No IP address");
            ok = false;
        }
		vTaskDelay(800 / portTICK_PERIOD_MS);
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
    char urc_buffer[600]; // chứa dữ liệu tạm
    int len;

    while (1) {
        len = uart_read_bytes(SIM7600_UART_NUM, rx_buf, sizeof(rx_buf) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            rx_buf[len] = '\0';
            trim_response((char*)rx_buf);
            ESP_LOGI(TAG, "UART recv: %s", rx_buf);

            if (sim7600_waiting_resp) {
                strncpy(sim7600_rx_buffer, (char*)rx_buf, BUF_SIZE);
                xSemaphoreGive(sim7600_resp_sem);
                continue;
            }

            // --- Ghép dữ liệu vào buffer URC ---
            strncat(urc_buffer, (char*)rx_buf, sizeof(urc_buffer) - strlen(urc_buffer) - 1);

            // --- Xử lý từng khối +CMQTTRXSTART ... +CMQTTRXEND nếu đủ ---
            char *start = strstr(urc_buffer, "+CMQTTRXSTART:");
            while (start) {
                char *end = strstr(start, "+CMQTTRXEND:");
                if (!end) break; // chưa đủ -> chờ thêm

                // cắt đoạn nguyên bản tin
                char message[300];
                int msg_len = end - start + strlen("+CMQTTRXEND:") + 1;
                strncpy(message, start, msg_len);
                message[msg_len] = '\0';

                // --- Parse topic + payload ---
                char topic[100] = {0};
                char payload[100] = {0};
                char *t_start = strstr(message, "+CMQTTRXTOPIC:");
                char *p_start = strstr(message, "+CMQTTRXPAYLOAD:");

                if (t_start) {
                    char *t_line = strstr(t_start, "\r\n");
                    if (t_line) {
                        t_line += 2;
                        char *t_end = strstr(t_line, "\r\n");
                        if (t_end) *t_end = '\0';
                        strncpy(topic, t_line, sizeof(topic) - 1);
                    }
                }
                if (p_start) {
                    char *p_line = strstr(p_start, "\r\n");
                    if (p_line) {
                        p_line += 2;
                        char *p_end = strstr(p_line, "\r\n");
                        if (p_end) *p_end = '\0';
                        strncpy(payload, p_line, sizeof(payload) - 1);
                    }
                }

                ESP_LOGI(TAG, "✅ MQTT RX (multi-line): topic=%s, payload=%s", topic, payload);

                // --- Gửi event cho task chính ---
                sim7600_event_t ev = {
                    .type = SIM7600_EVENT_MQTT_SUBRECV
                };
                strncpy(ev.topic, topic, sizeof(ev.topic));
                strncpy(ev.payload, payload, sizeof(ev.payload));
                if (sim7600_event_queue)
                    xQueueSend(sim7600_event_queue, &ev, 0);

                // --- Xóa phần đã xử lý ---
                memmove(urc_buffer, end + strlen("+CMQTTRXEND:"), strlen(end + strlen("+CMQTTRXEND:")) + 1);
                start = strstr(urc_buffer, "+CMQTTRXSTART:");
            }

            // --- Nếu không phải MQTT, xử lý các URC khác ---
            if (strstr((char*)rx_buf, "+CMQTTDISC:") ||
                strstr((char*)rx_buf, "+CMQTTCONNLOST:") ||
                strstr((char*)rx_buf, "+CMQTTNONET") ||
                strstr((char*)rx_buf, "+CIPEVENT: NETWORK CLOSED UNEXPECTEDLY") ||
                strstr((char*)rx_buf, "+SIMCARD: NOT AVAILABLE")) {
                sim7600_event_t ev = { .type = SIM7600_EVENT_MQTT_DISCONNECTED };
                if (sim7600_event_queue)
                    xQueueSend(sim7600_event_queue, &ev, 0);
            }
            else if (strstr((char*)rx_buf, "+CMQTTCONNECT:")) {
                int client_id = -1, result = -1;
                if (sscanf((char*)rx_buf, "+CMQTTCONNECT: %d,%d", &client_id, &result) == 2) {
                    sim7600_event_t ev = {
                        .type = (result == 0)
                            ? SIM7600_EVENT_MQTT_CONNECTED
                            : SIM7600_EVENT_MQTT_DISCONNECTED
                    };
                    if (sim7600_event_queue)
                        xQueueSend(sim7600_event_queue, &ev, 0);
                    ESP_LOGI(TAG, "MQTT connect result=%d", result);
                } else {
                    ESP_LOGW(TAG, "Malformed +CMQTTCONNECT response: %s", rx_buf);
                }
            }
            else if (strstr((char*)rx_buf, "+CMQTTPUB:")) {
                int client_id = -1, result = -1;
                if (sscanf((char*)rx_buf, "+CMQTTPUB: %d,%d", &client_id, &result) == 2){
                    sim7600_event_t ev = {
                        .type = (result == 0) ? SIM7600_EVENT_MQTT_PUB_SUCCESS : SIM7600_EVENT_MQTT_PUB_FAILED
                    };
                    if (sim7600_event_queue)
                        xQueueSend(sim7600_event_queue, &ev, 0);
                    ESP_LOGI(TAG, "MQTT connect result=%d", result);
                } else {
                    ESP_LOGW(TAG, "Malformed +CMQTTPUB response: %s", rx_buf);
                }
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


// ---------------- Basic Check ----------------
bool sim7600_basic_check(void) {
    for (int i = 0; i < 5; i++) {
		ESP_LOGI("SIM7600", "Start check basic AT");
        if (sim7600_send_cmd_str("AT", "OK", 1, 1000)) {
            ESP_LOGI("SIM7600", "AT OK (basic check pass)");
            sim7600_send_cmd_str("ATE0", "OK", 3, 2000);
            return true;
        }
        ESP_LOGI("SIM7600","In debug fail, never exit");
        ESP_LOGW("SIM7600", "AT failed, retry %d/5...", i + 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    return false;
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
    if (sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 1, 5000)) return true;
    char buf[BUF_SIZE];
    if (strstr(buf, "+CMQTTSTART: 23")) {
		vTaskDelay(500 / portTICK_PERIOD_MS);
        sim7600_send_cmd_str("AT+CMQTTSTOP", "OK", 1, 3000);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        return sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 1, 5000);
    }
    return false;
}

bool sim7600_mqtt_connect(const char *client_id, const char *broker, int keepalive, int cleansession) {
    if (!sim7600_mqtt_start()) return false;
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"", client_id);
    if (!sim7600_send_cmd_str(cmd, "OK", 3, 2000)) return false;
    vTaskDelay(300 / portTICK_PERIOD_MS);
    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",%d,%d,\"\",\"\"", broker, keepalive, cleansession);
    if (!sim7600_send_cmd_str(cmd, "OK", 3, 5000)) return false;

    sim7600_event_t msg;
    for (int i = 0; i < 10; i++) {
        if (xQueueReceive(sim7600_event_queue, &msg, 1000/portTICK_PERIOD_MS)) {
            if (strstr(msg.payload, "+CMQTTCONNECT: 0,0")) return true;
        }
    }
    return false;
}

bool sim7600_mqtt_publish(const char *topic, const char *payload, int qos) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", (int)strlen(topic));
    if (!sim7600_send_cmd_str(cmd, ">", 1, 2000)) return false;
    if (!sim7600_send_cmd_str(topic, "OK", 1, 2000)) return false;
	vTaskDelay(500 / portTICK_PERIOD_MS);
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", (int)strlen(payload));
    if (!sim7600_send_cmd_str(cmd, ">", 1, 2000)) return false;
    if (!sim7600_send_cmd_str(payload, "OK", 1, 2000)) return false;
	vTaskDelay(500 / portTICK_PERIOD_MS);
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,%d,60", qos);
    return sim7600_send_cmd_str(cmd, "OK", 1, 5000);
}

bool sim7600_mqtt_subscribe(const char *topic, int qos) {
    char cmd[128];

    // Bước 1: gửi lệnh khai báo độ dài topic và QoS
    snprintf(cmd, sizeof(cmd), "AT+CMQTTSUBTOPIC=0,%d,%d", (int)strlen(topic), qos);
    if (!sim7600_send_cmd_str(cmd, ">", 1, 2000)) return false;

    // Bước 2: gửi nội dung topic
    if (!sim7600_send_cmd_str(topic, "OK", 1, 2000)) return false;

    // Bước 3: thực hiện lệnh subscribe
    // msgid = 1, timeout = 60
    snprintf(cmd, sizeof(cmd), "AT+CMQTTSUB=0");
    if (!sim7600_send_cmd_str(cmd, "OK", 1, 5000)) return false;

    // Có thể đợi URC: +CMQTTSUB: 0,0 báo subscribe thành công (nếu muốn)
    return true;
}

bool sim7600_mqtt_disconnect(void) {
    sim7600_send_cmd_str("AT+CMQTTDISC=0,60", "OK", 1, 3000);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    sim7600_send_cmd_str("AT+CMQTTREL=0", "OK", 1, 2000);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    sim7600_send_cmd_str("AT+CMQTTSTOP", "OK", 1, 3000);
    vTaskDelay(500 / portTICK_PERIOD_MS);
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

bool sim7600_is_network_registered(int retry, int delay_ms) {
    for (int i = 0; i < retry; i++) {
        // Kiểm tra CREG
        if (sim7600_send_cmd_str("AT+CREG?", "+CREG: 0,1", 1, 1000) ||
            sim7600_send_cmd_str("AT+CREG?", "+CREG: 0,5", 1, 1000)) {
            ESP_LOGI("SIM7600", "Network registered (CREG OK)");
            return true;
        }
        ESP_LOGW("SIM7600", "Network not ready, retry %d/%d", i+1, retry);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
    return false;
}

bool sim7600_check_signal_expect(void) {
    ESP_LOGI("SIM7600", "Checking signal strength...");
    // Lệnh AT+CSQ sẽ trả về dạng "+CSQ: x,y"
    if (sim7600_send_cmd_str("AT+CSQ", "+CSQ:", 1, 1000)) {
        ESP_LOGI("SIM7600", "Signal strength command OK (+CSQ received)");
        return true;
    } else {
        ESP_LOGW("SIM7600", "Failed to get signal strength (+CSQ not found)");
        return false;
    }
}


bool sim7600_check_sim_ready(int retry, int delay_ms) {
    for (int i = 0; i < retry; i++) {
        if (sim7600_send_cmd_str("AT+CPIN?", "+CPIN: READY", 1, 1000)) {
            ESP_LOGI("SIM7600", "✅ SIM sẵn sàng (+CPIN: READY)");
            return true;
        } else if (sim7600_send_cmd_str("AT+CPIN?", "+CPIN: SIM PIN", 1, 1000)) {
            ESP_LOGE("SIM7600", "🔐 SIM bị khóa PIN! Cần nhập mã PIN.");
            return false;
        } else if (sim7600_send_cmd_str("AT+CPIN?", "+CPIN: SIM PUK", 1, 1000)) {
            ESP_LOGE("SIM7600", "🔐 SIM bị khóa PUK! Cần nhập mã PUK.");
            return false;
        } else if (sim7600_send_cmd_str("AT+CPIN?", "+CPIN: NOT INSERTED", 1, 1000)) {
            ESP_LOGE("SIM7600", "❌ sim not inserted!");
            return false;
        } else {
            ESP_LOGW("SIM7600", "Sim not ready %d/%d", i + 1, retry);
            vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        }
    }
    ESP_LOGE("SIM7600", "❌ SIM không sẵn sàng sau %d lần thử!", retry);
    return false;
}
