#include "sim7600.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

static const char *TAG = "SIM7600";
static bool sim7600_is_on = false;

static QueueHandle_t sim7600_resp_queue;
static QueueHandle_t sim7600_urc_queue;
static QueueHandle_t uart_event_queue;

typedef struct {
    char data[BUF_SIZE];
} sim7600_msg_t;

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

// ---------------- UART Task ----------------
static void sim7600_uart_event_task(void *arg) {
    uart_event_t event;
    uint8_t buf[BUF_SIZE];

    while (1) {
        if (!xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) continue;

        switch (event.type) {
            case UART_DATA: {
                int len = uart_read_bytes(SIM7600_UART_NUM, buf, sizeof(buf)-1, 20/portTICK_PERIOD_MS);
                if (len <= 0) break;
                buf[len] = '\0';
                ESP_LOGI(TAG, "UART RX raw: %s", buf);

                sim7600_msg_t msg;
                strncpy(msg.data, (char*)buf, sizeof(msg.data)-1);
                msg.data[sizeof(msg.data)-1] = '\0';

                // URC detection
                if (strstr(msg.data, "RDY") || strstr(msg.data, "+CPIN:") ||
                    strstr(msg.data, "Call Ready") || strstr(msg.data, "SMS")) {
                    xQueueSend(sim7600_urc_queue, &msg, 0);
                } else {
                    xQueueSend(sim7600_resp_queue, &msg, 0);
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART overflow");
                uart_flush_input(SIM7600_UART_NUM);
                xQueueReset(uart_event_queue);
                break;
            default: break;
        }
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

    uart_driver_install(SIM7600_UART_NUM, BUF_SIZE*2, 0, 20, &uart_event_queue, 0);
    uart_param_config(SIM7600_UART_NUM, &uart_cfg);
    uart_set_pin(SIM7600_UART_NUM, SIM7600_TXD_PIN, SIM7600_RXD_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    sim7600_resp_queue = xQueueCreate(5, sizeof(sim7600_msg_t));
    sim7600_urc_queue  = xQueueCreate(5, sizeof(sim7600_msg_t));

    xTaskCreate(sim7600_uart_event_task, "sim7600_uart_evt", 4096, NULL, 12, NULL);
    ESP_LOGI(TAG, "UART init done, baud=%d", SIM7600_BAUDRATE);
}

// ---------------- Command ----------------
bool sim7600_send_cmd(const uint8_t *cmd, size_t len, const char *expect, int retry, int timeout_ms) {
    char buffer[BUF_SIZE];

    for (int i=0; i<retry; i++) {
        xQueueReset(sim7600_resp_queue);
        uart_write_bytes(SIM7600_UART_NUM, (const char*)cmd, len);
        ESP_LOGI(TAG, "Send HEX (%d bytes)", (int)len);
        ESP_LOG_BUFFER_HEX(TAG, cmd, len);

        int attempts = 3;
        while (attempts--) {
            int l = sim7600_read_resp(buffer, sizeof(buffer), timeout_ms);
            if (l > 0 && strstr(buffer, expect)) {
                ESP_LOGI(TAG, "CMD OK");
                return true;
            }
        }
        ESP_LOGW(TAG, "Retry %d/%d failed", i+1, retry);
    }

    ESP_LOGE(TAG, "CMD FAILED");
    return false;
}

bool sim7600_send_cmd_str(const char *cmd, const char *expect, int retry, int timeout_ms) {
    uint8_t buf[128];
    size_t len = snprintf((char*)buf, sizeof(buf), "%s\r\n", cmd);
    return sim7600_send_cmd(buf, len, expect, retry, timeout_ms);
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

void sim7600_reset_module(void) {
    if (!sim7600_is_on) return;
    ESP_LOGW(TAG, "Resetting SIM7600...");
    sim7600_power_off();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    sim7600_power_on();
}

// ---------------- MQTT ----------------
bool sim7600_mqtt_start(void) {
    if (sim7600_send_cmd_str("AT+CMQTTSTART", "OK", 3, 5000)) return true;
    char buf[BUF_SIZE];
    sim7600_read_resp(buf, sizeof(buf), 1000);
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
