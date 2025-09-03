#include "sim7600.h"
#include "gpio_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SIM7600";

void sim7600_uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = SIM7600_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(SIM7600_UART_NUM, 4096, 4096, 0, NULL, 0);
    uart_param_config(SIM7600_UART_NUM, &uart_config);
    uart_set_pin(SIM7600_UART_NUM, SIM7600_TXD_PIN, SIM7600_RXD_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART2 init done, baud=%d", SIM7600_BAUDRATE);
}

int sim7600_read_resp(char *buffer, int max_len, int timeout_ms) {
    int len = uart_read_bytes(SIM7600_UART_NUM, (uint8_t *)buffer, max_len - 1,
                              timeout_ms / portTICK_PERIOD_MS);
    if (len > 0) {
        buffer[len] = '\0';
        ESP_LOGI(TAG, "Recv: %s", buffer);
    }
    return len;
}

bool sim7600_send_cmd(const char *cmd, const char *expect, int retry, int timeout_ms) {
    char buffer[512];

    for (int i = 0; i < retry; i++) {
        char send_buf[128];
        snprintf(send_buf, sizeof(send_buf), "%s\r\n", cmd);
        uart_write_bytes(SIM7600_UART_NUM, send_buf, strlen(send_buf));
        ESP_LOGI(TAG, "Send: %s", cmd);

        vTaskDelay(200 / portTICK_PERIOD_MS);

        int len = sim7600_read_resp(buffer, sizeof(buffer), timeout_ms);
        if (len > 0 && strstr(buffer, expect)) {
            return true;
        }
        ESP_LOGW(TAG, "Retry %d failed", i+1);
    }
    return false;
}

void sim7600_reset_module(void) {
    ESP_LOGW(TAG, "Resetting SIM7600...");
    sim7600_power_on();   // dùng lại hàm bật lại từ gpio_config.c
}

// ------------------ MQTT functions ------------------

bool sim7600_mqtt_connect(const char *client_id, const char *broker) {
    if (!sim7600_send_cmd("AT+CMQTTSTART", "OK", 3, 5000)) return false;

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\"", client_id);
    if (!sim7600_send_cmd(cmd, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",60,1,\"\",\"\"", broker);
    if (!sim7600_send_cmd(cmd, "OK", 3, 5000)) return false;

    return true;
}

bool sim7600_mqtt_publish(const char *topic, const char *payload) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%d", strlen(topic));
    if (!sim7600_send_cmd(cmd, ">", 3, 2000)) return false;
    if (!sim7600_send_cmd(topic, "OK", 3, 2000)) return false;

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%d", strlen(payload));
    if (!sim7600_send_cmd(cmd, ">", 3, 2000)) return false;
    if (!sim7600_send_cmd(payload, "OK", 3, 2000)) return false;

    if (!sim7600_send_cmd("AT+CMQTTPUB=0,1,60", "OK", 3, 5000)) return false;

    return true;
}

bool sim7600_mqtt_disconnect(void) {
    if (!sim7600_send_cmd("AT+CMQTTDISC=0,60", "OK", 3, 3000)) return false;
    if (!sim7600_send_cmd("AT+CMQTTREL=0", "OK", 3, 2000)) return false;
    if (!sim7600_send_cmd("AT+CMQTTSTOP", "OK", 3, 3000)) return false;
    return true;
}
