#ifndef SIM7600_H
#define SIM7600_H

#include "driver/uart.h"
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// UART config
#define SIM7600_UART_NUM UART_NUM_2
#define SIM7600_TXD_PIN  GPIO_NUM_17
#define SIM7600_RXD_PIN  GPIO_NUM_16
#define SIM7600_BAUDRATE 115200
#define BUF_SIZE         512
#define SIM7600_PWRKEY_GPIO   GPIO_NUM_23   // ví dụ dùng GPIO4 để bật module

typedef enum {
    SIM7600_EVENT_MQTT_DISCONNECTED,
    SIM7600_EVENT_MQTT_CONNECTED,
    SIM7600_EVENT_NETWORK_LOST,
    SIM7600_EVENT_NETWORK_READY,
    SIM7600_EVENT_MQTT_SUBRECV,
} sim7600_event_type_t;

typedef struct {
    sim7600_event_type_t type;
    char topic[64];
    char payload[256];
} sim7600_event_t;

typedef struct {
    char topic[256];
    char payload[512];
    bool receiving;  // đang trong quá trình nhận bản tin
    int topic_len_expected;
    int payload_len_expected;
    int topic_len_received;
    int payload_len_received;
} sim7600_mqtt_rx_t;

static sim7600_mqtt_rx_t mqtt_rx_state = {0};
// ---------------- UART + Queue ----------------
// Khởi tạo UART và task reader
void sim7600_uart_init(void);

// ---------------- AT Command ----------------
// Gửi AT command dưới dạng HEX hoặc string
bool sim7600_send_cmd_str(const char *cmd, const char *expect, int retry, int timeout_ms);

// ---------------- Power & Reset ----------------
void sim7600_power_on(void);
void sim7600_power_off(void);
void sim7600_reset_module(void);

// ---------------- Check module ----------------
bool sim7600_basic_check(void);
bool sim7600_check_signal(int *out_rssi);
bool sim7600_ready_for_mqtt(void);
bool sim7600_ready_for_mqtt_retry(int retries, int delay_ms);

// ---------------- MQTT ----------------
bool sim7600_mqtt_start(void);
bool sim7600_mqtt_connect(const char *client_id, const char *broker, int keepalive, int cleansession);
bool sim7600_mqtt_publish(const char *topic, const char *payload, int qos);
bool sim7600_mqtt_disconnect(void);
void sim7600_uart_reader_task(void *arg);
void sim7600_set_event_queue(QueueHandle_t queue);
bool sim7600_mqtt_subscribe(const char *topic, int qos);
bool sim7600_is_network_registered(int retry, int delay_ms)
#endif // SIM7600_H
