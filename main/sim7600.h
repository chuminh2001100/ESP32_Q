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

// Khởi tạo UART + task reader
void sim7600_uart_init(void);

// Gửi AT command, chờ expect, retry nếu fail
bool sim7600_send_cmd(const char *cmd, const char *expect, int retry, int timeout_ms);

// Đọc response (sẽ được thay bằng queue)
int sim7600_read_resp(char *buffer, int max_len, int timeout_ms);

// Reset module
void sim7600_reset_module(void);
void sim7600_power_on(void);
void sim7600_power_off(void);

// Kiểm tra module ready
bool sim7600_check_signal(void);
bool sim7600_check_network(void);
bool sim7600_check_gprs(void);
bool sim7600_ready_for_mqtt(void);
bool sim7600_ready_for_mqtt_retry(int retries, int delay_ms);

// MQTT functions
bool sim7600_mqtt_start(void);
bool sim7600_mqtt_connect(const char *client_id, const char *broker, int keepalive, int cleansession);
bool sim7600_mqtt_publish(const char *topic, const char *payload, int qos);
bool sim7600_mqtt_disconnect(void);

#endif // SIM7600_H
