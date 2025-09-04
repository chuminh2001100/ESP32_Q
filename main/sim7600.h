#ifndef SIM7600_H
#define SIM7600_H

#include "driver/uart.h"
#include <stdbool.h>
#include <string.h>

#define SIM7600_UART_NUM UART_NUM_2
#define SIM7600_TXD_PIN  GPIO_NUM_17
#define SIM7600_RXD_PIN  GPIO_NUM_16
#define SIM7600_BAUDRATE 115200
#define BUF_SIZE            512

void sim7600_uart_init(void);
bool sim7600_send_cmd(const char *cmd, const char *expect, int retry, int timeout_ms);
int sim7600_read_resp(char *buffer, int max_len, int timeout_ms);
void sim7600_reset_module(void);

// MQTT functions
bool sim7600_mqtt_disconnect(void);
bool sim7600_check_signal(void);
bool sim7600_ready_for_mqtt(void);
bool sim7600_mqtt_start(void);
bool sim7600_mqtt_connect(const char *client_id, const char *broker, int keepalive, int cleansession);
bool sim7600_mqtt_publish(const char *topic, const char *payload, int qos);

/**
 * @brief Kiểm tra đã đăng ký mạng (AT+CREG?), true nếu stat = 1 hoặc 5
 */
bool sim7600_check_network(void);

/**
 * @brief Kiểm tra GPRS attach (AT+CGATT?), true nếu +CGATT:1
 */
bool sim7600_check_gprs(void);
#endif
