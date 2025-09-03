#ifndef SIM7600_H
#define SIM7600_H

#include "driver/uart.h"
#include <stdbool.h>
#include <string.h>

#define SIM7600_UART_NUM UART_NUM_2
#define SIM7600_TXD_PIN  GPIO_NUM_17
#define SIM7600_RXD_PIN  GPIO_NUM_16
#define SIM7600_BAUDRATE 115200

void sim7600_uart_init(void);
bool sim7600_send_cmd(const char *cmd, const char *expect, int retry, int timeout_ms);
int sim7600_read_resp(char *buffer, int max_len, int timeout_ms);
void sim7600_reset_module(void);

// MQTT functions
bool sim7600_mqtt_connect(const char *client_id, const char *broker);
bool sim7600_mqtt_publish(const char *topic, const char *payload);
bool sim7600_mqtt_disconnect(void);

#endif
