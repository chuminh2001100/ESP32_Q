#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT       UART_NUM_1   // Dùng UART1 (bạn có thể đổi)
#define UART_TX_PIN     17           // Chân TX (GPIO17 ví dụ)
#define UART_RX_PIN     16           // Chân RX (GPIO16 ví dụ)
#define UART_BAUD_RATE  115200
#define BUF_SIZE        1024

static const char *TAG = "UART_EXAMPLE";

void uart_task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1) {
        // Đọc dữ liệu từ UART (timeout 20ms)
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Kết thúc chuỗi
            ESP_LOGI(TAG, "Recv: %s", (char *)data);
        }

        // Gửi "Hello" mỗi 2s
        static int counter = 0;
        if (counter++ >= 100) {   // 100 × 20ms = 2000ms
            const char *msg = "Hello\r\n";
            uart_write_bytes(UART_PORT, msg, strlen(msg));
            ESP_LOGI(TAG, "Sent: %s", msg);
            counter = 0;
        }
    }
}

void app_main(void)
{
    // Cấu hình UART
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Cài đặt UART driver
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART example started.");

    // Tạo task UART
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
}
