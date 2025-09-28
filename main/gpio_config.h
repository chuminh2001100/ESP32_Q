#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "driver/gpio.h"

#define SIM7600_PWRKEY_GPIO   GPIO_NUM_4   // ví dụ dùng GPIO4 để bật module

void sim7600_gpio_init(void);

#endif