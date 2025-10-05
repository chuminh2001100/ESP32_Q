#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define ADC_CHANNEL     ADC1_CHANNEL_7 // GPIO35
#define ADC_ATTEN       ADC_ATTEN_DB_11
#define ADC_UNIT        ADC_UNIT_1
#define ADC_WIDTH       ADC_WIDTH_BIT_12

void sim7600_gpio_init(void);
float rawToBatteryVoltage(int raw);
float voltageToPercent(float vbat, float vmin, float vmax);
float rawToPercent(int raw, float vmin, float vmax);
float read_battery_voltage();
void init_adc();
#endif