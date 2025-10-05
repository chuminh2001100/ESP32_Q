#include "gpio_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define SIM7600_PWRKEY_GPIO   GPIO_NUM_23   // ví dụ dùng GPIO4 để bật module
esp_adc_cal_characteristics_t adc_chars;
void sim7600_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << SIM7600_PWRKEY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(SIM7600_PWRKEY_GPIO, 1); // default HIGH
}


const float R_TOP = 470000.0;
const float R_BOTTOM = 100000.0;
const float RATIO = (R_TOP + R_BOTTOM) / R_BOTTOM; // = 5.7
const int ADC_MAX = 4095;
const float VREF = 3.3; // dùng esp_adc_cal để có Vref chính xác

float rawToBatteryVoltage(int raw) {
  uint32_t vout_mV = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  float vbat_mV = (float)vout_mV * RATIO;
  float vbat_V = vbat_mV / 1000.0f;
  return vbat_V;
}

float voltageToPercent(float vbat, float vmin, float vmax) {
  float p = (vbat - vmin) / (vmax - vmin) * 100.0;
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

float rawToPercent(int raw, float vmin, float vmax) {
  float vbat = rawToBatteryVoltage(raw);
  return voltageToPercent(vbat, vmin, vmax);
}

void init_adc() {
    // Cấu hình ADC
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);
}

// Đọc ADC và trả về điện áp pin
float read_battery_voltage() {
    int raw = adc1_get_raw(ADC_CHANNEL);
    float vbat = rawToBatteryVoltage(raw);
    return vbat;
}