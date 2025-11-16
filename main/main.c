/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "ble.h"
#include <string.h>
#include "esp_log.h"
#include <string.h>  // 确保memcpy可用
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"
#include <math.h>
#define SOF  0xFF
#define CANNON_OPEN  0x01
#define CANNON_CLOSE 0x00
#define CANNON_SHOOT 0x03
#define GPIO_SHOOT  5 //控制发射引脚
#define GPIO_CHARGE 4

uint16_t g_voltage_value = 123;   // 发送给小程序的电压值
float g_voltage_value_f = 0.0f;   // 发送给小程序的电压值

uint8_t g_cmd = 0xFF;

// 定义控制任务的日志标签
static const char *TAG_CONTROL = "control";


uint16_t float_to_uint16(float x)
{
    // 四舍五入
    int value = (int)(x + 0.5f);

    // 饱和限制（确保不会越界）
    if (value < 0)
        value = 0;
    else if (value > 65535)
        value = 65535;

    return (uint16_t)value;
}

// ble接收处理任务
void ble_task(void* param) {
    for(;;)
    {
        uint8_t tx_data[3] = {SOF,0x00,0x00};
        g_voltage_value = float_to_uint16(g_voltage_value_f);
        memcpy(&tx_data[1],&g_voltage_value,2 * sizeof(uint8_t));
        ble_send_ch2_data(tx_data,sizeof(tx_data));
        // 接收任务
        if(g_ble_recive_flag == 1)
        {
            g_ble_recive_flag = 0;
            if(sv1_char1_value[0] == SOF && sv1_char1_value_len == 2) {
                g_cmd = sv1_char1_value[1];
            }

            // 处理接收到的数据
            
            // 获取BLE数据长度（在ble.h中定义了sv1_char1_value_len）
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

float adc_data_to_vin(uint16_t data)
{
    // ---- 将你的拟合系数放这里 ----
    const float a = -1.1078771331e-11;
    const float b = 2.8331331216e-08;
    const float c = 8.4934727972e-04;
    const float d = 6.3687307270e-03;

    float x = (float)data;

    // 三次多项式拟合: Vin = a*x^3 + b*x^2 + c*x + d
    return (((a * x + b) * x + c) * x + d)/0.00596421f;
}

static adc_channel_t channel[1] = {ADC_CHANNEL_0};
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#define ADC_READ_LEN                        256
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
void sample_task(void* param) {

    uint32_t ret_num = 0;
    uint8_t result[ADC_READ_LEN] = {0};
    memset(result, 0xcc, ADC_READ_LEN);

    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1000,  // 降低采样率到1kHz
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

    uint8_t channel_num = sizeof(channel) / sizeof(adc_channel_t);
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12; //12dB衰减，采集3v时是2805
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    
    // 启动ADC连续转换
    ESP_ERROR_CHECK(adc_continuous_start(handle));



    char unit[] = EXAMPLE_ADC_UNIT_STR(ADC_UNIT_1);
    for(;;) {

        esp_err_t ret = adc_continuous_read(handle, result, ADC_READ_LEN, &ret_num, pdMS_TO_TICKS(10));
        if (ret == ESP_OK) {
            ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                    ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit,chan_num, data);
                    g_voltage_value_f = adc_data_to_vin(data);
                    ESP_LOGI("Voltage","%f",g_voltage_value_f);
                } else {
                    ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ADC read timeout ");
        }
    }
}


void control_task(void* param) {
    
    for(;;) {
        switch(g_cmd) {
            case(CANNON_OPEN):
                ESP_LOGI(TAG_CONTROL, "CANNON OPEN");
                g_cmd = 0xFF;
            break;
            case(CANNON_CLOSE):
                ESP_LOGI(TAG_CONTROL, "CANNON CLOSE");
                g_cmd = 0xFF;
            break;
            case(CANNON_SHOOT):
                ESP_LOGI(TAG_CONTROL, "!!!!!CANNON SHOOT!!!!");
                g_cmd = 0xFF;
            break;
            default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    // nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // BLE
    ble_cfg_net_init();

    xTaskCreate(sample_task, "sample_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(ble_task,"ble_task",6144,NULL,3,NULL,1);

}
