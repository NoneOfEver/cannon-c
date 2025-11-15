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
#include "esp_adc/adc_continuous.h"

#define SOF  0xFF
#define CANNON_OPEN  0x01
#define CANNON_CLOSE 0x00
#define CANNON_SHOOT 0x03

uint16_t g_voltage_value = 123;   // 发送给小程序的电压值
uint8_t g_cmd = 0xFF;

// ble接收处理任务
void ble_task(void* param) {
    for(;;)
    {
        uint8_t tx_data[3] = {SOF,0x00,0x00};
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

void sample_task(void* param) {
    // 初始化 ADC 监视器句柄
    adc_monitor_handle_t adc_monitor_handle = NULL;

    // 配置 ADC 监视器
    adc_monitor_config_t zero_crossing_config = {
        .adc_unit = EXAMPLE_ADC_UNIT_1,      // 指定要监视的 ADC 单元
        .channel = EXAMPLE_ADC_CHANNEL_0,    // 指定要监视的 ADC 通道
        .h_threshold = 1100,                 // 设置监视的高阈值为接近偏置值，请根据实际情况进行调整
        .l_threshold = 900,                 // 设置监视的低阈值为接近偏置值，请根据实际情况进行调整
    };

    // 创建 ADC 监视器
    ESP_ERROR_CHECK(adc_new_continuous_monitor(&zero_crossing_config, &adc_monitor_handle));

    // 注册回调函数
    adc_monitor_evt_cbs_t zero_crossing_cbs = {
        .on_over_high_thresh = example_on_exceed_high_thresh,
        .on_below_low_thresh = example_on_below_low_thresh,
    };

    ESP_ERROR_CHECK(adc_continuous_monitor_register_event_callbacks(adc_monitor_handle, &zero_crossing_cbs, NULL));

    // 启用 ADC 监视器
    ESP_ERROR_CHECK(adc_continuous_monitor_enable(adc_monitor_handle));

}

void control_task(void* param) {
    
    for(;;) {
        switch(g_cmd) {
            case(CANNON_OPEN):
                ESP_LOGI(TAG, "CANNON OPEN");
                g_cmd = 0xFF;
            break;
            case(CANNON_CLOSE):
                ESP_LOGI(TAG, "CANNON CLOSE");
                g_cmd = 0xFF;
            break;
            case(CANNON_SHOOT):
                ESP_LOGI(TAG, "!!!!!CANNON SHOOT!!!!");
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
