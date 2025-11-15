/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "ble.h"

// ble接收处理任务
void ble_task(void* param)
{

    while(1)
    {
        // 接收任务
        if(g_ble_recive_flag == 1)
        {
            g_ble_recive_flag = 0;


            // 处理接收到的数据
            
            // 获取BLE数据长度（在ble.h中定义了sv1_char1_value_len）
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // BLE
    ble_cfg_net_init();

    xTaskCreatePinnedToCore(ble_task,"ble_task",6144,NULL,3,NULL,1);

}
