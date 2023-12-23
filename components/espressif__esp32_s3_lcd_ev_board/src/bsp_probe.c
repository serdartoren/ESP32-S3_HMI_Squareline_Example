/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_psram.h"

#include "bsp_probe.h"
#include "bsp_err_check.h"
#include "bsp/esp32_s3_lcd_ev_board.h"

#define MODULE_PSRAM_SIZE_R8    (8 * 1024 * 1024)

static const char *TAG = "bsp_probe";
static bsp_module_type_t module_type = MODULE_TYPE_UNKNOW;

bsp_module_type_t bsp_probe_module_type(void)
{
    if (module_type != MODULE_TYPE_UNKNOW) {
        return module_type;
    }

    int psram_size = esp_psram_get_size();
    if (psram_size > MODULE_PSRAM_SIZE_R8) {
        ESP_LOGI(TAG, "Detect module with 16MB PSRAM");
        module_type = MODULE_TYPE_R16;
    } else {
        ESP_LOGI(TAG, "Detect module with 8MB PSRAM");
        module_type = MODULE_TYPE_R8;
    }

    return module_type;
}

bsp_sub_board_type_t bsp_probe_sub_board_type(void)
{

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
  
    return SUB_BOARD_TYPE_3_800_480;
}
