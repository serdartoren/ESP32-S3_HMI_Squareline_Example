/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "bsp/esp32_s3_lcd_ev_board.h"
#include "lvgl.h"
#include "ui/ui.h"


#define TAG "ESP-EXAMPLE"

/*******************************************************************************
* Private functions
*******************************************************************************/

// *INDENT-OFF*
void app_lvgl_display(void)
{
    bsp_display_lock(0);

    ui_init();

    bsp_display_unlock();
}

void app_main(void)
{
    // GPIO yapılandırması
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 2),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    /* Initialize I2C (for touch and audio) */
    bsp_i2c_init();

    /* Initialize display and LVGL */
    bsp_display_start();

    /* Turn on display backlight */
    bsp_display_backlight_on();

    /* Add and show objects on display */
    app_lvgl_display();

    ESP_LOGI(TAG, "Example initialization done.");
}
// *INDENT-ON*
