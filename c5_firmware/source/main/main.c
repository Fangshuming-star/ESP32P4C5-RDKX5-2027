/*
 * ESP32-C5 wireless co-processor firmware (ESP-Hosted-MCU "co-processor" role).
 *
 * Runs the ESP-Hosted SDIO-slave stack: it exposes the C5's Wi-Fi to the
 * ESP32-P4 host over the internal SDIO link of the OSPTE ESP32-P4C5-Core board.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_log.h"
#include "esp_event.h"
#include "esp_err.h"
#include "nvs_flash.h"

static const char *TAG = "c5_coproc";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-Hosted C5 co-processor starting (SDIO slave)...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* The ESP-Hosted co-processor stack (SDIO slave + Wi-Fi) is started by the
     * esp_hosted component itself; there is nothing more to do here. */
    ESP_LOGI(TAG, "Co-processor ready, waiting for host over SDIO...");
}
