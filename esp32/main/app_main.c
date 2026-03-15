#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "kinco_dual.h"
#include "microros_node.h"

static const char *TAG = "MAIN";

void app_main(void) {
    /* NVS required by ESP-IDF internals */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Bring up both Kinco drives — blocks until both nodes are ready */
    ESP_LOGI(TAG, "Initializing motor stack...");
    ESP_ERROR_CHECK(kinco_dual_init());
    ESP_LOGI(TAG, "Motor stack ready");

    /* Start micro-ROS task
     * Stack: 8192 bytes — micro-ROS needs more than a typical task
     * Priority: 5 — below can_rx_task (10) so CAN RX is never starved */
    xTaskCreate(microros_task,
                "microros_task",
                8192,
                NULL,
                5,
                NULL);
}
