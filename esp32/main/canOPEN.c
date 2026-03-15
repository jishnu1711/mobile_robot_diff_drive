#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/twai.h"

#include "can_cfg.h"
#include "kinco_dual.h"

static const char *TAG = "KINCO_MIN";

/* -------- exact Ubuntu-style COB-IDs -------- */
#define COB_NMT                0x000U
#define COB_EMCY(node)         (0x080U + (node))
#define COB_TPDO1(node)        (0x180U + (node))   /* speed(int32) + position(int32) */
#define COB_RPDO1(node)        (0x200U + (node))   /* controlword(uint16 LE) + mode(uint8) */
#define COB_TPDO2(node)        (0x280U + (node))   /* current(int16 LE) */
#define COB_RPDO2(node)        (0x300U + (node))   /* target velocity(int32 LE) */
#define COB_HB(node)           (0x700U + (node))

/* -------- exact mode values from Ubuntu stack -------- */
#define NMT_OPERATIONAL        0x01
#define NMT_PRE_OPERATIONAL    0x80

#define MODE_POSITION          0x01
#define MODE_VELOCITY          0x03

#define CONTROLWORD_POWER_ON   0x000F
#define CONTROLWORD_POWER_OFF  0x0006
#define CONTROLWORD_RESET      0x0086

#if (CAN_BITRATE_KBPS != 125)
#error "This stack expects CAN_BITRATE_KBPS = 125"
#endif

static kinco_feedback_t g_left = {
    .node_id = LEFT_NODE_ID
};

static kinco_feedback_t g_right = {
    .node_id = RIGHT_NODE_ID
};

static portMUX_TYPE g_fb_lock = portMUX_INITIALIZER_UNLOCKED;
static bool g_kinco_ready = false;

/* ---------------- little-endian helpers ---------------- */
static int16_t le_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t le_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t le_i32(const uint8_t *p) {
    return (int32_t)(
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24)
    );
}

/* ---------------- raw TX ---------------- */
static esp_err_t send_std_frame(uint16_t can_id, const uint8_t *data, uint8_t len) {
    twai_message_t msg = {0};
    msg.identifier = can_id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = len;

    if (len > 0 && data != NULL) {
        memcpy(msg.data, data, len);
    }

    return twai_transmit(&msg, pdMS_TO_TICKS(200));
}

/* ---------------- exact Ubuntu-style TX helpers ---------------- */
static esp_err_t kinco_send_nmt(uint8_t node_id, uint8_t state) {
    const uint8_t d[2] = {state, node_id};
    return send_std_frame(COB_NMT, d, 2);
}

static esp_err_t kinco_send_mode(uint8_t node_id, uint16_t controlword, uint8_t mode) {
    uint8_t d[3];
    d[0] = (uint8_t)(controlword & 0xFF);
    d[1] = (uint8_t)((controlword >> 8) & 0xFF);
    d[2] = mode;
    return send_std_frame((uint16_t)COB_RPDO1(node_id), d, 3);
}

static esp_err_t kinco_fault_reset(uint8_t node_id) {
    return kinco_send_mode(node_id, CONTROLWORD_RESET, MODE_POSITION);
}

static esp_err_t kinco_enable_velocity_mode(uint8_t node_id) {
    return kinco_send_mode(node_id, CONTROLWORD_POWER_ON, MODE_VELOCITY);
}

static esp_err_t kinco_send_velocity(uint8_t node_id, int32_t vel_raw) {
    uint8_t d[4];
    d[0] = (uint8_t)(vel_raw & 0xFF);
    d[1] = (uint8_t)((vel_raw >> 8) & 0xFF);
    d[2] = (uint8_t)((vel_raw >> 16) & 0xFF);
    d[3] = (uint8_t)((vel_raw >> 24) & 0xFF);
    return send_std_frame((uint16_t)COB_RPDO2(node_id), d, 4);
}

/* ---------------- stable bringup path ---------------- */
static esp_err_t kinco_reset_then_bringup_velocity_zero(uint8_t node_id) {
    esp_err_t err;

    ESP_LOGI(TAG, "Reset+bringup start for node 0x%02X", node_id);

    err = kinco_fault_reset(node_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fault reset failed node=0x%02X: %s", node_id, esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(KINCO_RESET_DELAY_MS));

    err = kinco_send_nmt(node_id, NMT_OPERATIONAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NMT operational failed node=0x%02X: %s", node_id, esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(KINCO_POST_NMT_DELAY_MS));

    err = kinco_enable_velocity_mode(node_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Enable velocity mode failed node=0x%02X: %s", node_id, esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(KINCO_POST_MODE_DELAY_MS));

    err = kinco_send_velocity(node_id, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Zero velocity failed node=0x%02X: %s", node_id, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Reset+bringup done for node 0x%02X", node_id);
    return ESP_OK;
}

/* ---------------- RX parsing ---------------- */
static void handle_emcy(kinco_feedback_t *m, const twai_message_t *msg) {
    if (msg->data_length_code < 8) return;

    portENTER_CRITICAL(&g_fb_lock);
    m->emcy_seen = true;
    m->emcy_code = le_u16(&msg->data[0]);
    m->emcy_reg  = msg->data[2];
    memcpy(m->emcy_data, &msg->data[3], 5);
    portEXIT_CRITICAL(&g_fb_lock);

    ESP_LOGW(TAG,
             "EMCY node=0x%02X code=0x%04X reg=0x%02X data=[%02X %02X %02X %02X %02X]",
             m->node_id,
             m->emcy_code,
             m->emcy_reg,
             m->emcy_data[0], m->emcy_data[1], m->emcy_data[2],
             m->emcy_data[3], m->emcy_data[4]);
}

static void parse_canopen_rx(const twai_message_t *msg) {
    if (!msg || msg->extd) return;

    const uint32_t id = msg->identifier & 0x7FFU;
    const uint8_t dlc = msg->data_length_code & 0x0FU;

    if (id == COB_HB(LEFT_NODE_ID) && dlc >= 1) {
        portENTER_CRITICAL(&g_fb_lock);
        g_left.hb_seen = true;
        g_left.hb_state = msg->data[0];
        portEXIT_CRITICAL(&g_fb_lock);
        return;
    }

    if (id == COB_HB(RIGHT_NODE_ID) && dlc >= 1) {
        portENTER_CRITICAL(&g_fb_lock);
        g_right.hb_seen = true;
        g_right.hb_state = msg->data[0];
        portEXIT_CRITICAL(&g_fb_lock);
        return;
    }

    if (id == COB_TPDO1(LEFT_NODE_ID) && dlc >= 8) {
        portENTER_CRITICAL(&g_fb_lock);
        g_left.tpdo1_seen = true;
        g_left.speed = le_i32(&msg->data[0]);
        g_left.position = le_i32(&msg->data[4]);
        portEXIT_CRITICAL(&g_fb_lock);
        return;
    }

    if (id == COB_TPDO1(RIGHT_NODE_ID) && dlc >= 8) {
        portENTER_CRITICAL(&g_fb_lock);
        g_right.tpdo1_seen = true;
        g_right.speed = le_i32(&msg->data[0]);
        g_right.position = le_i32(&msg->data[4]);
        portEXIT_CRITICAL(&g_fb_lock);
        return;
    }

    if (id == COB_TPDO2(LEFT_NODE_ID) && dlc >= 2) {
        portENTER_CRITICAL(&g_fb_lock);
        g_left.tpdo2_seen = true;
        g_left.current = le_i16(&msg->data[0]);
        portEXIT_CRITICAL(&g_fb_lock);
        return;
    }

    if (id == COB_TPDO2(RIGHT_NODE_ID) && dlc >= 2) {
        portENTER_CRITICAL(&g_fb_lock);
        g_right.tpdo2_seen = true;
        g_right.current = le_i16(&msg->data[0]);
        portEXIT_CRITICAL(&g_fb_lock);
        return;
    }

    if (id == COB_EMCY(LEFT_NODE_ID)) {
        handle_emcy(&g_left, msg);
        return;
    }

    if (id == COB_EMCY(RIGHT_NODE_ID)) {
        handle_emcy(&g_right, msg);
        return;
    }
}

/* ---------------- RX task ---------------- */
static void can_rx_task(void *arg) {
    (void)arg;

    while (1) {
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1000));

        if (err == ESP_OK) {
            parse_canopen_rx(&msg);
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "twai_receive failed: %s", esp_err_to_name(err));
        }
    }
}

/* ---------------- TWAI init ---------------- */
static esp_err_t can_init_start(void) {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, CAN_TWAI_MODE);

    g_config.tx_queue_len = 32;
    g_config.rx_queue_len = 128;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG,
             "TWAI started: bitrate=%d kbps tx=%d rx=%d mode=%d",
             CAN_BITRATE_KBPS,
             (int)CAN_TX_GPIO,
             (int)CAN_RX_GPIO,
             (int)CAN_TWAI_MODE);

    return ESP_OK;
}

/* ---------------- public API ---------------- */
esp_err_t kinco_dual_init(void) {
    if (g_kinco_ready) {
        return ESP_OK;
    }

    if (LEFT_NODE_ID == RIGHT_NODE_ID) {
        ESP_LOGE(TAG, "Left/right node IDs must differ");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(can_init_start());

    BaseType_t ok = xTaskCreate(can_rx_task, "can_rx_task", 4096, NULL, 10, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create can_rx_task");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(kinco_reset_then_bringup_velocity_zero(LEFT_NODE_ID));
    vTaskDelay(pdMS_TO_TICKS(KINCO_NODE_GAP_DELAY_MS));

    ESP_ERROR_CHECK(kinco_reset_then_bringup_velocity_zero(RIGHT_NODE_ID));
    vTaskDelay(pdMS_TO_TICKS(KINCO_NODE_GAP_DELAY_MS));

    g_kinco_ready = true;
    ESP_LOGI(TAG, "Both Kinco nodes initialized");
    return ESP_OK;
}

esp_err_t kinco_set_velocity_raw(uint8_t node_id, int32_t vel_raw) {
    if (!g_kinco_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    if (node_id != LEFT_NODE_ID && node_id != RIGHT_NODE_ID) {
        return ESP_ERR_INVALID_ARG;
    }

    return kinco_send_velocity(node_id, vel_raw);
}

esp_err_t kinco_left_set_velocity_raw(int32_t vel_raw) {
    return kinco_set_velocity_raw(LEFT_NODE_ID, vel_raw);
}

esp_err_t kinco_right_set_velocity_raw(int32_t vel_raw) {
    return kinco_set_velocity_raw(RIGHT_NODE_ID, vel_raw);
}

esp_err_t kinco_stop_all(void) {
    esp_err_t err;

    err = kinco_send_velocity(LEFT_NODE_ID, 0);
    if (err != ESP_OK) return err;

    err = kinco_send_velocity(RIGHT_NODE_ID, 0);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

void kinco_get_left_feedback(kinco_feedback_t *out) {
    if (!out) return;

    portENTER_CRITICAL(&g_fb_lock);
    *out = g_left;
    portEXIT_CRITICAL(&g_fb_lock);
}

void kinco_get_right_feedback(kinco_feedback_t *out) {
    if (!out) return;

    portENTER_CRITICAL(&g_fb_lock);
    *out = g_right;
    portEXIT_CRITICAL(&g_fb_lock);
}

// /* ---------------- demo app_main ---------------- */
// /* Later, your micro-ROS app can keep this init and replace the while-loop logic. */
// void app_main(void) {
//     esp_err_t err = nvs_flash_init();
//     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         err = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(err);

//     ESP_ERROR_CHECK(kinco_dual_init());

//     while (1) {
// #if KINCO_PRINT_SUMMARY
//         kinco_feedback_t left, right;
//         kinco_get_left_feedback(&left);
//         kinco_get_right_feedback(&right);

//         ESP_LOGI(TAG,
//                  "LEFT  node=0x%02X speed=%" PRId32 " pos=%" PRId32 " current=%d emcy=%d code=0x%04X",
//                  left.node_id, left.speed, left.position, (int)left.current,
//                  left.emcy_seen, left.emcy_code);

//         ESP_LOGI(TAG,
//                  "RIGHT node=0x%02X speed=%" PRId32 " pos=%" PRId32 " current=%d emcy=%d code=0x%04X",
//                  right.node_id, right.speed, right.position, (int)right.current,
//                  right.emcy_seen, right.emcy_code);
// #endif
//         vTaskDelay(pdMS_TO_TICKS(KINCO_SUMMARY_PERIOD_MS));
//     }
// }

/*just a basic main function to test them motors*/