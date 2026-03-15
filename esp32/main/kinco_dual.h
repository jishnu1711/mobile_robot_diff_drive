#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t node_id;

    bool hb_seen;
    uint8_t hb_state;

    bool tpdo1_seen;
    int32_t speed;
    int32_t position;

    bool tpdo2_seen;
    int16_t current;

    bool emcy_seen;
    uint16_t emcy_code;
    uint8_t emcy_reg;
    uint8_t emcy_data[5];
} kinco_feedback_t;

/* init both drives into velocity mode with zero command */
esp_err_t kinco_dual_init(void);

/* raw velocity commands (exact RPDO2 payload int32 LE) */
esp_err_t kinco_set_velocity_raw(uint8_t node_id, int32_t vel_raw);
esp_err_t kinco_left_set_velocity_raw(int32_t vel_raw);
esp_err_t kinco_right_set_velocity_raw(int32_t vel_raw);

/* runtime stop: send zero velocity to both */
esp_err_t kinco_stop_all(void);

/* cached feedback getters */
void kinco_get_left_feedback(kinco_feedback_t *out);
void kinco_get_right_feedback(kinco_feedback_t *out);