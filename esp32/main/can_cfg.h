#pragma once

#include "driver/twai.h"

/* -------- TWAI hardware config -------- */
#define CAN_TX_GPIO         21
#define CAN_RX_GPIO         22
#define CAN_BITRATE_KBPS    125
#define CAN_TWAI_MODE       TWAI_MODE_NORMAL

/* -------- node mapping -------- */
/* Change these if your physical bench setup is swapped */
#define LEFT_NODE_ID        0x06
#define RIGHT_NODE_ID       0x07

/* -------- startup timing -------- */
#define KINCO_RESET_DELAY_MS        1500
#define KINCO_POST_NMT_DELAY_MS     20
#define KINCO_POST_MODE_DELAY_MS    20
#define KINCO_NODE_GAP_DELAY_MS     500

/* -------- debug -------- */
/* Set to 0 when you integrate micro-ROS and want near-zero console overhead */
#define KINCO_PRINT_SUMMARY         1
#define KINCO_SUMMARY_PERIOD_MS     1000