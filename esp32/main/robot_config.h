#pragma once

/* ================================================================
 * robot_config.h
 * All robot-specific and transport-specific constants in one place.
 * ================================================================ */


/* ----------------------------------------------------------------
 * micro-ROS UART transport
 * Using UART2 keeps UART0 free for ESP-IDF debug logs.
 * ---------------------------------------------------------------- */
#define UROS_UART_PORT      UART_NUM_0
#define UROS_UART_TX_GPIO   1
#define UROS_UART_RX_GPIO   3
#define UROS_UART_BAUD      576000


/* ----------------------------------------------------------------
 * Differential drive geometry  — measure and replace these
 * ---------------------------------------------------------------- */
#define WHEEL_RADIUS_M      0.075f   /* metres — replace with yours */
#define WHEEL_BASE_M        0.35f    /* metres — replace with yours */


/* ----------------------------------------------------------------
 * Velocity scaling: rad/s  <->  Kinco internal unit (vel_raw)
 *
 * Kinco formula (manual §8.2, 0x606C / 0x60FF):
 *   DEC = (RPM * 512 * Encoder_Resolution) / 1875
 *
 * FD134S encoder = 2500 PPR (manual p.583: "A: 2500 PRR Incremental Encoder")
 *
 * Derivation:
 *   1 RPM  = (512 * 2500) / 1875        = 682.667 DEC
 *   1 RPS  = 60 * 682.667               = 40,960  DEC
 *   1 rad/s = 40,960 / (2 * pi)        ≈ 6,519.0 DEC
 *
 * Sanity check against your motor test (vel_raw = 500,000):
 *   500,000 / 682.667 ≈ 732 RPM  — consistent with a loaded servo test
 * ---------------------------------------------------------------- */
#define VEL_RAW_PER_RAD_S   6519.0f


/* ----------------------------------------------------------------
 * Encoder scaling: TPDO1 position ticks -> radians
 *
 * Position actual value (0x6064) is in raw encoder counts.
 * 2500 PPR * x4 quadrature = 10,000 counts/rev.
 *
 *   radians = ticks * (2 * pi / ENCODER_TICKS_PER_REV)
 * ---------------------------------------------------------------- */
#define ENCODER_TICKS_PER_REV   10000.0f


/* ----------------------------------------------------------------
 * micro-ROS node / topic names
 * ---------------------------------------------------------------- */
#define UROS_NODE_NAME          "kinco_diff_drive"
#define UROS_NODE_NAMESPACE     ""

#define UROS_WHEEL_VEL_CMD_TOPIC  "/wheel_vel_cmd"
#define UROS_JOINT_STATE_TOPIC    "/esp32/joint_states"

/* JointState publish rate */
#define UROS_PUBLISH_HZ         50        /* 50 Hz → 20 ms period */

/* Safety: stop motors if no /cmd_vel received within this window */
#define CMD_VEL_TIMEOUT_MS      500
