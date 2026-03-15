#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>
#include "esp32_serial_transport.h"

#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "kinco_dual.h"
#include "robot_config.h"

static const char *TAG = "UROS";
static size_t g_uart_port = UROS_UART_PORT;
/* ----------------------------------------------------------------
 * Safety: track last /wheel_vel_cmd timestamp
 * ---------------------------------------------------------------- */
static volatile int64_t g_last_cmd_ms = 0;

static int64_t now_ms(void) {
    return (int64_t)(esp_timer_get_time() / 1000LL);
}

/* ----------------------------------------------------------------
 * /wheel_vel_cmd subscriber callback
 * msg->data.data[0] = left  wheel rad/s
 * msg->data.data[1] = right wheel rad/s
 * No kinematics — DiffDriveController already did that.
 * ---------------------------------------------------------------- */
static std_msgs__msg__Float64MultiArray g_wheel_cmd_msg;
static double g_wheel_cmd_data[2];   /* static backing array */

static void wheel_vel_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *msg =
        (const std_msgs__msg__Float64MultiArray *)msgin;

    if (msg->data.size < 2) return;

    g_last_cmd_ms = now_ms();

    int32_t left_raw  = (int32_t)(msg->data.data[0] * VEL_RAW_PER_RAD_S);
    int32_t right_raw = (int32_t)(msg->data.data[1] * VEL_RAW_PER_RAD_S);

    kinco_left_set_velocity_raw(left_raw);
    kinco_right_set_velocity_raw(right_raw);
}

/* ----------------------------------------------------------------
 * /esp32/joint_states publisher
 * 2 joints: "left_wheel", "right_wheel"
 * Publishes: position [rad], velocity [rad/s]
 * ---------------------------------------------------------------- */
#define NUM_JOINTS 2

static sensor_msgs__msg__JointState g_js_msg;
static double g_js_position[NUM_JOINTS];
static double g_js_velocity[NUM_JOINTS];

static char g_name_left[32]  = "left_wheel";
static char g_name_right[32] = "right_wheel";

static bool joint_state_msg_init(void) {
    sensor_msgs__msg__JointState__init(&g_js_msg);

    if (!rosidl_runtime_c__String__Sequence__init(&g_js_msg.name, NUM_JOINTS)) {
        return false;
    }
    rosidl_runtime_c__String__assign(&g_js_msg.name.data[0], g_name_left);
    rosidl_runtime_c__String__assign(&g_js_msg.name.data[1], g_name_right);

    g_js_msg.position.data     = g_js_position;
    g_js_msg.position.size     = NUM_JOINTS;
    g_js_msg.position.capacity = NUM_JOINTS;

    g_js_msg.velocity.data     = g_js_velocity;
    g_js_msg.velocity.size     = NUM_JOINTS;
    g_js_msg.velocity.capacity = NUM_JOINTS;

    g_js_msg.effort.data     = NULL;
    g_js_msg.effort.size     = 0;
    g_js_msg.effort.capacity = 0;

    return true;
}

static void joint_state_msg_fini(void) {
    g_js_msg.position.data = NULL;
    g_js_msg.velocity.data = NULL;
    sensor_msgs__msg__JointState__fini(&g_js_msg);
}

static void fill_joint_state(void) {
    kinco_feedback_t left_fb, right_fb;
    kinco_get_left_feedback(&left_fb);
    kinco_get_right_feedback(&right_fb);

    const double ticks_to_rad = (2.0 * M_PI) / ENCODER_TICKS_PER_REV;
    g_js_position[0] = (double)left_fb.position  * ticks_to_rad;
    g_js_position[1] = (double)right_fb.position * ticks_to_rad;

    g_js_velocity[0] = (double)left_fb.speed  * (2.0 * M_PI / 60.0);
    g_js_velocity[1] = (double)right_fb.speed * (2.0 * M_PI / 60.0);
}

static void stamp_now(builtin_interfaces__msg__Time *t) {
    int64_t ns = rmw_uros_epoch_nanos();
    t->sec     = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

/* ----------------------------------------------------------------
 * RCLC error helper
 * ---------------------------------------------------------------- */
#define RCCHECK(fn) \
    do { \
        rcl_ret_t _rc = (fn); \
        if (_rc != RCL_RET_OK) { \
            ESP_LOGE(TAG, "rcl error %d at %s:%d", (int)_rc, __FILE__, __LINE__); \
            return false; \
        } \
    } while (0)

/* ----------------------------------------------------------------
 * ROS entities
 * ---------------------------------------------------------------- */
static rcl_node_t            g_node;
static rcl_subscription_t    g_sub_wheel_cmd;
static rcl_publisher_t       g_pub_joint_state;
static rclc_executor_t       g_executor;
static rclc_support_t        g_support;
static rcl_allocator_t       g_allocator;

static bool ros_entities_create(void) {
    g_allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

    RCCHECK(rclc_node_init_default(&g_node,
                                   UROS_NODE_NAME,
                                   UROS_NODE_NAMESPACE,
                                   &g_support));

    /* Subscribe to /wheel_vel_cmd — Float64MultiArray */
    RCCHECK(rclc_subscription_init_default(
        &g_sub_wheel_cmd,
        &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        UROS_WHEEL_VEL_CMD_TOPIC));

    /* Publish to /esp32/joint_states */
    RCCHECK(rclc_publisher_init_default(
        &g_pub_joint_state,
        &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        UROS_JOINT_STATE_TOPIC));

    RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));

    /* init backing array for incoming message before handing to executor */
    g_wheel_cmd_msg.data.data     = g_wheel_cmd_data;
    g_wheel_cmd_msg.data.size     = 0;
    g_wheel_cmd_msg.data.capacity = 2;

    RCCHECK(rclc_executor_add_subscription(&g_executor,
                                           &g_sub_wheel_cmd,
                                           &g_wheel_cmd_msg,
                                           &wheel_vel_cmd_callback,
                                           ON_NEW_DATA));

    ESP_LOGI(TAG, "ROS entities created");
    return true;
}

static void ros_entities_destroy(void) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    rcl_subscription_fini(&g_sub_wheel_cmd,   &g_node);
    rcl_publisher_fini(&g_pub_joint_state,    &g_node);
    rclc_executor_fini(&g_executor);
    rcl_node_fini(&g_node);
    rclc_support_fini(&g_support);
#pragma GCC diagnostic pop
    ESP_LOGI(TAG, "ROS entities destroyed");
}

/* ----------------------------------------------------------------
 * micro-ROS task
 * ---------------------------------------------------------------- */
void microros_task(void *arg) {
    (void)arg;

    if (!joint_state_msg_init()) {
        ESP_LOGE(TAG, "JointState msg init failed");
        vTaskDelete(NULL);
        return;
    }

    const TickType_t publish_period = pdMS_TO_TICKS(1000 / UROS_PUBLISH_HZ);

    while (1) {
        ESP_LOGI(TAG, "Waiting for micro-ROS agent...");

        rmw_uros_set_custom_transport(
            true,
            (void *) &g_uart_port,
            esp32_serial_open,
            esp32_serial_close,
            esp32_serial_write,
            esp32_serial_read
        );

        while (rmw_uros_ping_agent(200, 5) != RMW_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        ESP_LOGI(TAG, "Agent found");

        if (!ros_entities_create()) {
            ESP_LOGE(TAG, "Entity creation failed — retrying");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        rmw_uros_sync_session(1000);
        g_last_cmd_ms = now_ms();

        TickType_t last_publish = xTaskGetTickCount();

        while (1) {
            if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
                ESP_LOGW(TAG, "Agent lost — stopping motors");
                kinco_stop_all();
                break;
            }

            rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

            if ((now_ms() - g_last_cmd_ms) > CMD_VEL_TIMEOUT_MS) {
                kinco_stop_all();
                g_last_cmd_ms = now_ms();
            }

            TickType_t now_ticks = xTaskGetTickCount();
            if ((now_ticks - last_publish) >= publish_period) {
                last_publish = now_ticks;
                fill_joint_state();
                stamp_now(&g_js_msg.header.stamp);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
                rcl_publish(&g_pub_joint_state, &g_js_msg, NULL);
#pragma GCC diagnostic pop
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }

        ros_entities_destroy();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    joint_state_msg_fini();
    vTaskDelete(NULL);
}