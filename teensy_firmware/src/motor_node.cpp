#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include "robot_config.h"
#include "motor_driver.h"

// =============================================================================
//  MOTOR NODE
//  Subscriber : /cmd_vel (geometry_msgs/Twist)
//
//  Reçoit vx (linear.x), vy (linear.y), wz (angular.z) en m/s et rad/s.
//  Normalise vers [-1.0, 1.0] selon les vitesses max configurées.
//  Appelle setMecanumVelocity() → moteurs.
//
//  Watchdog : si aucun cmd_vel reçu depuis WATCHDOG_MS, stop automatique.
// =============================================================================

#define WATCHDOG_MS       500   // ms sans cmd_vel → STOP
#define MAX_LINEAR_MS     0.30f // m/s max en translation
#define MAX_ANGULAR_RS    1.00f // rad/s max en rotation

static rcl_subscription_t cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;
static unsigned long last_cmd_time = 0;

// -----------------------------------------------------------------------------
//  Callback /cmd_vel
// -----------------------------------------------------------------------------
static void cmdVelCallback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg =
        (const geometry_msgs__msg__Twist*)msgin;

    last_cmd_time = millis();

    // Normalisation vers [-1.0, 1.0]
    float vx = constrain(msg->linear.x  / MAX_LINEAR_MS, -1.0f, 1.0f);
    float vy = constrain(msg->linear.y  / MAX_LINEAR_MS, -1.0f, 1.0f);
    float wz = constrain(msg->angular.z / MAX_ANGULAR_RS, -1.0f, 1.0f);

    setMecanumVelocity(vx, vy, wz);
}

// -----------------------------------------------------------------------------
//  Init (appelé depuis main.cpp)
// -----------------------------------------------------------------------------
void motorNodeInit(rcl_node_t* node, rclc_executor_t* executor) {
    rclc_subscription_init_default(
        &cmd_vel_sub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_CMD_VEL
    );

    rclc_executor_add_subscription(
        executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmdVelCallback,
        ON_NEW_DATA
    );

    last_cmd_time = millis();
}

// -----------------------------------------------------------------------------
//  Watchdog — appelé dans encoderNodeSpin() ou directement dans loop()
// -----------------------------------------------------------------------------
void motorNodeWatchdog() {
    if (millis() - last_cmd_time > WATCHDOG_MS) {
        stopAll();
    }
}