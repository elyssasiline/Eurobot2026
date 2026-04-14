#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "robot_config.h"
#include "encoders.h"

// =============================================================================
//  ENCODER NODE
//  Publisher : /wheel_odom (nav_msgs/Odometry)   — 50 ms
//  Publisher : /wheel_ticks (Float32MultiArray)  — 50 ms [avl, avr, arl, arr, vavl, vavr, varl, varr]
//
//  L'odométrie publiée est une intégration locale (pas de correction EKF).
//  Le node de navigation ROS2 côté Raspberry fusionne avec le lidar.
// =============================================================================

static rcl_publisher_t odom_pub;
static rcl_publisher_t ticks_pub;
static nav_msgs__msg__Odometry odom_msg;
static std_msgs__msg__Float32MultiArray ticks_msg;

// Pose intégrée (frame odom)
static float pose_x     = 0.0f;
static float pose_y     = 0.0f;
static float pose_theta = 0.0f;

static unsigned long last_pub_time      = 0;
static unsigned long last_pub_time_prev = 0;  // pour calcul dt réel

// Float array data (8 valeurs : 4 ticks + 4 vitesses)
static float ticks_data[8];

// -----------------------------------------------------------------------------
//  Init
// -----------------------------------------------------------------------------
void encoderNodeInit(rcl_node_t* node, rclc_support_t* support) {
    (void)support;

    rclc_publisher_init_default(
        &odom_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_WHEEL_ODOM
    );

    rclc_publisher_init_default(
        &ticks_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/wheel_ticks"
    );

    // Init message odom
    odom_msg.header.frame_id.data     = (char*)"odom";
    odom_msg.header.frame_id.size     = 4;
    odom_msg.header.frame_id.capacity = 5;
    odom_msg.child_frame_id.data      = (char*)"base_link";
    odom_msg.child_frame_id.size      = 9;
    odom_msg.child_frame_id.capacity  = 10;

    // Covariance diagonale simple (sera affinée après calibration)
    for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i]   = 0.0;
    for (int i = 0; i < 36; i++) odom_msg.twist.covariance[i]  = 0.0;
    odom_msg.pose.covariance[0]  = 0.001; // x
    odom_msg.pose.covariance[7]  = 0.001; // y
    odom_msg.pose.covariance[35] = 0.01;  // yaw

    // Init Float32MultiArray
    ticks_msg.data.data     = ticks_data;
    ticks_msg.data.size     = 8;
    ticks_msg.data.capacity = 8;

    last_pub_time      = millis();
    last_pub_time_prev = millis();
}

// -----------------------------------------------------------------------------
//  Spin — appelé dans loop()
// -----------------------------------------------------------------------------
void encoderNodeSpin() {
    unsigned long now = millis();
    if (now - last_pub_time < PERIOD_ODOM_MS) return;

    // dt réel entre deux publications (évite la division par zéro au démarrage)
    float dt = (now - last_pub_time) / 1000.0f;
    if (dt < 0.001f) dt = PERIOD_ODOM_MS / 1000.0f;

    last_pub_time_prev = last_pub_time;
    last_pub_time      = now;

    // --- Odométrie ---
    OdomDelta delta = computeOdomDelta();

    // Intégration pose (frame odom, repère robot)
    float dx_m = delta.dx_mm / 1000.0f;
    float dy_m = delta.dy_mm / 1000.0f;

    pose_x     += dx_m * cosf(pose_theta) - dy_m * sinf(pose_theta);
    pose_y     += dx_m * sinf(pose_theta) + dy_m * cosf(pose_theta);
    pose_theta += delta.dtheta;

    // Normalise theta dans [-pi, pi]
    while (pose_theta >  3.14159265f) pose_theta -= 2.0f * 3.14159265f;
    while (pose_theta < -3.14159265f) pose_theta += 2.0f * 3.14159265f;

    // Quaternion depuis yaw (roulis/tangage = 0)
    float cy = cosf(pose_theta * 0.5f);
    float sy = sinf(pose_theta * 0.5f);

    // Remplissage message odom
    odom_msg.header.stamp.sec     = (int32_t)(now / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((now % 1000) * 1000000);

    odom_msg.pose.pose.position.x    = pose_x;
    odom_msg.pose.pose.position.y    = pose_y;
    odom_msg.pose.pose.position.z    = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.pose.pose.orientation.w = cy;

    odom_msg.twist.twist.linear.x  = (vit_AVL + vit_AVR + vit_ARL + vit_ARR) / 4000.0f;  // mm/s → m/s
    odom_msg.twist.twist.linear.y  = (-vit_AVL + vit_AVR + vit_ARL - vit_ARR) / 4000.0f;
    odom_msg.twist.twist.angular.z = delta.dtheta / dt;

    (void)rcl_publish(&odom_pub, &odom_msg, NULL);

    // --- Ticks bruts + vitesses ---
    EncoderSnapshot snap = getTicksSnapshot();
    ticks_data[0] = (float)snap.avl;
    ticks_data[1] = (float)snap.avr;
    ticks_data[2] = (float)snap.arl;
    ticks_data[3] = (float)snap.arr;
    ticks_data[4] = vit_AVL;
    ticks_data[5] = vit_AVR;
    ticks_data[6] = vit_ARL;
    ticks_data[7] = vit_ARR;

    (void)rcl_publish(&ticks_pub, &ticks_msg, NULL);
}