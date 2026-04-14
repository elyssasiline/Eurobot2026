#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>

#include "robot_config.h"

// =============================================================================
//  SENSOR NODE
//  Publishers :
//    /ultrasound/front  (sensor_msgs/Range)   — 100 ms
//    /ultrasound/rear   (sensor_msgs/Range)   — 100 ms
//    /ir_line           (std_msgs/UInt8)       — 100 ms  (bitmask : bit0=G, bit1=C, bit2=D)
//    /battery           (std_msgs/Float32)     — 1000 ms (tension en V)
//    /current           (std_msgs/Float32)     — 1000 ms (courant en A)
// =============================================================================

// -----------------------------------------------------------------------------
//  Publishers
// -----------------------------------------------------------------------------
static rcl_publisher_t sonar_front_pub;
static rcl_publisher_t sonar_rear_pub;
static rcl_publisher_t ir_line_pub;
static rcl_publisher_t battery_pub;
static rcl_publisher_t current_pub;

// Messages
static sensor_msgs__msg__Range sonar_front_msg;
static sensor_msgs__msg__Range sonar_rear_msg;
static std_msgs__msg__UInt8    ir_line_msg;
static std_msgs__msg__Float32  battery_msg;
static std_msgs__msg__Float32  current_msg;

static unsigned long last_sensor_time  = 0;
static unsigned long last_battery_time = 0;

// -----------------------------------------------------------------------------
//  Constantes sonar HC-SR04
// -----------------------------------------------------------------------------
#define SONAR_MIN_M    0.02f   // 2 cm
#define SONAR_MAX_M    4.00f   // 4 m
#define SONAR_FOV_RAD  0.2618f // ~15°
#define SONAR_TIMEOUT_US 25000 // 4m max

// -----------------------------------------------------------------------------
//  Lecture sonar HC-SR04 (bloquant max 25 ms — acceptable à 100 ms)
// -----------------------------------------------------------------------------
static float readSonar(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, SONAR_TIMEOUT_US);
    if (duration == 0) return SONAR_MAX_M; // timeout = pas d'obstacle détecté

    float dist = (duration * 0.0343f) / 2.0f / 100.0f; // cm → m
    return constrain(dist, SONAR_MIN_M, SONAR_MAX_M);
}

// -----------------------------------------------------------------------------
//  Lecture IR ligne noire
//  Retourne bitmask : bit 0 = gauche, bit 1 = centre, bit 2 = droite
//  Les capteurs IR sont actifs LOW (ligne noire = LOW)
// -----------------------------------------------------------------------------
static uint8_t readIrLine() {
    uint8_t mask = 0;
    if (!digitalRead(IR_LINE_LEFT))   mask |= 0x01;
    if (!digitalRead(IR_LINE_CENTER)) mask |= 0x02;
    if (!digitalRead(IR_LINE_RIGHT))  mask |= 0x04;
    return mask;
}

// -----------------------------------------------------------------------------
//  Lecture batterie (diviseur résistif, à calibrer selon ton pont)
//  Exemple : 3S LiPo (12.6V max), ADC 3.3V, diviseur 1/5
//  → tension_réelle = (adc / 4096.0) * 3.3 * 5.0
// -----------------------------------------------------------------------------
#define BATTERY_DIVIDER  5.0f
#define CURRENT_SENSE_MV_A  66.0f   // ACS712-30A : 66 mV/A

static float readBatteryVoltage() {
    int raw = analogRead(PIN_BATTERY_ADC);
    return (raw / 4096.0f) * 3.3f * BATTERY_DIVIDER;
}

static float readCurrentAmps() {
    int raw = analogRead(PIN_CURRENT_ADC);
    float mv = (raw / 4096.0f) * 3300.0f;
    float mv_offset = 1650.0f; // ACS712 : 0A = VCC/2
    return (mv - mv_offset) / CURRENT_SENSE_MV_A;
}

// -----------------------------------------------------------------------------
//  Init
// -----------------------------------------------------------------------------
void sensorNodeInit(rcl_node_t* node, rclc_support_t* support) {
    (void)support;

    // Pins capteurs
    pinMode(SONAR_TRIG_FRONT, OUTPUT); pinMode(SONAR_ECHO_FRONT, INPUT);
    pinMode(SONAR_TRIG_REAR,  OUTPUT); pinMode(SONAR_ECHO_REAR,  INPUT);
    pinMode(IR_LINE_LEFT,   INPUT_PULLUP);
    pinMode(IR_LINE_CENTER, INPUT_PULLUP);
    pinMode(IR_LINE_RIGHT,  INPUT_PULLUP);
    analogReadResolution(12);

    // Publishers
    rclc_publisher_init_default(&sonar_front_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), TOPIC_ULTRASOUND "/front");

    rclc_publisher_init_default(&sonar_rear_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), TOPIC_ULTRASOUND "/rear");

    rclc_publisher_init_default(&ir_line_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), TOPIC_IR_LINE);

    rclc_publisher_init_default(&battery_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TOPIC_BATTERY);

    rclc_publisher_init_default(&current_pub, node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/current");

    // Init messages Range
    auto initRange = [](sensor_msgs__msg__Range& msg, const char* frame) {
        msg.radiation_type     = sensor_msgs__msg__Range__ULTRASOUND;
        msg.field_of_view      = SONAR_FOV_RAD;
        msg.min_range          = SONAR_MIN_M;
        msg.max_range          = SONAR_MAX_M;
        msg.header.frame_id.data     = (char*)frame;
        msg.header.frame_id.size     = strlen(frame);
        msg.header.frame_id.capacity = strlen(frame) + 1;
    };
    initRange(sonar_front_msg, "sonar_front");
    initRange(sonar_rear_msg,  "sonar_rear");

    last_sensor_time  = millis();
    last_battery_time = millis();
}

// -----------------------------------------------------------------------------
//  Spin — appelé dans loop()
// -----------------------------------------------------------------------------
void sensorNodeSpin() {
    unsigned long now = millis();

    // Capteurs ultrasons + IR (100 ms)
    if (now - last_sensor_time >= PERIOD_SENSORS_MS) {
        last_sensor_time = now;

        // Sonar avant
        float dist_front = readSonar(SONAR_TRIG_FRONT, SONAR_ECHO_FRONT);
        sonar_front_msg.range = dist_front;
        sonar_front_msg.header.stamp.sec     = (int32_t)(now / 1000);
        sonar_front_msg.header.stamp.nanosec = (uint32_t)((now % 1000) * 1000000);
        (void)rcl_publish(&sonar_front_pub, &sonar_front_msg, NULL);

        // Sonar arrière
        float dist_rear = readSonar(SONAR_TRIG_REAR, SONAR_ECHO_REAR);
        sonar_rear_msg.range = dist_rear;
        sonar_rear_msg.header.stamp.sec     = sonar_front_msg.header.stamp.sec;
        sonar_rear_msg.header.stamp.nanosec = sonar_front_msg.header.stamp.nanosec;
        (void)rcl_publish(&sonar_rear_pub, &sonar_rear_msg, NULL);

        // IR ligne noire
        ir_line_msg.data = readIrLine();
        (void)rcl_publish(&ir_line_pub, &ir_line_msg, NULL);
    }

    // Batterie + courant (1000 ms)
    if (now - last_battery_time >= PERIOD_BATTERY_MS) {
        last_battery_time = now;

        battery_msg.data = readBatteryVoltage();
        (void)rcl_publish(&battery_pub, &battery_msg, NULL);

        current_msg.data = readCurrentAmps();
        (void)rcl_publish(&current_pub, &current_msg, NULL);
    }
}