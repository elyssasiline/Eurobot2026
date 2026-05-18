#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include "ir_config.h"

// ============================================================
//  micro-ROS — objets globaux
// ============================================================
static rcl_allocator_t      allocator;
static rclc_support_t       support;
static rcl_node_t           node;
static rclc_executor_t      executor;
static rcl_publisher_t      mask_pub;          // /ir_line         — masque brut 15 bits
static rcl_publisher_t      pos_pub;           // /ir_position     — position 0–14000 / -1 perdu / -2 intersection
static rcl_publisher_t      intersect_pub;     // /ir_intersection — Bool
static std_msgs__msg__UInt16 mask_msg;
static std_msgs__msg__Int32  pos_msg;
static std_msgs__msg__Bool   intersect_msg;
static rcl_timer_t           timer;

typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} ros_state_t;

static ros_state_t ros_state = WAITING_AGENT;

// ============================================================
//  Lecture RC parallèle — seuils individuels par capteur
//  Retourne masque 15 bits, bit actif = ligne noire détectée
//  Les capteurs ignorés (IR_IGNORE_MASK) sont toujours à 0
// ============================================================
static uint16_t readIRMask() {
    // 1. Charger tous les condensateurs
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(IR_PINS[i], OUTPUT);
        digitalWrite(IR_PINS[i], HIGH);
    }
    delayMicroseconds(10);

    // 2. Passer en entrée et démarrer chrono
    uint32_t start = micros();
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(IR_PINS[i], INPUT);
    }

    // 3. Mesure de décharge en parallèle
    uint32_t decay[IR_COUNT];
    bool     done[IR_COUNT];
    for (int i = 0; i < IR_COUNT; i++) {
        decay[i] = RC_TIMEOUT_US;
        done[i]  = false;
    }
    uint8_t remaining = IR_COUNT;

    while (remaining > 0) {
        uint32_t elapsed = micros() - start;
        if (elapsed >= RC_TIMEOUT_US) break;
        for (int i = 0; i < IR_COUNT; i++) {
            if (!done[i] && !digitalRead(IR_PINS[i])) {
                decay[i] = elapsed;
                done[i]  = true;
                remaining--;
            }
        }
    }

    // 4. Seuillage individuel → masque, capteurs ignorés forcés à 0
    uint16_t mask = 0;
    for (int i = 0; i < IR_COUNT; i++) {
        if (IR_IGNORE_MASK & (1 << i)) continue;  // capteur ignoré
        if (decay[i] >= IR_THRESHOLDS[i]) {
            mask |= (uint16_t)(1 << i);
        }
    }
    return mask;
}

// ============================================================
//  Détection intersection
//  Vrai si >= IR_INTERSECTION_RATIO des capteurs VALIDES sont noirs
// ============================================================
static bool detectIntersection(uint16_t mask) {
    uint8_t black_count = 0;
    for (int i = 0; i < IR_COUNT; i++) {
        if (IR_IGNORE_MASK & (1 << i)) continue;
        if (mask & (1 << i)) black_count++;
    }
    return ((float)black_count / IR_VALID_COUNT) >= IR_INTERSECTION_RATIO;
}

// ============================================================
//  Calcul position centre de masse pondéré
//  Retourne 0–14000, LINE_LOST, ou LINE_INTERSECTION
// ============================================================
static int32_t computePosition(uint16_t mask) {
    if (detectIntersection(mask)) return LINE_INTERSECTION;

    int32_t numerator   = 0;
    int32_t denominator = 0;
    for (int i = 0; i < IR_COUNT; i++) {
        if (IR_IGNORE_MASK & (1 << i)) continue;
        if (mask & (1 << i)) {
            numerator   += i * 1000;
            denominator += 1000;
        }
    }
    if (denominator == 0) return LINE_LOST;
    return numerator / (denominator / 1000);
}

// ============================================================
//  Timer callback — 50 Hz
// ============================================================
static void timerCallback(rcl_timer_t* /*t*/, int64_t /*last*/) {
    uint16_t mask = readIRMask();
    int32_t  pos  = computePosition(mask);
    bool     intersection = (pos == LINE_INTERSECTION);

    mask_msg.data      = mask;
    pos_msg.data       = pos;
    intersect_msg.data = intersection;

    rcl_publish(&mask_pub,      &mask_msg,      NULL);
    rcl_publish(&pos_pub,       &pos_msg,        NULL);
    rcl_publish(&intersect_pub, &intersect_msg,  NULL);
}

// ============================================================
//  Init / Destroy micro-ROS
// ============================================================
static bool rosInit() {
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "teensy_ir", "", &support) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &mask_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
            "/ir_line") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &pos_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/ir_position") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &intersect_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "/ir_intersection") != RCL_RET_OK) return false;

    if (rclc_timer_init_default(
            &timer, &support,
            RCL_MS_TO_NS(PUB_PERIOD_MS),
            timerCallback) != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    rclc_executor_add_timer(&executor, &timer);
    return true;
}

static void rosDestroy() {
    rcl_publisher_fini(&mask_pub,      &node);
    rcl_publisher_fini(&pos_pub,       &node);
    rcl_publisher_fini(&intersect_pub, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ============================================================
//  Setup / Loop
// ============================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(CTRL_ODD_PIN,  OUTPUT); digitalWrite(CTRL_ODD_PIN,  HIGH);
    pinMode(CTRL_EVEN_PIN, OUTPUT); digitalWrite(CTRL_EVEN_PIN, HIGH);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    ros_state = WAITING_AGENT;
}

void loop() {
    switch (ros_state) {
        case WAITING_AGENT:
            if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                ros_state = AGENT_AVAILABLE;
            digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
            break;

        case AGENT_AVAILABLE:
            ros_state = rosInit() ? AGENT_CONNECTED : WAITING_AGENT;
            if (ros_state == AGENT_CONNECTED) digitalWrite(LED_BUILTIN, HIGH);
            break;

        case AGENT_CONNECTED:
            if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                ros_state = AGENT_DISCONNECTED;
                break;
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            break;

        case AGENT_DISCONNECTED:
            rosDestroy();
            ros_state = WAITING_AGENT;
            digitalWrite(LED_BUILTIN, LOW);
            break;
    }
}