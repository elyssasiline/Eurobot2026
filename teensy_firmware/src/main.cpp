#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "robot_config.h"
#include "motor_driver.h"
#include "encoders.h"

void motorNodeInit(rcl_node_t* node, rclc_executor_t* executor);
void encoderNodeInit(rcl_node_t* node, rclc_support_t* support);
void sensorNodeInit(rcl_node_t* node, rclc_support_t* support);
void encoderNodeSpin();
void sensorNodeSpin();
void motorNodeWatchdog();

static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

// États de la machine micro-ROS
typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } ros_state_t;
static ros_state_t ros_state = WAITING_AGENT;

// Tente d'initialiser micro-ROS — retourne true si OK
static bool rosInit() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "teensy_robot", "", &support) != RCL_RET_OK) return false;
    if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK) return false;

    motorNodeInit(&node, &executor);
    encoderNodeInit(&node, &support);
    // sensorNodeInit(&node, &support);  // désactivé tant que pas de capteurs

    return true;
}

static void rosDestroy() {
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    stopAll();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    motorDriverInit();
    encodersInit();

    // micro-ROS sur USB Serial
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    ros_state = WAITING_AGENT;
}

void loop() {
    switch (ros_state) {

        case WAITING_AGENT:
            // Clignote lentement — en attente de l'agent sur la Raspberry
            if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
                ros_state = AGENT_AVAILABLE;
            }
            digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
            break;

        case AGENT_AVAILABLE:
            if (rosInit()) {
                ros_state = AGENT_CONNECTED;
                digitalWrite(LED_BUILTIN, HIGH); // LED fixe = connecté
            } else {
                ros_state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            // Vérifie que l'agent est toujours là
            if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                ros_state = AGENT_DISCONNECTED;
                break;
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            motorNodeWatchdog();
            majVitesse();
            encoderNodeSpin();
            // sensorNodeSpin();
            break;

        case AGENT_DISCONNECTED:
            rosDestroy();
            ros_state = WAITING_AGENT;
            digitalWrite(LED_BUILTIN, LOW);
            break;
    }
}