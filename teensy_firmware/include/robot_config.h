#pragma once

// =============================================================================
//  ROBOT MECANUM — Configuration centrale
//  Teensy 4.1 + 2x VNH5019
// =============================================================================

// -----------------------------------------------------------------------------
//  PINOUT MOTEURS
// -----------------------------------------------------------------------------
#define M_ARR_INA      9
#define M_ARR_INB      10
#define M_ARR_PWM      8
#define M_ARR_EN_DIAG  32

#define M_ARL_INA      11
#define M_ARL_INB      27
#define M_ARL_PWM      12
#define M_ARL_EN_DIAG  33

#define M_AVL_INA      23
#define M_AVL_INB      24
#define M_AVL_PWM      22
#define M_AVL_EN_DIAG  26

#define M_AVR_INA      25
#define M_AVR_INB      29
#define M_AVR_PWM      28
#define M_AVR_EN_DIAG  31

// -----------------------------------------------------------------------------
//  PINOUT ENCODEURS
// -----------------------------------------------------------------------------
#define ENC_ARL_A      4
#define ENC_ARL_B      5
#define ENC_ARR_A      6
#define ENC_ARR_B      7
#define ENC_AVL_A      2
#define ENC_AVL_B      3
#define ENC_AVR_A      0
#define ENC_AVR_B      1

// -----------------------------------------------------------------------------
//  PINOUT CAPTEURS (à adapter selon câblage réel)
// -----------------------------------------------------------------------------
#define SONAR_TRIG_FRONT   34
#define SONAR_ECHO_FRONT   35
#define SONAR_TRIG_REAR    36
#define SONAR_ECHO_REAR    37

#define IR_LINE_LEFT       38
#define IR_LINE_CENTER     39
#define IR_LINE_RIGHT      40

#define PIN_BATTERY_ADC    A0
#define PIN_CURRENT_ADC    A1

// -----------------------------------------------------------------------------
//  CONSTANTES MOTEURS
// -----------------------------------------------------------------------------
#define PWM_MAX              4095
#define PWM_FREQ_HZ          20000
#define PWM_RESOLUTION_BITS  12

// -----------------------------------------------------------------------------
//  CONSTANTES ENCODEURS / ODOMÉTRIE
// -----------------------------------------------------------------------------
#define TICKS_PAR_TOUR      33600.0f
#define DIAMETRE_ROUE_MM    80.0f
#define PERIMETRE_ROUE_MM   (DIAMETRE_ROUE_MM * 3.14159265f)
#define MM_PAR_TICK         (PERIMETRE_ROUE_MM / TICKS_PAR_TOUR)

// Distance entre roues gauche et droite (à mesurer sur le robot, en m)
#define WHEEL_BASE_M        0.20f
// Distance entre roues avant et arrière (à mesurer, en m)
#define WHEEL_TRACK_M       0.18f

// -----------------------------------------------------------------------------
//  PÉRIODES DE PUBLICATION (ms)
// -----------------------------------------------------------------------------
#define PERIOD_ODOM_MS      50
#define PERIOD_SENSORS_MS   100
#define PERIOD_BATTERY_MS   1000

// -----------------------------------------------------------------------------
//  INVERSIONS MOTEURS
//  Les moteurs droite sont montés en miroir mécanique.
//  Modifier ici après calibration (ou via commande série en mode diag).
// -----------------------------------------------------------------------------
#define INV_AVL  true
#define INV_AVR  false
#define INV_ARL  true
#define INV_ARR  false

// -----------------------------------------------------------------------------
//  TOPICS ROS2
// -----------------------------------------------------------------------------
#define TOPIC_CMD_VEL       "/cmd_vel"
#define TOPIC_WHEEL_ODOM    "/wheel_odom"
#define TOPIC_ULTRASOUND    "/ultrasound"
#define TOPIC_IR_LINE       "/ir_line"
#define TOPIC_BATTERY       "/battery"