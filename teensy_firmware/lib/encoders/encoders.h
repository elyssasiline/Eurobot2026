#pragma once
#include <Arduino.h>
#include "robot_config.h"

// =============================================================================
//  ENCODEURS — Interface
//  Quadrature 4x sur les 4 roues Mecanum.
//  Résolution : TICKS_PAR_TOUR (défini dans robot_config.h)
// =============================================================================

// -----------------------------------------------------------------------------
//  Ticks bruts (accès externe en lecture)
// -----------------------------------------------------------------------------
extern volatile long ticks_AVL;
extern volatile long ticks_AVR;
extern volatile long ticks_ARL;
extern volatile long ticks_ARR;

// -----------------------------------------------------------------------------
//  Vitesses calculées (mm/s, mises à jour par majVitesse())
// -----------------------------------------------------------------------------
extern float vit_AVL;
extern float vit_AVR;
extern float vit_ARL;
extern float vit_ARR;

// -----------------------------------------------------------------------------
//  Init : attache les interruptions sur les 8 broches
// -----------------------------------------------------------------------------
void encodersInit();

// -----------------------------------------------------------------------------
//  Reset ticks et historique vitesse (à appeler avant un test)
// -----------------------------------------------------------------------------
void resetTicks();

// -----------------------------------------------------------------------------
//  Mise à jour des vitesses — à appeler dans loop() ou dans un timer.
//  Ne fait rien si PERIOD_ODOM_MS ne s'est pas écoulé.
//  Retourne true si une mise à jour a eu lieu.
// -----------------------------------------------------------------------------
bool majVitesse();

// -----------------------------------------------------------------------------
//  Lecture snapshot thread-safe
// -----------------------------------------------------------------------------
struct EncoderSnapshot {
    long avl, avr, arl, arr;
};
EncoderSnapshot getTicksSnapshot();

// -----------------------------------------------------------------------------
//  Odométrie différentielle simplifiée (Mecanum — approximation)
//  Retourne déplacement en mm et rotation en rad depuis le dernier appel.
//  À intégrer dans encoder_node.cpp pour construire le message odom.
// -----------------------------------------------------------------------------
struct OdomDelta {
    float dx_mm;   // déplacement avant/arrière
    float dy_mm;   // déplacement latéral (straffe)
    float dtheta;  // rotation (rad)
};
OdomDelta computeOdomDelta();