#pragma once
#include <Arduino.h>
#include "robot_config.h"

// =============================================================================
//  MOTOR DRIVER — Interface
//  Gère les 4 moteurs Mecanum via 2x VNH5019.
//
//  Convention de signe :
//    +pwm = ce moteur pousse le robot vers l'AVANT
//    L'inversion miroir (moteurs droite) est gérée ici, pas dans les fonctions
//    de mouvement.
//
//  Cinématique Mecanum (roues à 45°) :
//    Mouvement        | AVL | AVR | ARL | ARR
//    Avancer          |  +  |  +  |  +  |  +
//    Reculer          |  -  |  -  |  -  |  -
//    Pivot gauche     |  -  |  +  |  -  |  +
//    Pivot droite     |  +  |  -  |  +  |  -
//    Straffe gauche   |  -  |  +  |  +  |  -
//    Straffe droite   |  +  |  -  |  -  |  +
//    Diag AV-G        |  0  |  +  |  +  |  0
//    Diag AV-D        |  +  |  0  |  0  |  +
//    Diag AR-G        |  0  |  -  |  -  |  0
//    Diag AR-D        |  -  |  0  |  0  |  -
// =============================================================================

// -----------------------------------------------------------------------------
//  Init
// -----------------------------------------------------------------------------
void motorDriverInit();

// -----------------------------------------------------------------------------
//  Inversions (modifiables à chaud depuis diag ou depuis ROS)
// -----------------------------------------------------------------------------
extern bool inv_AVL;
extern bool inv_AVR;
extern bool inv_ARL;
extern bool inv_ARR;

// -----------------------------------------------------------------------------
//  Wrappers individuels (signe = intention robot)
// -----------------------------------------------------------------------------
void moteurAVL(int v);
void moteurAVR(int v);
void moteurARL(int v);
void moteurARR(int v);
void stopAll();

// -----------------------------------------------------------------------------
//  Mouvements Mecanum de haut niveau
// -----------------------------------------------------------------------------
void avancer(int pwm);
void reculer(int pwm);
void pivotGauche(int pwm);
void pivotDroite(int pwm);
void straffeGauche(int pwm);
void straffeDroite(int pwm);
void diagAvGauche(int pwm);
void diagAvDroite(int pwm);
void diagArGauche(int pwm);
void diagArDroite(int pwm);

// -----------------------------------------------------------------------------
//  Commande Mecanum générique depuis vecteur vitesse
//  vx   : vitesse longitudinale  [-1.0 .. 1.0]
//  vy   : vitesse latérale       [-1.0 .. 1.0]
//  wz   : vitesse angulaire      [-1.0 .. 1.0]
//  Chaque composante est normalisée, la mise à l'échelle vers PWM_MAX est faite
//  en interne. Sature si la somme dépasse PWM_MAX.
// -----------------------------------------------------------------------------
void setMecanumVelocity(float vx, float vy, float wz);

// -----------------------------------------------------------------------------
//  Diagnostic VNH5019
// -----------------------------------------------------------------------------
bool checkDriverFault(int en_diag_pin);
void printDriverStatus();