#pragma once
#include <stdint.h>

// ============================================================
//  QTRX-HD-15RC — Configuration pins et constantes
//  Teensy 4.1
// ============================================================

// 15 pins de lecture RC (bit0=C01 extrême gauche, bit14=C15 extrême droite)
static const uint8_t IR_PINS[15] = {
    21, 20, 19, 18, 17, 16, 15,    // C01–C07
    41, 40, 39, 38, 37, 36, 35, 34 // C08–C15
};

// Contrôle émetteurs IR
static const uint8_t CTRL_ODD_PIN  = 22;
static const uint8_t CTRL_EVEN_PIN = 23;

// Timeout de décharge RC en µs (5ms couvre le pire cas)
static const uint32_t RC_TIMEOUT_US = 5000;

// ------------------------------------------------------------------
//  Seuils individuels par capteur (µs) — noir si decay >= seuil
//
//  C01, C02, C14 : valeurs naturellement élevées → seuil 300µs
//  C11, C15      : pins flottants/ignorés        → seuil 0xFFFFFFFF
//  Tous les autres                               → seuil 180µs
// ------------------------------------------------------------------
static const uint32_t IR_THRESHOLDS[15] = {
    300,        // C01 — valeurs élevées
    0xFFFFFFFF, // C02 — valeurs élevées
    180,        // C03
    180,        // C04
    180,        // C05
    180,        // C06
    180,        // C07
    180,        // C08
    180,        // C09
    180,        // C10
    0xFFFFFFFF, // C11 — ignoré (pin flottant)
    300,        // C12
    0xFFFFFFFF, // C13
    300,        // C14 — valeurs élevées
    0xFFFFFFFF, // C15 — ignoré (pin flottant)
};

// Masque des capteurs ignorés (C11=bit10, C15=bit14)
// Ces bits sont toujours traités comme "blanc" dans tous les calculs
static const uint16_t IR_IGNORE_MASK = (1 << 10) | (1 << 14);

// Nombre de capteurs valides (15 - 2 ignorés)
static const uint8_t IR_VALID_COUNT = 13;

// Seuil intersection : fraction de capteurs VALIDES détectant du noir
// 0.85 ≈ 11 capteurs sur 13 → intersection détectée
static const float IR_INTERSECTION_RATIO = 0.85f;

// Fréquence de publication micro-ROS (50 Hz)
static const uint32_t PUB_PERIOD_MS = 20;

// Nombre total de capteurs
static const uint8_t IR_COUNT = 15;

// Position centre (milieu de 0–14000)
static const int32_t IR_CENTER = 7000;

// Valeur publiée sur /ir_position quand aucun capteur ne détecte la ligne
static const int32_t LINE_LOST = -1;

// Valeur publiée sur /ir_position quand intersection détectée
static const int32_t LINE_INTERSECTION = -2;