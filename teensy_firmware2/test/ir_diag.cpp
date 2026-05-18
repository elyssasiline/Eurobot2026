/**
 * ir_diag.cpp — Diagnostic QTRX-HD-15RC
 */

#include <Arduino.h>
#include "ir_config.h"

// Seuil modifiable en live depuis le moniteur série
static uint32_t threshold = IR_THRESHOLDS[0];
static bool     showBars  = true;

// ============================================================
//  Lecture brute — retourne les 15 temps de décharge en µs
// ============================================================
static void readRaw(uint32_t out[IR_COUNT]) {
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(IR_PINS[i], OUTPUT);
        digitalWrite(IR_PINS[i], HIGH);
    }
    delayMicroseconds(10);

    uint32_t start = micros();
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(IR_PINS[i], INPUT);
        out[i] = RC_TIMEOUT_US;
    }

    bool    done[IR_COUNT] = {false};
    uint8_t remaining      = IR_COUNT;

    while (remaining > 0 && (micros() - start) < RC_TIMEOUT_US) {
        for (int i = 0; i < IR_COUNT; i++) {
            if (!done[i] && !digitalRead(IR_PINS[i])) {
                out[i] = micros() - start;
                done[i]  = true;
                remaining--;
            }
        }
    }
}

// ============================================================
//  Affichage d'une barre proportionnelle avec indicateur B/N
// ============================================================
static void printBar(uint32_t val, uint32_t thresh) {
    bool isBlack = (val >= thresh);
    int  bars    = (int)map((long)constrain((long)val, 0L, (long)RC_TIMEOUT_US),
                             0L, (long)RC_TIMEOUT_US, 0L, 16L);

    Serial.print(isBlack ? " N|" : " B|");
    for (int i = 0; i < bars; i++)    Serial.print('#');
    for (int i = bars; i < 16; i++)   Serial.print(' ');
    Serial.print('|');
}

// ============================================================
//  Routine de calibration guidée
// ============================================================
static void runCalibration() {
    uint32_t decay[IR_COUNT];
    uint32_t whiteMin = RC_TIMEOUT_US, whiteMax = 0;
    uint32_t blackMin = RC_TIMEOUT_US, blackMax = 0;

    Serial.println();
    Serial.println("=== CALIBRATION ===");
    Serial.println("1. Pose le robot sur surface BLANCHE et appuie sur Entrée...");
    while (!Serial.available()) {}
    while (Serial.available()) Serial.read();

    for (int r = 0; r < 10; r++) {
        readRaw(decay);
        for (int i = 0; i < IR_COUNT; i++) {
            if (decay[i] > 1000) continue;
            if (decay[i] < whiteMin) whiteMin = decay[i];
            if (decay[i] > whiteMax) whiteMax = decay[i];
        }
        delay(50);
    }
    Serial.print("  Blanc — min: "); Serial.print(whiteMin);
    Serial.print("µs  max: "); Serial.print(whiteMax); Serial.println("µs");

    Serial.println("2. Pose le robot sur la LIGNE NOIRE et appuie sur Entrée...");
    while (!Serial.available()) {}
    while (Serial.available()) Serial.read();

    for (int r = 0; r < 10; r++) {
        readRaw(decay);
        for (int i = 0; i < IR_COUNT; i++) {
            if (decay[i] > 1000) continue;
            if (decay[i] < blackMin) blackMin = decay[i];
            if (decay[i] > blackMax) blackMax = decay[i];
        }
        delay(50);
    }
    Serial.print("  Noir  — min: "); Serial.print(blackMin);
    Serial.print("µs  max: "); Serial.print(blackMax); Serial.println("µs");

    if (blackMin > whiteMax) {
        threshold = (whiteMax + blackMin) / 2;
        Serial.print("  ✓ Seuil recommandé : ");
        Serial.print(threshold);
        Serial.println("µs  (reporté dans ir_config.h : IR_THRESHOLDS)");
    } else {
        Serial.println("  ✗ Overlap blanc/noir — augmente la hauteur du capteur ou vérifie l'éclairage");
    }
    Serial.println("===================");
    Serial.println();
}

// ============================================================
//  Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    pinMode(CTRL_ODD_PIN,  OUTPUT); digitalWrite(CTRL_ODD_PIN,  HIGH);
    pinMode(CTRL_EVEN_PIN, OUTPUT); digitalWrite(CTRL_EVEN_PIN, HIGH);
    Serial.print("CTRL_ODD_PIN ("); Serial.print(CTRL_ODD_PIN); Serial.print(") = ");
    Serial.println(digitalRead(CTRL_ODD_PIN));
    Serial.print("CTRL_EVEN_PIN ("); Serial.print(CTRL_EVEN_PIN); Serial.print(") = ");
    Serial.println(digitalRead(CTRL_EVEN_PIN));

    Serial.println("========================================");
    Serial.println("  QTRX-HD-15RC — Diagnostic IR");
    Serial.println("========================================");
    Serial.println("  '+'  seuil +50µs");
    Serial.println("  '-'  seuil -50µs");
    Serial.println("  'r'  toggle barres visuelles");
    Serial.println("  'c'  calibration guidée auto");
    Serial.println("========================================");
    Serial.println();
}

// ============================================================
//  Loop
// ============================================================
void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '+') {
            threshold += 50;
            Serial.print("► Seuil → "); Serial.print(threshold); Serial.println("µs");
        } else if (c == '-') {
            threshold = max(50u, threshold - 50);
            Serial.print("► Seuil → "); Serial.print(threshold); Serial.println("µs");
        } else if (c == 'r') {
            showBars = !showBars;
            Serial.println(showBars ? "► Barres ON" : "► Barres OFF");
        } else if (c == 'c') {
            runCalibration();
        }
    }

    uint32_t decay[IR_COUNT];
    readRaw(decay);

    // --- En-tête capteurs ---
    Serial.print("Seuil=");
    Serial.print(threshold);
    Serial.print("µs | ");
    for (int i = 0; i < IR_COUNT; i++) {
        Serial.print("C");
        if (i < 9) Serial.print('0');
        Serial.print(i + 1);
        Serial.print("     ");
    }
    Serial.println();

    // --- Valeurs brutes µs ---
    Serial.print("           | ");
    for (int i = 0; i < IR_COUNT; i++) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%4luµs ", (unsigned long)decay[i]);
        Serial.print(buf);
    }
    Serial.println();

    // --- Barres visuelles B/N — utilise IR_THRESHOLDS[i] par capteur ---
    if (showBars) {
        Serial.print("           |");
        for (int i = 0; i < IR_COUNT; i++) {
            printBar(decay[i], IR_THRESHOLDS[i]);
        }
        Serial.println();
    }

    // --- Position centre de masse — utilise IR_THRESHOLDS[i] par capteur ---
    int32_t num = 0, den = 0;
    for (int i = 0; i < IR_COUNT; i++) {
        if (IR_IGNORE_MASK & (1 << i)) continue;
        if (decay[i] >= IR_THRESHOLDS[i]) {
            num += i * 1000;
            den += 1000;
        }
    }
    Serial.print("           | Position: ");
    if (den == 0) {
        Serial.println("LIGNE PERDUE");
    } else {
        int32_t pos = num / (den / 1000);
        float   err = (pos - 7000) / 7000.0f;
        Serial.print(pos);
        Serial.print(" / 14000  erreur: ");
        Serial.print(err, 3);
        Serial.println(err > 0 ? "  (droite)" : "  (gauche)");
    }

    Serial.println("-----------|---------------------------"
                   "--------------------------------------------");
    Serial.println();
    delay(300);
}