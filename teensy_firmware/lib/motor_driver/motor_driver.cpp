#include "motor_driver.h"

// =============================================================================
//  MOTOR DRIVER — Implémentation
// =============================================================================

bool inv_AVL = INV_AVL;
bool inv_AVR = INV_AVR;
bool inv_ARL = INV_ARL;
bool inv_ARR = INV_ARR;

// -----------------------------------------------------------------------------
//  Init hardware
// -----------------------------------------------------------------------------
void motorDriverInit() {
    // Direction pins
    pinMode(M_AVL_INA, OUTPUT); pinMode(M_AVL_INB, OUTPUT);
    pinMode(M_AVR_INA, OUTPUT); pinMode(M_AVR_INB, OUTPUT);
    pinMode(M_ARL_INA, OUTPUT); pinMode(M_ARL_INB, OUTPUT);
    pinMode(M_ARR_INA, OUTPUT); pinMode(M_ARR_INB, OUTPUT);

    // EN/DIAG (entrées avec pull-up — HIGH = OK, LOW = fault)
    pinMode(M_AVL_EN_DIAG, INPUT_PULLUP);
    pinMode(M_AVR_EN_DIAG, INPUT_PULLUP);
    pinMode(M_ARL_EN_DIAG, INPUT_PULLUP);
    pinMode(M_ARR_EN_DIAG, INPUT_PULLUP);

    // PWM 20 kHz, résolution 12 bits
    analogWriteFrequency(M_AVL_PWM, PWM_FREQ_HZ);
    analogWriteFrequency(M_AVR_PWM, PWM_FREQ_HZ);
    analogWriteFrequency(M_ARL_PWM, PWM_FREQ_HZ);
    analogWriteFrequency(M_ARR_PWM, PWM_FREQ_HZ);
    analogWriteResolution(PWM_RESOLUTION_BITS);

    stopAll();
}

// -----------------------------------------------------------------------------
//  Driver bas niveau
//  vitesse > 0 → INA=H, INB=L
//  vitesse < 0 → INA=L, INB=H
//  vitesse = 0 → frein actif (INA=L, INB=L, PWM=MAX)
// -----------------------------------------------------------------------------
static void setMoteur(int ina, int inb, int pwm_pin, int vitesse) {
    vitesse = constrain(vitesse, -PWM_MAX, PWM_MAX);
    if (vitesse > 0) {
        digitalWrite(ina, HIGH); digitalWrite(inb, LOW);
        analogWrite(pwm_pin, vitesse);
    } else if (vitesse < 0) {
        digitalWrite(ina, LOW); digitalWrite(inb, HIGH);
        analogWrite(pwm_pin, -vitesse);
    } else {
        digitalWrite(ina, LOW); digitalWrite(inb, LOW);
        analogWrite(pwm_pin, PWM_MAX);
    }
}

// -----------------------------------------------------------------------------
//  Wrappers avec gestion inversion miroir
// -----------------------------------------------------------------------------
void moteurAVL(int v) { setMoteur(M_AVL_INA, M_AVL_INB, M_AVL_PWM, inv_AVL ? -v : v); }
void moteurAVR(int v) { setMoteur(M_AVR_INA, M_AVR_INB, M_AVR_PWM, inv_AVR ? -v : v); }
void moteurARL(int v) { setMoteur(M_ARL_INA, M_ARL_INB, M_ARL_PWM, inv_ARL ? -v : v); }
void moteurARR(int v) { setMoteur(M_ARR_INA, M_ARR_INB, M_ARR_PWM, inv_ARR ? -v : v); }

void stopAll() {
    moteurAVL(0); moteurAVR(0); moteurARL(0); moteurARR(0);
}

// -----------------------------------------------------------------------------
//  Mouvements Mecanum
// -----------------------------------------------------------------------------
void avancer(int pwm)       { moteurAVL( pwm); moteurAVR( pwm); moteurARL( pwm); moteurARR( pwm); }
void reculer(int pwm)       { moteurAVL(-pwm); moteurAVR(-pwm); moteurARL(-pwm); moteurARR(-pwm); }
void pivotGauche(int pwm)   { moteurAVL(-pwm); moteurAVR( pwm); moteurARL(-pwm); moteurARR( pwm); }
void pivotDroite(int pwm)   { moteurAVL( pwm); moteurAVR(-pwm); moteurARL( pwm); moteurARR(-pwm); }
void straffeGauche(int pwm) { moteurAVL(-pwm); moteurAVR( pwm); moteurARL( pwm); moteurARR(-pwm); }
void straffeDroite(int pwm) { moteurAVL( pwm); moteurAVR(-pwm); moteurARL(-pwm); moteurARR( pwm); }
void diagAvGauche(int pwm)  { moteurAVL(   0); moteurAVR( pwm); moteurARL( pwm); moteurARR(   0); }
void diagAvDroite(int pwm)  { moteurAVL( pwm); moteurAVR(   0); moteurARL(   0); moteurARR( pwm); }
void diagArGauche(int pwm)  { moteurAVL(   0); moteurAVR(-pwm); moteurARL(-pwm); moteurARR(   0); }
void diagArDroite(int pwm)  { moteurAVL(-pwm); moteurAVR(   0); moteurARL(   0); moteurARR(-pwm); }

// -----------------------------------------------------------------------------
//  Commande Mecanum générique
//  vx, vy, wz normalisés [-1.0, 1.0]
//
//  Cinématique :
//    AVL = vx - vy - wz
//    AVR = vx + vy + wz
//    ARL = vx + vy - wz
//    ARR = vx - vy + wz
//
//  On normalise après pour ne pas saturer asymétriquement.
// -----------------------------------------------------------------------------
void setMecanumVelocity(float vx, float vy, float wz) {
    float avl = vx - vy - wz;
    float avr = vx + vy + wz;
    float arl = vx + vy - wz;
    float arr = vx - vy + wz;

    // Normalisation si l'une des roues dépasse 1.0
    float max_val = max(max(fabsf(avl), fabsf(avr)), max(fabsf(arl), fabsf(arr)));
    if (max_val > 1.0f) {
        avl /= max_val;
        avr /= max_val;
        arl /= max_val;
        arr /= max_val;
    }

    moteurAVL((int)(avl * PWM_MAX));
    moteurAVR((int)(avr * PWM_MAX));
    moteurARL((int)(arl * PWM_MAX));
    moteurARR((int)(arr * PWM_MAX));
}

// -----------------------------------------------------------------------------
//  Diagnostic VNH5019
// -----------------------------------------------------------------------------
bool checkDriverFault(int en_diag_pin) {
    return digitalRead(en_diag_pin) == LOW; // LOW = fault
}

void printDriverStatus() {
    struct { int pin; const char* nom; } drivers[] = {
        { M_AVL_EN_DIAG, "AVL (m1)" },
        { M_AVR_EN_DIAG, "AVR (m2)" },
        { M_ARL_EN_DIAG, "ARL (m3)" },
        { M_ARR_EN_DIAG, "ARR (m4)" },
    };
    Serial.println("[DIAG VNH5019]");
    for (auto& d : drivers) {
        Serial.print("  "); Serial.print(d.nom); Serial.print(" : ");
        Serial.println(digitalRead(d.pin) == HIGH ? "OK" : "!! DÉFAUT !!");
    }
}