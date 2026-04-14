#include "encoders.h"

// =============================================================================
//  ENCODEURS — Implémentation
// =============================================================================

volatile long ticks_AVL = 0;
volatile long ticks_AVR = 0;
volatile long ticks_ARL = 0;
volatile long ticks_ARR = 0;

float vit_AVL = 0.0f;
float vit_AVR = 0.0f;
float vit_ARL = 0.0f;
float vit_ARR = 0.0f;

static long prev_AVL = 0, prev_AVR = 0, prev_ARL = 0, prev_ARR = 0;
static unsigned long t_prev_vit = 0;

// Pour l'odométrie delta
static long odom_prev_AVL = 0, odom_prev_AVR = 0;
static long odom_prev_ARL = 0, odom_prev_ARR = 0;

// -----------------------------------------------------------------------------
//  ISR quadrature
//  Décodage par comparaison A==B au moment du front :
//    Front A : A==B → sens-, A!=B → sens+
//    Front B : A==B → sens+, A!=B → sens-
// -----------------------------------------------------------------------------
void FASTRUN isr_AVL_A() { digitalReadFast(ENC_AVL_A)==digitalReadFast(ENC_AVL_B) ? ticks_AVL-- : ticks_AVL++; }
void FASTRUN isr_AVL_B() { digitalReadFast(ENC_AVL_A)==digitalReadFast(ENC_AVL_B) ? ticks_AVL++ : ticks_AVL--; }
void FASTRUN isr_AVR_A() { digitalReadFast(ENC_AVR_A)==digitalReadFast(ENC_AVR_B) ? ticks_AVR-- : ticks_AVR++; }
void FASTRUN isr_AVR_B() { digitalReadFast(ENC_AVR_A)==digitalReadFast(ENC_AVR_B) ? ticks_AVR++ : ticks_AVR--; }
void FASTRUN isr_ARL_A() { digitalReadFast(ENC_ARL_A)==digitalReadFast(ENC_ARL_B) ? ticks_ARL-- : ticks_ARL++; }
void FASTRUN isr_ARL_B() { digitalReadFast(ENC_ARL_A)==digitalReadFast(ENC_ARL_B) ? ticks_ARL++ : ticks_ARL--; }
void FASTRUN isr_ARR_A() { digitalReadFast(ENC_ARR_A)==digitalReadFast(ENC_ARR_B) ? ticks_ARR-- : ticks_ARR++; }
void FASTRUN isr_ARR_B() { digitalReadFast(ENC_ARR_A)==digitalReadFast(ENC_ARR_B) ? ticks_ARR++ : ticks_ARR--; }

// -----------------------------------------------------------------------------
//  Init
// -----------------------------------------------------------------------------
void encodersInit() {
    pinMode(ENC_AVL_A, INPUT_PULLUP); pinMode(ENC_AVL_B, INPUT_PULLUP);
    pinMode(ENC_AVR_A, INPUT_PULLUP); pinMode(ENC_AVR_B, INPUT_PULLUP);
    pinMode(ENC_ARL_A, INPUT_PULLUP); pinMode(ENC_ARL_B, INPUT_PULLUP);
    pinMode(ENC_ARR_A, INPUT_PULLUP); pinMode(ENC_ARR_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_AVL_A), isr_AVL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_AVL_B), isr_AVL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_AVR_A), isr_AVR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_AVR_B), isr_AVR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_ARL_A), isr_ARL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_ARL_B), isr_ARL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_ARR_A), isr_ARR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_ARR_B), isr_ARR_B, CHANGE);

    t_prev_vit = millis();
}

// -----------------------------------------------------------------------------
//  Reset
// -----------------------------------------------------------------------------
void resetTicks() {
    noInterrupts();
    ticks_AVL = 0; ticks_AVR = 0; ticks_ARL = 0; ticks_ARR = 0;
    prev_AVL = 0;  prev_AVR = 0;  prev_ARL = 0;  prev_ARR = 0;
    odom_prev_AVL = 0; odom_prev_AVR = 0;
    odom_prev_ARL = 0; odom_prev_ARR = 0;
    interrupts();
}

// -----------------------------------------------------------------------------
//  Vitesses
// -----------------------------------------------------------------------------
bool majVitesse() {
    unsigned long now = millis();
    if (now - t_prev_vit < PERIOD_ODOM_MS) return false;

    float dt = (now - t_prev_vit) / 1000.0f;
    t_prev_vit = now;

    noInterrupts();
    long dAVL = ticks_AVL - prev_AVL; prev_AVL = ticks_AVL;
    long dAVR = ticks_AVR - prev_AVR; prev_AVR = ticks_AVR;
    long dARL = ticks_ARL - prev_ARL; prev_ARL = ticks_ARL;
    long dARR = ticks_ARR - prev_ARR; prev_ARR = ticks_ARR;
    interrupts();

    vit_AVL = (dAVL * MM_PAR_TICK) / dt;
    vit_AVR = (dAVR * MM_PAR_TICK) / dt;
    vit_ARL = (dARL * MM_PAR_TICK) / dt;
    vit_ARR = (dARR * MM_PAR_TICK) / dt;

    return true;
}

// -----------------------------------------------------------------------------
//  Snapshot thread-safe
// -----------------------------------------------------------------------------
EncoderSnapshot getTicksSnapshot() {
    EncoderSnapshot s;
    noInterrupts();
    s.avl = ticks_AVL; s.avr = ticks_AVR;
    s.arl = ticks_ARL; s.arr = ticks_ARR;
    interrupts();
    return s;
}

// -----------------------------------------------------------------------------
//  Odométrie Mecanum
//
//  Pour des roues Mecanum à 45° :
//    dx    = (AVL + AVR + ARL + ARR) / 4
//    dy    = (-AVL + AVR + ARL - ARR) / 4
//    dtheta= (-AVL + AVR - ARL + ARR) / (4 * (Lx+Ly))
//
//  Lx = demi-empattement longitudinal (WHEEL_BASE_M / 2)
//  Ly = demi-voie transversale        (WHEEL_TRACK_M / 2)
//
//  Les ticks sont convertis en mm avant calcul.
// -----------------------------------------------------------------------------
OdomDelta computeOdomDelta() {
    noInterrupts();
    long avl = ticks_AVL - odom_prev_AVL; odom_prev_AVL = ticks_AVL;
    long avr = ticks_AVR - odom_prev_AVR; odom_prev_AVR = ticks_AVR;
    long arl = ticks_ARL - odom_prev_ARL; odom_prev_ARL = ticks_ARL;
    long arr = ticks_ARR - odom_prev_ARR; odom_prev_ARR = ticks_ARR;
    interrupts();

    float d_avl = avl * MM_PAR_TICK;
    float d_avr = avr * MM_PAR_TICK;
    float d_arl = arl * MM_PAR_TICK;
    float d_arr = arr * MM_PAR_TICK;

    float Lxy = (WHEEL_BASE_M + WHEEL_TRACK_M) * 500.0f; // (m → mm) * 2 roues

    OdomDelta delta;
    delta.dx_mm  = -( d_avl + d_avr + d_arl + d_arr) / 4.0f;
    delta.dy_mm  = -(-d_avl + d_avr + d_arl - d_arr) / 4.0f;
    delta.dtheta = -(-d_avl + d_avr - d_arl + d_arr) / (4.0f * Lxy);

    return delta;
}