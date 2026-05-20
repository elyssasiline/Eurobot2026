/*
  Sequence pince simplifiee - Teensy 4.1
  Execution unique au demarrage
*/

#include <Tic.h>
#include <Servo.h>

// Prototypes
void allerA(int32_t cible);
void fermerPince();
void ouvrirPince();
void maintenirPince();
void stopPinceLOW();
float lireCourant();
void tournerServo(int angle);

// ============================================================
// >>>>>>>>>> VARIABLES A AJUSTER <<<<<<<<
// ============================================================

const int32_t DESCENTE_PAS              = 2350;
const int32_t HAUTEUR_INTERMEDIAIRE_PAS = 800;
const float   I_SERRAGE_A               = 0.7f;
const int     ANGLE_ROTATION            = 180;

const uint8_t  PWM_FERMETURE        = 240;
const uint8_t  PWM_OUVERTURE        = 200;
const uint8_t  PWM_MAINTIEN         = 80;    // a ajuster selon le ressort
const uint16_t SURSERRAGE_MS        = 0;   // a ajuster selon le ressort
const float    I_SECURITE_A         = 1.5f;
const uint16_t T_IGNORE_PIC_MS      = 300;
const uint16_t TIMEOUT_FERMETURE_MS = 5000;
const uint16_t T_OUVERTURE_MS       = 1000;

// ============================================================
// CONFIGURATION HARDWARE
// ============================================================

#define TIC_SERIAL Serial1
TicSerial tic(TIC_SERIAL);

const int   motorPin1 = 2;
const int   motorPin2 = 3;
const int   PIN_SHUNT = A0;
const float R_SHUNT   = 1.0f;
const float VREF      = 3.3f;
const int   ADC_MAX   = 4095;
const int   N_MOY     = 16;

Servo         monServo;
const uint8_t PIN_SERVO    = 9;
const int     US_MIN        = 500;
const int     US_MAX        = 2500;
const int     ANGLE_INITIAL = 0;

bool pince_serree = false;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("==========================================");
  Serial.println("  SEQUENCE PINCE");
  Serial.println("==========================================");
  Serial.print("Descente        : "); Serial.print(DESCENTE_PAS);              Serial.println(" pas");
  Serial.print("Hauteur interm. : "); Serial.print(HAUTEUR_INTERMEDIAIRE_PAS); Serial.println(" pas");
  Serial.print("Courant serrage : "); Serial.print(I_SERRAGE_A * 1000);        Serial.println(" mA");
  Serial.print("Angle rotation  : "); Serial.print(ANGLE_ROTATION);            Serial.println(" deg");

  TIC_SERIAL.begin(9600);
  delay(20);
  tic.exitSafeStart();
  tic.energize();
  tic.haltAndSetPosition(0);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(PIN_SHUNT, INPUT);
  analogReadResolution(12);
  stopPinceLOW();

  monServo.attach(PIN_SERVO, US_MIN, US_MAX);
  tournerServo(ANGLE_INITIAL);
  delay(500);

  Serial.println("\nDebut sequence dans 2s...\n");
  delay(2000);

  // [1] Descente
  Serial.println("[1] DESCENTE");
  allerA(DESCENTE_PAS);
  delay(300);

  // [2] Serrage
  Serial.println("\n[2] SERRAGE PINCE");
  fermerPince();
  delay(300);

  // [3] Montee intermediaire — maintien actif pendant le deplacement
  Serial.println("\n[3] MONTEE intermediaire");
  allerA(HAUTEUR_INTERMEDIAIRE_PAS);
  delay(300);

  // [4] Rotation servo — maintien actif
  Serial.println("\n[4] ROTATION servo");
  tournerServo(ANGLE_ROTATION);
  delay(800);
  tic.haltAndSetPosition(HAUTEUR_INTERMEDIAIRE_PAS);
  Serial.println("    Position recalibree");

  // [5] Redescente — maintien actif
  Serial.println("\n[5] REDESCENTE");
  allerA(DESCENTE_PAS);
  delay(300);

  // [6] Ouverture
  Serial.println("\n[6] OUVERTURE PINCE");
  ouvrirPince();
  delay(300);

  // [7] Remontee
  Serial.println("\n[7] REMONTEE position haute");
  allerA(0);

  // [8] Retour servo
  Serial.println("\n[8] RETOUR servo position initiale");
  tournerServo(ANGLE_INITIAL);

  Serial.println("\n=== SEQUENCE TERMINEE ===");
}

void loop() {}

// ============================================================
// STEPPER
// ============================================================
void allerA(int32_t cible) {
  tic.exitSafeStart();
  tic.setTargetPosition(cible);

  unsigned long t0 = millis();
  while (tic.getCurrentPosition() != cible) {
    tic.resetCommandTimeout();
    if (pince_serree) maintenirPince();
    delay(10);
    if (millis() - t0 > 15000) {
      Serial.println("    !! TIMEOUT stepper");
      return;
    }
  }
  Serial.print("    Cible "); Serial.print(cible); Serial.println(" atteinte");
}

// ============================================================
// PINCE - FERMETURE
// ============================================================
void fermerPince() {
  unsigned long t0 = millis();

  analogWrite(motorPin1, PWM_FERMETURE);
  digitalWrite(motorPin2, LOW);

  while (true) {
    float i = lireCourant();
    unsigned long t = millis() - t0;

    Serial.print("    t="); Serial.print(t);
    Serial.print(" ms   I="); Serial.print(i * 1000, 1); Serial.println(" mA");

    if (i >= I_SECURITE_A) {
      pince_serree = true;
      Serial.println("    !! SECURITE — passage maintien");
      return;
    }

    if (t > T_IGNORE_PIC_MS && i >= I_SERRAGE_A) {
      Serial.println("    Serrage detecte — sur-serrage...");
      delay(SURSERRAGE_MS);
      pince_serree = true;
      Serial.print("    >>> SERREE — maintien PWM (I=");
      Serial.print(lireCourant() * 1000, 1); Serial.println(" mA)");
      return;
    }

    if (t > TIMEOUT_FERMETURE_MS) {
      pince_serree = true;
      Serial.println("    !! TIMEOUT — passage maintien");
      return;
    }

    delay(20);
  }
}

// ============================================================
// PINCE - OUVERTURE
// ============================================================
void ouvrirPince() {
  pince_serree = false;
  digitalWrite(motorPin1, LOW);
  analogWrite(motorPin2, PWM_OUVERTURE);
  delay(T_OUVERTURE_MS);
  stopPinceLOW();
  Serial.println("    Pince relachee");
}

// ============================================================
// PINCE - MAINTIEN PWM
// ============================================================
void maintenirPince() {
  analogWrite(motorPin1, PWM_MAINTIEN);
  digitalWrite(motorPin2, LOW);
}

// ============================================================
// PINCE - STOP LOW
// ============================================================
void stopPinceLOW() {
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

// ============================================================
// MESURE COURANT
// ============================================================
float lireCourant() {
  long somme = 0;
  for (int i = 0; i < N_MOY; i++) {
    somme += analogRead(PIN_SHUNT);
  }
  float v = (somme / (float)N_MOY) * VREF / ADC_MAX;
  return v / R_SHUNT;
}

// ============================================================
// SERVO
// ============================================================
void tournerServo(int angle) {
  int us = map(angle, 0, 180, US_MIN, US_MAX);
  monServo.writeMicroseconds(us);
}