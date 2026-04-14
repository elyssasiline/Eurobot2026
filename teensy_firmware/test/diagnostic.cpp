// diag.cpp
#include <Arduino.h>
#include "robot_config.h"
#include "motor_driver.h"
#include "encoders.h"

#define PWM_TEST          2000
#define SYNCHRO_SEUIL_PCT 10.0f
#define SEQ_DUREE_MS      1200

// Remplacer les #define de pins en tête de diagnostic.cpp par :
#define M_ARL_INA      9
#define M_ARL_INB      10
#define M_ARL_PWM      8
#define M_ARL_EN_DIAG  32

#define M_ARR_INA      11
#define M_ARR_INB      27
#define M_ARR_PWM      12
#define M_ARR_EN_DIAG  33

#define M_AVL_INA      23
#define M_AVL_INB      24
#define M_AVL_PWM      22
#define M_AVL_EN_DIAG  26

#define M_AVR_INA      25
#define M_AVR_INB      29
#define M_AVR_PWM      28
#define M_AVR_EN_DIAG  31

#define ENC_ARL_A      4
#define ENC_ARL_B      5
#define ENC_ARR_A      6
#define ENC_ARR_B      7
#define ENC_AVL_A      2
#define ENC_AVL_B      3
#define ENC_AVR_A      0
#define ENC_AVR_B      1

// Inversions correctes (validées physiquement) :
bool inv_AVL = true;
bool inv_AVR = false;
bool inv_ARL = true;
bool inv_ARR = false;

void diagInit() {
    // rien à init pour l'instant
}

static void printInversions() {
    Serial.println("[INVERSIONS]");
    Serial.print("  AVL: "); Serial.println(inv_AVL ? "INVERSÉ" : "direct");
    Serial.print("  AVR: "); Serial.println(inv_AVR ? "INVERSÉ" : "direct");
    Serial.print("  ARL: "); Serial.println(inv_ARL ? "INVERSÉ" : "direct");
    Serial.print("  ARR: "); Serial.println(inv_ARR ? "INVERSÉ" : "direct");
}

static void testMoteurSeul(const char* nom, void (*mFn)(int), volatile long* ticks) {
    Serial.print("\n[TEST] "); Serial.print(nom);
    Serial.println(" — 2s, observe la roue.");
    noInterrupts(); *ticks = 0; interrupts();
    mFn(PWM_TEST);
    delay(2000);
    stopAll();
    delay(200);
    noInterrupts(); long t = *ticks; interrupts();
    Serial.print("  Ticks : "); Serial.print(t);
    Serial.println(t > 0 ? " (sens +)" : " (sens -)");
}

static void testSynchronisation(int pwm) {
    Serial.println("\n[SYNCHRO] Avance 3s...");
    resetTicks();
    avancer(pwm);
    delay(3000);
    stopAll();
    delay(200);
    noInterrupts();
    long avl=ticks_AVL, avr=ticks_AVR, arl=ticks_ARL, arr=ticks_ARR;
    interrupts();
    float absVals[4] = { (float)abs(avl),(float)abs(avr),(float)abs(arl),(float)abs(arr) };
    long  rawVals[4] = { avl, avr, arl, arr };
    bool  inv[4]     = { inv_AVL, inv_AVR, inv_ARL, inv_ARR };
    const char* noms[4] = {"AVL","AVR","ARL","ARR"};
    float moy = (absVals[0]+absVals[1]+absVals[2]+absVals[3]) / 4.0f;
    if (moy == 0) { Serial.println("  ERREUR : aucun tick."); return; }
    for (int i=0; i<4; i++) {
        Serial.print("  "); Serial.print(noms[i]); Serial.print(" : ");
        Serial.print(rawVals[i]); Serial.print(" ticks  (");
        Serial.print(absVals[i] * MM_PAR_TICK, 1); Serial.print(" mm)  ");
        float ecart = abs(absVals[i] - moy) / moy * 100.0f;
        bool signeOk = inv[i] ? (rawVals[i] < 0) : (rawVals[i] > 0);
        if (!signeOk) Serial.print("!! SIGNE INATTENDU !!");
        else if (ecart > SYNCHRO_SEUIL_PCT) { Serial.print(ecart,1); Serial.print("% !! ECART !!"); }
        else { Serial.print(ecart,1); Serial.print("% OK"); }
        Serial.println();
    }
}

static void testRampePWM() {
    Serial.println("\n[RAMPE] Montée/descente PWM sur AVL.");
    resetTicks();
    const int steps=15, step_ms=300;
    auto mesure = [&](int pwm) {
        noInterrupts(); long t0=ticks_AVL; interrupts();
        moteurAVL(pwm);
        delay(step_ms);
        noInterrupts(); long t1=ticks_AVL; interrupts();
        long delta=abs(t1-t0);
        float v=(delta*MM_PAR_TICK)/(step_ms/1000.0f);
        Serial.print("  PWM "); Serial.print(pwm); Serial.print(" → ");
        if (delta==0) Serial.println("--- (zone morte)");
        else { Serial.print(v,0); Serial.println(" mm/s"); }
    };
    Serial.println("  >> Montée :");
    for (int i=0; i<=steps; i++) mesure((PWM_MAX*i)/steps);
    Serial.println("  >> Descente :");
    for (int i=steps; i>=0; i--) mesure((PWM_MAX*i)/steps);
    stopAll();
}

static void testSequenceAuto(int pwm) {
    Serial.println("\n[SEQ] Lance la séquence (tape 's' pour interrompre).");
    struct { void (*fn)(int); const char* nom; } seq[] = {
        {avancer,"Avancer"},{reculer,"Reculer"},{pivotGauche,"Pivot G"},
        {pivotDroite,"Pivot D"},{straffeGauche,"Straffe G"},{straffeDroite,"Straffe D"},
        {diagAvGauche,"Diag AV-G"},{diagAvDroite,"Diag AV-D"},
        {diagArGauche,"Diag AR-G"},{diagArDroite,"Diag AR-D"},
    };
    for (auto& s : seq) {
        if (Serial.available()) {
            String in=Serial.readStringUntil('\n'); in.trim();
            if (in=="s") { stopAll(); Serial.println("[SEQ] Interrompue."); return; }
        }
        Serial.print("  → "); Serial.println(s.nom);
        resetTicks();
        s.fn(pwm);
        delay(SEQ_DUREE_MS);
        stopAll();
        noInterrupts();
        long avl=ticks_AVL,avr=ticks_AVR,arl=ticks_ARL,arr=ticks_ARR;
        interrupts();
        Serial.print("  ENC: AVL:"); Serial.print(avl);
        Serial.print(" AVR:"); Serial.print(avr);
        Serial.print(" ARL:"); Serial.print(arl);
        Serial.print(" ARR:"); Serial.println(arr);
        delay(400);
    }
    Serial.println("[SEQ] Terminée.");
}

void diagParse(String cmd) {
    cmd.trim();
    if (cmd.length()==0) return;

    if (cmd=="diag") { printDriverStatus(); return; }
    if (cmd=="didc") {
        struct { void (*fn)(int); int pin; const char* nom; } motors[] = {
            {moteurAVL,M_AVL_EN_DIAG,"AVL"},{moteurAVR,M_AVR_EN_DIAG,"AVR"},
            {moteurARL,M_ARL_EN_DIAG,"ARL"},{moteurARR,M_ARR_EN_DIAG,"ARR"},
        };
        for (auto& m : motors) {
            stopAll(); delay(200);
            m.fn(PWM_TEST); delay(500);
            int e=digitalRead(m.pin); stopAll();
            Serial.print("  "); Serial.print(m.nom);
            Serial.println(e==HIGH?" : OK":" : !! DÉFAUT !!");
            delay(300);
        }
        return;
    }

    if (cmd.length()>=2) {
        String p2=cmd.substring(0,2);
        String p3=(cmd.length()>=3)?cmd.substring(0,3):"";
        int val=(cmd.indexOf(' ')>0)
                ?constrain(cmd.substring(cmd.indexOf(' ')+1).toInt(),0,PWM_MAX)
                :PWM_TEST;
        if (p2=="sl"){straffeGauche(val);return;}
        if (p2=="sr"){straffeDroite(val);return;}
        if (p3=="dfl"){diagAvGauche(val);return;}
        if (p3=="dfr"){diagAvDroite(val);return;}
        if (p3=="dbl"){diagArGauche(val);return;}
        if (p3=="dbr"){diagArDroite(val);return;}
        if (p2=="pl"){pivotGauche(val);return;}
        if (p2=="pr"){pivotDroite(val);return;}
        if (p2=="sy"){testSynchronisation(val);return;}
        if (p2=="sq"){testSequenceAuto(val);return;}
        if (p2=="vn"){testRampePWM();return;}
    }

    if (cmd.startsWith("inv")&&cmd.length()>=4) {
        char n=cmd.charAt(3);
        if (n=='1'){inv_AVL=!inv_AVL;Serial.print("AVL ");Serial.println(inv_AVL?"INVERSÉ":"direct");}
        if (n=='2'){inv_AVR=!inv_AVR;Serial.print("AVR ");Serial.println(inv_AVR?"INVERSÉ":"direct");}
        if (n=='3'){inv_ARL=!inv_ARL;Serial.print("ARL ");Serial.println(inv_ARL?"INVERSÉ":"direct");}
        if (n=='4'){inv_ARR=!inv_ARR;Serial.print("ARR ");Serial.println(inv_ARR?"INVERSÉ":"direct");}
        printInversions(); return;
    }

    char action=cmd.charAt(0);
    int val=(cmd.indexOf(' ')>0)
            ?constrain(cmd.substring(cmd.indexOf(' ')+1).toInt(),0,PWM_MAX)
            :PWM_TEST;

    switch(action) {
        case 'f': Serial.print("[AVANT] PWM=");Serial.println(val);avancer(val); break;
        case 'b': Serial.print("[RECUL] PWM=");Serial.println(val);reculer(val); break;
        case 's': stopAll(); break;
        case 't':
            if (cmd.length()>1) {
                char n=cmd.charAt(1);
                if (n=='1') testMoteurSeul("AV-G (m1)",moteurAVL,&ticks_AVL);
                if (n=='2') testMoteurSeul("AV-D (m2)",moteurAVR,&ticks_AVR);
                if (n=='3') testMoteurSeul("AR-G (m3)",moteurARL,&ticks_ARL);
                if (n=='4') testMoteurSeul("AR-D (m4)",moteurARR,&ticks_ARR);
            }
            break;
        case 'e': {
            EncoderSnapshot s=getTicksSnapshot();
            Serial.println("[ENC]");
            Serial.print("  AVL: ");Serial.print(s.avl);Serial.print(" | ");Serial.print(vit_AVL,1);Serial.println(" mm/s");
            Serial.print("  AVR: ");Serial.print(s.avr);Serial.print(" | ");Serial.print(vit_AVR,1);Serial.println(" mm/s");
            Serial.print("  ARL: ");Serial.print(s.arl);Serial.print(" | ");Serial.print(vit_ARL,1);Serial.println(" mm/s");
            Serial.print("  ARR: ");Serial.print(s.arr);Serial.print(" | ");Serial.print(vit_ARR,1);Serial.println(" mm/s");
            break;
        }
        case 'z': resetTicks();Serial.println("[ZERO]"); break;
        case 'i': printInversions(); break;
        case 'h':
        default:
            Serial.println("\n--- AIDE ---");
            Serial.println("t1..t4  : tester moteur seul");
            Serial.println("inv1..4 : inverser un moteur");
            Serial.println("f/b <pwm> : avant/arrière");
            Serial.println("pl/pr   : pivot G/D");
            Serial.println("sl/sr   : straffe G/D");
            Serial.println("dfl/dfr/dbl/dbr : diagonales");
            Serial.println("sy/sq/vn : synchro / séquence / rampe");
            Serial.println("diag / didc : diagnostic drivers");
            Serial.println("s:stop  e:encodeurs  z:reset  i:inversions");
    }
}