#include <Arduino.h>
#include "FastAccelStepper.h"
#include <VL53L0X.h>
#include <math.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <WiFi.h>

float CORRECTION_DERIVE = 1;

// --- CONFIGURATION SEQUENCE ---
int ZONE_DEBUT = 1;
int ZONE_FIN = 5;
int zone_actuelle = 1;
bool mode_sequence = false;

// Configuration Pins
#define I2C_SDA 2
#define I2C_SCL 3
#define TCAADDR 0x70
#define BUTTON_PIN 6

// Bouton
bool buttonState = false;

// LED de diagnostic
// const int LED_PIN = 2;

const char *ssid = "NOM_DU_WIFI";
const char *password = "MDP";

// Constantes Robot
const float k_LIN = 14.5513; // Pas par mm
const float k_ROT = 12.381;  // Pas par degré
int SYMETRIE = 1;

const int DISTANCE_ARRET = 10;
const int DISTANCE_LOIN = 160;
const int PAS_ROTATION = 15;

// Vitesse Max
const float VITESSE_MAX = 1000;
const float ACCELERATION = 2000;

// Pins Moteurs
const int enablePin = 4;
const int mode1Pin = 28;
const int mode2Pin = 8;

// Moteur Gauche (L)
const int stepPin_L = 26;
const int dirPin_L = 25;

// Moteur Droit (R)
const int stepPin_R = 23;
const int dirPin_R = 24;

const int stby = 5;

// --- FastAccelStepper ---
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperL = NULL;
FastAccelStepper *stepperR = NULL;

rgb_lcd lcd;

// Capteurs
VL53L0X sensorG;
VL53L0X sensorD;

// Variables Globales
float robot_x = 0;
float robot_y = 0;
float robot_a = 0;
float cible_x = 0;
float cible_y = 0;

// Timers
unsigned long timer_capteur = 0;
unsigned long timer_pause = 0;
unsigned long timer_debut_mouvement = 0;

// Timer pour le démarrage automatique
unsigned long timer_depart_auto = 0;
bool robot_a_demarre = false;

// Evitement
int compteur_obs = 0;
int sens_esquive = 1;
float angle_scan_total = 0;

// Machine à états
enum Etat
{
  ATTENTE,
  CALCUL_ROT,
  ROTATION,
  PAUSE_ROT,
  CALCUL_AV,
  AVANCE,
  PAUSE_SEQUENCE,
  OBS_STOP,
  OBS_ANALYSE,
  OBS_TOURNE,
  OBS_AVANCE_SORTIE
};
Etat etat = ATTENTE;

// Gestion du TCA9548A pour multiplexage I2C
void tcaselect(uint8_t i)
{
  if (i > 3)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(5);
}

// setup
void setup()
{
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  delay(1500);
  Serial.println("\nROBOT DEMARRAGE...");

  pinMode(stby, OUTPUT);
  digitalWrite(stby, LOW);

  digitalWrite(enablePin, LOW);
  pinMode(enablePin, OUTPUT_OPEN_DRAIN);

  pinMode(mode1Pin, OUTPUT);
  pinMode(mode2Pin, OUTPUT);

  pinMode(dirPin_L, OUTPUT);
  pinMode(dirPin_R, OUTPUT);
  pinMode(stepPin_L, OUTPUT);
  pinMode(stepPin_R, OUTPUT);

  digitalWrite(dirPin_L, HIGH);
  digitalWrite(dirPin_R, HIGH);
  digitalWrite(stepPin_L, HIGH);
  digitalWrite(stepPin_R, HIGH);

  digitalWrite(mode1Pin, HIGH);
  digitalWrite(mode2Pin, HIGH);

  delay(5000); // Attente de 5 secondes

  digitalWrite(stby, HIGH);
  delay(100);
  digitalWrite(enablePin, HIGH);

  // --- Initialisation FastAccelStepper ---
  engine.init();

  // Moteur Gauche : direction inversée (remplace setPinsInverted(true, false, false))
  stepperL = engine.stepperConnectToPin(stepPin_L);
  if (stepperL)
  {
    stepperL->setDirectionPin(dirPin_L, true); // true = inversé
    stepperL->setAcceleration((uint32_t)ACCELERATION);
    stepperL->setSpeedInHz((uint32_t)VITESSE_MAX);
  }

  // Moteur Droit : direction normale
  stepperR = engine.stepperConnectToPin(stepPin_R);
  if (stepperR)
  {
    stepperR->setDirectionPin(dirPin_R, false); // false = normal
    stepperR->setAcceleration((uint32_t)ACCELERATION);
    stepperR->setSpeedInHz((uint32_t)VITESSE_MAX);
  }

  digitalWrite(stby, HIGH);

  // Initialisation I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // Initialisation LCD (Canal 3)
  tcaselect(3);
  lcd.begin(16, 2);
  Serial.println("LCD OK");
  lcd.setRGB(255, 255, 0);

  // Initialisation Capteurs ToF
  tcaselect(0);
  sensorG.init();
  sensorG.setTimeout(500);
  sensorG.startContinuous();

  tcaselect(1);
  sensorD.init();
  sensorD.setTimeout(500);
  sensorD.startContinuous();

  if (SYMETRIE == 1)
  {
    robot_x = 52.5;
    robot_y = 53.75;
    robot_a = 0;
  }
  else
  {
    robot_x = 2947.5;
    robot_y = 53.75;
    robot_a = 180;
  }

  Serial.println("PRET ! ATTENTE 5 SECONDES AVANT SEQUENCE 1-5...");
  timer_depart_auto = millis();
}

// OUTILS

int lire_capteur(int id)
{
  int d;
  if (id == 1)
  {
    tcaselect(0);
    d = sensorG.readRangeContinuousMillimeters();
  }
  else
  {
    tcaselect(1);
    d = sensorD.readRangeContinuousMillimeters();
  }
  if (sensorG.timeoutOccurred() || sensorD.timeoutOccurred())
    return 8888;
  if (d == 0 || d > 8000 || d > 65000 || d < 20)
    return 8888;
  return 8888;
}

void reset_moteurs()
{
  // FastAccelStepper : setCurrentPosition() remet le compteur à zéro
  stepperL->setCurrentPosition(0);
  stepperR->setCurrentPosition(0);
}

void maj_pos()
{
  // FastAccelStepper : getCurrentPosition() remplace currentPosition()
  float mmL = stepperL->getCurrentPosition() / k_LIN;
  float mmR = stepperR->getCurrentPosition() / (k_LIN * CORRECTION_DERIVE);
  float mm = (mmL + mmR) / 2.0;

  robot_x += mm * cos(robot_a * PI / 180.0);
  robot_y += mm * sin(robot_a * PI / 180.0);
}

void aller_vers(float x, float y)
{
  if (SYMETRIE == -1)
    x = 3000.0 - x;
  cible_x = x;
  cible_y = y;
  etat = CALCUL_ROT;
}

void aller_a_la_zone(int z)
{
  Serial.print("Navigation vers Zone: ");
  Serial.println(z);
  switch (z)
  {
  case 0:
    aller_vers(52.5, 53.75);
    break;
  case 1:
    aller_vers(100, 1200);
    break;
  case 2:
    aller_vers(700, 1900);
    break;
  case 3:
    aller_vers(800, 1200);
    break;
  case 4:
    aller_vers(1250, 550);
    break;
  case 5:
    aller_vers(1500, 1200);
    break;
  case 6:
    aller_vers(1500, 1900);
    break;
  case 7:
    aller_vers(1750, 550);
    break;
  case 8:
    aller_vers(2200, 1200);
    break;
  case 9:
    aller_vers(2300, 1900);
    break;
  case 10:
    aller_vers(2900, 1200);
    break;
  }
}

// LOGIQUE DE MOUVEMENT

void gestion_etats()
{
  // FastAccelStepper est piloté par timer hardware : plus besoin de stepperL.run() ici

  switch (etat)
  {
  case ATTENTE:
    break;

  case CALCUL_ROT:
  {
    float dx = cible_x - robot_x;
    float dy = cible_y - robot_y;
    float dist = sqrt(dx * dx + dy * dy);

    if (dist < 20)
    {
      etat = AVANCE;
      return;
    }

    float angle_vise = atan2(dx, dy) * 180.0 / PI;
    float rot = angle_vise - robot_a;
    while (rot > 180)
      rot -= 360;
    while (rot <= -180)
      rot += 360;

    reset_moteurs();
    long pas = (long)(rot * k_ROT);

    // Avec FastAccelStepper : setSpeedInHz() avant move()
    stepperL->setSpeedInHz((uint32_t)VITESSE_MAX);
    stepperR->setSpeedInHz((uint32_t)VITESSE_MAX);

    stepperL->move(-pas);
    stepperR->move(pas);

    robot_a += rot;
    while (robot_a > 180)
      robot_a -= 360;
    while (robot_a <= -180)
      robot_a += 360;

    etat = ROTATION;
  }
  break;

  case ROTATION:
    // distanceToGo() == 0 remplacé par !isRunning()
    if (!stepperL->isRunning() && !stepperR->isRunning())
    {
      timer_pause = millis();
      etat = PAUSE_ROT;
    }
    break;

  case PAUSE_ROT:
    if (millis() - timer_pause > 100)
      etat = CALCUL_AV;
    break;

  case CALCUL_AV:
  {
    float dx = cible_x - robot_x;
    float dy = cible_y - robot_y;
    float dist = sqrt(dx * dx + dy * dy);

    reset_moteurs();

    long pas_G = (long)(dist * k_LIN);
    long pas_D = (long)(dist * k_LIN * CORRECTION_DERIVE);

    stepperL->setSpeedInHz((uint32_t)VITESSE_MAX);
    stepperR->setSpeedInHz((uint32_t)(VITESSE_MAX * CORRECTION_DERIVE));

    stepperL->move(pas_G);
    stepperR->move(pas_D);

    timer_debut_mouvement = millis();
    timer_capteur = millis();
    compteur_obs = 0;
    etat = AVANCE;
  }
  break;

  case AVANCE:
    if (!stepperL->isRunning() && !stepperR->isRunning())
    {
      robot_x = cible_x;
      robot_y = cible_y;

      if (mode_sequence)
      {
        Serial.print("Arrive zone ");
        Serial.println(zone_actuelle);
        timer_pause = millis();
        etat = PAUSE_SEQUENCE;
      }
      else
      {
        Serial.println("Fini.");
        etat = ATTENTE;
      }
    }
    else if (millis() - timer_capteur > 50)
    {
      timer_capteur = millis();
      if (millis() - timer_debut_mouvement > 500)
      {
        int dG = lire_capteur(1);
        int dD = lire_capteur(2);
        if ((dG != 8888 && dG < DISTANCE_ARRET) || (dD != 8888 && dD < DISTANCE_ARRET))
        {
          compteur_obs++;
          if (compteur_obs >= 2)
          {
            etat = OBS_STOP;
            Serial.println("OBSTACLE !");
          }
        }
        else
        {
          compteur_obs = 0;
        }
      }
    }
    break;

  // --- LOGIQUE DE SEQUENCE ---
  case PAUSE_SEQUENCE:
    if (millis() - timer_pause > 1000)
    {
      zone_actuelle++;
      if (zone_actuelle <= ZONE_FIN)
      {
        aller_a_la_zone(zone_actuelle);
      }
      else
      {
        Serial.println("SEQUENCE TERMINEE (1 a 5).");
        mode_sequence = false;
        etat = ATTENTE;
      }
    }
    break;

  // OBSTACLE
  case OBS_STOP:
    // stopMove() décélère proprement ; on attend l'arrêt complet
    stepperL->stopMove();
    stepperR->stopMove();
    while (stepperL->isRunning() || stepperR->isRunning())
      delay(1);
    maj_pos();
    reset_moteurs();
    etat = OBS_ANALYSE;
    break;

  case OBS_ANALYSE:
  {
    int dG = lire_capteur(1);
    int dD = lire_capteur(2);
    sens_esquive = (dG < dD) ? 1 : -1;
    etat = OBS_TOURNE;
  }
  break;

  case OBS_TOURNE:
    if (!stepperL->isRunning() && !stepperR->isRunning())
    {
      int dG = lire_capteur(1);
      int dD = lire_capteur(2);
      if (dG > DISTANCE_LOIN && dD > DISTANCE_LOIN)
      {
        reset_moteurs();
        float dist_esquive = (DISTANCE_ARRET + 50.0) * 1.5;
        long pas = (long)(dist_esquive * k_LIN);
        stepperL->setSpeedInHz((uint32_t)VITESSE_MAX);
        stepperR->setSpeedInHz((uint32_t)VITESSE_MAX);
        stepperL->move(pas);
        stepperR->move(pas);
        timer_debut_mouvement = millis();
        etat = OBS_AVANCE_SORTIE;
      }
      else
      {
        reset_moteurs();
        long p = (long)(PAS_ROTATION * k_ROT * sens_esquive);
        stepperL->setSpeedInHz((uint32_t)VITESSE_MAX);
        stepperR->setSpeedInHz((uint32_t)VITESSE_MAX);
        stepperL->move(-p);
        stepperR->move(p);
        angle_scan_total += PAS_ROTATION;
        robot_a += PAS_ROTATION * sens_esquive;
      }
    }
    break;

  case OBS_AVANCE_SORTIE:
    if (!stepperL->isRunning() && !stepperR->isRunning())
    {
      maj_pos();
      etat = CALCUL_ROT;
    }
    break;
  }
}

void gestion_entrees()
{
  // --- DEMARRAGE AUTOMATIQUE DE LA SEQUENCE ---
  if (!robot_a_demarre)
  {
    if (millis() - timer_depart_auto > 5000)
    {
      Serial.println("GO ! DEBUT SEQUENCE ZONES 1 A 5");
      mode_sequence = true;
      zone_actuelle = ZONE_DEBUT;
      aller_a_la_zone(zone_actuelle);
      robot_a_demarre = true;
    }
  }

  // Debug via Série
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'S' || c == 's')
    {
      // stopMove() remplace stop() (décélération douce)
      stepperL->stopMove();
      stepperR->stopMove();
      reset_moteurs();
      etat = ATTENTE;
      mode_sequence = false;
      Serial.println("STOP URGENCE");
    }
  }
}

void loop()
{
  gestion_entrees();
  gestion_etats();
}