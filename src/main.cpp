#include <Arduino.h>
#include "BluetoothSerial.h"
#include "rgb_lcd.h"
#include <ESP32Encoder.h>

/* Vérification matérielle de l'activation de la pile Bluetooth */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Le module Bluetooth n'est pas active dans la configuration systeme !
#endif

// --- Instanciation des objets ---
BluetoothSerial SerialBT; // Interface UART virtuelle via Bluetooth Classic
rgb_lcd lcd;              // Interface I2C pour l'afficheur
ESP32Encoder encoder;     // Gestion matérielle des interruptions du codeur

// --- Définition du mapping matériel (Pinout) ---
const int PIN_ON_MOTEUR  = 25; // Autorisation de puissance du pont en H
const int PIN_DIR_MOTEUR = 26; // Contrôle du sens de rotation
const int PIN_PWM_MOTEUR = 27; // Signal de commande de hachage (Vitesse)
const int PIN_ENC_A      = 19; // Canal A du codeur incrémental
const int PIN_ENC_B      = 23; // Canal B du codeur incrémental
const int PIN_IR0        = 36; // Entrée analogique du capteur infrarouge CNY70

// --- Paramétrage du périphérique PWM matériel (ledc) ---
const int PWM_CHANNEL    = 0;
const int PWM_FREQ       = 25000; // Fréquence de découpage inaudible (25 kHz)
const int PWM_RESOLUTION = 11;    // Résolution 11 bits (dynamique de commande de 0 à 2047)

// --- Variables globales partagées (Volatile pour l'accès inter-tâches RTOS) ---
volatile int consigneVitesse = 0; // Vitesse cible (en impulsions/100ms)
volatile bool sensAvance = true;  // État logique du sens de rotation
volatile bool moteurActif = false;// Autorisation de marche globale
volatile bool resetPID = false;   // Drapeau de synchronisation pour l'inversion de sens

// --- Variables de gestion du temps (Télémétrie) ---
unsigned long dernierEnvoiBT = 0;
const int PERIODE_ENVOI_BT = 200; // Période d'échantillonnage pour l'envoi à l'IHM (200 ms)

/**
 * Tâche FreeRTOS : Régulateur Proportionnel-Intégral (PI)
 * Exécutée de manière asynchrone avec une période stricte pour garantir l'intégration mathématique.
 */
void vTaskAsserv(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t periode = pdMS_TO_TICKS(100); // Période d'échantillonnage Te = 100ms
    
    long ancienCodeur = 0;
    float erreurSomme = 0.0;     // Mémoire de l'action intégrale
    float consigneFiltree = 0.0; // Variable d'état pour le filtre de la consigne
    
    // Synthèse du correcteur (valeurs empiriques)
    const float Kp = 10.0; // Gain Proportionnel (réactivité)
    const float Ki = 15.0; // Gain Intégral (annulation de l'erreur statique)

    while (1) {
        // Attente bloquante jusqu'à la prochaine période (garantit le temps réel)
        vTaskDelayUntil(&xLastWakeTime, periode); 

        // --- GESTION DES SÉCURITÉS MÉCANIQUES (Inversion de sens) ---
        if (resetPID) {
            ledcWrite(PWM_CHANNEL, 0); // Coupure du PWM
            digitalWrite(PIN_ON_MOTEUR, LOW);
            digitalWrite(PIN_DIR_MOTEUR, sensAvance ? LOW : HIGH); 
            
            erreurSomme = 0.0;       // Purge de la mémoire intégrale pour éviter un à-coup
            consigneFiltree = 0.0;   // Réinitialisation de la rampe
            ancienCodeur = encoder.getCount(); 
            
            resetPID = false;
            vTaskDelay(pdMS_TO_TICKS(150)); // Temps mort pour laisser l'inertie mécanique retomber
            continue;
        }

        // 1. Dérivation numérique de la position pour obtenir la vitesse mesurée
        long nouveauCodeur = encoder.getCount();
        long vitesseReelle = abs(nouveauCodeur - ancienCodeur);
        ancienCodeur = nouveauCodeur;

        // 2. Traitement de l'arrêt d'urgence ou de l'état de repos
        if (!moteurActif || consigneVitesse == 0) {
            ledcWrite(PWM_CHANNEL, 0);
            digitalWrite(PIN_ON_MOTEUR, LOW);
            erreurSomme = 0; 
            consigneFiltree = 0.0; 
            continue;
        }

        // 3. Filtrage de la consigne (Filtre passe-bas du 1er ordre / Rampe)
        // Adoucit les échelons de vitesse brutaux envoyés par le slider de l'IHM
        consigneFiltree = (0.7 * consigneFiltree) + (0.3 * consigneVitesse);

        // 4. Calcul de l'erreur d'asservissement
        float erreur = consigneFiltree - vitesseReelle;
        erreurSomme += erreur;
        
        // 5. Action Intégrale avec sécurité Anti-Windup
        // Empêche la saturation de l'intégrateur en cas de blocage de l'arbre moteur
        if (erreurSomme > 150) erreurSomme = 150;
        if (erreurSomme < -150) erreurSomme = -150;

        // 6. Loi de commande PI
        float commande = (erreur * Kp) + (erreurSomme * Ki);
        
        // Saturation matérielle pour le rapport cyclique du PWM (0 à 2047 max)
        if (commande < 0) commande = 0;
        if (commande > 2000) commande = 2000;

        // 7. Application aux actionneurs (Pont en H)
        digitalWrite(PIN_DIR_MOTEUR, sensAvance ? LOW : HIGH); 
        digitalWrite(PIN_ON_MOTEUR, HIGH);
        ledcWrite(PWM_CHANNEL, (int)commande);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialisation du bus I2C et de l'écran LCD Grove
    Wire1.setPins(15, 5);
    lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
    lcd.setRGB(0, 0, 255); 
    lcd.print("Init. BT...");

    // Configuration des sorties de puissance
    pinMode(PIN_ON_MOTEUR, OUTPUT);
    pinMode(PIN_DIR_MOTEUR, OUTPUT);
    digitalWrite(PIN_ON_MOTEUR, LOW); // Sécurité : moteur coupé au démarrage
    
    // Attachement du générateur PWM matériel à la broche
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM_MOTEUR, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0); 

    // Configuration des entrées capteurs
    pinMode(PIN_IR0, INPUT); // Broche 36 : Input Only
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachHalfQuad(PIN_ENC_A, PIN_ENC_B);
    encoder.setCount(0); // Référencement de l'origine de position

    // Initialisation du serveur Bluetooth avec profil SPP
    if (!SerialBT.begin("ESP32_Pupitre_AdamHelal")) {
        Serial.println("Erreur fatale : Initialisation Bluetooth échouée");
        while (1); // Blocage de sécurité
    }

    // Réduction du timeout série à 100ms.
    // Permet à parseInt() de ne pas bloquer la boucle principale trop longtemps
    // tout en laissant le temps de recevoir une trame complète.
    SerialBT.setTimeout(100);

    // Déploiement de la tâche de régulation sur le planificateur de l'ESP32
    // Priorité 1, allocation de 4096 octets de pile
    xTaskCreate(vTaskAsserv, "RegulVitesse", 4096, NULL, 1, NULL);

    // Mise à jour de l'IHM locale
    lcd.setRGB(0, 255, 0); 
    lcd.setCursor(0, 0);
    lcd.print("IHM Connectee!  ");
}

void loop() {
    // --- 1. DÉCODAGE DES TRAMES ENTRANTES (Smartphone -> ESP32) ---
    if (SerialBT.available()) {
        char commande = SerialBT.read();
        
        switch(commande) {
            case '0': moteurActif = false; break; 
            case '1': moteurActif = true; break;  
            case 'F': 
                if (!sensAvance) { 
                    sensAvance = true; 
                    resetPID = true; // Demande d'inversion sécurisée
                }
                break;   
            case 'R': 
                if (sensAvance) { 
                    sensAvance = false; 
                    resetPID = true; // Demande d'inversion sécurisée
                }
                break;  
            case 'V': {
                // Extraction de la valeur du slider et mise à l'échelle
                // Conversion de la plage IHM [0-2000] vers la consigne d'impulsions [0-100]
                long valeurSlider = SerialBT.parseInt(); 
                consigneVitesse = map(valeurSlider, 0, 2000, 0, 100);
                break;
            }
            // Changement de la couleur du rétroéclairage LCD via I2C
            case 'C': lcd.setRGB(0, 255, 255); break; 
            case 'M': lcd.setRGB(255, 0, 255); break; 
            case 'Y': lcd.setRGB(255, 255, 0); break; 
        }
    }

    // --- 2. MULTIPLEXAGE DE LA TÉLÉMÉTRIE (ESP32 -> Smartphone) ---
    // Utilisation de millis() pour un cadencement non-bloquant
    if (millis() - dernierEnvoiBT > PERIODE_ENVOI_BT) {
        dernierEnvoiBT = millis();

        // Acquisition des données capteurs
        long positionBras = encoder.getCount();
        int valeurIR = analogRead(PIN_IR0); 
        
        // Binarisation du signal IR avec un seuil calibré expérimentalement (2000)
        // Repos ambiant mesuré : ~780 | Balle détectée mesurée : 4095
        bool presenceBalle = (valeurIR > 2000); 

        // Protocole d'envoi vers l'application "Bluetooth Electronics"
        // Trame Canal 'P' : Mise à jour de la position numérique
        SerialBT.print("*P"); 
        SerialBT.print(positionBras);
        SerialBT.print("*");

        // Trame Canal 'D' : Modification de la couleur du voyant dynamique
        // Formatage RGB requis par l'application cliente
        if (presenceBalle) {
            SerialBT.print("*DR255G255B0*"); // Présence -> Voyant Jaune
        } else {
            SerialBT.print("*DR50G50B50*");  // Repos -> Voyant Gris
        }
    }
    
    // Temporisation de la boucle principale
    delay(10); 
}