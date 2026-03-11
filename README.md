# TP 6 : Asservissement de vitesse et IHM Bluetooth sur ESP32

## Description du TP
Ce dépôt contient le code final du TP d'informatique embarquée. L'objectif était de piloter le moteur d'un pupitre trieur depuis une tablette Android via Bluetooth, tout en gérant un asservissement de vitesse en boucle fermée.

## Matériel utilisé
- Carte ESP32 (programmation sous VS Code / PlatformIO)
- Moteur à courant continu + Pont en H (commande PWM)
- Codeur incrémental pour la mesure de vitesse/position
- Capteur infrarouge CNY70 (détection de balle)
- Écran LCD Grove I2C
- Tablette avec l'application "Bluetooth Electronics"

## Fonctionnalités implémentées

### 1. Régulation de vitesse (FreeRTOS)
- Mise en place d'un correcteur PI (Proportionnel-Intégral).
- La régulation tourne dans une tâche `vTaskAsserv` avec FreeRTOS pour garantir une période d'échantillonnage stricte de 100 ms.
- Ajout d'une sécurité "Anti-Windup" pour limiter le terme intégral.
- Ajout d'une rampe d'accélération (filtre passe-bas sur la consigne) et d'une coupure moteur de 150ms lors de l'inversion de sens pour éviter les à-coups mécaniques.

### 2. Communication Bluetooth (Profil SPP)
- **Réception (Tablette -> ESP32) :** L'ESP32 lit des caractères pour activer/couper le moteur (`1`/`0`), changer de sens (`F`/`R`), changer la couleur du LCD (`C`, `M`, `Y`), et lit un entier pour la consigne de vitesse (`V` + valeur du slider).
- **Émission (ESP32 -> Tablette) :** Envoi de la position du bras (`*P...*`) et de l'état du capteur CNY70 (`*D...*`). 
- Le seuil du capteur CNY70 a été calibré expérimentalement à 2000 (environ 780 à vide, 4095 avec la balle) pour allumer le voyant de l'application en jaune ou l'éteindre en gris.

---
**Auteur :** Adam Helal - BUT GEII 2A
