# Xplore Rover — Mémoire Projet

Ce fichier est mis à jour automatiquement pour garder le contexte entre sessions.

---

## Architecture générale

| | Repo | Machine | Package |
|--|------|---------|---------|
| **Rover** | `xplore_rasberry` | Raspberry Pi 4 — Docker ROS2 Humble | `rover_xplore` |
| **PC** | `xplore_pub` | PC/VM Ubuntu 24 — Docker ROS2 Humble | `rover_xplore_pub` |

Communication : ROS2 DDS via réseau (`--net=host` des deux côtés).

---

## Composants hardware

| Composant | Qté | Notes |
|-----------|-----|-------|
| Moteurs roues JGA25-370 (DC + encodeur) | 4 | encodeur 6 fils, quadrature A/B |
| Moteurs bras robotisé | 5 | type à confirmer avec team élec |
| Moteur benne (conteneur) | 1 | tilt pour vider |
| Caméra Pi | 1 | MJPEG bridge host → Docker |
| Capteurs ultrasoniques (US) | 5 (prévu) | 3 avant + 2 côtés à 45° |
| IMU | 1 | fusion avec encodeurs via EKF |
| Raspberry Pi 4 | 1 | compute principal, ROS2 |
| Arduino Nano | TBD | rôle à confirmer avec team élec |

> **À confirmer avec team élec :** type moteurs bras, type drivers, nb Arduino Nano, communication RPi↔Arduino (USB série ou I2C).

---

## Défis du rover

| Défi | Mode | Description |
|------|------|-------------|
| Course FPV | `race` | L'opérateur pilote en regardant le flux caméra |
| Ramassage d'objets | `arm` | Bras robotique commandé à distance, dépôt à la base |
| Collaboratif | TBD | Avec d'autres rovers — specs pas encore connues |
| Navigation autonome | `autonomous` | Le rover navigue seul sans intervention humaine |

---

## Architecture des modes — controller_node (PC)

Un seul node `controller_node` gère tout le flow côté PC :

```
Démarrage
  ├── [A] Autonome   → publie /rover/mode "autonomous"
  └── [T] Commandé
        ├── [R] Race → publie /rover/mode "race"
        └── [B] Bras → publie /rover/mode "arm"
  [M] à tout moment → retour menu → publie /rover/mode "idle"
```

### Topics
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/rover/mode` | `std_msgs/String` | PC → RPi | Mode actif : autonomous / race / arm / idle |
| `/rover/cmd_vel` | `geometry_msgs/Twist` | PC → RPi | Commandes de déplacement |

### Modes reconnus par mode_manager_node (RPi)
| Mode | Action RPi |
|------|-----------|
| `autonomous` | démarrer navigation autonome |
| `race` | activer réception cmd_vel |
| `arm` | activer arm_node |
| `idle` | stopper tous les actionneurs |

---

## Téléopération — profil Race

### Touches clavier
| Touche | Action |
|--------|--------|
| W | avancer |
| S | reculer |
| A | pivot gauche |
| D | pivot droite |
| Q | arc avant gauche |
| E | arc avant droite |
| Y | arc arrière gauche |
| X | arc arrière droite |
| 8 / 9 / 0 | vitesse lente (0.33) / moyenne (0.66) / pleine (1.0) |
| Espace | stop moteurs immédiat |
| M | retour menu |
| P | quitter |

### Mécanisme hold-to-move
- Key repeat OS (~30ms) + timeout par axe (200ms)
- Le multiplicateur de vitesse est appliqué côté PC avant publication du Twist
- La RPi reçoit les valeurs déjà scalées — pas besoin de connaître le niveau de vitesse

### Sécurité réseau
- `teleop_receiver_node` stoppe les moteurs si aucune commande reçue depuis 500ms

---

## Téléopération — profil Bras

- Contrôle clavier à implémenter
- Bouton "position base" → bras revient au-dessus du conteneur automatiquement
- Bouton "décharger" → tilt benne pour vider

---

## Contrôle moteurs roues — PID

Architecture prévue par roue :

```
cmd_vel Twist
    ↓
cinématique différentielle → vitesse cible roue gauche + droite
    ↓
PID par roue (cible vs RPM mesuré par encodeur)
    ↓
PWM → driver moteur → moteur
    ↓
encodeur → RPM réel → feedback PID + odométrie
```

---

## Localisation — EKF

Fusion encodeurs + IMU via `robot_localization` (package ROS2) :

```
encoder_node  →  /odom       ─┐
imu_node      →  /imu/data   ─┤→ EKF (robot_localization) → /odometry/filtered
```

Pas besoin de coder le Kalman manuellement.

---

## Stratégie navigation autonome

### Défi autonome — spécifications
- **Map** : 8m × 5m, terrain plat, entouré de barrières (barreaux) + scotch au sol
- **Objectif** : aller jusqu'à une structure avec ArUco tag, revenir au point de départ, sans toucher les obstacles
- **Obstacles** : 10–50cm de hauteur, placement aléatoire et secret
- **ArUco** : sur un poteau, visible depuis la position de départ (à confirmer)
- **Bonus** : ramasser une bouteille d'eau près de l'ArUco (position donnée en x,y,z relatif à l'ArUco) et la ramener
- **Restarts** : 2 autorisés mais remet les points à zéro

### Workflow autonome
```
[1] DETECT_ARUCO
    → Caméra détecte l'ArUco depuis le départ
    → Calcule bearing + distance estimée

[2] NAVIGATE_TO_ARUCO
    → Navigation guidée par caméra vers l'ArUco
    → US évitent les obstacles en temps réel
    → Odométrie track la position

[3] ARRIVE_ARUCO
    → Recalage position absolue grâce à l'ArUco (corrige drift odométrie)
    → (Bonus) calcule position bouteille via offset x,y,z fourni

    (Bonus) NAVIGATE_TO_BOTTLE → PICK_UP

[4] NAVIGATE_TO_START
    → Retour vers (0,0) via odométrie
    → US évitent obstacles

[5] ARRIVÉE BASE
    → (Bonus) déposer bouteille → tilt benne
    → Mission terminée
```

### Rôle des capteurs
| Besoin | Capteur |
|--------|---------|
| Naviguer vers ArUco | Caméra (ArUco detection) |
| Éviter obstacles | 3 US avant + 2 US côtés (45°) |
| Tracker position | Encodeurs + IMU + EKF |
| Recalage position absolue | ArUco |
| Détecter bord map | Odométrie (limites software 8×5m) — US non fiables sur barreaux |
| Détecter scotch au sol | À confirmer (couleur ?) |
| Position bouteille | ArUco + offset x,y,z donné |

### Nœuds autonomes à créer (RPi)
| Nœud | Rôle |
|------|------|
| `aruco_node` | détecte ArUco, publie pose relative |
| `ultrasonic_node` | publie distances 5 capteurs |
| `encoder_node` | odométrie roues |
| `imu_node` | données IMU |
| `obstacle_avoidance_node` | réactif, tourne en permanence |
| `autonomous_node` | machine d'état des 5 phases |

### Points à confirmer
- Visibilité ArUco depuis le départ (change toute la stratégie si non)
- Couleur du scotch au sol
- Nombre de bouteilles à ramasser

---

## Problème caméra — état actuel

**Cause racine** : libcamera Python bindings compilés pour Python 3.12 (Ubuntu 24.04 host),
incompatibles avec Docker ROS2 Humble (Python 3.10). Pas de solution connue dans la communauté.

**Solution choisie : MJPEG bridge**
```
Host RPi
  └── camera_server.py  (picamera2, PYTHONPATH fixé, Python 3.12)
        ↓ HTTP MJPEG — 127.0.0.1:8080 (loopback, ~0ms latence réseau)
Docker ROS2 Humble
  └── camera_node.py  (cv2.VideoCapture HTTP → publie /camera/image_raw)
        ↓ ROS2 DDS
PC Docker ROS2 Humble
  └── video_viewer_node.py  (déjà compatible, aucune modif nécessaire)
```

**Lancer camera_server.py sur le host RPi :**
```bash
PYTHONPATH=/usr/lib/aarch64-linux-gnu/python3.12/site-packages \
python3.12 camera_server.py
```

---

## État du code

### Repo Rover (`xplore_rasberry`)
| Fichier | État |
|---------|------|
| `rover_xplore/rover_xplore/camera_node.py` | À mettre à jour → lire MJPEG bridge |
| `rover_xplore/rover_xplore/mode_manager_node.py` | Prêt — gère autonomous/race/arm/idle |
| `rover_xplore/rover_xplore/teleop_receiver_node.py` | Prêt — reçoit Twist, timeout sécurité 500ms |
| `camera_server.py` (racine) | À créer — picamera2 HTTP server |
| `rover_xplore/rover_xplore/autonomous_node.py` | À créer |
| `rover_xplore/rover_xplore/aruco_node.py` | À créer |
| `rover_xplore/rover_xplore/obstacle_avoidance_node.py` | À créer |
| `rover_xplore/rover_xplore/ultrasonic_node.py` | À créer |
| `rover_xplore/rover_xplore/imu_node.py` | À créer |
| `rover_xplore/rover_xplore/encoder_node.py` | À créer |
| `rover_xplore/rover_xplore/arm_node.py` | À créer |

### Repo PC (`xplore_pub`)
| Fichier | État |
|---------|------|
| `rover_xplore_pub/rover_xplore_pub/controller_node.py` | Prêt — menu + race + bras |
| `rover_xplore_pub/rover_xplore_pub/video_viewer_node.py` | Prêt |
| `rover_xplore_pub/rover_xplore_pub/teleop_node.py` | Remplacé par controller_node |
| `rover_xplore_pub/rover_xplore_pub/mode_selector_node.py` | Remplacé par controller_node |

---

## Infrastructure

- Docker : `docker_humble_desktop/` — ROS 2 Humble Desktop (les deux repos)
- CI : GitHub Actions `docker_ci.yml` — build sur push vers `master` uniquement
- Branches actives : `feat/mode-selection` (les deux repos)

---

## Conventions

- Package rover : `rover_xplore` uniquement
- Package PC : `rover_xplore_pub` uniquement
- Commentaires en français, variables/fonctions en anglais
- Commits sans co-author

---

*Dernière mise à jour : 2026-04-11*
