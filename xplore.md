# Xplore Rover — Mémoire Projet

Ce fichier est mis à jour automatiquement pour garder le contexte entre sessions.

---

## Architecture générale

| | Repo | Chemin local | Machine | Package |
|--|------|-------------|---------|---------|
| **Rover** | `xplore_rasberry` | `/Users/hadyazzi/Desktop/Personal/xplore/xplore_rasberry` | Raspberry Pi 4 — Ubuntu 22.04, ROS2 Humble natif | `rover_xplore` |
| **PC** | `xplore_pub` | `/Users/hadyazzi/Desktop/Personal/xplore/xplore_pub` | VM/Mac — ROS2 Humble | `rover_xplore_pub` |

Communication : ROS2 DDS via réseau local (même subnet, ROS_DOMAIN_ID identique).

---

## Composants hardware

| Composant | Qté | Notes |
|-----------|-----|-------|
| Moteurs roues JGA25-370 (DC + encodeur) | 4 | motor1..4 dans le protocole série |
| Servos | 4 | servo_1..4 dans le protocole série |
| Stepper | 1 | stepper dans le protocole série |
| Moteurs bras robotisé | 5 | type à confirmer avec team élec |
| Moteur benne (conteneur) | 1 | tilt pour vider |
| Caméra Pi | 1 | libcamera (picamera2) natif Ubuntu 22 |
| Capteurs ultrasoniques (US) | 5 | distance_1..5 dans le protocole série |
| IMU (accel + gyro) | 1 | accel_x/y/z + gyro_x/y/z dans le protocole série |
| Raspberry Pi 4 | 1 | compute principal, ROS2 |
| Micro-contrôleur (équipe élec) | 1 | communication série binaire avec le Pi |

---

## Communication RPi ↔ Micro (protocole série binaire)

Protocole défini avec l'équipe élec — structs binaires little-endian via UART/USB.

### `data_from_rasberry` — Pi → Micro (18 octets)
```c
struct data_from_rasberry {
    uint16_t servo_1, servo_2, servo_3, servo_4;   // 4 × uint16
    int16_t  motor1_speed, motor2_speed, motor3_speed, motor4_speed;  // 4 × int16
    int16_t  stepper;                               // 1 × int16
};  // total : 18 octets, format Python : "<4H5h"
```

### `data_for_rasberry` — Micro → Pi (30 octets)
```c
struct data_for_rasberry {
    uint16_t accel_x, accel_y, accel_z;            // IMU accéléromètre
    uint16_t gyro_x, gyro_y, gyro_z;               // IMU gyroscope
    int16_t  motor1_speed, motor2_speed, motor3_speed, motor4_speed;  // encodeurs roues
    uint16_t distance_1, distance_2, distance_3, distance_4, distance_5;  // 5 US
};  // total : 30 octets, format Python : "<6H4h5H"
```

> **À confirmer avec team élec :** disposition des 4 moteurs (gauche/droite ou avant/arrière), port série (`/dev/ttyUSB0`), baudrate.

---

## Défis du rover

| Défi | Mode | Description |
|------|------|-------------|
| Course FPV | `race` | L'opérateur pilote en regardant le flux caméra |
| Ramassage d'objets | `arm` | Bras robotique commandé à distance, dépôt à la base |
| Collaboratif | TBD | Avec d'autres rovers — specs pas encore connues |
| Navigation autonome | `autonomous` | Le rover navigue seul sans intervention humaine |

---

## Architecture des modes — rover_gui (PC, GUI PySide6)

`rover_gui` (interface graphique) remplace `controller_node` + `video_viewer_node`.
Le menu est cliquable à la souris ; les commandes en mode race restent au clavier.

```
Menu principal (souris)
  ├── [AUTONOME]      → publie /rover/mode "autonomous"
  ├── [TÉLÉOPÉRATION]
  │     ├── [RACE]    → publie /rover/mode "race"  + affiche flux caméra
  │     └── [BRAS]    → publie /rover/mode "arm"
  └── [QUITTER]
  Retour menu (souris ou touche M en race) → publie /rover/mode "idle"
```

### Palette XPlore (rover_gui)
| Rôle | Hex |
|------|-----|
| Fond principal (bleu marine logo) | `#0C1427` |
| Surface (cards) | `#121E36` |
| Surface élevée (hover) | `#182440` |
| Border | `#1E2E4A` |
| Primary (rouge identité) | `#E53935` |
| Accent (cyan) | `#00AEEF` |
| Succès (ArUco détecté) | `#5AE8B5` |
| Texte | `#F3F6F9` |
| Texte muted | `#6B7A95` |

### Animations rover_gui
- **PulsingDot** : point qui pulse en opacité dans les badges de mode (race/auto)
- **ArUco status** : pulse vert quand détecté, statique gris sinon
- **Footer dot** : couleur dynamique selon mode (idle gris / race rouge / auto vert / arm cyan)
- Pas de fade-in entre pages (conflit QPainter avec PulsingDot enfants)

`controller_node` (terminal) reste disponible en fallback mais n'est plus le mode par défaut.

### Topics
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/rover/mode` | `std_msgs/String` | PC → RPi | Mode actif : autonomous / race / arm / idle |
| `/rover/cmd_vel` | `geometry_msgs/Twist` | PC → RPi | Commandes de déplacement |
| `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | RPi → RPi | Flux vidéo qualité max — pour aruco_node, reste local |
| `/camera/image_compressed` | `sensor_msgs/CompressedImage` | RPi → PC | Flux vidéo JPEG, QoS BEST_EFFORT |
| `/aruco_detected` | `std_msgs/Float32MultiArray` | RPi → RPi | `[found, id, cx, cy, area]` — marker le plus proche |
| `/rover/nav_goal` | `std_msgs/Int32MultiArray` | PC → RPi | `[start_row, start_col, target_row, target_col]` — envoyé au Start |
| `/rover/grid_state` | `std_msgs/Int32MultiArray` | RPi → PC | Grille 12×8 aplatie — états cases (0=UNDISCOVERED..4=BORDER) |
| `/rover/grid_pos` | `std_msgs/Int32MultiArray` | RPi → PC | `[row, col]` — position courante du rover sur la grille |
| `/ultrasonic` | `std_msgs/Float32MultiArray` | RPi → RPi | `[d1, d2, d3, d4, d5]` en cm — publié par serial_bridge_node |
| `/imu/raw` | `sensor_msgs/Imu` | RPi → RPi | accel + gyro bruts — publié par serial_bridge_node |
| `/wheel_encoders` | `std_msgs/Int32MultiArray` | RPi → RPi | `[m1, m2, m3, m4]` vitesses encodeurs — publié par serial_bridge_node |

### Modes reconnus par mode_manager_node (RPi)
| Mode | Action RPi |
|------|-----------|
| `autonomous` | démarrer navigation autonome |
| `race` | activer réception cmd_vel |
| `arm` | activer arm_node |
| `idle` | stopper tous les actionneurs |

---

## Téléopération — profil Race

### Touches clavier (dans `rover_gui` mode race)
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

> Les indicateurs de touches s'allument en rouge en temps réel dans la GUI quand pressés.

### Mécanisme hold-to-move
- Key repeat OS (~30ms) + timeout par axe (200ms)
- Le multiplicateur de vitesse est appliqué côté PC avant publication du Twist
- La RPi reçoit les valeurs déjà scalées — pas besoin de connaître le niveau de vitesse

### Sécurité réseau
- `teleop_receiver_node` stoppe les moteurs si aucune commande reçue depuis 500ms

---

## Téléopération — profil Bras

### Touches clavier (dans `rover_gui` mode bras)
| Touche | Action |
|--------|--------|
| O / K | UP / DOWN (axe Z) |
| J / L | CLOSE / OPEN (pinces) |
| I / P | FLIP / UNFLIP (axe Y) |
| ↑ / ↓ | Benne monte / descend |
| 6 / 7 / 8 / 9 / 0 | Niveaux de vitesse |
| Espace | Stop tout (bras + benne) |
| M | Retour menu |

- Bouton "POSITION DE VIDAGE" → tilt benne pour vider
- Topic publié : `/rover/arm_cmd` (`Float32MultiArray` — [z, y, pince, speed, dump, bin_dir])

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

### Grille de navigation

- Grille interne **10 lignes × 6 colonnes** (cellules 80×80cm) — 8m/0.8=10, 5m/0.8=6
- Grille totale **12×8** avec rangée `BORDER` tout autour — élimine toute vérification out-of-bounds
- Départ : bas-droite — Cible : haut-gauche (ArUco)
- Chaque cellule : état (`BORDER|UNDISCOVERED|FREE|OBSTACLE|AMBUSH`) + priorité (Chebyshev inversé vers cible)
- Chaque cellule divisée en **4 mini-cases 40×40cm** (TL/TR/BL/BR) pour la détection fine des obstacles
- `AMBUSH` = cul-de-sac vécu : le rover y est entré et toutes les voisines étaient bloquées → évité à l'avenir

### Algorithme de navigation (implémenté dans MapWidget + autonomous_node)

**BFS global vers meilleure UNDISCOVERED** à chaque tick :

1. Depuis la position courante, BFS à travers les cases FREE pour trouver la case UNDISCOVERED avec la meilleure priorité (Chebyshev min vers cible) atteignable
2. Si la meilleure est un voisin direct → move classique (1 step), case → FREE, push position au stack
3. Si la meilleure est accessible via des cases FREE → **transit silencieux** : le rover repasse sur les cases déjà visitées (sans changer leur état) jusqu'à la UNDISCOVERED cible
4. Si aucune UNDISCOVERED atteignable nulle part → marquer `AMBUSH` + backtrack via stack
5. Déplacement réel : IMU pour cap → encodeurs pour distance → case mise à jour

**Règles diagonale** : une diagonale (dr,dc) n'est autorisée que si les deux cases "coin" adjacentes `(cr, cc+dc)` et `(cr+dr, cc)` sont libres (non BORDER/OBSTACLE/AMBUSH).

**Failles connues :**
- Pas de condition d'arrêt si la cible est inatteignable (rover boucle en AMBUSH indéfiniment)
- Pas de pondération distance de transit vs gain de priorité (long transit pour gain marginal)
- Stack peut contenir des doublons (transits multiples depuis la même case)
- Transit non invalidé si un obstacle est placé sur la destination pendant le transit

### Architecture autonome — séparation PC / Rover

**PC (rover_gui)** :
- L'utilisateur place départ + arrivée sur la grille → appuie Start
- Publie `/rover/nav_goal` → rover reçoit les coordonnées
- Reçoit `/rover/grid_state` + `/rover/grid_pos` → affiche la grille en temps réel
- Le PC est **uniquement un affichage**, il ne décide rien

**Rover (autonomous_node)** :
- Reçoit le goal, tourne l'algo BFS en local
- US → cases OBSTACLE dans la grille interne
- Encodeurs + IMU → position dans la grille (quelle case)
- Publie la grille + sa position pour le PC

### Workflow autonome
```
[1] DETECT_ARUCO   → caméra identifie la cible, confirme position dans grille
[2] NAVIGATE_GRID  → algo BFS (voir section algorithme ci-dessus)
[3] ARRIVE_ARUCO   → recalage position absolue, (Bonus) ramassage bouteille
[4] NAVIGATE_RETURN → retour via grille déjà remplie + US double-check
[5] ARRIVEE_BASE   → (Bonus) déposer bouteille → tilt benne
```

### Rôle des capteurs
| Besoin | Capteur |
|--------|---------|
| Identifier la cible | Caméra (ArUco detection) |
| Détecter obstacles | 3 US avant (−45°/0°/+45°) + US gauche + US droite |
| Tracker position | Encodeurs + IMU + EKF (`robot_localization`) |
| Recalage position absolue | ArUco |
| Détecter bord map | Cellules BORDER (software) — US non fiables sur barreaux |
| Position bouteille | ArUco + offset x,y,z donné |

### Nœuds à créer (RPi)
| Nœud | Rôle |
|------|------|
| `serial_bridge_node` | pont binaire série ↔ ROS2 : reçoit struct 30 octets (IMU+encodeurs+US), publie `/ultrasonic` `/imu/raw` `/wheel_encoders` ; reçoit commandes moteurs/servos/stepper, envoie struct 18 octets |
| `autonomous_node` | grille 12×8 + algo BFS + machine d'état — s'abonne à `/rover/nav_goal`, publie `/rover/grid_state` + `/rover/grid_pos` |

### Points à confirmer
- Visibilité ArUco depuis le départ (change toute la stratégie si non)
- Couleur du scotch au sol
- Nombre de bouteilles à ramasser
- Modèle exact des US (angle de détection)

---

## Caméra — architecture cible

### Problème confirmé (session 2026-04-16)

La Pi Camera CSI ne fonctionne **pas** dans Docker sur Ubuntu 22.04 :
- `python3-libcamera` absent des repos Ubuntu 22.04
- `libcamera-tools` dans les repos Ubuntu = version 2020 (`0~git20200629`) → segfault
- V4L2 direct (video14+) → timeout, ce sont des devices ISP, pas capture
- GStreamer `libcamerasrc` → dépend aussi de libcamera
- `camera_auto_detect=1` est bien dans `/boot/firmware/config.txt` → hardware OK

Le problème n'était pas Ubuntu 24 → c'était Docker qui isole libcamera.

### Solution retenue : bridge natif via DDS

`camera_node` tourne **nativement** sur le Pi (hors Docker). Le container Docker utilise déjà `--net=host` (`run.sh`) → les topics ROS2 publiés en natif sont visibles dans Docker via DDS automatiquement.

```
RPi Ubuntu 22 — NATIF (hors Docker)
  └── camera_node.py
        libcamera / picamera2  ←  capture Pi Camera CSI
        ↓ /camera/image_raw
        ↓ ROS2 DDS (--net=host → visible dans Docker)

RPi Ubuntu 22 — Docker
  └── tous les autres nodes (mode_manager, motor_controller, etc.)

VM/Mac — Docker
  └── video_viewer_node.py  ←  affiche le flux FPV
```

### État déploiement Pi (2026-04-25)

| Composant | État |
|-----------|------|
| ROS2 Humble (`rclpy`) | ✅ Installé |
| cv2 (python3-opencv) | ✅ Installé |
| `source /opt/ros/humble/setup.bash` dans `.bashrc` | ✅ Fait |
| libcamera (compilé depuis sources RPi) | ✅ Installé dans `/usr/local/lib` |
| picamera2 | ✅ Installé (pip, patch pykms appliqué) |
| colcon build rover_xplore | ✅ Fait — branche `feat/mode-selection` |
| camera_node fonctionnel | ✅ Publie `/camera/image_compressed` (JPEG) @ 640x480 30 FPS |
| VM voit le topic `/camera/image_compressed` | ✅ DDS fonctionne |

### Notes importantes post-install

- `PYTHONPATH=/usr/local/lib/python3/dist-packages` dans `~/.bashrc` — nécessaire pour que libcamera soit trouvé
- picamera2 pip patché : `/home/pi-hady/.local/lib/python3.10/site-packages/picamera2/previews/__init__.py` — import `DrmPreview` rendu optionnel (pykms absent sur Ubuntu 22)
- Workspace ROS2 : `~/dev_ws/src/xplore_rasberry` (branche `feat/mode-selection`)

### Lancer la caméra (commande unique)

```bash
source ~/.bashrc && source ~/dev_ws/install/setup.bash && ros2 run rover_xplore camera_node
```

### Fix freeze viewer (2026-04-25)

Deux causes identifiées et corrigées :
1. **Bande passante** : BGR non compressé = 27 MB/s sur WiFi → saturait DDS UDP
2. **QoS RELIABLE** : DDS retransmettait les paquets perdus → lag s'accumulait

Fix appliqué dans les deux repos :
- `camera_node` : publie `CompressedImage` JPEG (qualité 80) sur `/camera/image_compressed` → ~1-2 MB/s
- `video_viewer_node` : subscribe en `CompressedImage` + `BEST_EFFORT` QoS (frames perdues ignorées, pas de retransmission)
- `cv_bridge` retiré des deux nodes (non nécessaire avec JPEG natif cv2)

À surveiller si freeze persiste : throttling thermique Pi, multicast DDS filtré par routeur.

### Dépendances RPi (natif)
```bash
sudo apt install ros-humble-ros-base ros-humble-cv-bridge python3-colcon-common-extensions python3-opencv libcap-dev
pip3 install picamera2  # après libcamera compilé
```

---

## État du code

### Repo Rover (`xplore_rasberry`)
| Fichier | État |
|---------|------|
| `rover_xplore/rover_xplore/camera_node.py` | Prêt — picamera2 (libcamera) + cv2, double pub `/camera/image_raw` (Image bgr8) + `/camera/image_compressed` (JPEG), publie uniquement en mode `race` ou `autonomous`, fallback V4L2 |
| `scripts/start_camera.sh` | Prêt — lance camera_node natif sur le Pi hors Docker |
| `rover_xplore/rover_xplore/mode_manager_node.py` | Prêt — gère autonomous/race/arm/idle |
| `rover_xplore/rover_xplore/motor_controller_node.py` | Prêt — cinématique diff, publie `/rover/motor_cmd` Int32MultiArray [FR,FL,BR,BL] (-255..255), gating race/autonomous, timeout 500ms. Serial retiré. |
| `rover_xplore/rover_xplore/aruco_node.py` | Prêt — cv2.aruco sur `/camera/image_raw`, publie `/aruco_detected` (marker le plus proche), s'active uniquement en mode `autonomous`, log sur transition détecté/perdu, dict configurable |
| `rover_xplore/rover_xplore/teleop_receiver_node.py` | Supprimé — remplacé par motor_controller_node |
| `rover_xplore/rover_xplore/serial_bridge_node.py` | Prêt — pont binaire 10 Hz, envoi struct 18B (4×uint16 servo + 4×int16 motor + int16 stepper), réception struct 30B (IMU+enc+US), publie /ultrasonic /imu/raw /wheel_encoders, force zeros en idle |
| `rover_xplore/rover_xplore/arm_node.py` | Prêt — reçoit /rover/arm_cmd Float32[z,y,pince,speed,dump,bin_dir], mappe stepper/servo1..4, publie /rover/arm_serial_cmd Int32[s1,s2,s3,s4,stepper], gating mode arm |
| `rover_xplore/rover_xplore/autonomous_node.py` | À créer — grille BFS + machine à états, s'abonne `/rover/nav_goal`, publie `/rover/grid_state` + `/rover/grid_pos` |

### Repo PC (`xplore_pub`)
| Fichier | État |
|---------|------|
| `rover_xplore_pub/rover_xplore_pub/rover_gui.py` | **Prêt — GUI PySide6 unifiée.** Menu → Téléop (Race / Bras) ou Autonome. **MapWidget** : grille 12×8 interactive, placement manuel départ/cible/obstacles (bouton OBSTACLE sticky, toggle), navigation simulée avec algo BFS global (transit via cases FREE vers meilleure UNDISCOVERED). **Architecture cible** : Start → publie `/rover/nav_goal`, affiche grille reçue depuis `/rover/grid_state` + position depuis `/rover/grid_pos` (à implémenter). **Animations** : GlowCard, GlowButton, PulsingDot, footer dynamique. |
| `rover_xplore_pub/rover_xplore_pub/xplore_logo.jpg` | Logo EPFL XPlore intégré dans la GUI (format paysage, fond #0C1427) |
| `rover_xplore_pub/rover_xplore_pub/controller_node.py` | Fallback terminal — toujours dispo (`ros2 run rover_xplore_pub controller_node`) |
| `rover_xplore_pub/rover_xplore_pub/video_viewer_node.py` | Fallback terminal — toujours dispo |
| `rover_xplore_pub/rover_xplore_pub/teleop_node.py` | Remplacé par rover_gui |
| `rover_xplore_pub/rover_xplore_pub/mode_selector_node.py` | Remplacé par rover_gui |

### Lancer rover_gui (à faire à chaque démarrage Docker)

```bash
# 1. Update les listes de packages
sudo apt update

# 2. Installer la dépendance Qt (perdue à chaque redémarrage Docker)
sudo apt install libxcb-cursor0 -y

# 3. Installer PySide6 (perdu à chaque redémarrage Docker)
pip3 install PySide6

# 4. Build (uniquement si le code a changé)
cd ~/dev_ws && colcon build --packages-select rover_xplore_pub

# 5. Sourcer + lancer
source install/setup.bash && ros2 run rover_xplore_pub rover_gui
```

> **Redémarrages suivants (sans changement de code) :** steps 1 → 3 + step 5 seulement.

---

## Infrastructure

- RPi : Ubuntu 22.04, ROS 2 Humble **natif** (plus de Docker)
- VM/Mac : ROS 2 Humble (Docker ou natif selon env)
- CI : GitHub Actions `docker_ci.yml` — build sur push vers `master` uniquement
- Branches actives : `feat/mode-selection` (les deux repos)

---

## Conventions

- Package rover : `rover_xplore` uniquement
- Package PC : `rover_xplore_pub` uniquement
- Commentaires en français, variables/fonctions en anglais
- **Commits sans co-author** — ne jamais ajouter `Co-Authored-By: Claude` dans les messages de commit

---

*Dernière mise à jour : 2026-05-09 (session 14 — serial_bridge_node + arm_node créés, motor_controller_node refactorisé : serial texte retiré, 4 moteurs [-255..255], publie /rover/motor_cmd)*
