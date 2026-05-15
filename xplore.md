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
- Commande calculée directement depuis `active_keys` à chaque tick du timer (100ms)
- Tant qu'une touche est maintenue → sa valeur est publiée. Relâché → 0.0 immédiat
- Le multiplicateur de vitesse est appliqué côté PC avant publication du Twist
- La RPi reçoit les valeurs déjà scalées — pas besoin de connaître le niveau de vitesse

### Mapping touches → Twist (MOVEMENT_KEYS)
| Touche | linear | angular | Effet |
|--------|--------|---------|-------|
| W | +1.0 | 0 | Avance |
| S | -1.0 | 0 | Recule |
| A | 0 | +8.0 | Rotation gauche sur place (×8 pour compenser WHEEL_BASE/2=0.125 → ±100% PWM) |
| D | 0 | -8.0 | Rotation droite sur place |
| Q | +1.0 | +1.0 | Arc avant-gauche |
| E | +1.0 | -1.0 | Arc avant-droite |
| Y | -1.0 | -1.0 | Arc arrière-gauche (angular négatif = roue droite plus rapide en arrière) |
| X | -1.0 | +1.0 | Arc arrière-droite |

> **Note** : le clamp `[-1, 1]` est appliqué uniquement sur `linear`, pas sur `angular` (pour permettre ±8.0 sur les rotations). `motor_controller_node` clamp les PWM à `[-255, 255]`.

### Sécurité réseau
- `teleop_receiver_node` stoppe les moteurs si aucune commande reçue depuis 500ms

---

## Téléopération — profil Bras

### Touches clavier (dans `rover_gui` mode bras)
| Touche | Action |
|--------|--------|
| O / K | UP / DOWN (axe Z — stepper) |
| J / L | CLOSE / OPEN (pinces — servo) |
| I / P | FLIP / UNFLIP (axe Y — servo) |
| ↑ / ↓ | Benne monte / descend (servo) |
| 6 / 7 / 8 / 9 / 0 | Niveaux de vitesse |
| Espace | Stop tout (bras + benne + angles remis à 0) |
| M | Retour menu |

### Contrôle servos — angle absolu accumulé
- **Servos** (pince, flip, benne) : le GUI accumule l'angle en local [-100..100] à chaque tick de 100ms. Tant qu'une touche est maintenue, l'angle augmente/diminue progressivement. Relâché → servo reste en place.
- **Stepper** (axe Z) : directionnel, pas d'angle accumulé.
- Bouton `↺ 0` à côté de chaque axe servo → reset individuel de l'angle à 0.
- Bouton "POSITION DE VIDAGE" → force benne à 100 (position maximale).
- Topic publié : `/rover/arm_cmd` (`Float32MultiArray` — [z, s1_angle, s23_angle, speed, dump, s4_angle])
  - `z` = direction stepper (-1/0/+1) × speed
  - `s1_angle`, `s23_angle`, `s4_angle` = angles absolus [-100..100] calculés par le GUI
- `arm_node` se contente de borner et transmettre les angles (`clamp(y, -100, 100)`) — pas de recalcul.

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

- Grille interne **10 lignes × 6 colonnes** (cellules non carrées : 800mm × 833mm) — 8m/10=800mm, 5m/6≈833mm
- Grille totale **12 lignes × 8 colonnes** (GRID_ROWS=12, GRID_COLS=8) avec rangée `BORDER` tout autour — élimine toute vérification out-of-bounds
- Convention : les **lignes (rows) augmentent dans le sens d'avancement** (8m), les colonnes = direction perpendiculaire (5m)
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

### Architecture lifecycle (RPi)

| Node | Type | Actif en |
|------|------|----------|
| `mode_manager_node` | Node (orchestrateur) | toujours |
| `serial_bridge_node` | Node (hardware owner) | toujours |
| `motor_controller_node` | **LifecycleNode** | race / arm / autonomous |
| `arm_node` | **LifecycleNode** | arm / autonomous |
| `aruco_node` | **LifecycleNode** | autonomous |
| `camera_node` | Node (natif hors Docker) | race / arm / autonomous (auto-géré) |

`mode_manager_node` orchestre les transitions lifecycle via les services ROS2 (`/<node>/change_state`). Aucun node géré ne connaît le mode — seul mode_manager décide qui tourne.

### Lancer le rover

```bash
# Terminal 1 — natif (hors Docker) — caméra Pi
ros2 run rover_xplore camera_node

# Terminal 2 — dans Docker — tous les autres nodes
ros2 launch rover_xplore rover.launch.py
```

### Repo Rover (`xplore_rasberry`)
| Fichier | État |
|---------|------|
| `rover_xplore/rover_xplore/camera_node.py` | Prêt — picamera2 (libcamera) + cv2, double pub `/camera/image_raw` (bgr8, pour aruco) + `/camera/image_compressed` (JPEG, pour FPV), fallback V4L2. **Lance nativement hors Docker.** |
| `rover_xplore/rover_xplore/mode_manager_node.py` | Prêt — orchestrateur lifecycle. Queue + worker thread unique (pas de race condition). Configure les nodes au démarrage, active/désactive selon le mode reçu sur `/rover/mode`. |
| `rover_xplore/rover_xplore/motor_controller_node.py` | Prêt — LifecycleNode. Cinématique diff → `/rover/motor_cmd` Int32[FR,FL,BR,BL] (-255..255). Timeout 500ms. Zeros garantis à Ctrl+C. |
| `rover_xplore/rover_xplore/arm_node.py` | Prêt — LifecycleNode. Reçoit `/rover/arm_cmd` Float32[z,s1_angle,s23_angle,speed,dump,s4_angle] → `/rover/arm_serial_cmd` Int32[s1,s2,s3,s4,stepper]. Angles servos envoyés directs (le GUI accumule), stepper reste directionnel. Zeros garantis à Ctrl+C. |
| `rover_xplore/rover_xplore/aruco_node.py` | Prêt — LifecycleNode. Détection ArUco sur `/camera/image_raw` (bgr8 non compressé). Publie `/aruco_detected` Float32[found,id,cx,cy,area]. |
| `rover_xplore/rover_xplore/serial_bridge_node.py` | Prêt — pont binaire 10 Hz. Envoi struct 18B, réception struct 30B. Lecture buffer entier par tick (pas de corruption de trame). Reconnexion automatique si USB se débranche. Timeout 1s sécurité moteurs. |
| `rover_xplore/launch/rover.launch.py` | Prêt — lance tous les nodes sauf camera_node (natif). |
| `rover_xplore/rover_xplore/autonomous_node.py` | À créer — grille BFS + machine à états, s'abonne `/rover/nav_goal`, publie `/rover/grid_state` + `/rover/grid_pos` |
| `rover_xplore/rover_xplore/teleop_receiver_node.py` | Supprimé — remplacé par motor_controller_node |

### Repo PC (`xplore_pub`)
| Fichier | État |
|---------|------|
| `rover_xplore_pub/rover_xplore_pub/rover_gui.py` | **Prêt — GUI PySide6 unifiée.** Menu → Téléop (Race / Bras) ou Autonome. **MapWidget** : grille 12×8 interactive, BFS simulé. **ArmPage** : angles servos accumulés localement, affichage numérique (+75/-30/0), boutons reset individuel par servo, vue rover top-down avec vitesses roues. **RacePage** : idem vue rover top-down. **Animations** : GlowCard, GlowButton, PulsingDot, footer dynamique. |
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

---

## Points d'amélioration à traiter plus tard

| # | Fichier | Problème | Fix |
|---|---------|----------|-----|
| 1 | `serial_bridge_node` | Pas de framing serial avec l'Arduino — si l'Arduino reboot ou envoie des données corrompues, la struct peut être décodée n'importe comment | Ajouter un byte de début de trame côté Arduino + vérification côté Pi (à coordonner avec l'équipe élec) |
| 2 | `mode_manager_node` | `time.sleep(2.0)` au démarrage avant de configurer les nodes lifecycle — fragile si le système est lent | Remplacer par une boucle de polling qui vérifie la disponibilité des services sans sleep fixe |
| 3 | `camera_node` | Gère son mode en interne (subscribe à `/rover/mode`) alors que tous les autres sont lifecycle — incohérence architecturale | Réécrire en LifecycleNode quand libcamera sera mieux maîtrisé |
| 4 | Tous | Zéro tests automatisés | Ajouter des tests unitaires sur la cinématique diff, le mapping arm, le packing/unpacking de la struct serial |
| 5 | `rover_gui.py` — `MapWidget` | La navigation BFS est simulée côté PC uniquement — les topics `/rover/nav_goal`, `/rover/grid_state`, `/rover/grid_pos` ne sont pas encore implémentés côté RPi | Implémenter un nœud `nav_node` sur le RPi qui publie l'état de la grille et reçoit les objectifs de navigation |
| 6 | `xplore_pub` Docker | PySide6 et `libxcb-cursor0` doivent être réinstallés à chaque redémarrage du container (pas de persistence) | Ajouter `RUN pip install PySide6 && apt-get install -y libxcb-cursor0` dans le Dockerfile pour éviter les steps manuels |

---

## Prochaine session — à implémenter

### 1. `odometry_node.py` (priorité)
Architecture complète documentée dans `autonome.md`.
- Dead-reckoning encodeurs + filtre complémentaire gyro (MPU9250)
- Publie `/rover/pose` [x_mm, y_mm, theta_rad] + `/rover/grid_pos` [col, row]
- Service `/rover/reset_pose` (std_srvs/Trigger)
- Plan détaillé : `/Users/hadyazzi/.claude/plans/virtual-chasing-bear.md`
- Fichiers à modifier : `rover_xplore/rover_xplore/odometry_node.py` (créer), `setup.py`, `package.xml`, `rover.launch.py`

### 2. `autonomous_node.py`
Architecture complète dans `autonome.md` (sections 4, 5, 9, 10).
- BFS + PID navigation case à case
- Souscrit `/rover/pose` + `/rover/grid_pos` + `/aruco_detected` + `/ultrasonic`
- Publie `/rover/cmd_vel` + `/rover_status` + `/rover/grid_state`
- États : IDLE → EXPLORING → VISUAL_ALIGNING → VISUAL_APPROACHING → ARUCO_REACHED → RETURNING → DONE

### 3. PID moteurs (feedback encodeurs)
- Boucle fermée dans `motor_controller_node` sur `/wheel_encoders`
- Actuellement : PWM brut sans feedback

### Notes approche ArUco finale
- Phase 1 : BFS vers case approx. donnée par opérateur avant mission
- Phase 2 (visuelle) : `cx - 320` pour bearing, US d2 < 300mm pour stop
- Option C (pose estimation tvec) documentée mais non implémentée — nécessite calibration caméra + taille marker physique

---

*Dernière mise à jour : 2026-05-15 (session 18 — mode bras : angle absolu accumulé côté GUI (servos), arm_node transmet direct, boutons reset servo, vue rover top-down avec vitesses roues dans Race et Arm. Fix téléop : Y/X arrière inversés, rotation A/D clamp supprimé (±8.0 → ±100% PWM). Architecture autonome complète dans autonome.md. Prochaine session : coder odometry_node.py)*
