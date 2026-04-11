# Plan — Mode Autonome : Navigation vers ArUco + Retour

## Contexte

Compétition robotique : terrain 8m×5m plat, obstacles 10-25cm de haut, ArUco 5x5_250 (14.4cm)
visible depuis le départ et en hauteur. Objectifs :
1. Atteindre la structure ArUco (< 1m) → 20 pts
2. Revenir à la position de départ → 40 pts
3. (Bonus) Attraper une bouteille et la ramener → 40 pts

**Contraintes techniques :**
- Tout le code dans `rover_xplore` uniquement
- Moteurs : RPi → serial Arduino (115200 baud, `/dev/ttyUSB0`) → PID roues côté Arduino
- Caméra : **problème de compatibilité libcamera non résolu** (Python 3.12 host vs Python 3.10 Docker).
  Plusieurs solutions envisagées (MJPEG bridge et autres) — pas encore décidée ni implémentée.
  Le `aruco_node.py` sera écrit pour consommer `/camera/image_raw` ; la source de ce topic
  (camera_node.py à adapter) sera résolue en parallèle.
- Odométrie : encodeurs JGA25-370 (quadrature A/B) + IMU → **filtre EKF**

---

## Architecture cible

```
[source caméra — à décider]
       ↓
  /camera/image_raw
       ↓
  aruco_node.py ─────────────────────────→ /aruco_detected
  encoder_node.py ───────────────────────→ /wheel_odom
  imu_node.py ───────────────────────────→ /imu/data
  ultrasonic_node.py ────────────────────→ /distances [fl, fc, fr, l, r] cm
       ↓
  ekf_odometry_node.py ──────────────────→ /odom [x, y, theta]
       ↓
  autonomous_node.py ────────────────────→ /cmd_vel (Twist)
       ↓
  motor_controller_node.py ──────────────→ serial Arduino → PID → PWM moteurs

  mode_manager_node.py ← /rover/mode → publie /rover_mode pour activer autonomous_node
```

---

## Machine à états — FSM compétition

```
IDLE
  │ mode = 'autonomous'
  ▼
LOCATE        ← cherche ArUco ; si pas vu dans 3s → rotation lente jusqu'à détection
  │ N=5 frames consécutives confirmées
  ▼
APPROACH      ← asservissement visuel (center_x) + évitement US + enregistrement waypoints
  │ dist_aruco_m < 1.5m
  ▼
ARRIVED       ← arrêt, log position finale ; → GRAB si bras prêt, sinon → RETURN
  ▼
GRAB          ← bras vers offset [x,y,z] relatif ArUco → saisie bouteille → rétraction
  │ (implémenté quand bras finalisé par équipe élec)
  ▼
RETURN        ← rejoue waypoints en sens inverse + évitement US actif
  │ waypoint[0] atteint (dist < 0.3m)
  ▼
DONE          ← moteurs stop, publie MISSION_DONE
```

**Gestion obstacles (AVOID) :** sous-état appelable depuis APPROACH et RETURN.
- Évalue fl/fc/fr/l/r → tourne vers côté le plus libre
- Anti-rebond : max 3 rotations avant dégagement forcé
- Retourne à l'état précédent quand voie libre devant

---

## Fichiers à créer

### 1. `rover_xplore/rover_xplore/aruco_node.py` — PRIORITÉ 1

**Subscribe :** `/camera/image_raw` (sensor_msgs/Image)
**Publish :** `/aruco_detected` (Float32MultiArray)
- Format : `[found(0/1), marker_id, cx_px, cy_px, area_px2, distance_m, angle_rad]`
- Dict : `cv2.aruco.DICT_5X5_250`, taille réelle : `0.144` m
- Distance : règle des triangles similaires (sans calibration → approximation suffisante à 5-8m)
  `dist_m = (0.144 * focal_px_estimé) / marker_width_px`
- Angle : `atan2((cx - 320) / focal_px, 1.0)` → angle horizontal en rad
- Si plusieurs markers → garder le plus grand (plus proche)

### 2. `rover_xplore/rover_xplore/ultrasonic_node.py`

**Publish :** `/distances` (Float32MultiArray `[fl, fc, fr, l, r]` en cm)
- GPIO TRIG/ECHO (pins à confirmer avec équipe élec)
- `dist_cm = echo_duration_us / 58.0`, max 400cm, min 2cm
- Fréquence : 10 Hz

### 3. `rover_xplore/rover_xplore/encoder_node.py`

**Publish :** `/wheel_odom` (Float32MultiArray `[v_left_ms, v_right_ms, dt_s]`)
- Interruptions GPIO sur front montant/descendant des signaux A/B
- `v = (delta_ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE / dt`
- Pins et TICKS_PER_REV à calibrer avec équipe élec

### 4. `rover_xplore/rover_xplore/imu_node.py`

**Publish :** `/imu/data` (Float32MultiArray `[roll, pitch, yaw, gyro_z_rads]`)
- I2C (MPU6050 ou autre — type à confirmer)
- Fréquence : 50 Hz

### 5. `rover_xplore/rover_xplore/ekf_odometry_node.py`

**Subscribe :** `/wheel_odom`, `/imu/data`
**Publish :** `/odom` (Float32MultiArray `[x, y, theta]`)

**EKF — état [x, y, θ] :**
```
État      : X = [x, y, θ]ᵀ
Prédiction: encodeurs (modèle cinématique différentielle)
Correction: cap IMU (yaw) comme mesure z = θ_imu

F = jacobien du modèle de mouvement
Q = bruit process (à calibrer : σ_x, σ_y, σ_θ)
R = bruit mesure IMU (σ_imu)

Predict:  X̂ = f(X, u),  P = F P Fᵀ + Q
Update:   K = P Hᵀ (H P Hᵀ + R)⁻¹
          X = X̂ + K (z - H X̂)
          P = (I - K H) P
```
- Fréquence : synchronisée sur `/wheel_odom` (10 Hz), mise à jour IMU dès réception

### 6. `rover_xplore/rover_xplore/motor_controller_node.py`

**Subscribe :** `/cmd_vel` (Twist)
**Logique :**
- `v_left  = linear - angular * WHEEL_BASE/2`
- `v_right = linear + angular * WHEEL_BASE/2`
- Envoyer via serial : `L<v_left_ms> R<v_right_ms>\n`
- Safety timeout 500ms → `L0.0 R0.0\n`
- **PID moteurs côté Arduino** (Arduino reçoit vitesse cible m/s, compare avec encodeurs, ajuste PWM)

```python
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 115200
WHEEL_BASE  = 0.25   # m — À CALIBRER
```

### 7. `rover_xplore/rover_xplore/autonomous_node.py`

**Subscribe :** `/aruco_detected`, `/distances`, `/odom`, `/rover/mode`
**Publish :** `/cmd_vel`, `/rover_status`

**Constantes (à calibrer sur hardware) :**
```python
TICK_RATE           = 10.0   # Hz
CONFIRM_N_FRAMES    = 5      # frames consécutives pour valider ArUco
LOCATE_TIMEOUT_S    = 3.0    # avant rotation si ArUco pas vu
LOCATE_ROTATE_SPEED = 0.3    # rad/s
APPROACH_DEADZONE   = 0.08   # ±8% image
APPROACH_LINEAR     = 0.2    # m/s
APPROACH_ANGULAR_KP = 1.2    # gain P visuel
ARRIVED_DIST_M      = 1.5    # distance ArUco → ARRIVED
US_FRONT_CM         = 30.0   # seuil obstacle avant
US_SIDE_CM          = 20.0   # seuil latéral
AVOID_TURN_SPEED    = 0.8    # rad/s
WAYPOINT_STEP_M     = 0.3    # espacement waypoints
RETURN_KP           = 1.5    # gain P cap retour
RETURN_ARRIVED_M    = 0.3    # rayon arrêt final
```

**Waypoints (APPROACH) :** Ajouter (x, y, θ) tous les 0.3m parcourus.
**RETURN :** Parcourir `self.waypoints` en sens inverse, cap proportionnel vers chaque waypoint.

---

## Fichiers à modifier

### `rover_xplore/rover_xplore/mode_manager_node.py`

Ajouter publisher `/rover_mode` (String).
Dans `mode_callback` cas `'autonomous'` → publier `'AUTO'` sur `/rover_mode`.

### `rover_xplore/setup.py`

Ajouter les entry points :
```python
'aruco_node            = rover_xplore.aruco_node:main',
'ultrasonic_node       = rover_xplore.ultrasonic_node:main',
'encoder_node          = rover_xplore.encoder_node:main',
'imu_node              = rover_xplore.imu_node:main',
'ekf_odometry_node     = rover_xplore.ekf_odometry_node:main',
'motor_controller_node = rover_xplore.motor_controller_node:main',
'autonomous_node       = rover_xplore.autonomous_node:main',
```

---

## Ordre d'implémentation recommandé

1. `aruco_node.py` → testable dès que caméra résolue
2. `motor_controller_node.py` → testable avec Arduino
3. `ultrasonic_node.py` → testable GPIO
4. `encoder_node.py` + `imu_node.py` → testable capteurs
5. `ekf_odometry_node.py` → testable avec topics simulés
6. `autonomous_node.py` → intègre tout, simulable via `ros2 topic pub`
7. Modifs `mode_manager_node.py` + `setup.py`

---

## Tests de vérification

```bash
# Test ArUco isolé
ros2 run rover_xplore aruco_node &
ros2 topic echo /aruco_detected
# → [1.0, id, cx, cy, area, dist_m, angle] en pointant vers marker

# Test moteurs
ros2 run rover_xplore motor_controller_node &
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
# → vérifier trame série Arduino + rotation roues

# Test FSM simulé
ros2 run rover_xplore autonomous_node &
ros2 topic pub /distances std_msgs/Float32MultiArray "{data: [100.0,100.0,100.0,100.0,100.0]}"
ros2 topic pub /odom std_msgs/Float32MultiArray "{data: [0.0,0.0,0.0]}"
ros2 topic pub /rover/mode std_msgs/String "{data: 'autonomous'}"
ros2 topic echo /rover_status
# → LOCATE → APPROACH dès qu'un ArUco est visible sur /aruco_detected
```

---

## À clarifier avec équipe électronique

- Protocole trame série Arduino (format exact des commandes vitesse)
- Pins GPIO : TRIG/ECHO × 5 ultrasoniques, A/B × encodeurs
- Type IMU + adresse I2C
- WHEEL_BASE, WHEEL_CIRCUMFERENCE, TICKS_PER_REV
- Interface bras (quand finalisé)
- Solution caméra retenue (MJPEG bridge ou autre)
