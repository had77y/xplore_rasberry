# Mode Autonome — Architecture & Décisions

> Document de référence pour l'implémentation du mode autonome sur le RPi.
> Reflète toutes les décisions, choix et incertitudes discutés en session.

---

## 1. Contexte général

Le mode autonome est **entièrement exécuté sur le Raspberry Pi** — pas dans le GUI (`xplore_pub`).  
Le rover doit explorer une grille et atteindre un **unique marqueur ArUco** (destination inconnue à l'avance).  
La logique autonome existait côté GUI (simulation BFS) — elle est ici remplacée par de vrais nodes ROS2 sur le RPi avec les vrais capteurs.

---

## 2. La grille

### Dimensions physiques
- Map réelle : **8 m × 5 m**
- Grille interne navigable : **10 colonnes × 6 lignes**
- Grille totale avec bordures : **12 × 8** (1 cellule de bordure tout autour)

### Convention d'orientation de la grille

Les **lignes (rows) augmentent dans le sens d'avancement du rover** (8m). Les **colonnes (cols)** représentent la direction perpendiculaire (5m).

```
GRID_ROWS = 12   (10 navigables + 2 bordures)   ← direction avancement (8m)
GRID_COLS = 8    (6 navigables  + 2 bordures)   ← direction perpendiculaire (5m)
```

### Taille des cellules
Les cellules sont **non carrées** pour couvrir toute la map :

```
CELL_ROW_MM = 800   (8000mm / 10 lignes navigables  — division exacte)
CELL_COL_MM = 833   (5000mm / 6 colonnes navigables — reste 2mm ignoré)
```

> **Pourquoi non carré ?** 5000 / 800 = 6.25 → une cellule carrée laisserait 200mm de map non couverte.
> L'ArUco pouvant être n'importe où, on choisit de couvrir toute la map.

### États des cases
Repris et adaptés depuis le GUI (`rover_gui.py`) :

| Valeur | Nom | Description |
|--------|-----|-------------|
| 0 | `UNDISCOVERED` | Jamais visitée — état initial |
| 1 | `FREE` | Visitée, libre |
| 2 | `OBSTACLE` | Obstacle détecté |
| 3 | `AMBUSH` | Impasse — toutes les sorties bloquées |
| 4 | `CELL_BORDER` | Bordure de map — hors grille |

### Coordonnées
- Origine : **coin de la map** (à définir précisément — voir §7 Incertitudes)
- `row = floor(y_mm / CELL_ROW_MM)` — y_mm = position dans la direction d'avancement (0..8000mm)
- `col = floor(x_mm / CELL_COL_MM)` — x_mm = position perpendiculaire (0..5000mm)
- `floor()` utilisé (et non `int()`) pour gérer correctement les valeurs négatives

---

## 3. Localisation — `odometry_node`

### Principe
Dead-reckoning par intégration des encodeurs de roues, corrigé par le gyroscope MPU9250.  
Node toujours actif (plain `Node`, pas `LifecycleNode`), comme `serial_bridge_node`.

### Topics
| Direction | Topic | Type | Contenu |
|-----------|-------|------|---------|
| In | `/wheel_encoders` | `Int32MultiArray` | [m1, m2, m3, m4] — Δticks / 100ms |
| In | `/imu/raw` | `sensor_msgs/Imu` | `angular_velocity.z` brut MPU9250 |
| Out | `/rover/pose` | `Float32MultiArray` | [x_mm, y_mm, theta_rad] |
| Out | `/rover/grid_pos` | `Int32MultiArray` | [col, row] |

### Service
- `/rover/reset_pose` (`std_srvs/Trigger`) → remet (x, y, θ) à la position initiale

### Paramètres physiques
| Paramètre | Valeur | Note |
|-----------|--------|------|
| `wheel_diameter_mm` | 120 | Diamètre roue 12 cm |
| `wheel_base_mm` | 250 | Entraxe gauche-droite 25 cm |
| `ticks_per_rev` | 1320 | JGA25-370 — **à calibrer** |
| `alpha` | 0.95 | Poids filtre complémentaire |
| `gyro_z_scale` | 0.000133 | MPU9250 ±250°/s → rad/s par LSB = π/(180×131) |

### Mapping encodeurs
```
m1 = Front Right (FR)    m3 = Back Right (BR)  →  v_right = mean(m1, m3)
m2 = Front Left  (FL)    m4 = Back Left  (BL)  →  v_left  = mean(m2, m4)
```

### Cinématique différentielle (tout en mm, θ en rad)

```
mm_per_tick = (π × wheel_diameter_mm) / ticks_per_rev   # ≈ 0.285 mm/tick

# Encodeurs
d_right = mean(Δm1, Δm3) × mm_per_tick
d_left  = mean(Δm2, Δm4) × mm_per_tick

d_center = (d_right + d_left) / 2.0
dθ_enc   = (d_right - d_left) / wheel_base_mm   # rad (mm/mm)

# Gyroscope MPU9250 — correction angulaire
raw_z = imu.angular_velocity.z
if raw_z > 32767: raw_z -= 65536    # uint16 → int16 signé (bug serial_bridge)
gyro_z_rads = (raw_z - gyro_bias_z) × gyro_z_scale
dθ_gyro = gyro_z_rads × 0.1        # dt = 100ms

# Filtre complémentaire
dθ = alpha × dθ_gyro + (1 - alpha) × dθ_enc

# Intégration pose — midpoint heading
x     += d_center × cos(θ + dθ/2)
y     += d_center × sin(θ + dθ/2)
θ     += dθ
θ      = atan2(sin(θ), cos(θ))     # normalisation (-π, π)

row = floor(y / CELL_ROW_MM)   # y = direction avancement
col = floor(x / CELL_COL_MM)   # x = direction perpendiculaire
```

> **Midpoint heading** : on utilise `θ + dθ/2` (angle médian) plutôt que l'angle de début ou de fin — plus précis dans les virages.

### Calibration biais gyro
Au démarrage, lire ~100 échantillons gyro avec le rover immobile → `gyro_bias_z = mean(raw_z)`.

### Calibration encodeurs
1. `ros2 service call /rover/reset_pose std_srvs/srv/Trigger`
2. Faire avancer exactement 1000 mm (ruban)
3. `ros2 topic echo /rover/pose --once` → lire `x_reporté`
4. `ticks_per_rev_calibré = ticks_per_rev × (1000.0 / x_reporté)`

> **Note** : le commentaire dans `serial_bridge_node.py` dit `[d1..d5] en cm` — à vérifier, l'Arduino envoie peut-être des mm.

---

## 4. Navigation — `autonomous_node`

### Principe général
Navigation case par case. Le rover va toujours au **centre de sa case courante** avant de décider de la case suivante.

Centre d'une case `(col, row)` :
```
x_centre = col × cell_width_mm  + cell_width_mm  / 2
y_centre = row × cell_height_mm + cell_height_mm / 2
```

### Algorithme (porté depuis `rover_gui.py`)

**Priorité des cases — distance de Chebyshev vers l'ArUco :**
```
priority[r][c] = max(|r - target_row|, |c - target_col|)
```
Distance 0 à l'ArUco, augmente en spirale. Précalculé statiquement quand la position de l'ArUco est connue.

> **Note** : tant que l'ArUco n'est pas détecté, la priorité est calculée dynamiquement ou on fait une exploration systématique.

**Sélection de la case suivante — BFS :**
- Traverse uniquement les cases `FREE`
- Parmi toutes les cases `UNDISCOVERED` atteignables, choisit celle avec la **plus petite priorité** (plus proche de l'ArUco)
- Mouvement **8 directions** (N, S, E, W + diagonales)
- Guard diagonal : pour un mouvement diagonal, les 2 cases cardinales adjacentes ne doivent pas être bloquées

**Trois cas possibles à chaque step :**

| Cas | Condition | Action |
|-----|-----------|--------|
| Transit en cours | `transit_path` non vide | Suivre le chemin d'une case |
| UNDISCOVERED adjacente | BFS retourne chemin de longueur 2 | Y aller, marquer FREE, push stack |
| UNDISCOVERED non adjacente | BFS retourne chemin plus long | Stocker `transit_path`, avancer d'une case |
| Impasse | BFS retourne vide | Marquer AMBUSH, backtrack via `path_stack` |

**Structures de données :**
```python
_cells[row][col]  # état de chaque case (UNDISCOVERED/FREE/OBSTACLE/AMBUSH/CELL_BORDER)
_priority[row][col]  # distance Chebyshev vers ArUco
_path_stack   # historique pour backtracking
_transit_path # chemin en cours à travers des cases FREE
```

### Modifications vs GUI
1. **Pas de QTimer 500ms** — le step est déclenché par l'arrivée au centre de la case (odométrie)
2. **Obstacles dynamiques** — les cases sont mises à jour en temps réel depuis les ultrasons
3. **Obstacle mi-chemin** — si un obstacle est détecté pendant un `transit_path`, stopper + relancer BFS
4. **Diagonales conservées** — avec le guard existant (les deux cardinales adjacentes doivent être libres)

---

## 5. Détection des obstacles — Ultrasons

### Capteurs disponibles
5 capteurs US publiés sur `/ultrasonic` (Float32MultiArray [d1..d5] en mm).  
Tous lus simultanément depuis la struct Arduino 30B reçue par `serial_bridge_node`.

### Configuration des capteurs (repère rover : x=avant, y=gauche, origine=centre rover)

```python
import math
# Format : (sx_mm, sy_mm, angle_rad, threshold_mm)
SENSORS = [
    ( 40,  +50, math.radians(+30), 2400),  # d1 — US avant-gauche
    ( 40,    0, math.radians(  0), 2400),  # d2 — US central avant
    ( 40,  -50, math.radians(-30), 2400),  # d3 — US avant-droit
    (-135, +150, math.radians(+90),  800), # d4 — US latéral gauche
    (-135, -150, math.radians(-90),  800), # d5 — US latéral droit
]
# ⚠ Mapping d1..d5 → capteur physique à confirmer avec team élec (câblage Arduino)
```

> Note : x=+40mm car les ~26cm avant sont occupés par le bras robotique.  
> Capteurs avant (d1/d2/d3) : seuil 2400mm (= 3 × CELL_ROW_MM) — couvre 2 cases devant dans le pire cas.  
> Capteurs latéraux (d4/d5) : seuil 800mm (= 1 case) — détection immédiate uniquement.

### Algorithme de détection par tick US

#### Debounce
`_us_hit_count: Dict[int, int]` — nb de lectures consécutives < seuil par capteur.  
`HIT_COUNT_REQUIRED = 2` lectures avant de marquer OBSTACLE (évite les faux positifs).

```python
HIT_COUNT_REQUIRED = 2

def _us_cb(self, msg):
    distances = msg.data  # [d1..d5] en mm
    for i, (sx, sy, sa, threshold) in enumerate(SENSORS):
        d = distances[i]
        if 10 < d < threshold:                          # filtre bruit (<10mm) + seuil
            self._us_hit_count[i] = self._us_hit_count.get(i, 0) + 1
            if self._us_hit_count[i] >= HIT_COUNT_REQUIRED:
                self._mark_obstacle(sx, sy, sa, d)
        else:
            self._us_hit_count[i] = 0                  # reset debounce si rien détecté
```

#### Projection dans la grille — `_mark_obstacle(sx, sy, sa, d)`

```python
from math import floor, cos, sin

def _mark_obstacle(self, sx, sy, sa, d):
    theta = self._theta  # cap rover dans repère monde (rad)

    # Étape 1 : position du capteur dans repère monde
    #   (rotation du vecteur capteur par le cap rover)
    sensor_wx = self._x + sx * cos(theta) - sy * sin(theta)
    sensor_wy = self._y + sx * sin(theta) + sy * cos(theta)

    # Étape 2 : position de l'obstacle (d mm depuis le capteur, dans la direction sa+theta)
    obs_x = sensor_wx + d * cos(theta + sa)
    obs_y = sensor_wy + d * sin(theta + sa)

    # Étape 3 : convertir en case grille
    obs_row = int(floor(obs_y / CELL_ROW_MM))   # y = direction avancement
    obs_col = int(floor(obs_x / CELL_COL_MM))   # x = direction perpendiculaire

    # Étape 4 : marquer (les cellules BORDER absorbent les débordements)
    if 0 <= obs_row < GRID_ROWS and 0 <= obs_col < GRID_COLS:
        if self._cells[obs_row][obs_col] not in (CELL_BORDER, OBSTACLE):
            self._cells[obs_row][obs_col] = OBSTACLE
            self._on_new_obstacle(obs_row, obs_col)
```

#### Comportement après détection — `_on_new_obstacle(row, col)`

Deux cas selon que l'obstacle tombe sur le transit en cours ou non :

```python
def _on_new_obstacle(self, row, col):
    if any(r == row and c == col for r, c in self._transit_path):
        # Obstacle sur le chemin en cours → invalider et replanifier
        self._transit_path.clear()
        self.get_logger().warn(f'Obstacle sur transit ({row},{col}) — replanification BFS')
    # Sinon : obstacle hors transit → juste marqué dans la grille
    # Au prochain step, le BFS en tiendra compte automatiquement
```

> **Comportement général :** le rover ne s'arrête pas quand il détecte un obstacle à distance.  
> Il marque la case OBSTACLE, continue son déplacement actuel (vers la case suivante).  
> Arrivé à la prochaine case, le BFS recalcule un chemin qui évite l'obstacle déjà dans la grille.

---

## 6. Initialisation de la grille

```python
CELL_BORDER  = 4
UNDISCOVERED = 0
FREE         = 1
OBSTACLE     = 2
AMBUSH       = 3

# Grille 12 lignes × 8 colonnes
# Rows 0 et 11 = BORDER, cols 0 et 7 = BORDER
# Zone navigable : rows 1-10 (10 × 800mm = 8m), cols 1-6 (6 × 833mm = 5m)
_cells = [[UNDISCOVERED] * GRID_COLS for _ in range(GRID_ROWS)]
for r in range(GRID_ROWS):
    for c in range(GRID_COLS):
        if r == 0 or r == GRID_ROWS - 1 or c == 0 or c == GRID_COLS - 1:
            _cells[r][c] = CELL_BORDER
```

---

## 7. Incertitudes & points non résolus

| Point | État | Notes |
|-------|------|-------|
| Position de départ sur la grille | **Non résolu** | Le rover est posé manuellement — pas de recalage automatique. La position initiale est déclarée via `reset_pose`, l'opérateur place le rover le plus près possible du centre de la case de départ. |
| Positions US avant (3 capteurs) | **En attente team meca** | Positions proposées : voir §5 |
| Mapping d1..d5 → capteur physique | **Non résolu** | Dépend du câblage Arduino |
| `ticks_per_rev` encodeurs | **À calibrer** | Valeur par défaut 1320 (JGA25-370 30:1) |
| Range MPU9250 gyro | **Assumé ±250°/s** | À vérifier dans le code Arduino |
| Valeur US en cm ou mm | **À vérifier** | `serial_bridge` dit cm, user dit mm |
| `wheel_base_mm` précis | **À calibrer** | Valeur initiale 250mm |
| Origine de la grille (coin exact) | **À définir** | Convention à établir avec la team |
| ArUco position inconnue | **Par design** | Le rover explore jusqu'à le trouver |

---

## 8. Architecture des nodes

```
serial_bridge_node  (toujours actif)
    ├── publie /wheel_encoders → odometry_node
    ├── publie /imu/raw        → odometry_node
    └── publie /ultrasonic     → autonomous_node

odometry_node  (toujours actif)
    ├── publie /rover/pose     → autonomous_node
    ├── publie /rover/grid_pos → autonomous_node + GUI
    └── service /rover/reset_pose

autonomous_node  (LifecycleNode, actif en mode autonomous)
    ├── souscrit /rover/pose
    ├── souscrit /rover/grid_pos
    ├── souscrit /aruco_detected
    ├── souscrit /ultrasonic
    ├── souscrit /rover/nav_goal   ← cible envoyée par le GUI au moment du START
    ├── publie /rover/cmd_vel      → motor_controller_node
    └── publie /rover/grid_state   → GUI (grille temps réel 96 valeurs row-major)

mode_manager_node  (toujours actif)
    └── active autonomous_node quand mode = 'autonomous'
```

### Démarrage du mode autonome — flux opérateur

```
1. Opérateur sélectionne mode "autonomous" dans le GUI
   → mode_manager active autonomous_node (Inactive → Active)
   → autonomous_node démarre en IDLE (attend nav_goal)

2. Opérateur oriente le rover dans la direction de départ, appuie reset_pose si besoin

3. Opérateur clique la case cible sur la MapWidget du GUI, appuie START
   → GUI publie /rover/nav_goal [row, col]
   → autonomous_node reçoit nav_goal, reset la grille, lance EXPLORING

4. Le rover explore et renvoie /rover/grid_state à chaque changement de case
   → GUI met à jour la MapWidget en temps réel
```

---

## 9. PID — deux niveaux

### PID niveau 1 — Vitesse moteurs (`motor_controller_node`)
- **Feedback** : `/wheel_encoders` (Δticks/100ms → vitesse réelle)
- **Consigne** : vitesse désirée issue de `cmd_vel`
- **Sortie** : PWM corrigé vers `/rover/motor_cmd`
- **Pourquoi** : actuellement open-loop — asymétrie moteurs et variation batterie non compensées

### PID niveau 2 — Navigation case à case (`autonomous_node`)
- **Feedback** : `/rover/pose` [x, y, θ] depuis `odometry_node`
- **Consigne** : centre de la case cible `(col × cell_width + cell_width/2, row × cell_height + cell_height/2)`
- **Sortie** : `cmd_vel` Twist (linear.x, angular.z)
- **Pourquoi** : garantit que le rover arrive au centre exact de chaque case

### Pipeline complet
```
autonomous_node
  → PID navigation → /rover/cmd_vel
      → motor_controller_node
          → PID vitesse → /rover/motor_cmd
              → serial_bridge → Arduino → moteurs
                  → /wheel_encoders ──────────────→ (feedback PID vitesse)
  ← /rover/pose ← odometry_node ← /wheel_encoders  (feedback PID navigation)
```

---

## 10. Mouvement physique case à case

### Principe général

Le rover tourne sur lui-même pour faire face à la case cible, puis avance (ou recule). Le déplacement se fait en deux sous-états séquentiels : `ROTATING` puis `MOVING`.

### Direction (dr, dc) → stratégie

`dr = row_cible - row_courant`, `dc = col_cible - col_courant`

| dr | dc | Rotation | Déplacement |
|----|----|----------|-------------|
| +1 | 0 | 0° (tout droit) | avance |
| -1 | 0 | 0° (tout droit) | recule |
| 0 | ±1 | ±90° | avance |
| +1 | ±1 | ±45° | avance |
| -1 | ±1 | ±45° (côté opposé) | recule |

**Formule générale :**
```python
heading_cible = atan2(dc, dr)   # 0° = avancer (rows croissants), 90° = droite
delta_theta   = normalize(heading_cible - theta_actuel)  # normalisé [-π, π]

if abs(delta_theta) <= pi/2:
    # Faire face à la cible et avancer
    rotate_to(heading_cible)
    move(FORWARD, distance)
else:
    # Faire face à l'opposé et reculer (rotation minimale)
    rotate_to(normalize(heading_cible + pi))
    move(REVERSE, distance)
```

**Distance à parcourir :**
```python
CELL_ROW_MM = 800
CELL_COL_MM = 833
CELL_DIAG_MM = sqrt(CELL_ROW_MM**2 + CELL_COL_MM**2)  # ≈ 1155mm

distance = CELL_DIAG_MM if (dr != 0 and dc != 0) else (CELL_ROW_MM if dc == 0 else CELL_COL_MM)
```

### Sous-états de mouvement

```
ROTATING : publie angular.z = Kp_rot × delta_theta (capped)
           linear.x = 0
           → fin quand |delta_theta| < ANGLE_TOL_RAD (≈ 0.1 rad)

MOVING   : publie linear.x = ±max(MIN_SPEED, Kp_lin × dist_au_centre)
           angular.z = Ka × (heading_cible - theta_actuel)   ← correction cap
           → fin quand dist_au_centre < ARRIVAL_TOL_MM
```

### Détection d'arrivée au centre de la case

```python
ARRIVAL_TOL_MM = 100   # ajustable selon précision odométrie

x_centre = col_cible * CELL_COL_MM + CELL_COL_MM / 2
y_centre = row_cible * CELL_ROW_MM + CELL_ROW_MM / 2

dist = sqrt((x - x_centre)**2 + (y - y_centre)**2)
arrived = dist < ARRIVAL_TOL_MM
```

---

### Mission complète — flux d'états haut niveau

```
IDLE
  └─ réception /rover/nav_goal [row, col] ──→ EXPLORING
       (reset grille + recalc priorités Chebyshev vers cible)

EXPLORING  (BFS prioritaire vers la zone cible — Phase 1)
  ├─ ArUco détecté (/aruco_detected found=1.0)
  │     → recalculer priorités Chebyshev vers la case nav_goal
  │     → continuer EXPLORING (BFS route naturellement vers ArUco)
  ├─ ArUco area >= AREA_NEAR  (Phase 2 : approche visuelle)
  │     → VISUAL_APPROACH  (TODO : asservissement ArUco centrage)
  └─ case nav_goal atteinte ──→ RETURNING

RETURNING  (BFS sur grille connue vers case de départ)
  └─ case départ atteinte ──→ DONE

DONE
  └─ stopper moteurs, publier grid_state final
```

> **Phase 1 → Phase 2** : le passage à l'approche visuelle est déclenché par la taille du marqueur ArUco dans l'image (`area` dans `/aruco_detected`). Dès que `area >= AREA_NEAR`, le rover est suffisamment proche pour que la camera guide la correction fine. `AREA_NEAR` est une constante à calibrer selon la résolution caméra.

> **Retour** : BFS sur la grille connue — plus court et plus fiable que rejouer le `path_stack` à l'envers.  
> Le `path_stack` reste réservé au backtracking AMBUSH pendant l'exploration.

---

### Améliorations

| # | Amélioration | Détail |
|---|-------------|--------|
| 1 | **Ralentissement à l'approche** | `linear.x = max(MIN_SPEED, Kp_lin × dist_au_centre)` — vitesse proportionnelle à la distance restante, évite l'overshoot sans vrai PID |
| 2 | **Correction angulaire pendant MOVING** | `angular.z = Ka × (heading_cible - theta_actuel)` calculé à chaque tick pendant MOVING — maintient le cap sans sous-état supplémentaire |
| 3 | **Retour par BFS** | Pour le chemin retour, relancer BFS sur la grille connue plutôt que d'inverser le `path_stack` (qui ne contient que les nouvelles cases, pas les transits) |

---

## 11. Référence — rover_gui.py (xplore_pub)

### Fonctions portables directement dans `autonomous_node.py`

| Fonction | Action |
|----------|--------|
| `_bfs_to_best_undiscovered(sr, sc)` | Copier quasi verbatim — code complet, propre, bien commenté |
| `_nav_step()` | Porter avec adaptations — logique des 3 cas (transit / direct / AMBUSH) |
| `_reset_grid()` | Porter directement |
| `_recalc_priorities()` | Porter directement |

### Constantes déjà alignées

```python
ROWS, COLS = 12, 8          # ✅ même convention que autonome.md
UNDISCOVERED, FREE, OBSTACLE, AMBUSH, CELL_BORDER = 0, 1, 2, 3, 4

# Positions par défaut (navigable area = rows 1..10, cols 1..6)
target = (1, 1)                      # haut-gauche
start  = (ROWS - 2, COLS - 2)       # bas-droite = (10, 6)
```

### Points à ne pas oublier à l'implémentation

**1. `_ambush_streak`** — compteur d'AMBUSH consécutifs (présent dans GUI mais inutilisé).  
Dans l'`autonomous_node`, l'utiliser pour détecter un rover bloqué en boucle infinie et déclencher un arrêt d'urgence ou une alerte opérateur.

```python
_ambush_streak: int = 0
AMBUSH_LIMIT = 5   # après N AMBUSH consécutifs → stopper et alerter

# Dans _nav_step, cas AMBUSH :
self._ambush_streak += 1
if self._ambush_streak >= AMBUSH_LIMIT:
    self.get_logger().error('Rover bloqué — arrêt autonome')
    self._stop()
```

**2. Edge case — transit annulé → `path_stack.pop()` obligatoire**

Quand un transit démarre, la position courante est pushée dans `path_stack` AVANT que le transit ne commence. Si on annule le transit (obstacle détecté dessus), il faut aussi pop ce dernier élément, sinon le backtracking sera décalé d'une case.

```python
def _on_new_obstacle(self, row, col):
    if any(r == row and c == col for r, c in self._transit_path):
        self._transit_path.clear()
        if self._path_stack:
            self._path_stack.pop()   # annuler le push fait au démarrage du transit
        self.get_logger().warn(f'Obstacle sur transit ({row},{col}) — replanification BFS')
```

---

## 12. Fichiers à créer / modifier

| Fichier | Action | État |
|---------|--------|------|
| `rover_xplore/rover_xplore/odometry_node.py` | Créer | ✅ Fait |
| `rover_xplore/rover_xplore/autonomous_node.py` | Créer | ✅ Fait |
| `rover_xplore/setup.py` | Ajouter entry points | ✅ Fait |
| `rover_xplore/package.xml` | Ajouter `<depend>std_srvs</depend>` | ✅ Fait |
| `rover_xplore/rover_xplore/mode_manager_node.py` | Ajouter `autonomous_node` dans `_MODE_MAP` | ✅ Fait |
| `rover_xplore/launch/rover.launch.py` | Ajouter `odometry_node` et `autonomous_node` | À faire |
| `xplore_pub/rover_gui.py` — `RosBridge/_RosNode` | Ajouter publisher `/rover/nav_goal`, subscribers `/rover/grid_pos` + `/rover/grid_state` | À faire |
| `xplore_pub/rover_gui.py` — `MapWidget` | Remplacer simulation BFS locale par monitoring ROS temps réel + bouton START envoie nav_goal | À faire |
| `xplore_pub/rover_gui.py` — `AutonomousPage` | Connecter signals ROS → MapWidget | À faire |
