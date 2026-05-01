# Navigation Autonome — Architecture

---

## Contexte du défi

- **Map** : 8m × 5m, terrain plat
- **Bords** : barrières physiques (barreaux) + scotch au sol
- **Objectif** : aller de bas-droite (départ) à haut-gauche (cible ArUco), ramasser une bouteille, revenir
- **Obstacles** : 10–50cm de hauteur, placement aléatoire et secret
- **Restarts** : 2 autorisés mais remettent les points à zéro

---

## Grille de navigation

### Cellules principales (80 × 80 cm)

- 8m / 0.8 = **10 lignes**, 5m / 0.8 = **6 colonnes** → grille interne 10×6
- Entourée d'une rangée de cellules `BORDER` → **grille totale 12×8**
- Départ : `(row=10, col=6)` bas-droite (grille paddée)
- Cible : `(row=1, col=1)` haut-gauche (grille paddée)

```
col→  0  1  2  3  4  5  6  7
row↓
 0   [B][B][B][B][B][B][B][B]
 1   [B][T][ ][ ][ ][ ][ ][B]   T = cible (ArUco)
 2   [B][ ][ ][ ][ ][ ][ ][B]
 3   [B][ ][ ][ ][ ][ ][ ][B]
 4   [B][ ][ ][ ][ ][ ][ ][B]
 5   [B][ ][ ][ ][ ][ ][ ][B]
 6   [B][ ][ ][ ][ ][ ][ ][B]
 7   [B][ ][ ][ ][ ][ ][ ][B]
 8   [B][ ][ ][ ][ ][ ][ ][B]
 9   [B][ ][ ][ ][ ][ ][ ][B]
10   [B][ ][ ][ ][ ][ ][S][B]   S = départ
11   [B][B][B][B][B][B][B][B]
```

### Attributs d'une cellule

```python
@dataclass
class Cell:
    state: CellState      # BORDER | UNDISCOVERED | FREE | OBSTACLE | AMBUSH
    priority: int         # Chebyshev inversé depuis cible — plus grand = plus désirable
    mini: list[MiniState] # [TL, TR, BL, BR] — état des 4 sous-cases 40×40cm
```

### États possibles

| État | Description |
|------|-------------|
| `BORDER` | Hors-map, priorité = −1, jamais sélectionné |
| `UNDISCOVERED` | Pas encore visité ni scanné |
| `FREE` | Passage confirmé libre |
| `OBSTACLE` | Obstacle détecté par US |
| `AMBUSH` | Cul-de-sac vécu : le rover y est entré mais toutes les voisines étaient bloquées |

### Priorité

Calculée une fois à l'initialisation via distance de Chebyshev vers la cible :

```
priority(cell) = max_dist − chebyshev(cell, target)
```

- `max_dist` = chebyshev(départ_interne, cible_interne) = max(9, 5) = 9
- Cible : priorité = 9 (maximum)
- Départ : priorité = 0
- `BORDER` : priorité = −1

---

## Sous-cases 40 × 40 cm

Chaque cellule 80×80 est divisée en 4 mini-cases :

```
+----+----+
| TL | TR |
+----+----+
| BL | BR |
+----+----+
```

Chaque mini-case : `FREE | OBSTACLE`

### Règle de passage

**Mouvement droit** (haut / bas / gauche / droite) :
```
→ vers le haut :  TL + TR  (cellule courante)  doivent être FREE
                + BL + BR  (cellule cible)      doivent être FREE
```

**Mouvement diagonal** — 4 mini-cases au coin partagé entre les 4 cellules impliquées :
```
→ vers haut-gauche :  TL  (cellule courante)
                    + TR  (cellule à gauche)
                    + BR  (cellule diagonale = cible)
                    + BL  (cellule au-dessus)
                    → toutes FREE
```

---

## Capteurs US

| Capteur | Position | Direction relative au cap |
|---------|----------|--------------------------|
| US_FL | avant-gauche | −45° |
| US_FC | avant-centre | 0° |
| US_FR | avant-droite | +45° |
| US_L  | gauche | −90° |
| US_R  | droite | +90° |

- **Portée** : 4m — suffisant pour anticiper la cellule suivante (0.8m)
- Avant chaque déplacement, le rover pivote vers la direction cible → US frontaux couvrent la zone de passage
- Cône de détection à confirmer selon modèle (typique HC-SR04 : ±15°)

---

## Localisation

```
Encodeurs (4 roues) ──┐
                       ├──→ EKF (robot_localization) ──→ /odometry/filtered
IMU ───────────────────┘
```

- EKF fusionne encodeurs + IMU → position (x, y) + cap θ en continu
- Position (x, y) mappée sur cellule courante `(row, col)` à chaque update
- Rotations : multiples de 45° selon direction cible, contrôlées via IMU
- Translations : 80cm (droit) ou 113cm (diagonal), contrôlées via encodeurs

---

## Algorithme de navigation

### Sélection de la prochaine cellule

```python
def next_cell(current, grid):
    neighbors = get_8_neighbors(current, grid)
    candidates = [c for c in neighbors
                  if c.state not in (BORDER, OBSTACLE)
                  and passage_libre(current, c, grid)]
    if not candidates:
        return None  # backtrack
    # Préférer FREE > UNDISCOVERED > éviter AMBUSH sauf dernier recours
    return max(candidates, key=lambda c: (c.state != AMBUSH, c.priority))
```

### Boucle principale (greedy)

```
INIT:
  calculer priorities sur toute la grille
  current = départ
  stack_chemin = [départ]
  ambush_streak = 0

LOOP:
  next = next_cell(current, grid)

  si next == None:
    marquer current comme AMBUSH
    ambush_streak += 1
    current = stack_chemin.pop()          ← backtrack d'une cellule

    si ambush_streak >= 3:
      → déclencher A* (voir ci-dessous)
    continuer

  sinon:
    ambush_streak = 0
    pivoter vers next (IMU)
    avancer vers centre de next (encodeurs)
    scanner US → update mini-cases + états cellules adjacentes
    stack_chemin.append(next)
    current = next

  si current == cible:
    PHASE SUIVANTE
```

### Replanning A*

Déclenché quand `ambush_streak >= 3` :

```
path = astar(current, target, grid)
  heuristique : Chebyshev(cell, target)
  coût : 1 (droit) ou √2 (diagonal)
  cellules exclues : BORDER, OBSTACLE, AMBUSH

si path trouvé:
  suivre path case par case (mêmes règles de passage)
  reprendre greedy après avoir atteint la dernière case du path
  ambush_streak = 0

sinon:
  continuer greedy (pas de chemin connu, explorer davantage)
```

---

## Machine d'état — autonomous_node

```
[1] DETECT_ARUCO
    → caméra détecte l'ArUco depuis le départ
    → calcule bearing + distance estimée (taille du tag)
    → confirme position cible dans la grille

[2] NAVIGATE_GRID
    → algo greedy + A* ci-dessus
    → US scannent à chaque déplacement
    → EKF track la position en continu

[3] ARRIVE_ARUCO
    → recalage position absolue via ArUco (corrige drift EKF)
    → (Bonus) calcule position bouteille via offset x,y,z fourni
    → (Bonus) bras ramasse la bouteille

[4] NAVIGATE_RETURN
    → retour vers départ via grille déjà remplie (chemin aller connu)
    → US en double-check temps réel

[5] ARRIVEE_BASE
    → (Bonus) déposer bouteille → tilt benne
    → mission terminée
```

---

## Nodes ROS2

| Node | Machine | Rôle |
|------|---------|------|
| `camera_node` | RPi (natif) | flux caméra → `/camera/image_raw` |
| `aruco_node` | RPi (Docker) | détecte ArUco → `/aruco_detected` |
| `encoder_node` | RPi (Docker) | odométrie roues → `/wheel_odom` |
| `imu_node` | RPi (Docker) | données IMU → `/imu/data` |
| `ultrasonic_node` | RPi (Docker) | distances 5 US → `/us/distances` |
| `autonomous_node` | RPi (Docker) | grille + algo + machine d'état |
| `motor_controller_node` | RPi (Docker) | reçoit `/rover/cmd_vel` → serial Arduino |

---

## Topics principaux

| Topic | Type | De → Vers |
|-------|------|-----------|
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node → aruco_node |
| `/aruco_detected` | `std_msgs/Float32MultiArray` | aruco_node → autonomous_node |
| `/wheel_odom` | `nav_msgs/Odometry` | encoder_node → robot_localization |
| `/imu/data` | `sensor_msgs/Imu` | imu_node → robot_localization |
| `/odometry/filtered` | `nav_msgs/Odometry` | robot_localization → autonomous_node |
| `/us/distances` | `std_msgs/Float32MultiArray` | ultrasonic_node → autonomous_node |
| `/rover/cmd_vel` | `geometry_msgs/Twist` | autonomous_node → motor_controller_node |
| `/rover/grid_state` | `std_msgs/String` (JSON) | autonomous_node → rover_gui (PC) |

---

*Dernière mise à jour : 2026-05-01*
