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

## Défis du rover

| Défi | Mode | Description |
|------|------|-------------|
| Course FPV | `teleop` | L'opérateur pilote en regardant le flux caméra |
| Ramassage d'objets | `teleop` | Bras robotique commandé à distance, dépôt à la base |
| Collaboratif | TBD | Avec d'autres rovers — specs pas encore connues |
| Navigation autonome | `autonomous` | Le rover navigue seul sans intervention humaine |

---

## Stratégie de navigation autonome

### Détection d'obstacles
- **Couche sécurité** : 3 capteurs ultrasoniques (US) — stoppe/évite en cas de danger immédiat
- **Détection principale** : ML via caméra (pas d'US comme détecteur principal)

### Localisation & Retour à la base
- **Odométrie** : encodeurs moteurs + IMU pour position précise
- **Retour** : navigation backwards via odométrie vers (0, 0)

### Capteurs
| Capteur | Topic ROS2 | Rôle |
|---------|-----------|------|
| Caméra Pi | `/camera/image_raw` | Détection ML obstacles + ArUco |
| 3× US | `/distances` | Sécurité (couche basse) |
| Encodeurs moteurs | TBD | Odométrie précise |
| IMU | TBD | Odométrie précise |

---

## Modes de fonctionnement

| Mode | Touche PC | Topic | Description |
|------|-----------|-------|-------------|
| `autonomous` | `A` | `/rover/mode` | Navigation autonome |
| `teleop` | `C` | `/rover/mode` | Commandé par l'opérateur (course FPV + défi bras) |

Flux : `mode_selector_node` (PC) → `/rover/mode` → `mode_manager_node` (Raspberry)

---

## Téléopération — architecture

### Topics
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/rover/mode` | `std_msgs/String` | PC → RPi | Sélection du mode |
| `/rover/cmd_vel` | `geometry_msgs/Twist` | PC → RPi | Commandes de déplacement |

### Contrôle clavier (teleop_node)
- Flèches du clavier — **hold-to-move** : maintenir = avance, relâcher = stop
- `↑/↓` → `linear.x` (avant/arrière)
- `←/→` → `angular.z` (rotation)
- `↑ + ←` = virage en arc, `←` seul = pivot sur place
- `Espace` = stop immédiat
- Mécanisme : key repeat OS (~30ms) + timeout par axe (200ms) — si plus de répétitions → axe remis à 0

### Gestion de la vitesse (à implémenter)
- Touches `1/2/3` pour changer le multiplicateur de vitesse pendant le roulage
- Option A : `1/2/3` changent linear ET angular ensemble (intuitif pour la course)
- Option B : linear et angular indépendants (précision pour le défi bras)
- **À décider** : mode course → tout ensemble, mode bras → séparé ?

### Sécurité réseau
- `teleop_receiver_node` stoppe les moteurs si aucune commande reçue depuis 500ms

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
| `rover_xplore/rover_xplore/mode_manager_node.py` | Prêt — subscribe /rover/mode, dispatche selon le mode |
| `rover_xplore/rover_xplore/teleop_receiver_node.py` | Prêt — reçoit Twist, log mouvements, timeout sécurité 500ms |
| `camera_server.py` (racine) | À créer — picamera2 HTTP server |
| `rover_xplore/rover_xplore/autonomous_node.py` | À créer |
| `rover_xplore/rover_xplore/arm_node.py` | À créer — main robotique (défi 2) |
| `rover_xplore/rover_xplore/ultrasonic_node.py` | À créer |
| `rover_xplore/rover_xplore/imu_node.py` | À créer |
| `rover_xplore/rover_xplore/encoder_node.py` | À créer |

### Repo PC (`xplore_pub`)
| Fichier | État |
|---------|------|
| `rover_xplore_pub/rover_xplore_pub/video_viewer_node.py` | Prêt — compatible avec /camera/image_raw |
| `rover_xplore_pub/rover_xplore_pub/mode_selector_node.py` | Prêt — clavier A/C → publie /rover/mode |
| `rover_xplore_pub/rover_xplore_pub/teleop_node.py` | Prêt — flèches clavier → publie /rover/cmd_vel |

---

## Infrastructure

- Docker : `docker_humble_desktop/` — ROS 2 Humble Desktop (les deux repos)
- CI : GitHub Actions `docker_ci.yml`
- Branches actives : `feat/mode-selection` (les deux repos), `fix/camera-picamera2` (rover uniquement)

---

## Conventions

- Package rover : `rover_xplore` uniquement (pas `rover_commands`)
- Package PC : `rover_xplore_pub` uniquement (pas `rover_commands`)
- Commentaires en français, variables/fonctions en anglais

---

*Dernière mise à jour : 2026-04-10*
