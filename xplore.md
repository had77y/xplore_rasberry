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
| `rover_xplore/rover_xplore/mode_manager_node.py` | Créé — subscribe /rover/mode, log le mode reçu |
| `camera_server.py` (racine) | À créer — picamera2 HTTP server |
| `rover_xplore/rover_xplore/autonomous_node.py` | À créer |
| `rover_xplore/rover_xplore/teleop_node.py` | À créer — commandes moteurs depuis PC |
| `rover_xplore/rover_xplore/arm_node.py` | À créer — main robotique (défi 2) |
| `rover_xplore/rover_xplore/ultrasonic_node.py` | À créer |
| `rover_xplore/rover_xplore/imu_node.py` | À créer |
| `rover_xplore/rover_xplore/encoder_node.py` | À créer |

### Repo PC (`xplore_pub`)
| Fichier | État |
|---------|------|
| `rover_xplore_pub/rover_xplore_pub/video_viewer_node.py` | Prêt — compatible avec /camera/image_raw |
| `rover_xplore_pub/rover_xplore_pub/mode_selector_node.py` | Créé — clavier A/C → publie /rover/mode |

---

## Infrastructure

- Docker : `docker_humble_desktop/` — ROS 2 Humble Desktop (les deux repos)
- CI : GitHub Actions `docker_ci.yml`
- Branche active (rover) : `fix/camera-picamera2`

---

## Conventions

- Package rover : `rover_xplore` uniquement (pas `rover_commands`)
- Package PC : `rover_xplore_pub` uniquement (pas `rover_commands`)
- Commentaires en français, variables/fonctions en anglais

---

## Modes de fonctionnement

| Mode | Touche PC | Topic | Description |
|------|-----------|-------|-------------|
| `autonomous` | `A` | `/rover/mode` | Navigation IA autonome |
| `teleop` | `C` | `/rover/mode` | Commandé par l'opérateur (course FPV + défi bras) |

Flux : `mode_selector_node` (PC) → `/rover/mode` → `mode_manager_node` (Raspberry)

---

*Dernière mise à jour : 2026-04-10*
