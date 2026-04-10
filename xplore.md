# Xplore Rover — Mémoire Projet (Claude)

Ce fichier est mis à jour automatiquement par Claude pour garder le contexte entre sessions.

---

## Architecture générale

- **Cible hardware** : Raspberry Pi 4, dans Docker ROS 2 Humble
- **Package actif** : `rover_xplore` (ne pas utiliser `rover_commands`)
- **Repo complémentaire** : un second repo tourne sur PC/VM Ubuntu 24 (aussi Docker ROS 2) pour la communication avec le rover

---

## Stratégie de navigation autonome

### Détection d'obstacles
- **Couche sécurité** : 3 capteurs ultrasoniques (US) — stoppe/évite en cas de danger immédiat
- **Détection principale** : ML via caméra (pas d'US comme détecteur principal)

### Localisation & Retour à la base
- **Odométrie** : encodeurs moteurs + IMU pour position précise
- **Retour** : navigation backwards via odométrie vers (0, 0) — run it backwards

### Capteurs
| Capteur | Topic ROS2 | Rôle |
|---------|-----------|------|
| Caméra Pi | `/camera/image_raw` | Détection ML obstacles + ArUco |
| 3× US | `/distances` | Sécurité (couche basse) |
| Encodeurs moteurs | TBD | Odométrie précise |
| IMU | TBD | Odométrie précise |

---

## Problèmes connus

### Camera — frames noires
- **Symptôme** : `picamera2` avec `create_still_configuration` → frames noires en continu
- **Cause probable** : `still_configuration` n'est pas fait pour le streaming continu (AEC/AWB pas convergé)
- **Fix appliqué** (branche `fix/camera-picamera2`) :
  - Utiliser `create_video_configuration` à la place
  - `time.sleep(2.0)` après `camera.start()` pour laisser converger l'AEC/AWB
  - 5 frames de warmup pour vider le buffer de démarrage
  - `capture_array("main")` avec le nom du stream explicite
- **Statut** : à tester sur hardware RPi

---

## État du code

### `rover_xplore/rover_xplore/camera_node.py`
- Branche `fix/camera-picamera2` : picamera2 avec video config + warmup
- Branche `master` : image synthétique (fallback de test)

### `rover_xplore/rover_xplore/` — à créer
- `autonomous_node.py` — machine à états navigation (inspiré de `rover_commands` mais refondu)
- `ultrasonic_node.py` — lecture 3 US (couche sécurité)
- `imu_node.py` — lecture IMU
- `encoder_node.py` — lecture encodeurs moteurs
- `aruco_node.py` — détection ArUco
- `ml_detection_node.py` — détection obstacles par ML

---

## Infrastructure

- Docker : `docker_humble_desktop/` — ROS 2 Humble Desktop
- CI : GitHub Actions `docker_ci.yml`
- Branche courante active : `fix/camera-picamera2`

---

## Conventions

- Langue du code : commentaires en français, variables/fonctions en anglais
- Package à utiliser : `rover_xplore` uniquement
- Pas de `rover_commands` pour les nouveaux développements

---

*Dernière mise à jour : 2026-04-10*
