# Améliorations — Xplore Rover

## 1. `_transition_sync` — état incohérent après timeout

**Fichier :** `rover_xplore/mode_manager_node.py:182`

**Problème :** quand une transition lifecycle timeout, la fonction retourne `False` et logue une erreur, mais `_node_active[name]` n'est pas mis à jour. Le `mode_manager` croit que le node est encore dans son ancien état. Au prochain changement de mode, il ne tente pas de le réactiver (ou désactiver) car il pense que c'est déjà fait.

**Fix — ajouter des clients `GetState` et une réconciliation au début de chaque `_apply_mode` :**

```python
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State

# Dans __init__, après les _lc_clients :
self._gs_clients: dict[str, GetState.Client] = {
    name: self.create_client(GetState, f'/{name}/get_state')
    for name in _MANAGED_NODES
}
```

```python
def _reconcile_states(self):
    """Resynchronise _node_active avec l'état lifecycle réel de chaque node."""
    for name in _MANAGED_NODES:
        client = self._gs_clients[name]
        if not client.service_is_ready():
            continue
        future = client.call_async(GetState.Request())
        deadline = time.monotonic() + _TRANSITION_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if future.done() and future.result() is not None:
            is_active = (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            )
            if self._node_active[name] != is_active:
                self.get_logger().warn(
                    f'[{name}] état corrigé : '
                    f'{"actif" if is_active else "inactif"}'
                )
            self._node_active[name] = is_active
```

```python
def _apply_mode(self, mode: str):
    with self._transition_lock:
        self._reconcile_states()   # ← ajouter ici avant toute transition
        self.get_logger().info(f'Changement de mode : {self._current_mode!r} → {mode!r}')
        ...
```

**Pourquoi ça règle le problème :** avant chaque changement de mode, le `mode_manager` interroge l'état réel de chaque node via le service `GetState` au lieu de faire confiance à sa mémoire interne. Si un timeout avait corrompu l'état, il est corrigé avant les transitions suivantes.

---

## 2. `autonomous_node` — odométrie non finalisée

**Fichier :** `rover_xplore/autonomous_node.py`

**Problème :** `autonomous_node` écoute `/rover/pose` pour connaître sa position, mais ce topic dépend d'un `odometry_node` basé sur les encodeurs roues. Si l'odométrie dérive (ce qui arrive toujours sur terrain réel), la navigation se désaligne progressivement par rapport à la grille physique.

**Améliorations à prévoir :**
- Fusionner les encodeurs roues (`/wheel_encoders`) avec l'IMU (`/imu/raw`) pour corriger la dérive angulaire
- Recaler la pose sur détection ArUco (le marker est à une position connue dans la grille)
- Valider les constantes `CELL_ROW_MM`, `CELL_COL_MM` sur le terrain réel

---

## 4. Synchronisation série ESP32 ↔ Pi ✅ RÉSOLU

**Fichier :** `rover_xplore/serial_bridge_node.py`

**Problème initial :** pas de framing. Si l'alignement était perdu (reset ESP32, bruit UART), `struct.unpack` décodait silencieusement des valeurs corrompues.

**Solution implémentée (2026-05-18) :** magic word `"rover"` (5 octets ASCII) en tête de chaque trame dans les **deux sens**.

- Choix de `"rover"` : les bytes `r/o/v/e/r` (0x72/0x6F/0x76/0x65/0x72) sont dans la plage 0x65..0x9B, **mathématiquement impossible** dans les données Pi→ESP32 (toutes clampées à -100..100).
- **Pi** (`_send_struct`) : préfixe `b'rover'` avant chaque struct. (`_recv_struct`) : `rfind(b'rover')` dans le buffer — prend toujours la trame la plus récente.
- **ESP32** : state machine octet par octet — scanne `rover` avant d'accepter les 18 octets de commande. Répond avec `rover` + 30 octets capteurs uniquement sur trame valide.

Resync automatique en ≤ 1 frame (20ms) sans intervention manuelle.

---

## 3. IMU non exploité

**Fichier :** `rover_xplore/serial_bridge_node.py`

**Problème :** l'IMU est publié brut sur `/imu/raw` (accéléromètre + gyroscope) mais n'est consommé par aucun node. C'est une donnée disponible qui pourrait améliorer l'estimation de cap dans `autonomous_node`.

**Amélioration :** intégrer le gyroscope (axe Z) dans l'odométrie pour corriger la dérive angulaire des encodeurs, notamment sur sol glissant ou lors de rotations rapides.
