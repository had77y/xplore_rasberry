# ══════════════════════════════════════════════════════════════════════════════
# aruco_node.py — Raspberry Pi
#
# RÔLE : détecter les marqueurs ArUco dans le flux vidéo de la caméra
#         et publier leur position pixel pour qu'autonomous_node puisse
#         naviguer vers eux.
#
# QU'EST-CE QU'UN MARKER ARUCO ?
#   Un tag papier imprimé (comme un QR code) avec un pattern de carrés noirs/blancs.
#   Il est posé sur ou près de la cible (bouteille d'eau).
#   OpenCV peut le détecter et donner sa position dans l'image (coordonnées pixel).
#
# PLACE DANS LE SYSTÈME :
#
#   [RPi] camera_node
#         ↓ publie /camera/image_raw (Image bgr8, 640×480)
#         ↓ SEULEMENT quand ce node est abonné (voir astuce subscription_count)
#   [RPi] aruco_node ← CE FICHIER
#         ├─ écoute /rover/mode → s'abonne/désabonne de /camera/image_raw
#         └─ publie /aruco_detected → [RPi] autonomous_node
#
# TOPIC PUBLIÉ :
#   /aruco_detected  (std_msgs/Float32MultiArray)
#   Format : [found, id, center_x, center_y, area]
#     found    : 1.0 si marker détecté, 0.0 sinon
#     id       : identifiant du marker (ex: 0, 1, 2...) — défini lors de l'impression
#     center_x : position horizontale du centre du marker dans l'image (en pixels, 0=gauche, 640=droite)
#     center_y : position verticale du centre dans l'image (en pixels, 0=haut, 480=bas)
#     area     : aire du marker en pixels² (plus grand = plus proche de la caméra)
#   Si rien détecté : [0.0, -1.0, 0.0, 0.0, 0.0]
#
# ASTUCE CLEF — SUBSCRIPTION DYNAMIQUE :
#   Ce node ne s'abonne à /camera/image_raw QUE quand le mode est 'autonomous'.
#   Conséquence : camera_node voit 0 subscribers hors autonomous → ne publie pas le RAW
#   → économie de ~27 MB/s de données et de CPU.
#   En mode 'race' ou 'arm' : aruco_node se désabonne → camera_node arrête le RAW.
#
# DICTIONNAIRE ARUCO :
#   DICT_4X4_50 (défaut) = 50 markers différents possibles avec une grille 4×4 de cases.
#   On peut changer via paramètre ROS2 :
#   ros2 run rover_xplore aruco_node --ros-args -p dictionary:=DICT_6X6_250
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import cv2

# Même QoS que camera_node : BEST_EFFORT + depth=1 = on veut toujours le frame le plus récent
VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DEFAULT_DICT = 'DICT_4X4_50'


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        # ── Paramètre ROS2 : dictionnaire ArUco ───────────────────────────────
        # Permet de changer le dictionnaire sans recompiler (via --ros-args -p dictionary:=...)
        self.declare_parameter('dictionary', DEFAULT_DICT)
        dict_name = self.get_parameter('dictionary').get_parameter_value().string_value

        # Récupère la constante OpenCV correspondante (ex: 'DICT_4X4_50' → cv2.aruco.DICT_4X4_50)
        dict_id = getattr(cv2.aruco, dict_name, None)
        if dict_id is None:
            self.get_logger().error(f'Dictionnaire ArUco inconnu: {dict_name}')
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')

        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

        # Compatibilité OpenCV : l'API ArUco a changé entre les versions.
        # >= 4.7 : nouvelle classe ArucoDetector (plus propre)
        # < 4.7  : ancienne fonction detectMarkers() en global
        if hasattr(cv2.aruco, 'ArucoDetector'):
            # OpenCV >= 4.7 : API objet
            parameters = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self._detect_fn = lambda gray: self._detector.detectMarkers(gray)
        else:
            # OpenCV < 4.7 : API globale (style ancien)
            parameters = cv2.aruco.DetectorParameters_create()
            self._detect_fn = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=parameters
            )
        # Dans les deux cas, _detect_fn(gray) retourne : (corners, ids, rejected)
        # corners : liste de tableaux (4 coins du marker en pixels)
        # ids     : tableau d'identifiants correspondants
        # rejected : markers "candidats" rejetés (non utilisés ici)

        # ── Subscription image — créée dynamiquement ──────────────────────────
        # Démarre à None → on crée/détruit selon le mode reçu sur /rover/mode
        self._sub_image = None
        self._marker_visible = False  # pour logger uniquement sur transition (pas à chaque frame)

        # Abonnement permanent au mode
        self.create_subscription(String, '/rover/mode', self._on_mode, 10)

        # Publisher des résultats de détection
        # → autonomous_node l'écoute pour savoir où est la cible
        self.pub = self.create_publisher(Float32MultiArray, '/aruco_detected', 10)

        self.get_logger().info(f'aruco_node démarré — dict={dict_name} — en attente mode autonomous')

    # ── Gestion du mode ───────────────────────────────────────────────────────

    def _on_mode(self, msg: String):
        """
        Reçoit le mode depuis /rover/mode (publié par le PC).
        - Si mode == 'autonomous' et qu'on n'est pas encore abonné → créer la subscription image
        - Si mode != 'autonomous' et qu'on est abonné → détruire la subscription image
        Créer/détruire la subscription change le subscription_count de camera_node,
        ce qui active/désactive la publication du RAW automatiquement.
        """
        if msg.data == 'autonomous' and self._sub_image is None:
            # On entre en mode autonomous → s'abonner à la caméra RAW
            self._sub_image = self.create_subscription(
                Image, '/camera/image_raw', self._on_image, VIDEO_QOS
            )
            self.get_logger().info('Mode autonomous — détection ArUco activée')

        elif msg.data != 'autonomous' and self._sub_image is not None:
            # On sort du mode autonomous → se désabonner de la caméra
            self.destroy_subscription(self._sub_image)
            self._sub_image = None
            # Publier une détection "vide" pour signaler à autonomous_node
            # qu'il n'y a plus de marker visible (reset de son état interne)
            out = Float32MultiArray()
            out.data = [0.0, -1.0, 0.0, 0.0, 0.0]
            self.pub.publish(out)
            self.get_logger().info(f'Mode {msg.data} — détection ArUco désactivée')

    # ── Traitement de chaque frame ────────────────────────────────────────────

    def _on_image(self, msg: Image):
        """
        Appelée à ~30 Hz quand camera_node publie un frame sur /camera/image_raw.
        1. Convertit le message ROS2 en tableau numpy (image BGR)
        2. Convertit en niveaux de gris (ArUco n'a pas besoin de couleur)
        3. Lance la détection ArUco
        4. Publie le résultat sur /aruco_detected
        """
        if msg.encoding != 'bgr8':
            # On s'attend à du BGR 8 bits (format OpenCV standard)
            # Si camera_node change de format, ce node loguera une erreur
            self.get_logger().warn(f'Encodage non géré: {msg.encoding} (attendu bgr8)')
            return

        # Reconstruction du tableau numpy depuis les données brutes du message ROS2
        # msg.data = bytes bruts | shape = (hauteur × largeur × 3 canaux)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        # Conversion en niveaux de gris : ArUco est basé sur les contrastes noir/blanc
        # → pas besoin de couleur, et le traitement est 3× plus rapide en gris
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Détection des markers ArUco dans l'image
        corners, ids, _ = self._detect_fn(gray)
        # corners : liste de N tableaux shape (1, 4, 2) — 4 coins (x,y) par marker
        # ids     : tableau shape (N, 1) — un ID par marker détecté
        # _       : rejected (ignoré)

        out = Float32MultiArray()

        if ids is None or len(ids) == 0:
            # Aucun marker détecté dans ce frame
            if self._marker_visible:
                # Transition : marker visible → perdu → on logue
                self.get_logger().info('ArUco perdu')
                self._marker_visible = False
            out.data = [0.0, -1.0, 0.0, 0.0, 0.0]  # [found=0, id=-1, cx=0, cy=0, area=0]

        else:
            # Un ou plusieurs markers détectés → garder le plus proche (plus grande aire)
            # L'aire en pixels² augmente quand on se rapproche du marker (perspective)
            areas = [cv2.contourArea(c[0]) for c in corners]
            # c[0] = tableau (4, 2) des 4 coins → contourArea calcule l'aire du quadrilatère
            idx = int(np.argmax(areas))  # index du marker avec la plus grande aire

            c = corners[idx][0]  # les 4 coins du marker choisi : shape (4, 2)
            # Centre du marker = moyenne des 4 coins en X et en Y
            cx = float(np.mean(c[:, 0]))  # moyenne des X des 4 coins
            cy = float(np.mean(c[:, 1]))  # moyenne des Y des 4 coins
            marker_id = int(ids[idx][0])

            if not self._marker_visible:
                # Transition : pas visible → visible → on logue
                self.get_logger().info(f'ArUco détecté ! ID={marker_id} centre=({cx:.0f},{cy:.0f})')
                self._marker_visible = True

            # Format publié : [found=1, id, cx_px, cy_px, area_px²]
            # autonomous_node interprète :
            #   cx < 320 → marker à gauche de l'image → tourner à gauche
            #   cx > 320 → marker à droite → tourner à droite
            #   area plus grande → rover plus proche du marker
            out.data = [1.0, float(marker_id), cx, cy, float(areas[idx])]

        # Publier le résultat → lu par autonomous_node sur le même RPi
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
