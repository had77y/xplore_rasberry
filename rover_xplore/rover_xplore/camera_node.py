# ══════════════════════════════════════════════════════════════════════════════
# camera_node.py — Raspberry Pi (natif, hors Docker)
#
# RÔLE : capturer les frames de la Pi Camera (CSI) et les publier sur deux
#         topics ROS2 selon les besoins des abonnés.
#
# POURQUOI HORS DOCKER : libcamera (le driver de la Pi Camera) ne fonctionne
#   pas dans un container Docker. Ce node tourne donc directement sur le RPi.
#   Les autres nodes (dans Docker avec --net=host) voient ses topics via DDS.
#
# PLACE DANS LE SYSTÈME :
#
#   [PC] controller_node / rover_gui
#         ↓ publie /rover/mode
#   [RPi] camera_node ← CE FICHIER
#         ├─ écoute /rover/mode → active/désactive la capture
#         ├─ publie /camera/image_compressed → [PC] rover_gui (affichage vidéo)
#         └─ publie /camera/image_raw        → [RPi] aruco_node (détection marker)
#
# DEUX TOPICS PUBLIÉS :
#
#   /camera/image_compressed  (sensor_msgs/CompressedImage, JPEG q80)
#     → Actif en modes : race, autonomous, arm
#     → Destinataire : rover_gui.py sur le PC (affichage du flux FPV)
#     → Taille : ~1-2 MB/s sur WiFi (JPEG compressé → économe en bande passante)
#
#   /camera/image_raw  (sensor_msgs/Image, bgr8, 640×480)
#     → Actif UNIQUEMENT si aruco_node est abonné (c-à-d seulement en mode autonomous)
#     → Destinataire : aruco_node (qui tourne aussi sur le RPi, reste local)
#     → Taille : 640×480×3 = ~900 KB/frame → ne passe PAS bien sur WiFi
#     → Astuce : on vérifie get_subscription_count() avant de publier
#                Si personne n'écoute → on ne publie pas → 0% CPU/bande passante
#
# GATING PAR MODE :
#   idle / arm hors liste ACTIVE_MODES → le timer tourne mais capture_and_publish() retourne immédiatement
#   → 0% CPU caméra quand inutile
#
# DEUX BACKENDS CAMÉRA :
#   libcamera (picamera2) → utilisé si une caméra CSI est détectée (Pi Camera)
#   V4L2 (cv2.VideoCapture) → fallback pour caméra USB ou tests sur VM Linux
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import cv2

# Import conditionnel : picamera2 n'existe que sur Raspberry Pi avec libcamera installé.
# Sur une VM ou un Mac, cet import échoue → on tombe sur le backend V4L2.
try:
    from picamera2 import Picamera2
    LIBCAMERA_AVAILABLE = True
except ImportError:
    LIBCAMERA_AVAILABLE = False

# ── Paramètres de capture ──────────────────────────────────────────────────────
FRAME_W = 640
FRAME_H = 480
TARGET_FPS = 30
WARMUP_FRAMES = TARGET_FPS  # ~1 seconde de "chauffe" pour laisser l'AEC/AWB converger
                              # (auto-exposition et auto-balance des blancs de la caméra)
JPEG_QUALITY = 80             # 80 = bon compromis qualité/taille pour le WiFi

# Modes où la caméra est active et publie des frames.
# arm ajouté pour afficher le flux vidéo pendant le contrôle du bras.
ACTIVE_MODES = {'race', 'autonomous', 'arm'}

# QoS vidéo : BEST_EFFORT = on abandonne les vieux frames si le réseau est lent.
# Opposé de RELIABLE qui re-transmettrait → accumulation de lag.
# depth=1 = on ne garde qu'un seul frame en file d'attente → toujours le plus récent.
VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Indicateur : est-ce qu'on doit publier des frames ?
        # Passe à True seulement quand le mode reçu est dans ACTIVE_MODES.
        self._active = False

        # image brute BGR8 → vers aruco_node (local RPi, ~27 MB/s, uniquement si abonné)
        self.pub_raw  = self.create_publisher(Image,            '/camera/image_raw',        VIDEO_QOS)
        # image compressée JPEG → vers le PC (via WiFi, ~1-2 MB/s)
        self.pub_jpeg = self.create_publisher(CompressedImage,  '/camera/image_compressed', VIDEO_QOS)

        # Abonnement au mode : quand l'opérateur change le mode depuis le PC,
        # ce callback met à jour self._active.
        self.create_subscription(String, '/rover/mode', self._on_mode, 10)

        # Détection du backend caméra disponible.
        # Picamera2.global_camera_info() retourne la liste des caméras CSI détectées.
        libcamera_cameras = Picamera2.global_camera_info() if LIBCAMERA_AVAILABLE else []
        if libcamera_cameras:
            # Pi Camera CSI connectée → utiliser libcamera (qualité max, 30 FPS stable)
            self._init_libcamera()
        else:
            if LIBCAMERA_AVAILABLE:
                # libcamera est installé mais aucune caméra physique détectée
                # → câble CSI débranché ou dtoverlay manquant dans /boot/config.txt
                self.get_logger().warn(
                    'libcamera disponible mais aucune caméra CSI détectée '
                    '(vérifier câble CSI / dtoverlay) — fallback V4L2'
                )
            # Fallback : caméra USB ou test sur VM
            self._init_v4l2()

        # Timer principal : appelle capture_and_publish() à 30 Hz.
        # Si self._active == False, la fonction retourne immédiatement → pas de CPU.
        self.timer = self.create_timer(1.0 / TARGET_FPS, self.capture_and_publish)
        backend = 'libcamera' if libcamera_cameras else 'V4L2'
        self.get_logger().info(
            f'camera_node démarré — {backend} {FRAME_W}x{FRAME_H} @ {TARGET_FPS} FPS '
            f'(JPEG q{JPEG_QUALITY}) — en attente du mode...'
        )

    # ── Réception du mode ──────────────────────────────────────────────────────

    def _on_mode(self, msg: String):
        """
        Appelée quand /rover/mode reçoit un nouveau message (depuis le PC).
        Active ou désactive la publication des frames selon le mode.
        """
        was_active = self._active
        # On est actif uniquement si le mode est dans la liste ACTIVE_MODES
        self._active = msg.data in ACTIVE_MODES

        if self._active and not was_active:
            self.get_logger().info(f'Mode {msg.data} — caméra activée')
        elif not self._active and was_active:
            self.get_logger().info(f'Mode {msg.data} — caméra désactivée')

    # ── Initialisation des backends caméra ────────────────────────────────────

    def _init_libcamera(self):
        """
        Initialise la Pi Camera via picamera2 (API Python de libcamera).
        Format RGB888 : numpy retourne du BGR (ordre OpenCV) malgré le nom.
        FrameDurationLimits : force exactement 30 FPS (en microsecondes : 1_000_000/30).
        """
        self.cam = Picamera2()
        frame_duration_us = int(1_000_000 / TARGET_FPS)
        cfg = self.cam.create_video_configuration(
            main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
            controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
        )
        self.cam.configure(cfg)
        self.cam.start()
        # Drain des premières frames : les capteurs d'exposition/balance ont besoin
        # d'environ 1 seconde pour converger. Sans ça, les premières frames sont
        # sur/sous-exposées ou avec des couleurs fausses.
        for _ in range(WARMUP_FRAMES):
            self.cam.capture_array("main")
        # On stocke la fonction de capture dans un attribut → même interface que V4L2
        self.capture_fn = lambda: self.cam.capture_array("main")

    def _init_v4l2(self):
        """
        Fallback : scan automatique de /dev/video0 à /dev/video31 pour trouver
        une caméra USB ou virtuelle compatible V4L2.
        MJPG : force le format MJPEG côté caméra pour réduire la bande passante USB.
        """
        self.cap = None
        for idx in range(32):
            path = f'/dev/video{idx}'
            cap = cv2.VideoCapture(path, cv2.CAP_V4L)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
                ret, _ = cap.read()  # test : est-ce qu'on peut vraiment lire ?
                if ret:
                    self.cap = cap
                    self.get_logger().info(f'V4L2: utilise {path} (MJPG)')
                    break
                cap.release()
            else:
                cap.release()
        if self.cap is None:
            self.get_logger().error('Aucune caméra V4L2 disponible !')
            raise RuntimeError('Camera introuvable')
        self.capture_fn = self._capture_v4l2

    def _capture_v4l2(self):
        ret, frame = self.cap.read()
        return frame if ret else None  # retourne None si la lecture échoue

    # ── Boucle principale (30 Hz) ──────────────────────────────────────────────

    def capture_and_publish(self):
        """
        Appelée par le timer à 30 Hz.
        Si le mode n'est pas actif → retour immédiat (0% CPU caméra).
        Sinon : capture un frame, encode en JPEG, publie les deux topics.
        """
        if not self._active:
            return  # mode idle ou non supporté → rien à faire

        frame = self.capture_fn()  # numpy array BGR, shape (480, 640, 3)
        if frame is None:
            self.get_logger().warn('Frame vide — skip')
            return

        stamp = self.get_clock().now().to_msg()  # timestamp ROS2 (nécessaire dans le header)

        # ── Publication JPEG (priorité 1) ──────────────────────────────────────
        # On encode en JPEG avant le RAW pour minimiser la latence du viewer PC.
        # Le JPEG part en premier sur le réseau → l'opérateur voit l'image le plus vite possible.
        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ret:
            self.get_logger().warn('Échec encodage JPEG — skip')
            return
        jpeg = CompressedImage()
        jpeg.header.stamp = stamp
        jpeg.header.frame_id = 'camera'
        jpeg.format = 'jpeg'
        jpeg.data = buf.tobytes()
        self.pub_jpeg.publish(jpeg)
        # → reçu par rover_gui.py (PC) via WiFi → affiché dans la fenêtre caméra

        # ── Publication RAW (uniquement si quelqu'un écoute) ──────────────────
        # get_subscription_count() retourne le nombre de nodes abonnés à /camera/image_raw.
        # En mode 'autonomous', aruco_node s'abonne → count > 0 → on publie.
        # En mode 'race' ou 'arm', personne n'est abonné → count == 0 → on ne publie pas.
        # Avantage : évite de générer 27 MB/s de données inutiles.
        if self.pub_raw.get_subscription_count() > 0:
            raw = Image()
            raw.header.stamp = stamp
            raw.header.frame_id = 'camera'
            raw.height = FRAME_H
            raw.width  = FRAME_W
            raw.encoding = 'bgr8'   # OpenCV BGR, 8 bits par canal
            raw.is_bigendian = 0
            raw.step = FRAME_W * 3  # nombre d'octets par ligne = largeur × 3 canaux
            raw.data = frame.tobytes()
            self.pub_raw.publish(raw)
            # → reçu par aruco_node (RPi, même machine) → détection du marker ArUco

    # ── Nettoyage à l'arrêt ────────────────────────────────────────────────────

    def destroy_node(self):
        """Arrête proprement la caméra avant de détruire le node."""
        if LIBCAMERA_AVAILABLE and hasattr(self, 'cam'):
            self.cam.stop()
        elif hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)  # bloque ici, traite les callbacks ROS2
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
