#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2026 İDA - Bozulmaya Karşı Korumalı Otomatik Veri ve Video Kayıtçısı
================================================================================
- Gerçek zamanlı kamera kaydı ve ZED stereo depth tabanlı engel haritası kaydı (Dosya 1 ve Dosya 3).
- Her ARM işleminde yeni zaman damgalı oturum başlatılır.
- Her DISARM işleminde tüm dosyalar anında mühürlenip kapatılır.
- CSV her satırda flush+fsync edilir. MP4 dosyalarının geçerli kapanması için
  görev sonunda kontrollü DISARM/shutdown yapılmalıdır.
- 'atexit' ve 'signal' kancaları ile işletim sistemi seviyesinde dosya güvenliği sağlar.
"""

import atexit
import csv
import datetime
import math
import os
import time
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from mavros_msgs.msg import PositionTarget, State
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from std_msgs.msg import Float32, Int8
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quaternion_to_rpy(qx, qy, qz, qw):
  sinr_cosp = 2.0 * (qw * qx + qy * qz)
  cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
  roll = math.atan2(sinr_cosp, cosr_cosp)

  sinp = 2.0 * (qw * qy - qz * qx)
  if abs(sinp) >= 1:
    pitch = math.copysign(math.pi / 2, sinp)
  else:
    pitch = math.asin(sinp)

  siny_cosp = 2.0 * (qw * qz + qx * qy)
  cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
  yaw = math.atan2(siny_cosp, cosy_cosp)

  return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TeknofestDataLoggerNode(Node):

  def __init__(self):
    super().__init__('teknofest_data_logger_node')

    # --- DİNAMİK MASAÜSTÜ YOLU ---
    self.declare_parameter('log_dir', '')
    param_dir = self.get_parameter('log_dir').value

    if param_dir:
      self.kayit_klasoru = os.path.abspath(os.path.expanduser(param_dir))
    else:
      home_dir = os.path.expanduser('~')
      desktop_dir = os.path.join(home_dir, 'Desktop')
      if os.path.exists(desktop_dir):
        self.kayit_klasoru = os.path.join(desktop_dir, 'teknofest_kayitlari')
      else:
        self.kayit_klasoru = os.path.join(home_dir, 'teknofest_kayitlari')

    os.makedirs(self.kayit_klasoru, exist_ok=True)

    self.declare_parameter('video_fps', 10.0)
    self.declare_parameter('seal_on_stop', False)
    self.declare_parameter('map_publish_rate_hz', 2.0)
    self.fps = float(self.get_parameter('video_fps').value)
    self.seal_on_stop = bool(self.get_parameter('seal_on_stop').value)
    self.map_publish_rate_hz = max(
        0.2, float(self.get_parameter('map_publish_rate_hz').value)
    )
    self.last_map_publish_time = None

    # --- ZED 2 DERİNLİK KAMERASI TABANLI YEREL ENGEL HARİTASI ---
    self.declare_parameter(
        'depth_topic', '/zed/zed_node/depth/depth_registered'
    )
    self.declare_parameter(
        'camera_info_topic', '/zed/zed_node/rgb/camera_info'
    )
    self.declare_parameter('map_width_m', 12.0)
    self.declare_parameter('map_forward_m', 10.0)
    self.declare_parameter('map_resolution_m', 0.05)
    self.declare_parameter('min_depth_m', 0.40)
    self.declare_parameter('max_depth_m', 15.0)
    self.declare_parameter('depth_sample_step', 4)
    self.declare_parameter('map_decay', 0.88)
    self.declare_parameter('roi_top_ratio', 0.25)
    self.declare_parameter('roi_bottom_ratio', 0.92)

    self.depth_topic = str(self.get_parameter('depth_topic').value)
    self.camera_info_topic = str(
        self.get_parameter('camera_info_topic').value
    )
    self.map_width_m = float(self.get_parameter('map_width_m').value)
    self.map_forward_m = float(self.get_parameter('map_forward_m').value)
    self.map_resolution_m = float(
        self.get_parameter('map_resolution_m').value
    )
    self.min_depth_m = float(self.get_parameter('min_depth_m').value)
    self.max_depth_m = float(self.get_parameter('max_depth_m').value)
    self.depth_sample_step = max(
        1, int(self.get_parameter('depth_sample_step').value)
    )
    self.map_decay = float(self.get_parameter('map_decay').value)
    self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
    self.roi_bottom_ratio = float(
        self.get_parameter('roi_bottom_ratio').value
    )

    self.map_width_px = max(
        200, int(round(self.map_width_m / self.map_resolution_m))
    )
    self.map_forward_px = max(
        200, int(round(self.map_forward_m / self.map_resolution_m))
    )
    self.local_costmap = np.zeros(
        (self.map_forward_px, self.map_width_px), dtype=np.float32
    )

    # Kamera tabanlı yerel engel haritası. Bu SLAM değildir; hareketli
    # base/camera frame'inde anlık yerel occupancy haritasıdır.
    self.local_map_pub = self.create_publisher(
        OccupancyGrid, '/camera_local_costmap', 10
    )

    self.cam_fx = None
    self.cam_fy = None
    self.cam_cx = None
    self.cam_cy = None
    self.camera_info_received = False

    self.csv_dosyasi = None
    self.csv_yazici = None
    self.kamera_writer = None
    self.costmap_writer = None

    self.mp4_kamera_yolu = None
    self.mp4_costmap_yolu = None
    self.csv_dosya_yolu = None

    self.kayit_aktif = False
    self.dosyalar_muhurlendi = True

    # MP4 gerçek-zaman senkronizasyonu.
    # OpenCV VideoWriter sabit FPS ile çalışır. Kaynak görüntü daha yavaş
    # gelirse aynı kare gerektiği kadar tekrar yazılarak video süresi gerçek
    # görev süresiyle eşit tutulur; böylece 2x/3x hızlı oynatma oluşmaz.
    self.kayit_baslangic_monotonic = None
    self.kamera_yazilan_kare = 0
    self.costmap_yazilan_kare = 0
    self.son_kamera_frame = None
    self.son_costmap_frame = None

    # TF2 Dinleyicisi
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # Durum Değişkenleri
    self.lat = 0.0
    self.lon = 0.0
    self.hiz = 0.0
    self.roll = 0.0
    self.pitch = 0.0
    self.yaw = 0.0
    self.hiz_setpoint = 0.0
    self.yon_setpoint = 0.0
    self.onceki_armed = False
    self.surus_yapildi = False

    # --- ABONELİKLER ---
    # QoS profili Best Effort (sensor_data)
    self.create_subscription(
        PoseStamped,
        '/mavros/local_position/pose',
        self.pose_cb,
        qos_profile_sensor_data,
    )
    self.create_subscription(
        State, '/mavros/state', self.mavros_state_cb, 10
    )
    self.create_subscription(
        NavSatFix,
        '/mavros/global_position/global',
        self.gps_cb,
        qos_profile_sensor_data,
    )
    self.create_subscription(
        TwistStamped,
        '/mavros/local_position/velocity_body',
        self.velocity_cb,
        qos_profile_sensor_data,
    )
    self.create_subscription(
        Float32, '/desired_yaw', self.desired_yaw_cb, 10
    )
    self.create_subscription(
        PositionTarget, '/mavros/setpoint_raw/local', self.setpoint_cb, 10
    )
    self.create_subscription(
        Image, '/obstacle_debug_image', self.kamera_image_cb, 10
    )
    # Engel haritası doğrudan ZED 2 stereo depth verisinden üretilir.
    self.create_subscription(
        CameraInfo,
        self.camera_info_topic,
        self.camera_info_cb,
        qos_profile_sensor_data,
    )
    self.create_subscription(
        Image,
        self.depth_topic,
        self.depth_image_cb,
        qos_profile_sensor_data,
    )
    self.create_subscription(Int8, '/stop', self.stop_cb, 10)

    # 2 Hz CSV Kayıt Zamanlayıcısı
    self.timer = self.create_timer(0.5, self.telemetri_kaydet)

    # --- GÜVENLİK KANCALARI ---
    atexit.register(self.dosyalari_muhurle_ve_kapat)

    self.get_logger().info(
        '📊 ARM/DISARM Tetiklemeli Gerçek Zamanlı Kayıtçı Hazır! Klasör:'
        f' {self.kayit_klasoru}'
    )

  # ---------------- OTURUM YÖNETİMİ VE GÜVENLİ MÜHÜRLEME ----------------
  def _yeni_kayit_oturumu_baslat(self):
    """ARM olunca yeni zaman damgasıyla temiz dosyalar açar."""
    if self.kayit_aktif:
      self.dosyalari_muhurle_ve_kapat()

    zaman_damgasi = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    self.csv_dosya_yolu = os.path.join(
        self.kayit_klasoru, f'Dosya2_telemetri_{zaman_damgasi}.csv'
    )
    self.mp4_kamera_yolu = os.path.join(
        self.kayit_klasoru, f'Dosya1_kamera_islenmis_{zaman_damgasi}.mp4'
    )
    self.mp4_costmap_yolu = os.path.join(
        self.kayit_klasoru, f'Dosya3_engel_haritasi_{zaman_damgasi}.mp4'
    )

    try:
      self.csv_dosyasi = open(
          self.csv_dosya_yolu, mode='w', newline='', encoding='utf-8'
      )
      self.csv_yazici = csv.writer(self.csv_dosyasi)
      self.csv_yazici.writerow([
          'Zaman_Damgasi',
          'Enlem_Lat',
          'Boylam_Lon',
          'Yer_Hizi_m_s',
          'Roll_deg',
          'Pitch_deg',
          'Heading_deg',
          'Hiz_Setpoint_m_s',
          'Yon_Setpoint_deg',
      ])
      self.csv_dosyasi.flush()
      os.fsync(self.csv_dosyasi.fileno())
      self.kamera_writer = None
      self.costmap_writer = None

      self.kayit_baslangic_monotonic = time.monotonic()
      self.kamera_yazilan_kare = 0
      self.costmap_yazilan_kare = 0
      self.son_kamera_frame = None
      self.son_costmap_frame = None

      self.kayit_aktif = True
      self.dosyalar_muhurlendi = False
      self.get_logger().info(
          f'🟢 [YENİ KAYIT BAŞLATILDI] ARM edildi -> {zaman_damgasi}'
      )
    except Exception as e:
      self.get_logger().error(f'❌ Kayıt oturumu başlatılamadı: {e}')

  def _video_gercek_zamanli_yaz(self, writer, frame, counter_attr):
    """Sabit FPS MP4'ü gerçek duvar zamanı ile senkron tutar.

    Kaynak FPS, VideoWriter FPS'inden düşükse kare tekrarı yapar.
    Kaynak daha hızlıysa gerektiğinde kare atlar. Böylece dosya 2x/3x
    hızlı veya yavaş oynatılmaz.
    """
    if writer is None or frame is None:
      return

    if self.kayit_baslangic_monotonic is None:
      self.kayit_baslangic_monotonic = time.monotonic()

    elapsed = max(
        0.0, time.monotonic() - self.kayit_baslangic_monotonic
    )
    hedef_kare = max(1, int(math.floor(elapsed * self.fps)) + 1)
    yazilan = int(getattr(self, counter_attr))

    tekrar = hedef_kare - yazilan
    if tekrar <= 0:
      return

    # Çok uzun bir callback kesintisinde tek seferde sınırsız yazımı önle.
    # 5 saniyelik boşluğu aşan durumda yine gerçek zamana yaklaşır ancak
    # executor'ü gereksiz yere kilitlemez.
    maksimum_tekrar = max(1, int(round(self.fps * 5.0)))
    tekrar = min(tekrar, maksimum_tekrar)

    for _ in range(tekrar):
      writer.write(frame)

    setattr(self, counter_attr, yazilan + tekrar)

  def _video_suresini_tamamla(self):
    """DISARM anında son kareleri görev bitiş zamanına kadar uzatır."""
    if self.kayit_baslangic_monotonic is None:
      return

    if self.kamera_writer is not None and self.son_kamera_frame is not None:
      self._video_gercek_zamanli_yaz(
          self.kamera_writer,
          self.son_kamera_frame,
          'kamera_yazilan_kare',
      )

    if self.costmap_writer is not None and self.son_costmap_frame is not None:
      self._video_gercek_zamanli_yaz(
          self.costmap_writer,
          self.son_costmap_frame,
          'costmap_yazilan_kare',
      )

  def dosyalari_muhurle_ve_kapat(self):
    """Dosyaları bozulmadan güvenle diske yazar ve kapatır."""
    if not self.kayit_aktif or self.dosyalar_muhurlendi:
      return

    self.dosyalar_muhurlendi = True
    self.kayit_aktif = False
    self.get_logger().info(
        '🔒 [GÜVENLİ MÜHÜRLEME] DISARM tespit edildi. Dosyalar kapatılıyor...'
    )

    # MP4 sürelerini DISARM anına kadar gerçek zamanla eşitle.
    try:
      self._video_suresini_tamamla()
    except Exception as e:
      self.get_logger().error(
          f'Video süre tamamlama hatası: {type(e).__name__}: {e}'
      )

    try:
      if self.csv_dosyasi and not self.csv_dosyasi.closed:
        self.csv_dosyasi.flush()
        os.fsync(self.csv_dosyasi.fileno())
        self.csv_dosyasi.close()
        self.get_logger().info('✅ [Dosya 2] Telemetri CSV Başarıyla Mühürlendi.')
    except Exception as e:
      self.get_logger().error(f'CSV Kapatma Hatası: {e}')

    try:
      if self.kamera_writer is not None:
        self.kamera_writer.release()
        self.kamera_writer = None
        self.get_logger().info(
            '✅ [Dosya 1] Ön Kamera Videosu Başarıyla Mühürlendi.'
        )
    except Exception as e:
      self.get_logger().error(f'Kamera Video Kapatma Hatası: {e}')

    try:
      if self.costmap_writer is not None:
        self.costmap_writer.release()
        self.costmap_writer = None
        self.get_logger().info(
            '✅ [Dosya 3] Engel Haritası Videosu Başarıyla Mühürlendi.'
        )
    except Exception as e:
      self.get_logger().error(f'Costmap Video Kapatma Hatası: {e}')

    # Yazıcı açılmamış olsa bile eski kod koşulsuz "3 dosya hazır" diyordu.
    # Hakeme teslimden önce gerçekten oluşan ve veri içeren dosyaları doğrula.
    kontroller = (
        ('Dosya 1 kamera MP4', self.mp4_kamera_yolu, 1024),
        ('Dosya 2 telemetri CSV', self.csv_dosya_yolu, 128),
        ('Dosya 3 engel haritası MP4', self.mp4_costmap_yolu, 1024),
    )
    eksikler = []
    for ad, yol, minimum_bayt in kontroller:
      try:
        boyut = os.path.getsize(yol) if yol and os.path.isfile(yol) else 0
      except OSError:
        boyut = 0
      if boyut < minimum_bayt:
        eksikler.append(f'{ad} ({boyut} bayt)')

    if eksikler:
      self.get_logger().error(
          '❌ HAKEM TESLİM DOSYALARI EKSİK/BOŞ: ' + '; '.join(eksikler)
      )
    else:
      self.get_logger().info(
          '📁 TÜM 3 DOSYA HAKEMLERE TESLİM İÇİN HAZIR! Konum:'
          f' {self.kayit_klasoru}'
      )

  # ---------------- CALLBACKS ----------------
  def pose_cb(self, msg):
    self.roll, self.pitch, self.yaw = quaternion_to_rpy(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    )

  def mavros_state_cb(self, msg):
    """KUMANDADAN ARM EDİLDİĞİNDE YENİ KAYIT BAŞLAT, DISARM EDİLDİĞİNDE MÜHÜRLE"""
    if msg.armed and not self.onceki_armed:
      self.surus_yapildi = True
      self.get_logger().info('🔓 [PIXHAWK ARMED] Yeni kayıt oturumu başlatılıyor...')
      self._yeni_kayit_oturumu_baslat()
    elif not msg.armed and self.onceki_armed:
      self.get_logger().warn('🔒 [PIXHAWK DISARMED] Kayıt oturumu kapatılıyor ve mühürleniyor...')
      self.dosyalari_muhurle_ve_kapat()

    self.onceki_armed = msg.armed

  def stop_cb(self, msg):
    """Opsiyonel /stop mühürleme.

    /stop=0; sensör kaybı, rota bekleme veya güvenli duruş için de kullanılabildiği
    için varsayılan olarak kayıt kapatmaz. Asıl oturum sınırı ARM/DISARM'dır.
    """
    if (
        self.seal_on_stop
        and msg.data == 0
        and self.surus_yapildi
        and self.kayit_aktif
    ):
      self.get_logger().info(
          '🛑 [STOP MÜHÜRLEME] /stop=0 alındı. Dosyalar kapatılıyor...'
      )
      self.dosyalari_muhurle_ve_kapat()

  def gps_cb(self, msg):
    self.lat = msg.latitude
    self.lon = msg.longitude

  def velocity_cb(self, msg):
    # Gerçek yer hızı (Ground Speed m/s)
    self.hiz = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

  def desired_yaw_cb(self, msg):
    # Şartnamedeki "heading" alanını 0..360 derece biçiminde kaydet.
    self.yon_setpoint = math.degrees(msg.data) % 360.0

  def setpoint_cb(self, msg):
    vx = msg.velocity.x
    vy = msg.velocity.y
    # Geri vites setpointini pozitif hız gibi göstermemek için gövde-X
    # işaretini koru. Normal seyirde pozitif, geri viteste negatif olur.
    buyukluk = math.hypot(vx, vy)
    self.hiz_setpoint = -buyukluk if vx < 0.0 else buyukluk

  @staticmethod
  def _image_to_bgr(msg):
    """bgr8/rgb8/bgra8/rgba8 Image mesajını CvBridge olmadan dönüştürür."""
    if msg.height == 0 or msg.width == 0 or len(msg.data) == 0:
      raise ValueError('Boş image mesajı')

    encoding = str(msg.encoding).lower()
    if encoding in ('bgra8', 'rgba8'):
      channels = 4
    elif encoding in ('bgr8', 'rgb8'):
      channels = 3
    else:
      raise ValueError(f'Desteklenmeyen image encoding: {msg.encoding}')

    expected_row = int(msg.width) * channels
    if int(msg.step) < expected_row:
      raise ValueError(
          f'Geçersiz image step={msg.step}, beklenen>={expected_row}'
      )

    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        int(msg.height), int(msg.step)
    )
    frame = rows[:, :expected_row].reshape(
        int(msg.height), int(msg.width), channels
    )

    if encoding == 'bgra8':
      return np.ascontiguousarray(frame[:, :, :3])
    if encoding == 'rgba8':
      return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if encoding == 'rgb8':
      return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(frame)

  @staticmethod
  def _image_to_depth_m(msg):
    """32FC1 veya 16UC1/mono16 depth mesajını metre cinsine dönüştürür."""
    if msg.height == 0 or msg.width == 0 or len(msg.data) == 0:
      raise ValueError('Boş depth mesajı')

    encoding = str(msg.encoding).lower()

    if encoding == '32fc1':
      dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
      row_elems = int(msg.step) // 4
      if row_elems < int(msg.width):
        raise ValueError('32FC1 step/width uyumsuz')
      depth = np.frombuffer(msg.data, dtype=dtype).reshape(
          int(msg.height), row_elems
      )[:, : int(msg.width)]
      return np.ascontiguousarray(depth.astype(np.float32, copy=False))

    if encoding in ('16uc1', 'mono16'):
      dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
      row_elems = int(msg.step) // 2
      if row_elems < int(msg.width):
        raise ValueError('16UC1 step/width uyumsuz')
      depth_mm = np.frombuffer(msg.data, dtype=dtype).reshape(
          int(msg.height), row_elems
      )[:, : int(msg.width)]
      return np.ascontiguousarray(
          depth_mm.astype(np.float32) / 1000.0
      )

    raise ValueError(f'Desteklenmeyen depth encoding: {msg.encoding}')

  def _publish_local_occupancy_grid(self, stamp):
    """Depth tamponunu zed_camera_link frame'inde yerel OccupancyGrid olarak yayınlar."""
    now = self.get_clock().now()
    if self.last_map_publish_time is not None:
      age = (now - self.last_map_publish_time).nanoseconds / 1e9
      if age < (1.0 / self.map_publish_rate_hz):
        return
    self.last_map_publish_time = now

    binary = self.local_costmap > 110.0

    # local_costmap görüntü düzeni: [far->near, left(image)->right(image)].
    # ROS grid: x=forward, y=left, origin x=0/y=-half_width.
    # Eksenleri ters çevirip transpose ederek [y, x] matrisine dönüştür.
    occupied_yx = np.flip(binary, axis=(0, 1)).T
    grid_np = np.where(occupied_yx, 100, -1).astype(np.int8)

    grid = OccupancyGrid()
    grid.header.stamp = stamp
    grid.header.frame_id = 'zed_camera_link'
    grid.info.resolution = float(self.map_resolution_m)
    grid.info.width = int(self.map_forward_px)
    grid.info.height = int(self.map_width_px)
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = -float(self.map_width_m) * 0.5
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0
    grid.data = grid_np.reshape(-1).tolist()
    self.local_map_pub.publish(grid)

  def kamera_image_cb(self, msg):
    if not self.kayit_aktif or self.dosyalar_muhurlendi:
      return
    try:
      cv_frame = self._image_to_bgr(msg)
      h, w = cv_frame.shape[:2]

      if self.kamera_writer is None and self.mp4_kamera_yolu is not None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.kamera_writer = cv2.VideoWriter(
            self.mp4_kamera_yolu, fourcc, self.fps, (w, h)
        )
        if not self.kamera_writer.isOpened():
          self.get_logger().error(
              f'❌ DOSYA 1 VideoWriter AÇILAMADI: {self.mp4_kamera_yolu}'
          )
          self.kamera_writer = None
          return

      if self.kamera_writer is not None:
        zaman_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(
            cv_frame,
            f'TIME: {zaman_str}',
            (20, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        self.son_kamera_frame = cv_frame.copy()
        self._video_gercek_zamanli_yaz(
            self.kamera_writer,
            self.son_kamera_frame,
            'kamera_yazilan_kare',
        )
    except Exception as e:
      self.get_logger().error(
          f'❌ DOSYA 1 KAMERA KAYIT HATASI: {type(e).__name__}: {e}'
      )

  def _costmap_frame_yaz(self, cv_frame):
    """Üretilmiş engel haritası karesini Dosya 3 videosuna yazar."""
    if not self.kayit_aktif or self.dosyalar_muhurlendi:
      return

    try:
      h, w = cv_frame.shape[:2]

      if self.costmap_writer is None and self.mp4_costmap_yolu is not None:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.costmap_writer = cv2.VideoWriter(
            self.mp4_costmap_yolu, fourcc, self.fps, (w, h)
        )
        if not self.costmap_writer.isOpened():
          self.get_logger().error(
              f'❌ DOSYA 3 VideoWriter AÇILAMADI: {self.mp4_costmap_yolu}'
          )
          self.costmap_writer = None
          return

      if self.costmap_writer is not None:
        self.son_costmap_frame = cv_frame.copy()
        self._video_gercek_zamanli_yaz(
            self.costmap_writer,
            self.son_costmap_frame,
            'costmap_yazilan_kare',
        )

    except Exception as e:
      self.get_logger().error(
          f'❌ DOSYA 3 HARİTA KAYIT HATASI: {type(e).__name__}: {e}'
      )

  def camera_info_cb(self, msg):
    """ZED kamera iç parametrelerini alır."""
    if len(msg.k) < 9:
      return

    fx = float(msg.k[0])
    fy = float(msg.k[4])
    cx = float(msg.k[2])
    cy = float(msg.k[5])

    if fx <= 0.0 or fy <= 0.0:
      return

    first_time = not self.camera_info_received
    self.cam_fx = fx
    self.cam_fy = fy
    self.cam_cx = cx
    self.cam_cy = cy
    self.camera_info_received = True

    if first_time:
      self.get_logger().info(
          '✅ ZED CameraInfo alındı: '
          f'fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}'
      )

  def _depth_to_meters(self, depth):
    """ZED depth görüntüsünü metre cinsine getirir."""
    if depth.dtype == np.uint16:
      return depth.astype(np.float32) * 0.001

    depth = depth.astype(np.float32, copy=False)
    finite = depth[np.isfinite(depth)]

    if finite.size > 0:
      median_val = float(np.median(finite))
      if median_val > 100.0:
        depth = depth * 0.001

    return depth

  def depth_image_cb(self, msg):
    """ZED depth_registered görüntüsünden kuşbakışı yerel engel haritası üretir.

    Harita ARM'dan bağımsız sürekli güncellenir; video yazımı yalnızca kayıt aktifken yapılır.
    """
    if not self.camera_info_received:
      self.get_logger().warn(
          '⚠️ Harita için CameraInfo bekleniyor: '
          f'{self.camera_info_topic}',
          throttle_duration_sec=5.0,
      )
      return

    try:
      depth = self._image_to_depth_m(msg)

      if depth.ndim != 2:
        self.get_logger().warn(
            f'⚠️ Beklenmeyen depth boyutu: {depth.shape}',
            throttle_duration_sec=5.0,
        )
        return

      depth_m = depth
      h, w = depth_m.shape

      fx = self.cam_fx
      cx = self.cam_cx

      if fx is None or cx is None:
        return

      self.local_costmap *= self.map_decay

      v0 = int(max(0, min(h - 1, round(h * self.roi_top_ratio))))
      v1 = int(max(v0 + 1, min(h, round(h * self.roi_bottom_ratio))))
      step = self.depth_sample_step

      vv, uu = np.mgrid[v0:v1:step, 0:w:step]
      zz = depth_m[v0:v1:step, 0:w:step]

      valid = (
          np.isfinite(zz)
          & (zz >= self.min_depth_m)
          & (zz <= self.max_depth_m)
      )

      if not np.any(valid):
        frame = self._render_depth_costmap(
            valid_points=0,
            depth_shape=(h, w),
        )
        self._publish_local_occupancy_grid(msg.header.stamp)
        self._costmap_frame_yaz(frame)
        return

      z = zz[valid]
      u = uu[valid].astype(np.float32)

      x_right = (u - cx) * z / fx

      half_width = self.map_width_m * 0.5
      in_map = (
          (x_right >= -half_width)
          & (x_right <= half_width)
          & (z >= 0.0)
          & (z <= self.map_forward_m)
      )

      x_right = x_right[in_map]
      z = z[in_map]

      if z.size:
        col = (
            (x_right + half_width) / self.map_resolution_m
        ).astype(np.int32)

        row = (
            (self.map_forward_m - z) / self.map_resolution_m
        ).astype(np.int32)

        inside = (
            (col >= 0)
            & (col < self.map_width_px)
            & (row >= 0)
            & (row < self.map_forward_px)
        )

        col = col[inside]
        row = row[inside]

        self.local_costmap[row, col] = 255.0

        binary = (self.local_costmap > 110.0).astype(np.uint8) * 255
        kernel = np.ones((3, 3), dtype=np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=1)
        self.local_costmap = np.maximum(
            self.local_costmap, dilated.astype(np.float32)
        )

      frame = self._render_depth_costmap(
          valid_points=int(z.size),
          depth_shape=(h, w),
      )
      self._publish_local_occupancy_grid(msg.header.stamp)
      self._costmap_frame_yaz(frame)

    except Exception as e:
      self.get_logger().error(
          f'❌ ZED DEPTH HARİTALANDIRMA HATASI: '
          f'{type(e).__name__}: {e}'
      )

  def _render_depth_costmap(self, valid_points, depth_shape):
    """Occupancy tamponunu okunabilir BGR görüntüsüne dönüştürür."""
    gray = np.clip(self.local_costmap, 0, 255).astype(np.uint8)
    cv_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    hmap, wmap = gray.shape
    vehicle_x = wmap // 2
    vehicle_y = hmap - 8

    one_meter_px = max(
        1, int(round(1.0 / self.map_resolution_m))
    )

    for x in range(0, wmap, one_meter_px):
      cv2.line(
          cv_frame, (x, 0), (x, hmap - 1), (40, 40, 40), 1
      )

    for y in range(0, hmap, one_meter_px):
      cv2.line(
          cv_frame, (0, y), (wmap - 1, y), (40, 40, 40), 1
      )

    cv2.circle(
        cv_frame, (vehicle_x, vehicle_y), 6, (0, 255, 255), -1
    )
    cv2.arrowedLine(
        cv_frame,
        (vehicle_x, vehicle_y),
        (vehicle_x, max(15, vehicle_y - 55)),
        (0, 255, 0),
        2,
        tipLength=0.25,
    )

    for dist_m in (5.0, 10.0):
      if dist_m < self.map_forward_m:
        y = int(
            round(
                (self.map_forward_m - dist_m)
                / self.map_resolution_m
            )
        )
        if 0 <= y < hmap:
          cv2.line(
              cv_frame, (0, y), (wmap - 1, y), (80, 80, 80), 1
          )
          cv2.putText(
              cv_frame,
              f'{dist_m:.0f}m',
              (5, max(15, y - 3)),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.4,
              (180, 180, 180),
              1,
          )

    zaman_str = datetime.datetime.now().strftime(
        '%Y-%m-%d %H:%M:%S.%f'
    )[:-3]

    cv2.putText(
        cv_frame,
        f'TIME: {zaman_str}',
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (200, 200, 200),
        1,
    )
    cv2.putText(
        cv_frame,
        f'ZED DEPTH MAP | points={valid_points}',
        (10, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (200, 200, 200),
        1,
    )
    cv2.putText(
        cv_frame,
        f'depth={depth_shape[1]}x{depth_shape[0]} '
        f'| range={self.min_depth_m:.1f}-{self.max_depth_m:.1f}m',
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (200, 200, 200),
        1,
    )
    cv2.putText(
        cv_frame,
        'FRONT',
        (vehicle_x - 25, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
    )

    return cv_frame

  def telemetri_kaydet(self):
    if not self.kayit_aktif or self.dosyalar_muhurlendi or self.csv_dosyasi is None:
      return

    try:
      t = self.tf_buffer.lookup_transform(
          'map', 'base_link', rclpy.time.Time()
      )
      self.roll, self.pitch, self.yaw = quaternion_to_rpy(
          t.transform.rotation.x,
          t.transform.rotation.y,
          t.transform.rotation.z,
          t.transform.rotation.w,
      )
    except TransformException:
      pass

    simdi_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    try:
      self.csv_yazici.writerow([
          simdi_str,
          f'{self.lat:.7f}',
          f'{self.lon:.7f}',
          f'{self.hiz:.2f}',
          f'{self.roll:.2f}',
          f'{self.pitch:.2f}',
          f'{self.yaw % 360.0:.2f}',
          f'{self.hiz_setpoint:.2f}',
          f'{self.yon_setpoint:.2f}',
      ])
      self.csv_dosyasi.flush()
      os.fsync(
          self.csv_dosyasi.fileno()
      )  # RAM tamponunu beklemeden doğrudan SSD/Flash diske yaz
    except Exception as e:
      self.get_logger().error(
          f'❌ TELEMETRİ CSV YAZMA HATASI: {type(e).__name__}: {e}',
          throttle_duration_sec=2.0,
      )

  def destroy_node(self):
    self.dosyalari_muhurle_ve_kapat()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = TeknofestDataLoggerNode()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, SystemExit):
    pass
  finally:
    node.dosyalari_muhurle_ve_kapat()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
