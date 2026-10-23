#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2026 İDA - ZED + YOLO Engel Kaçma / Kamikaze Düğümü

Güvenlik ilkeleri:
- RGB/depth/model/CameraInfo hazır değilse araç kör şekilde devam etmez.
- RGB/depth akışı bayatlarsa /stop=0 yayınlanır.
- ZED CameraInfo iç parametreleri kullanılır; sabit/hatalı fx ile geometri kurulmaz.
- Depth ölçüminde tek piksele güvenilmez; median ve minimum geçerli piksel sayısı kullanılır.
- Kamikaze son yaklaşım süresi parametrelidir; eski 20 saniyelik kör sürüş kaldırılmıştır.
"""

import math
import os

import cv2
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


def quaternion_to_yaw(qx, qy, qz, qw):
    t3 = 2.0 * (qw * qz + qx * qy)
    t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(t3, t4)


class ZedYoloObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('zed_yolo_obstacle_avoidance_node')

        # ---------------- PARAMETRELER ----------------
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('conf_threshold', 0.45)
        self.declare_parameter('device', '')
        self.declare_parameter('carpilacak_hedef', '')

        self.declare_parameter('guvenli_mesafe', 1.5)
        self.declare_parameter('geri_vites_tetik_mesafe', 0.5)
        self.declare_parameter('koridor_varsayilan_yaricap', 2.5)
        self.declare_parameter('pruva_koridor_yaricap', 0.60)

        self.declare_parameter('perception_timeout_sec', 0.8)
        self.declare_parameter('guidance_timeout_sec', 1.0)
        self.declare_parameter('rgb_depth_max_skew_sec', 0.35)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('min_valid_depth_pixels', 5)

        # ZED 2 min depth 0.3 m olduğu için 0.15 m eşiği erişilemezdi.
        self.declare_parameter('final_charge_trigger_m', 0.55)
        self.declare_parameter('final_charge_duration_sec', 1.0)

        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.device_param = str(self.get_parameter('device').value)
        self.hedef_renk = self._renk_normalize_et(
            self.get_parameter('carpilacak_hedef').value
        )

        self.guvenli_mesafe = float(self.get_parameter('guvenli_mesafe').value)
        self.geri_vites_tetik = float(
            self.get_parameter('geri_vites_tetik_mesafe').value
        )
        self.r_koridor = float(
            self.get_parameter('koridor_varsayilan_yaricap').value
        )
        self.pruva_koridor_yaricap = float(
            self.get_parameter('pruva_koridor_yaricap').value
        )

        self.perception_timeout_sec = float(
            self.get_parameter('perception_timeout_sec').value
        )
        self.guidance_timeout_sec = float(
            self.get_parameter('guidance_timeout_sec').value
        )
        self.rgb_depth_max_skew_sec = float(
            self.get_parameter('rgb_depth_max_skew_sec').value
        )
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.min_valid_depth_pixels = max(
            1, int(self.get_parameter('min_valid_depth_pixels').value)
        )
        self.final_charge_trigger_m = float(
            self.get_parameter('final_charge_trigger_m').value
        )
        self.final_charge_duration_sec = float(
            self.get_parameter('final_charge_duration_sec').value
        )

        # ---------------- YOLO MODEL ----------------
        self.model = None
        self.model_device = 'cpu'

        if not YOLO_AVAILABLE:
            self.get_logger().error(
                '❌ ultralytics/YOLO import edilemedi. Engel algılama güvenli STOP durumunda kalacak.'
            )
        else:
            try:
                bulunan_model_yolu = self.model_path
                if not os.path.exists(bulunan_model_yolu):
                    kod_dizini_model = os.path.join(
                        os.path.dirname(os.path.abspath(__file__)),
                        self.model_path,
                    )
                    desktop_model = os.path.join(
                        os.path.expanduser('~'), 'Desktop', self.model_path
                    )
                    if os.path.exists(kod_dizini_model):
                        bulunan_model_yolu = kod_dizini_model
                    elif os.path.exists(desktop_model):
                        bulunan_model_yolu = desktop_model

                if not os.path.exists(bulunan_model_yolu):
                    raise FileNotFoundError(
                        f'YOLO modeli bulunamadı: {bulunan_model_yolu}'
                    )

                secilen_device = (
                    self.device_param
                    if self.device_param
                    else (0 if torch.cuda.is_available() else 'cpu')
                )
                self.model = YOLO(bulunan_model_yolu)
                self.model_device = secilen_device
                self.get_logger().info(
                    f'✅ YOLO model yüklendi: {bulunan_model_yolu} | device={secilen_device}'
                )
            except Exception as e:
                self.model = None
                self.get_logger().error(
                    f'❌ YOLO MODEL YÜKLEME HATASI: {type(e).__name__}: {e}'
                )

        # ---------------- ALGILAMA DURUMU ----------------
        self.cv_image = None
        self.depth_image = None
        self.last_rgb_time = None
        self.last_depth_time = None

        # ZED sensör mesajlarının gerçek capture timestamp'leri.
        self.last_rgb_stamp_sec = None
        self.last_depth_stamp_sec = None

        self.cam_fx = None
        self.cam_cx = None
        self.camera_info_received = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Guidance girişleri
        self.raw_desired_yaw = 0.0
        self.raw_stop = 0
        self.raw_rotate = False
        self.kamikaze_aktif = False
        self.last_raw_yaw_time = None
        self.last_raw_stop_time = None

        # Kamikaze / arama durumu
        self.tarama_modu = False
        self.tarama_guncel_yaw = None
        self.tarama_hizi = math.radians(25.0)

        self.live_tekne_yaw = 0.0
        self.pose_alindi = False
        self.last_pose_time = None

        self.geri_vites_aktif = False
        self.geri_vites_baslangic = None

        self.carpma_gerceklesti = False
        self.carpma_zamani = None
        self.son_hedef_mesafe = None
        self.son_hedef_gorulme_zamani = None
        self.son_hedef_x = None
        self.son_hedef_dunya_yaw = None
        self.son_hamle_aktif = False
        self.son_hamle_baslangic = None

        # ---------------- ABONELİKLER ----------------
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32, '/desired_yaw_raw', self.raw_yaw_cb, 10
        )
        self.create_subscription(
            Int8, '/stop_raw', self.raw_stop_cb, 10
        )
        self.create_subscription(
            Bool, '/rotate_in_place_raw', self.raw_rotate_cb, 10
        )
        self.create_subscription(
            Bool, '/kamikaze_mode', self.kamikaze_mode_cb, 10
        )
        self.create_subscription(
            String, '/carpilacakhedef', self.carpilacak_hedef_cb, 10
        )

        # SensorData QoS, RELIABLE veya BEST_EFFORT kamera yayıncıları ile uyumludur.
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.rgb_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        # ---------------- YAYINCILAR ----------------
        self.pub_desired_yaw = self.create_publisher(
            Float32, '/desired_yaw', 10
        )
        self.pub_stop = self.create_publisher(Int8, '/stop', 10)
        self.pub_rotate = self.create_publisher(
            Bool, '/rotate_in_place', 10
        )
        self.pub_reverse = self.create_publisher(
            Bool, '/reverse_mode', 10
        )
        self.pub_debug_img = self.create_publisher(
            Image, '/obstacle_debug_image', 10
        )
        self.pub_costmap_img = self.create_publisher(
            Image, '/costmap_debug_image', 10
        )

        self.timer = self.create_timer(0.1, self.karar_dongusu)
        self.get_logger().info(
            '🧠 ZED + YOLO engel kaçma düğümü güvenli watchdog ile başlatıldı.'
        )

    # ------------------------------------------------------------------
    # Yardımcılar
    # ------------------------------------------------------------------
    def _renk_normalize_et(self, raw_renk):
        if not raw_renk:
            return None

        r = str(raw_renk).strip().lower()

        # Haberleşme protokolü ile aynı numerik eşleme:
        # 1=SIYAH, 2=KIRMIZI, 3=YESIL
        if r in ['siyah', 'black', 'ral9005', '1']:
            return 'siyah'
        if r in ['kirmizi', 'kırmızı', 'red', 'ral3026', '2']:
            return 'kirmizi'
        if r in ['yesil', 'yeşil', 'green', 'ral6037', '3']:
            return 'yesil'
        return None

    def _sinif_kategorize_et(self, sinif_adi):
        s = str(sinif_adi).lower()
        if any(k in s for k in ['turuncu', 'orange', 'kenar', 'boundary']):
            return 'turuncu'
        if any(k in s for k in ['sari', 'sarı', 'yellow', 'engel']):
            return 'sari'
        if any(k in s for k in ['kirmizi', 'kırmızı', 'red']):
            return 'kirmizi'
        if any(k in s for k in ['yesil', 'yeşil', 'green']):
            return 'yesil'
        if any(k in s for k in ['siyah', 'black']):
            return 'siyah'
        return 'bilinmeyen'

    def _reset_kamikaze_state(self):
        self.tarama_modu = False
        self.tarama_guncel_yaw = None
        self.geri_vites_aktif = False
        self.geri_vites_baslangic = None
        self.carpma_gerceklesti = False
        self.carpma_zamani = None
        self.son_hedef_mesafe = None
        self.son_hedef_gorulme_zamani = None
        self.son_hedef_x = None
        self.son_hedef_dunya_yaw = None
        self.son_hamle_aktif = False
        self.son_hamle_baslangic = None

    def _publish_outputs(
        self,
        desired_yaw,
        stop,
        rotate=False,
        reverse=False,
    ):
        self.pub_desired_yaw.publish(
            Float32(data=float(desired_yaw))
        )
        self.pub_stop.publish(Int8(data=int(stop)))
        self.pub_rotate.publish(Bool(data=bool(rotate)))
        self.pub_reverse.publish(Bool(data=bool(reverse)))

    def _guvenli_stop_yayinla(self):
        # /stop semantiği: 0 = motor hareket komutu verme.
        self._publish_outputs(
            desired_yaw=self.raw_desired_yaw,
            stop=0,
            rotate=False,
            reverse=False,
        )

    @staticmethod
    def _sure_saniye(simdi, once):
        if once is None:
            return float('inf')
        return max(0.0, (simdi - once).nanoseconds / 1e9)

    @staticmethod
    def _header_stamp_saniye(msg):
        try:
            sec = int(msg.header.stamp.sec)
            nsec = int(msg.header.stamp.nanosec)
            if sec == 0 and nsec == 0:
                return None
            return float(sec) + float(nsec) * 1e-9
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Callback'ler
    # ------------------------------------------------------------------
    def pose_cb(self, msg):
        self.live_tekne_yaw = quaternion_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        self.pose_alindi = True
        self.last_pose_time = self.get_clock().now()

    def camera_info_cb(self, msg):
        if len(msg.k) < 9:
            return

        fx = float(msg.k[0])
        cx = float(msg.k[2])
        if fx <= 0.0:
            return

        first = not self.camera_info_received
        self.cam_fx = fx
        self.cam_cx = cx
        self.camera_info_received = True

        if first:
            self.get_logger().info(
                f'✅ ZED CameraInfo: fx={fx:.2f}, cx={cx:.2f}'
            )

    def carpilacak_hedef_cb(self, msg):
        yeni = self._renk_normalize_et(msg.data)
        if yeni is None:
            self.get_logger().warn(
                f'⚠️ Geçersiz hedef renk mesajı: {msg.data!r}',
                throttle_duration_sec=2.0,
            )
            return

        if yeni != self.hedef_renk:
            self.hedef_renk = yeni
            # Yeni hedef geldiğinde eski hedefe ait final yaklaşım hafızası silinsin.
            self._reset_kamikaze_state()
            self.get_logger().info(
                f'🎯 Hedef renk güncellendi: {self.hedef_renk}'
            )

    def kamikaze_mode_cb(self, msg):
        yeni = bool(msg.data)
        if yeni and not self.kamikaze_aktif:
            self._reset_kamikaze_state()
            self.get_logger().warn('🚀 KAMİKAZE MODU AKTİF')
        elif not yeni and self.kamikaze_aktif:
            self._reset_kamikaze_state()
        self.kamikaze_aktif = yeni

    def raw_yaw_cb(self, msg):
        self.raw_desired_yaw = float(msg.data)
        self.last_raw_yaw_time = self.get_clock().now()

    def raw_stop_cb(self, msg):
        self.raw_stop = int(msg.data)
        self.last_raw_stop_time = self.get_clock().now()

    def raw_rotate_cb(self, msg):
        self.raw_rotate = bool(msg.data)

    def rgb_cb(self, msg):
        """sensor_msgs/Image -> BGR NumPy. Satır padding'i (step) dikkate alınır."""
        try:
            if msg.height == 0 or msg.width == 0 or len(msg.data) == 0:
                return

            encoding = str(msg.encoding).lower()

            if encoding in ('bgra8', 'rgba8'):
                channels = 4
            elif encoding in ('bgr8', 'rgb8'):
                channels = 3
            else:
                self.get_logger().error(
                    f'❌ Desteklenmeyen RGB encoding: {msg.encoding}',
                    throttle_duration_sec=2.0,
                )
                return

            expected_row = int(msg.width) * channels
            if int(msg.step) < expected_row:
                raise ValueError(
                    f'RGB step geçersiz: step={msg.step}, beklenen>={expected_row}'
                )

            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                int(msg.height), int(msg.step)
            )
            frame = rows[:, :expected_row].reshape(
                int(msg.height), int(msg.width), channels
            )

            if encoding == 'bgra8':
                bgr = frame[:, :, :3]
            elif encoding == 'rgba8':
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif encoding == 'rgb8':
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                bgr = frame

            self.cv_image = np.ascontiguousarray(bgr)
            self.last_rgb_time = self.get_clock().now()
            self.last_rgb_stamp_sec = self._header_stamp_saniye(msg)

        except Exception as e:
            self.get_logger().error(
                f'❌ RGB CALLBACK HATASI: {type(e).__name__}: {e}',
                throttle_duration_sec=2.0,
            )

    def depth_cb(self, msg):
        """sensor_msgs/Image -> float32 metre depth. Satır padding'i dikkate alınır."""
        try:
            if msg.height == 0 or msg.width == 0 or len(msg.data) == 0:
                return

            encoding = str(msg.encoding).lower()

            if encoding == '32fc1':
                dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
                row_elems = int(msg.step) // 4
                if row_elems < int(msg.width):
                    raise ValueError(
                        f'DEPTH step geçersiz: step={msg.step}, width={msg.width}'
                    )
                depth = np.frombuffer(msg.data, dtype=dtype).reshape(
                    int(msg.height), row_elems
                )[:, : int(msg.width)]
                depth = depth.astype(np.float32, copy=False)

            elif encoding in ('16uc1', 'mono16'):
                dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
                row_elems = int(msg.step) // 2
                if row_elems < int(msg.width):
                    raise ValueError(
                        f'DEPTH step geçersiz: step={msg.step}, width={msg.width}'
                    )
                depth_mm = np.frombuffer(msg.data, dtype=dtype).reshape(
                    int(msg.height), row_elems
                )[:, : int(msg.width)]
                depth = depth_mm.astype(np.float32) / 1000.0

            else:
                self.get_logger().error(
                    f'❌ Desteklenmeyen DEPTH encoding: {msg.encoding}',
                    throttle_duration_sec=2.0,
                )
                return

            self.depth_image = np.ascontiguousarray(depth)
            self.last_depth_time = self.get_clock().now()
            self.last_depth_stamp_sec = self._header_stamp_saniye(msg)

        except Exception as e:
            self.get_logger().error(
                f'❌ DEPTH CALLBACK HATASI: {type(e).__name__}: {e}',
                throttle_duration_sec=2.0,
            )

    # ------------------------------------------------------------------
    # Algılama / Geometri
    # ------------------------------------------------------------------
    def yolo_ile_tespit_et(self, frame):
        tespit_sonuclari = {
            'turuncu': [],
            'sari': [],
            'kirmizi': [],
            'yesil': [],
            'siyah': [],
        }
        if self.model is None or frame is None:
            return None, frame

        debug_frame = frame.copy()

        try:
            results = self.model(
                frame,
                conf=self.conf_threshold,
                device=self.model_device,
                verbose=False,
            )[0]
        except Exception as e:
            self.get_logger().error(
                f'❌ YOLO INFERENCE HATASI: {type(e).__name__}: {e}',
                throttle_duration_sec=2.0,
            )
            return None, debug_frame

        fh, fw = frame.shape[:2]

        # RGB CameraInfo ile gerçek kamera intrinsiği kullanılır.
        if self.camera_info_received and self.cam_fx and self.cam_fx > 0.0:
            fx = float(self.cam_fx)
            principal_x = float(self.cam_cx)
        else:
            return None, debug_frame

        boxes = (
            results.boxes.xyxy.cpu().numpy()
            if results.boxes is not None
            else []
        )
        classes = (
            results.boxes.cls.cpu().numpy()
            if results.boxes is not None
            else []
        )
        confs = (
            results.boxes.conf.cpu().numpy()
            if results.boxes is not None
            else []
        )

        for box, cls_id, conf in zip(boxes, classes, confs):
            x1, y1, x2, y2 = map(int, box)
            raw_class_name = self.model.names[int(cls_id)]
            kategori = self._sinif_kategorize_et(raw_class_name)

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            if self.depth_image is None:
                continue

            dh, dw = self.depth_image.shape[:2]
            depth_cx = int(cx * (dw / float(fw)))
            depth_cy = int(cy * (dh / float(fh)))

            depth_cx = min(max(depth_cx, 0), dw - 1)
            depth_cy = min(max(depth_cy, 0), dh - 1)

            py1, py2 = max(0, depth_cy - 3), min(dh, depth_cy + 4)
            px1, px2 = max(0, depth_cx - 3), min(dw, depth_cx + 4)

            derinlik_penceresi = self.depth_image[py1:py2, px1:px2]
            gecerli = derinlik_penceresi[
                np.isfinite(derinlik_penceresi)
                & (derinlik_penceresi >= 0.30)
                & (derinlik_penceresi <= self.max_depth_m)
            ]

            if gecerli.size < self.min_valid_depth_pixels:
                continue

            z_m = float(np.median(gecerli))
            if not math.isfinite(z_m):
                continue

            # RGB ve registered depth aynı yayın çözünürlüğünde olmalı; yine de
            # farklı çözünürlük halinde principal point/fx ölçeklenir.
            scale_x = dw / float(fw)
            fx_depth = fx * scale_x
            principal_depth_x = principal_x * scale_x
            x_m = (depth_cx - principal_depth_x) * z_m / fx_depth

            if kategori in tespit_sonuclari:
                tespit_sonuclari[kategori].append({
                    'x_m': float(x_m),
                    'z_m': z_m,
                    'bbox': (x1, y1, x2 - x1, y2 - y1),
                    'center': (cx, cy),
                    'conf': float(conf),
                })

            if kategori == 'sari':
                color = (0, 255, 255)
            elif kategori == 'turuncu':
                color = (0, 165, 255)
            elif kategori == 'yesil':
                color = (0, 255, 0)
            elif kategori == 'siyah':
                color = (80, 80, 80)
            else:
                color = (0, 0, 255)

            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                debug_frame,
                f'{kategori} {z_m:.2f}m',
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

        return tespit_sonuclari, debug_frame

    def costmap_2d_uret(self, tespitler, hedef_x, hedef_z):
        map_size = 600
        costmap = np.full((map_size, map_size, 3), 20, dtype=np.uint8)
        center_x = map_size // 2
        boat_y = map_size - 60
        scale = 45.0

        for r_m in [2, 4, 6, 8, 10]:
            cv2.circle(
                costmap,
                (center_x, boat_y),
                int(r_m * scale),
                (45, 45, 45),
                1,
            )

        tekne_pts = np.array(
            [
                [center_x, boat_y - 18],
                [center_x - 10, boat_y + 12],
                [center_x + 10, boat_y + 12],
            ],
            np.int32,
        )
        cv2.fillPoly(costmap, [tekne_pts], (255, 200, 0))

        for kategori, color in (
            ('sari', (0, 255, 255)),
            ('turuncu', (0, 165, 255)),
        ):
            for duba in tespitler.get(kategori, []):
                px = int(center_x + duba['x_m'] * scale)
                py = int(boat_y - duba['z_m'] * scale)
                if 0 <= px < map_size and 0 <= py < map_size:
                    cv2.circle(costmap, (px, py), 9, color, -1)

        if hedef_x is not None and hedef_z is not None:
            tgt_px = int(center_x + hedef_x * scale)
            tgt_py = int(boat_y - hedef_z * scale)
            cv2.arrowedLine(
                costmap,
                (center_x, boat_y),
                (tgt_px, tgt_py),
                (0, 255, 0),
                2,
                tipLength=0.2,
            )

        return costmap

    def geometrik_guvenli_hedef_hesapla(
        self,
        turuncu_dubalar,
        sari_dubalar,
    ):
        sol_turuncular = [d for d in turuncu_dubalar if d['x_m'] < 0]
        sag_turuncular = [d for d in turuncu_dubalar if d['x_m'] > 0]

        sol_sinir_x = min(
            [d['x_m'] for d in sol_turuncular],
            key=lambda x: abs(x),
            default=-self.r_koridor,
        )
        sag_sinir_x = min(
            [d['x_m'] for d in sag_turuncular],
            key=lambda x: abs(x),
            default=self.r_koridor,
        )

        aktif_sarilar = [
            d
            for d in sari_dubalar
            if 0.5 < d['z_m'] < self.guvenli_mesafe
        ]

        if not aktif_sarilar:
            return None

        en_yakin_z = min(d['z_m'] for d in aktif_sarilar)
        yakin_sira_sarilar = [
            d for d in aktif_sarilar
            if d['z_m'] <= en_yakin_z + 1.0
        ]

        sol_sari_x = min(d['x_m'] for d in yakin_sira_sarilar)
        sag_sari_x = max(d['x_m'] for d in yakin_sira_sarilar)

        sol_gecit_genislik = sol_sari_x - sol_sinir_x
        sag_gecit_genislik = sag_sinir_x - sag_sari_x

        if (
            sag_gecit_genislik >= sol_gecit_genislik
            and sag_gecit_genislik > 0.90
        ):
            hedef_x = (sag_sari_x + sag_sinir_x) / 2.0
        elif sol_gecit_genislik > 0.90:
            hedef_x = (sol_sinir_x + sol_sari_x) / 2.0
        else:
            # İki tarafta da açık güvenli geçit görünmüyorsa sınırların
            # geometrik merkezine yönel; geri-vites watchdog'u çok yakın
            # engelde ayrıca devreye girer.
            hedef_x = (sol_sinir_x + sag_sinir_x) / 2.0

        # Kamera X'i sağa pozitif, ROS yaw sola/CCW pozitiftir.
        hedef_aci = math.atan2(-hedef_x, en_yakin_z)
        return hedef_aci, hedef_x, en_yakin_z

    # ------------------------------------------------------------------
    # Debug görüntü yayını
    # ------------------------------------------------------------------
    def _debug_goruntuleri_yayinla(self, debug_frame, costmap_frame):
        try:
            debug_frame = np.ascontiguousarray(
                debug_frame, dtype=np.uint8
            )
            costmap_frame = np.ascontiguousarray(
                costmap_frame, dtype=np.uint8
            )

            now_msg = self.get_clock().now().to_msg()

            debug_msg = Image()
            debug_msg.header.stamp = now_msg
            debug_msg.header.frame_id = 'zed_camera_link'
            debug_msg.height = int(debug_frame.shape[0])
            debug_msg.width = int(debug_frame.shape[1])
            debug_msg.encoding = 'bgr8'
            debug_msg.is_bigendian = False
            debug_msg.step = int(debug_frame.shape[1] * 3)
            debug_msg.data = debug_frame.tobytes()

            costmap_msg = Image()
            costmap_msg.header.stamp = now_msg
            costmap_msg.header.frame_id = 'base_link'
            costmap_msg.height = int(costmap_frame.shape[0])
            costmap_msg.width = int(costmap_frame.shape[1])
            costmap_msg.encoding = 'bgr8'
            costmap_msg.is_bigendian = False
            costmap_msg.step = int(costmap_frame.shape[1] * 3)
            costmap_msg.data = costmap_frame.tobytes()

            self.pub_debug_img.publish(debug_msg)
            self.pub_costmap_img.publish(costmap_msg)

        except Exception as e:
            self.get_logger().error(
                f'❌ DEBUG IMAGE PUBLISH HATASI: {type(e).__name__}: {e}',
                throttle_duration_sec=2.0,
            )

    # ------------------------------------------------------------------
    # Ana karar döngüsü
    # ------------------------------------------------------------------
    def karar_dongusu(self):
        simdi = self.get_clock().now()

        rgb_yasi = self._sure_saniye(simdi, self.last_rgb_time)
        depth_yasi = self._sure_saniye(simdi, self.last_depth_time)

        algi_hazir = (
            self.cv_image is not None
            and self.depth_image is not None
            and self.model is not None
            and self.camera_info_received
        )
        algi_taze = (
            rgb_yasi <= self.perception_timeout_sec
            and depth_yasi <= self.perception_timeout_sec
        )

        # RGB/depth senkron farkını callback geliş saatinden değil,
        # ZED capture timestamp'lerinden hesapla.
        if (
            self.last_rgb_stamp_sec is not None
            and self.last_depth_stamp_sec is not None
        ):
            frame_skew = abs(
                self.last_rgb_stamp_sec - self.last_depth_stamp_sec
            )
        elif self.last_rgb_time is None or self.last_depth_time is None:
            frame_skew = float('inf')
        else:
            frame_skew = abs(
                (self.last_rgb_time - self.last_depth_time).nanoseconds
                / 1e9
            )

        if (
            not algi_hazir
            or not algi_taze
            or frame_skew > self.rgb_depth_max_skew_sec
        ):
            self.get_logger().warn(
                '⚠️ ALGILAMA GÜVENLİ STOP | '
                f'RGB={self.cv_image is not None} age={rgb_yasi:.2f}s | '
                f'DEPTH={self.depth_image is not None} age={depth_yasi:.2f}s | '
                f'CAMINFO={self.camera_info_received} | '
                f'MODEL={self.model is not None} | skew={frame_skew:.2f}s',
                throttle_duration_sec=2.0,
            )
            self._guvenli_stop_yayinla()
            return

        stop_yasi = self._sure_saniye(simdi, self.last_raw_stop_time)
        yaw_yasi = self._sure_saniye(simdi, self.last_raw_yaw_time)
        guidance_bayat = (
            stop_yasi > self.guidance_timeout_sec
            or (
                not self.kamikaze_aktif
                and yaw_yasi > self.guidance_timeout_sec
            )
        )
        if guidance_bayat:
            self.get_logger().warn(
                '⚠️ LOS/GUIDANCE BAYAT -> güvenli STOP | '
                f'stop_age={stop_yasi:.2f}s yaw_age={yaw_yasi:.2f}s '
                f'kamikaze={self.kamikaze_aktif}',
                throttle_duration_sec=2.0,
            )
            self._guvenli_stop_yayinla()
            return

        if self.pose_alindi:
            tekne_yaw = self.live_tekne_yaw
        else:
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                tekne_yaw = quaternion_to_yaw(
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                )
            except TransformException:
                self.get_logger().warn(
                    '⚠️ Tekne yaw alınamadı; güvenli STOP.',
                    throttle_duration_sec=2.0,
                )
                self._guvenli_stop_yayinla()
                return

        tespitler, debug_frame = self.yolo_ile_tespit_et(self.cv_image)
        if tespitler is None or debug_frame is None:
            self._guvenli_stop_yayinla()
            return

        turuncu_dubalar = tespitler['turuncu']
        sari_dubalar = tespitler['sari']

        nihai_desired_yaw = self.raw_desired_yaw
        nihai_stop = self.raw_stop
        nihai_rotate = self.raw_rotate
        nihai_reverse = False
        hedef_2d_x = 0.0
        hedef_2d_z = 4.0

        if self.kamikaze_aktif:
            self.geri_vites_aktif = False

            # Hedef rengi gelmeden tekneyi döndürmek/hareket ettirmek yerine bekle.
            if not self.hedef_renk:
                self.get_logger().warn(
                    '⚠️ [KAMİKAZE] /carpilacakhedef bekleniyor. Araç duruyor.',
                    throttle_duration_sec=2.0,
                )
                nihai_stop = 0
                nihai_rotate = False

            elif self.carpma_gerceklesti:
                nihai_stop = 0
                nihai_rotate = False

            elif self.son_hamle_aktif:
                gecen = self._sure_saniye(
                    simdi, self.son_hamle_baslangic
                )

                if gecen < self.final_charge_duration_sec:
                    nihai_stop = 1
                    nihai_rotate = False
                    nihai_desired_yaw = (
                        self.son_hedef_dunya_yaw
                        if self.son_hedef_dunya_yaw is not None
                        else self.raw_desired_yaw
                    )
                    hedef_2d_x = (
                        self.son_hedef_x
                        if self.son_hedef_x is not None
                        else 0.0
                    )
                    hedef_2d_z = (
                        self.son_hedef_mesafe
                        if self.son_hedef_mesafe is not None
                        else self.final_charge_trigger_m
                    )
                    self.get_logger().info(
                        f'💥 [KAMİKAZE SON HAMLE] {gecen:.2f}/'
                        f'{self.final_charge_duration_sec:.2f}s',
                        throttle_duration_sec=0.5,
                    )
                else:
                    self.son_hamle_aktif = False
                    self.carpma_gerceklesti = True
                    self.carpma_zamani = simdi
                    nihai_stop = 0
                    nihai_rotate = False
                    self.get_logger().warn(
                        '💥 [KAMİKAZE TAMAMLANDI] Son yaklaşım süresi tamamlandı; motor komutu kesildi.'
                    )

            else:
                hedef_dubalar = tespitler.get(self.hedef_renk, [])

                if hedef_dubalar:
                    self.tarama_modu = False
                    self.tarama_guncel_yaw = None
                    nihai_rotate = False
                    nihai_stop = 1

                    en_yakin = min(
                        hedef_dubalar, key=lambda d: d['z_m']
                    )
                    x_m = en_yakin['x_m']
                    z_m = en_yakin['z_m']
                    hedef_2d_x, hedef_2d_z = x_m, z_m

                    hedef_goreli_aci = math.atan2(-x_m, z_m)
                    hedef_dunya_yaw = tekne_yaw + hedef_goreli_aci

                    self.son_hedef_x = x_m
                    self.son_hedef_mesafe = z_m
                    self.son_hedef_gorulme_zamani = simdi
                    self.son_hedef_dunya_yaw = hedef_dunya_yaw
                    nihai_desired_yaw = hedef_dunya_yaw

                    self.get_logger().info(
                        f'🎯 [KAMİKAZE] Hedef: {self.hedef_renk} | '
                        f'Mesafe={z_m:.2f}m | '
                        f'Açı={math.degrees(hedef_goreli_aci):.1f}°',
                        throttle_duration_sec=1.0,
                    )

                    if z_m <= self.final_charge_trigger_m:
                        self.son_hamle_aktif = True
                        self.son_hamle_baslangic = simdi
                        self.get_logger().warn(
                            f'💥 [KAMİKAZE SON HAMLE BAŞLADI] '
                            f'z={z_m:.2f}m <= {self.final_charge_trigger_m:.2f}m'
                        )

                else:
                    son_gorulme_yasi = self._sure_saniye(
                        simdi, self.son_hedef_gorulme_zamani
                    )

                    # ZED'in yakın kör bölgesine yeni girmiş bir hedef kaybolduysa,
                    # son güvenilir dünya yaw'ında kısa ve sınırlı final yaklaşımı.
                    if (
                        self.son_hedef_mesafe is not None
                        and self.son_hedef_mesafe
                        <= self.final_charge_trigger_m
                        and son_gorulme_yasi <= 0.75
                        and self.son_hedef_dunya_yaw is not None
                    ):
                        self.son_hamle_aktif = True
                        self.son_hamle_baslangic = simdi
                        nihai_stop = 1
                        nihai_rotate = False
                        nihai_desired_yaw = self.son_hedef_dunya_yaw
                    else:
                        self.tarama_modu = True
                        nihai_rotate = True
                        nihai_stop = 1

                        if self.tarama_guncel_yaw is None:
                            self.tarama_guncel_yaw = tekne_yaw

                        donus_yonu = (
                            -1.0
                            if (
                                self.son_hedef_x is not None
                                and self.son_hedef_x > 0.0
                            )
                            else 1.0
                        )
                        adim = donus_yonu * self.tarama_hizi * 0.1
                        self.tarama_guncel_yaw = math.atan2(
                            math.sin(
                                self.tarama_guncel_yaw + adim
                            ),
                            math.cos(
                                self.tarama_guncel_yaw + adim
                            ),
                        )
                        nihai_desired_yaw = self.tarama_guncel_yaw

        else:
            tum_engeller = sari_dubalar + turuncu_dubalar
            pruva_engelleri = [
                d['z_m']
                for d in tum_engeller
                if (
                    abs(d['x_m']) < self.pruva_koridor_yaricap
                    and d['z_m'] > 0
                )
            ]
            en_yakin_pruva = min(pruva_engelleri, default=99.0)

            if not self.geri_vites_aktif:
                if en_yakin_pruva < self.geri_vites_tetik:
                    self.geri_vites_aktif = True
                    self.geri_vites_baslangic = simdi
                    self.get_logger().warn(
                        f'🔙 Geri vites tetiklendi: {en_yakin_pruva:.2f}m'
                    )
            else:
                gecen_sure = self._sure_saniye(
                    simdi, self.geri_vites_baslangic
                )
                if gecen_sure > 1.5 and en_yakin_pruva > 1.2:
                    self.geri_vites_aktif = False

            if self.geri_vites_aktif:
                nihai_stop = 1
                nihai_desired_yaw = self.raw_desired_yaw
                nihai_reverse = True
                hedef_2d_z = -2.0
            else:
                res = self.geometrik_guvenli_hedef_hesapla(
                    turuncu_dubalar,
                    sari_dubalar,
                )
                if res is not None:
                    gorsel_hedef_acisi, hedef_2d_x, hedef_2d_z = res
                    dunya_gorsel_yaw = (
                        tekne_yaw + gorsel_hedef_acisi
                    )
                    fark = math.atan2(
                        math.sin(
                            dunya_gorsel_yaw
                            - self.raw_desired_yaw
                        ),
                        math.cos(
                            dunya_gorsel_yaw
                            - self.raw_desired_yaw
                        ),
                    )
                    fark_sinirli = float(
                        np.clip(
                            fark,
                            -math.radians(45),
                            math.radians(45),
                        )
                    )
                    nihai_desired_yaw = (
                        self.raw_desired_yaw + fark_sinirli
                    )
                else:
                    nihai_desired_yaw = self.raw_desired_yaw
                    hedef_2d_x = 0.0
                    hedef_2d_z = 4.0

        costmap_frame = self.costmap_2d_uret(
            tespitler, hedef_2d_x, hedef_2d_z
        )

        self._publish_outputs(
            nihai_desired_yaw,
            nihai_stop,
            nihai_rotate,
            nihai_reverse,
        )
        self._debug_goruntuleri_yayinla(
            debug_frame, costmap_frame
        )


def main(args=None):
    rclpy.init(args=args)
    node = ZedYoloObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
