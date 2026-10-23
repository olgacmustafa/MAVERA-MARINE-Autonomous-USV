#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2026 İDA - Küresel Rota Takip ve Kamikaze Tetikleme Düğümü (LOSNode)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointList, State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int8, Bool
from tf2_ros import Buffer, TransformListener, TransformException

R_EARTH = 6378137.0
MAVROS_TIMEOUT_SEC = 2.0
POSE_TIMEOUT_SEC = 1.0


def quaternion_to_yaw(qx, qy, qz, qw):
    t3 = 2.0 * (qw * qz + qx * qy)
    t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(t3, t4)


class LOSNode(Node):
    def __init__(self):
        super().__init__('los_node')

        # --- PARAMETRELER ---
        self.declare_parameter('acceptance_radius', 0.75)
        self.declare_parameter('start_waypoint_index', 1)
        # Su üstünde takılmayı önlemek için yerinde dönüşten çıkış eşiği.
        self.declare_parameter('donus_esigi_derece', 30.0)
        self.declare_parameter('estop_timeout_sec', 1.5)

        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.current_waypoint_index = self.get_parameter('start_waypoint_index').value
        self.rotation_threshold = math.radians(self.get_parameter('donus_esigi_derece').value)
        self.estop_timeout_sec = max(
            0.5, float(self.get_parameter('estop_timeout_sec').value)
        )

        qos_profili = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_profili_rota = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.tf_tamponu = Buffer()
        self.tf_dinleyici = TransformListener(self.tf_tamponu, self)

        # Abonelikler
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profili)
        self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.yki_rota_callback, qos_profili_rota)
        self.create_subscription(State, '/mavros/state', self.mavros_durum_callback, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gercek_gps_callback, qos_profili)
        self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)

        # Yayıncılar
        self.publisher = self.create_publisher(Float32, '/desired_yaw_raw', 10)
        self.kapatpublisher = self.create_publisher(Int8, '/stop_raw', 10)
        self.rotate_publisher = self.create_publisher(Bool, '/rotate_in_place_raw', 10)
        self.kamikaze_pub = self.create_publisher(Bool, '/kamikaze_mode', 10)

        self.tekne_x = 0.0
        self.tekne_y = 0.0
        self.tekne_yaw = 0.0
        self.pose_alindi = False
        self.son_pose_zamani = None
        self.baslangic_konumu_alindi = False
        self.baslangic_lat = 0.0
        self.baslangic_lon = 0.0
        self.connected = False
        self.is_armed = False
        self.onceki_armed = False
        self.mode = None
        # Fail-safe: İlk E-Stop heartbeat'i gelene kadar otonomi hareket etmez.
        self.estop = True
        self.son_estop_zamani = None

        self.gps_waypoints = []
        self.rota_imzasi = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.kamikaze_aktif = False
        self.donus_modu = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_hesaplandi = False
        self.son_mavros_mesaj_zamani = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.rotasyon_kontrol)
        self.get_logger().info("🧭 LOS Düğümü Başlatıldı.")

    def pose_callback(self, msg):
        self.tekne_x = msg.pose.position.x
        self.tekne_y = msg.pose.position.y
        self.tekne_yaw = quaternion_to_yaw(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        )
        self.pose_alindi = True
        self.son_pose_zamani = self.get_clock().now()

    def estop_callback(self, msg):
        self.estop = bool(msg.data)
        self.son_estop_zamani = self.get_clock().now()

    def mavros_durum_callback(self, mesaj):
        self.son_mavros_mesaj_zamani = self.get_clock().now()
        self.is_armed = bool(mesaj.armed)
        self.mode = mesaj.mode
        self.connected = bool(mesaj.connected)

        # Aynı waypoint listesi MAVROS tarafından tekrar yayınlansa bile rota
        # callback'i ilerlemeyi sıfırlamasın. Yeni ARM oturumu bilinçli reset noktasıdır.
        if self.is_armed and not self.onceki_armed and self.gps_waypoints:
            start_idx = int(self.get_parameter('start_waypoint_index').value)
            if start_idx >= len(self.gps_waypoints):
                start_idx = 1 if len(self.gps_waypoints) > 1 else 0
            self.current_waypoint_index = start_idx
            self.kamikaze_aktif = False
            self.donus_modu = False
            self.offset_hesaplandi = False
            self.get_logger().info(
                f"🔓 Yeni ARM oturumu: rota başlangıç indeksi {start_idx}"
            )

        self.onceki_armed = self.is_armed

    def yki_rota_callback(self, mesaj):
        if not mesaj.waypoints:
            return
        yeni_rota = [
            (float(wp.x_lat), float(wp.y_long))
            for wp in mesaj.waypoints
            if wp.command == 16
        ]
        if not yeni_rota:
            return

        yeni_imza = tuple(
            (round(lat, 8), round(lon, 8))
            for lat, lon in yeni_rota
        )
        if yeni_imza == self.rota_imzasi:
            return

        self.rota_imzasi = yeni_imza
        self.gps_waypoints = yeni_rota
        self.kamikaze_aktif = False
        self.offset_hesaplandi = False
        self.donus_modu = False
        start_idx = int(self.get_parameter('start_waypoint_index').value)
        # 0. İndeks Home (Kalkış) noktasıdır. İlk hedef 1. indekstir.
        if start_idx >= len(self.gps_waypoints):
            start_idx = 1 if len(self.gps_waypoints) > 1 else 0
        self.current_waypoint_index = start_idx
        self.baslangic_lat, self.baslangic_lon = self.gps_waypoints[0]
        self.baslangic_konumu_alindi = True
        self.hedefi_metreye_cevir()

    def gercek_gps_callback(self, mesaj):
        # GPS FIX kontrolü (status >= 0: STATUS_FIX / DGPS / RTK)
        if mesaj.status.status < 0:
            return
        # EKF / Local Pose henüz gelmemişse kalibrasyon yapma
        if not self.pose_alindi:
            return

        if self.gps_waypoints and not self.baslangic_konumu_alindi:
            self.baslangic_lat, self.baslangic_lon = self.gps_waypoints[0]
            self.baslangic_konumu_alindi = True
            self.hedefi_metreye_cevir()

        if self.baslangic_konumu_alindi and not self.offset_hesaplandi:
            self._ofseti_kalibre_et(mesaj)

    def _latlon_to_enu(self, lat, lon):
        dlat = math.radians(lat - self.baslangic_lat)
        dlon = math.radians(lon - self.baslangic_lon)
        x = dlon * R_EARTH * math.cos(math.radians(self.baslangic_lat))
        y = dlat * R_EARTH
        return x, y

    def _ofseti_kalibre_et(self, gps_mesaji):
        if not self.pose_alindi:
            return
        curr_x = self.tekne_x
        curr_y = self.tekne_y
        gps_x, gps_y = self._latlon_to_enu(gps_mesaji.latitude, gps_mesaji.longitude)
        self.offset_x = curr_x - gps_x
        self.offset_y = curr_y - gps_y
        self.offset_hesaplandi = True
        self.hedefi_metreye_cevir()
        self.get_logger().info(
            f"🛰️ [GPS & EKF KALİBRE EDİLDİ] Başlangıç Lat/Lon: {self.baslangic_lat:.6f}, {self.baslangic_lon:.6f} | "
            f"Ofset X: {self.offset_x:.2f}m, Y: {self.offset_y:.2f}m"
        )

    def hedefi_metreye_cevir(self):
        if not self.baslangic_konumu_alindi or not self.gps_waypoints:
            return
        if not self.offset_hesaplandi:
            return
        if self.current_waypoint_index >= len(self.gps_waypoints):
            return
        target_lat, target_lon = self.gps_waypoints[self.current_waypoint_index]
        x, y = self._latlon_to_enu(target_lat, target_lon)
        self.target_x = x + self.offset_x
        self.target_y = y + self.offset_y

    def _dur_komutu_gonder(self):
        self.kapatpublisher.publish(Int8(data=0))
        self.rotate_publisher.publish(Bool(data=False))
        self.kamikaze_pub.publish(Bool(data=False))

    def rotasyon_kontrol(self):
        su_an = self.get_clock().now()
        gecen_sure = (su_an - self.son_mavros_mesaj_zamani).nanoseconds / 1e9

        estop_bayat = (
            self.son_estop_zamani is None
            or (su_an - self.son_estop_zamani).nanoseconds / 1e9
            > self.estop_timeout_sec
        )

        if (
            gecen_sure > MAVROS_TIMEOUT_SEC
            or not self.connected
            or self.estop
            or estop_bayat
        ):
            if estop_bayat:
                self.get_logger().error(
                    '❌ /emergency_stop heartbeat bayat/yok. LOS fail-safe STOP.',
                    throttle_duration_sec=2.0,
                )
            self._dur_komutu_gonder()
            return

        pose_bayat = (
            self.son_pose_zamani is None
            or (su_an - self.son_pose_zamani).nanoseconds / 1e9 > POSE_TIMEOUT_SEC
        )
        if pose_bayat:
            self.get_logger().warn(
                "⚠️ MAVROS local pose bayat/yok. LOS güvenli STOP.",
                throttle_duration_sec=2.0,
            )
            self._dur_komutu_gonder()
            return

        if not self.baslangic_konumu_alindi or not self.gps_waypoints or not self.offset_hesaplandi:
            self._dur_komutu_gonder()
            return

        self.kamikaze_pub.publish(Bool(data=bool(self.kamikaze_aktif)))

        if self.kamikaze_aktif:
            self.kapatpublisher.publish(Int8(data=1))
            return

        if self.pose_alindi:
            tekne_x, tekne_y, tekne_yaw = self.tekne_x, self.tekne_y, self.tekne_yaw
        else:
            try:
                t = self.tf_tamponu.lookup_transform('map', 'base_link', rclpy.time.Time())
                tekne_x = t.transform.translation.x
                tekne_y = t.transform.translation.y
                tekne_yaw = quaternion_to_yaw(
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w
                )
            except TransformException:
                self._dur_komutu_gonder()
                return

        distance_to_target = math.hypot(self.target_x - tekne_x, self.target_y - tekne_y)

        if distance_to_target < self.acceptance_radius:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.gps_waypoints):
                self.hedefi_metreye_cevir()
                self.donus_modu = True
                self.get_logger().info(f"🎯 [HEDEF DEĞİŞTİ] Yeni Waypoint İndeksi: {self.current_waypoint_index}")
                return
            else:
                self.kamikaze_aktif = True
                self.donus_modu = False
                self.get_logger().warn("🚀 [TÜM WAYPOINTLER TAMAMLANDI] KAMİKAZE MODU AKTİF!")
                return

        dx = self.target_x - tekne_x
        dy = self.target_y - tekne_y
        desired_yaw = math.atan2(dy, dx)

        # Hedefe kalan mesafeyi ve açıları periyodik logla
        self.get_logger().info(
            f"📍 [WAYPOINT {self.current_waypoint_index}/{len(self.gps_waypoints)-1}] "
            f"Kalan Mesafe: {distance_to_target:.2f} m | "
            f"Hedef Yaw: {math.degrees(desired_yaw):.1f}° | Anlık: {math.degrees(tekne_yaw):.1f}°",
            throttle_duration_sec=1.0
        )

        if self.donus_modu:
            yaw_hata = math.atan2(math.sin(desired_yaw - tekne_yaw), math.cos(desired_yaw - tekne_yaw))
            if abs(yaw_hata) < self.rotation_threshold:
                self.donus_modu = False

        self.rotate_publisher.publish(Bool(data=self.donus_modu))
        self.publisher.publish(Float32(data=desired_yaw))
        self.kapatpublisher.publish(Int8(data=1))


def main(args=None):
    rclpy.init(args=args)
    node = LOSNode()
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
