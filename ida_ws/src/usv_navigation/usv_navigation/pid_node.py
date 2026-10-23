#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TEKNOFEST 2026 İDA - Pixhawk Sürüş Kontrol Düğümü (PID / Guidance / Geri Vites Destekli)
========================================================================================
- Kumanda ile MANUEL <-> GUIDED geçişlerini anlık takip eder.
- /reverse_mode == True geldiğinde motorlara ters itki (-0.6 m/s) uygulayarak geriye çeker.
- DISARM durumunda motor setpointlerini sıfırlar ve hafızayı temizler.
- Pixhawk'a MAVLink PositionTarget (Hız + Hedef Yaw + Yaw Rate) gönderir.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State, DebugValue
from std_msgs.msg import Float32, Int8, Bool
from tf2_ros import Buffer, TransformListener, TransformException

COMMAND_TIMEOUT_SEC = 1.0
MAVROS_TIMEOUT_SEC = 2.0
POSE_TIMEOUT_SEC = 1.0


def quaternion_to_yaw(qx, qy, qz, qw):
    t3 = 2.0 * (qw * qz + qx * qy)
    t4 = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(t3, t4)


class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')

        # --- PARAMETRELER (PID, SÜRÜŞ VE MANEVRA AYARLARI) ---
        self.declare_parameter('Kp', 1.8)
        self.declare_parameter('Ki', 0.05)
        self.declare_parameter('Kd', 0.25)
        self.declare_parameter('normal_seyir_hizi', 1.3)
        self.declare_parameter('geri_vites_hizi', 0.5)
        self.declare_parameter('max_donus_hizi', 1.5)  # rad/s cinsinden max yaw rate
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('estop_timeout_sec', 1.5)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.normal_seyir_hizi = self.get_parameter('normal_seyir_hizi').value
        self.geri_vites_hizi = self.get_parameter('geri_vites_hizi').value
        self.max_donus_hizi = self.get_parameter('max_donus_hizi').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.estop_timeout_sec = max(
            0.5, float(self.get_parameter('estop_timeout_sec').value)
        )

        # --- TF2 DİNLEYİCİSİ ---
        self.tf_tamponu = Buffer()
        self.tf_dinleyici = TransformListener(self.tf_tamponu, self)

        # Durum Değişkenleri
        self.desired_yaw = 0.0
        self.integral = 0.0
        self.onceki_aci = None
        self.last_time = self.get_clock().now()
        self.donus_modu = False
        self.onceki_konum = None
        self.reverse_mode = False

        # Canlı Konum ve Açı Değişkenleri
        self.su_anki_x = 0.0
        self.su_anki_y = 0.0
        self.su_anki_aci = 0.0
        self.pose_alindi = False
        self.son_pose_zamani = None

        self.motorlari_kapat = 0
        # Fail-safe: E-Stop düğümünden güncel durum gelmeden motor komutu üretme.
        self.estop = True
        self.son_estop_zamani = None
        self.is_armed = False
        self.mode = "MANUAL"
        self.onceki_mode = "MANUAL"
        self.onceki_armed = False
        self.connected = False
        self.son_mavros_mesaj_zamani = None
        self.son_yaw_mesaj_zamani = None
        self.son_stop_mesaj_zamani = None

        # --- ABONELİKLER ---
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)
        self.create_subscription(State, '/mavros/state', self.mavros_durum_callback, 10)
        self.create_subscription(Int8, '/stop', self.durma_sinyali_callback, 10)
        self.create_subscription(Float32, '/desired_yaw', self.hedef_aci_callback, 10)
        self.create_subscription(Bool, '/rotate_in_place', self.donus_modu_callback, 10)
        self.create_subscription(Bool, '/reverse_mode', self.geri_vites_callback, 10)

        # --- YAYINCILAR ---
        self.setpoint_raw_yayinci = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.mp_log_yayinci = self.create_publisher(DebugValue, '/mavros/debug_value/send', 10)

        # 10 Hz Kontrol Döngüsü
        self.timer = self.create_timer(0.1, self.kontrol_dongusu)
        self.get_logger().info("🎮 Düzeltilmiş Hız + Yaw Rate Sürüş Düğümü Aktif.")

    # ---------------- CALLBACKS ----------------
    def pose_callback(self, msg):
        self.su_anki_x = msg.pose.position.x
        self.su_anki_y = msg.pose.position.y
        self.su_anki_aci = quaternion_to_yaw(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        )
        self.pose_alindi = True
        self.son_pose_zamani = self.get_clock().now()

    def estop_callback(self, msg):
        self.estop = bool(msg.data)
        self.son_estop_zamani = self.get_clock().now()

    def geri_vites_callback(self, msg):
        self.reverse_mode = msg.data

    def mavros_durum_callback(self, mesaj):
        self.son_mavros_mesaj_zamani = self.get_clock().now()
        self.connected = mesaj.connected

        if mesaj.mode != self.onceki_mode:
            if mesaj.mode == "GUIDED":
                self.get_logger().info("🟢 [KUMANDA -> OTONOM] GUIDED Moduna Geçildi!")
                self._kontrol_durumunu_sifirla()
            else:
                self.get_logger().warn(f"🟡 [OTONOM -> KUMANDA] {mesaj.mode} Modu.")
                self._sifir_hiz_yayinla()
                self._kontrol_durumunu_sifirla()
            self.onceki_mode = mesaj.mode

        if mesaj.armed != self.onceki_armed:
            if mesaj.armed:
                self.get_logger().info("🔓 [PIXHAWK ARMED] Motorlar Açıldı!")
            else:
                self.get_logger().warn("🔒 [PIXHAWK DISARMED] Motorlar Kapatıldı!")
                self._sifir_hiz_yayinla()
                self._kontrol_durumunu_sifirla()
            self.onceki_armed = mesaj.armed

        self.is_armed = mesaj.armed
        self.mode = mesaj.mode

    def durma_sinyali_callback(self, mesaj):
        self.motorlari_kapat = mesaj.data
        self.son_stop_mesaj_zamani = self.get_clock().now()

    def hedef_aci_callback(self, mesaj):
        self.desired_yaw = mesaj.data
        self.son_yaw_mesaj_zamani = self.get_clock().now()

    def donus_modu_callback(self, mesaj):
        self.donus_modu = mesaj.data

    # ---------------- YARDIMCI FONKSİYONLAR ----------------
    def _ham_setpoint_mesaji_olustur(self, hiz=0.0, yaw_rate=0.0, yaw_kullan=True):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED

        if yaw_kullan:
            # POSITION_TARGET_TYPEMASK:
            # PX/PY/PZ ignored, VZ ignored, AX/AY/AZ ignored, YAW ignored.
            # VX + VY + YAW_RATE aktif.
            msg.type_mask = 1511
        else:
            # PX/PY/PZ, VZ, AX/AY/AZ, YAW ve YAW_RATE ignored.
            # VX + VY aktif. Eski 3535 maskesi VX'i yanlışlıkla ignore ediyordu.
            msg.type_mask = 3559

        # Gövde (Body) ekseninde doğrudan ileri itki (Vx) ve sıfır yanal hız (Vy)
        msg.velocity.x = float(hiz)
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0

        msg.yaw = float(self.desired_yaw)
        msg.yaw_rate = float(yaw_rate)
        return msg

    def stop_motors(self):
        stop_msg = self._ham_setpoint_mesaji_olustur(hiz=0.0, yaw_rate=0.0, yaw_kullan=False)
        for _ in range(3):
            self.setpoint_raw_yayinci.publish(stop_msg)

    def _sifir_hiz_yayinla(self):
        self.setpoint_raw_yayinci.publish(self._ham_setpoint_mesaji_olustur(hiz=0.0, yaw_rate=0.0, yaw_kullan=False))

    def _kontrol_durumunu_sifirla(self):
        self.integral = 0.0
        self.onceki_aci = None
        self.onceki_konum = None
        self.last_time = self.get_clock().now()

    # ---------------- ANA KONTROL DÖNGÜSÜ ----------------
    def kontrol_dongusu(self):
        simdi = self.get_clock().now()

        # 1. MAVROS Kontrolü
        mavros_bayat = (self.son_mavros_mesaj_zamani is None or 
                        (simdi - self.son_mavros_mesaj_zamani).nanoseconds / 1e9 > MAVROS_TIMEOUT_SEC)
        if not self.connected or mavros_bayat:
            self._sifir_hiz_yayinla()
            self._kontrol_durumunu_sifirla()
            return

        # 2. Acil Stop. E-Stop node'u ölür veya topic kesilirse fail-safe STOP.
        estop_bayat = (
            self.son_estop_zamani is None
            or (simdi - self.son_estop_zamani).nanoseconds / 1e9
            > self.estop_timeout_sec
        )
        if self.estop or estop_bayat:
            if estop_bayat:
                self.get_logger().error(
                    '❌ /emergency_stop heartbeat bayat/yok. PID fail-safe STOP.',
                    throttle_duration_sec=2.0,
                )
            self._sifir_hiz_yayinla()
            self._kontrol_durumunu_sifirla()
            return

        # 3. Manuel Mod ve Disarm
        if self.mode != 'GUIDED' or not self.is_armed:
            self._kontrol_durumunu_sifirla()
            return

        # 4. Komut Zaman Aşımı & Stop Sinyali
        komut_bayat = (self.son_yaw_mesaj_zamani is None or self.son_stop_mesaj_zamani is None or
                       (simdi - self.son_yaw_mesaj_zamani).nanoseconds / 1e9 > COMMAND_TIMEOUT_SEC or
                       (simdi - self.son_stop_mesaj_zamani).nanoseconds / 1e9 > COMMAND_TIMEOUT_SEC)
        if komut_bayat or self.motorlari_kapat == 0:
            self._sifir_hiz_yayinla()
            self._kontrol_durumunu_sifirla()
            return

        # --- CANLI KONUM VE AÇI ---
        pose_bayat = (
            self.son_pose_zamani is None
            or (simdi - self.son_pose_zamani).nanoseconds / 1e9 > POSE_TIMEOUT_SEC
        )
        if pose_bayat:
            self.get_logger().warn(
                "⚠️ MAVROS local pose bayat/yok. PID güvenli STOP.",
                throttle_duration_sec=2.0,
            )
            self._sifir_hiz_yayinla()
            self._kontrol_durumunu_sifirla()
            return

        su_anki_zaman = self.get_clock().now()
        gecen_sure = (su_anki_zaman - self.last_time).nanoseconds / 1e9
        if gecen_sure <= 0.0:
            return

        if self.pose_alindi:
            su_anki_aci = self.su_anki_aci
        else:
            try:
                t = self.tf_tamponu.lookup_transform('map', 'base_link', rclpy.time.Time())
                su_anki_aci = quaternion_to_yaw(
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w
                )
            except TransformException:
                self._sifir_hiz_yayinla()
                self._kontrol_durumunu_sifirla()
                return

        # Açı Hatası [-pi, pi]
        error = math.atan2(math.sin(self.desired_yaw - su_anki_aci), math.cos(self.desired_yaw - su_anki_aci))
        abs_error = abs(error)

        if self.onceki_aci is None:
            aci_turevi = 0.0
        else:
            aci_farki = math.atan2(math.sin(su_anki_aci - self.onceki_aci), math.cos(su_anki_aci - self.onceki_aci))
            aci_turevi = -aci_farki / gecen_sure
        self.onceki_aci = su_anki_aci

        # PID Hesaplama
        onerilen_donus = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * aci_turevi)
        cikis_doymus = abs(onerilen_donus) >= self.max_donus_hizi
        if not (cikis_doymus and (error * onerilen_donus > 0)):
            self.integral += error * gecen_sure
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)

        donus_hizi = max(min(onerilen_donus, self.max_donus_hizi), -self.max_donus_hizi)
        self.last_time = su_anki_zaman

        # --- YENİ DİNAMİK HIZ HESAPLAMA (0 m/s KİLİTLENMESİ DÜZELTİLDİ) ---
        if self.reverse_mode:
            ileri_hiz = -abs(self.geri_vites_hizi)
            mod_etiketi = "🔙 GERİ VİTES"
        elif self.donus_modu:
            # Sadece açıkça yerinde dönüş istenmişse ileri hızı sıfırla
            ileri_hiz = 0.0
            mod_etiketi = "🔄 NOKTADA DÖNÜŞ"
        else:
            # Açı hatası 180° bile olsa teknenin dümeni tutması için en az %30 taban hız verilir
            hiz_orani = max(0.3, 1.0 - (abs_error / math.pi))
            ileri_hiz = self.normal_seyir_hizi * hiz_orani
            mod_etiketi = "➡️ SEYİR"

        # Setpoint Yayınlama
        setpoint_msg = self._ham_setpoint_mesaji_olustur(
            hiz=ileri_hiz,
            yaw_rate=donus_hizi,
            yaw_kullan=True
        )
        self.setpoint_raw_yayinci.publish(setpoint_msg)

        # Loglama
        self.get_logger().info(
            f"{mod_etiketi} | 🎯 Hedef Yaw: {math.degrees(self.desired_yaw):.1f}° | "
            f"📍 Anlık: {math.degrees(su_anki_aci):.1f}° | ❌ Hata: {math.degrees(error):.1f}° | "
            f"🚀 Hız: {ileri_hiz:.2f} m/s | 🔄 Yaw Rate: {donus_hizi:.2f} rad/s",
            throttle_duration_sec=1.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch SIGINT sırasında ROS context daha önce kapanmış olabilir.
        if rclpy.ok():
            try:
                node.stop_motors()
            except Exception as e:
                print(f"Motor durdurma yayın hatası: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
