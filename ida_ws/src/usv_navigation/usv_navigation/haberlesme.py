#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MAVERA MARINE - YKİ -> İDA hedef renk köprüsü (MAVROS RAW / MAV_CMD_USER_1)

Mimari:
    YKİ
      -> RFD
      -> Cube Orange+ TELEM
      -> Cube USB
      -> MAVROS
      -> /mavros/mavlink/from
      -> bu node
      -> /carpilacakhedef (std_msgs/String)
      -> obstacle_avoidance_node

ÖNEMLİ:
- Bu node Cube seri portunu AÇMAZ.
- Cube USB'nin tek sahibi MAVROS'tur.
- Böylece MAVROS ile ikinci bir pymavlink seri bağlantısı çakışmaz.
- Jetson sistem/bileşen kimliği (202/191) heartbeat'i
  /mavros/mavlink/to üzerinden Cube'a gönderilir.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

from datetime import datetime, timezone
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import String
from mavros_msgs.msg import Mavlink, State

from pymavlink import mavutil
from mavros import mavlink


# ---------------------------------------------------------------------------
# Protokol
# ---------------------------------------------------------------------------

IDA_JETSON_SISTEM_ID = 202
IDA_JETSON_BILESEN_ID = 191
YKI_SISTEM_ID = 201
YKI_BILESEN_ID = 190

PROTOKOL_SURUMU = 1
PROTOKOL_IMZASI = 2026

KOMUT_ID = getattr(
    mavutil.mavlink,
    "MAV_CMD_USER_1",
    31010,
)

RENKLER = {
    1: "SIYAH",
    2: "KIRMIZI",
    3: "YESIL",
}

ROS_RENKLER = {
    1: "siyah",
    2: "kirmizi",
    3: "yesil",
}

VARSAYILAN_KAYIT_YOLU = (
    Path.home() / "mavera_ida" / "data" / "hedef_renk.json"
)


def atomik_json_yaz(kayit_yolu: Path, veri: dict) -> None:
    kayit_yolu.parent.mkdir(parents=True, exist_ok=True)
    gecici = kayit_yolu.with_suffix(".tmp")
    gecici.write_text(
        json.dumps(veri, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    gecici.replace(kayit_yolu)


class IdaHedefRenkAlici(Node):
    def __init__(self):
        super().__init__("ida_hedef_renk_al")

        self.declare_parameter(
            "kayit_yolu",
            str(VARSAYILAN_KAYIT_YOLU),
        )
        self.declare_parameter(
            "beklenen_yki_sistem_id",
            YKI_SISTEM_ID,
        )
        self.declare_parameter(
            "beklenen_yki_bilesen_id",
            YKI_BILESEN_ID,
        )

        self.kayit_yolu = Path(
            self.get_parameter("kayit_yolu").value
        ).expanduser().resolve()

        self.beklenen_yki_sistem_id = int(
            self.get_parameter(
                "beklenen_yki_sistem_id"
            ).value
        )
        self.beklenen_yki_bilesen_id = int(
            self.get_parameter(
                "beklenen_yki_bilesen_id"
            ).value
        )

        self.kilitli_kod = None
        self.mavros_connected = False
        self.son_yki_sistem_id = YKI_SISTEM_ID
        self.son_yki_bilesen_id = YKI_BILESEN_ID
        self.heartbeat_ilk_log_yazildi = False

        # Önceki koşudaki hedef yanlışlıkla kullanılmasın.
        atomik_json_yaz(
            self.kayit_yolu,
            {
                "durum": "BEKLENIYOR",
                "kod": None,
                "renk": None,
                "baslangic_utc": datetime.now(
                    timezone.utc
                ).isoformat(),
            },
        )

        # /carpilacakhedef son değeri tutulur.
        # obstacle node zaten canlı yayını da alır; ayrıca her 1 sn tekrar edilir.
        renk_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.renk_pub = self.create_publisher(
            String,
            "/carpilacakhedef",
            renk_qos,
        )

        # MAVROS raw RX.
        # SensorData QoS, MAVROS tarafı BEST_EFFORT olsa da uyumlu kalır.
        self.raw_rx_sub = self.create_subscription(
            Mavlink,
            "/mavros/mavlink/from",
            self._mavlink_from_cb,
            qos_profile_sensor_data,
        )

        # MAVROS raw TX. Heartbeat, Cube'un 202/191 Jetson bilesenine giden
        # paketleri USB/MAVROS hattina yonlendirmeyi ogrenmesini saglar.
        # COMMAND_ACK ise YKI'nin tekrarlı hedef komutunu durdurur.
        self.raw_tx_pub = self.create_publisher(
            Mavlink,
            "/mavros/mavlink/to",
            10,
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            10,
        )

        # Pymavlink yalnizca ROS Mavlink mesajlarini kodlayip/cozmek icindir.
        # Seri portu MAVROS acmaya devam eder; bu node seri porta dokunmaz.
        self.decoder = mavutil.mavlink.MAVLink(None)
        self.encoder = mavutil.mavlink.MAVLink(
            None,
            srcSystem=IDA_JETSON_SISTEM_ID,
            srcComponent=IDA_JETSON_BILESEN_ID,
        )

        # Cube'un MAVLink yonlendirme tablosunda 202/191 rotasini canlı tut.
        self.create_timer(
            1.0,
            self._heartbeat_timer,
        )

        # Kilitlenen rengi düzenli tekrar yayınla.
        self.create_timer(
            1.0,
            self._renk_tekrar_timer,
        )

        self.get_logger().info(
            "📡 YKİ hedef renk alıcısı hazır."
        )
        self.get_logger().info(
            "MAVLink RX: /mavros/mavlink/from"
        )
        self.get_logger().info(
            "MAVLink TX: /mavros/mavlink/to (HEARTBEAT + COMMAND_ACK)"
        )
        self.get_logger().info(
            "Hedef topic: /carpilacakhedef"
        )
        self.get_logger().info(
            "Jetson RX modu: MAVROS /mavros/mavlink/from"
        )
        self.get_logger().info(
            "⏳ YKİ'den MAV_CMD_USER_1 hedef renk komutu bekleniyor..."
        )

    # ------------------------------------------------------------------
    # MAVROS
    # ------------------------------------------------------------------

    def _mavlink_yayinla(self, mesaj, aciklama: str) -> bool:
        """Pymavlink mesajini MAVROS raw TX topic'ine guvenle yollar."""
        try:
            # convert_to_rosmsg paketlenmis bir pymavlink mesaji bekler.
            mesaj.pack(self.encoder)
            try:
                ros_mesaj = mavlink.convert_to_rosmsg(
                    mesaj,
                    stamp=self.get_clock().now().to_msg(),
                )
            except TypeError:
                # Eski MAVROS Python yardimcisi stamp parametresi almayabilir.
                ros_mesaj = mavlink.convert_to_rosmsg(mesaj)
            self.raw_tx_pub.publish(ros_mesaj)
            return True
        except Exception as e:
            self.get_logger().error(
                f"{aciklama} MAVROS TX hatasi: "
                f"{type(e).__name__}: {e}",
                throttle_duration_sec=2.0,
            )
            return False

    def _heartbeat_timer(self):
        heartbeat = self.encoder.heartbeat_encode(
            getattr(
                mavutil.mavlink,
                "MAV_TYPE_ONBOARD_CONTROLLER",
                mavutil.mavlink.MAV_TYPE_GENERIC,
            ),
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        if self._mavlink_yayinla(heartbeat, "HEARTBEAT"):
            if not self.heartbeat_ilk_log_yazildi:
                self.heartbeat_ilk_log_yazildi = True
                self.get_logger().info(
                    "✅ IDA Jetson HEARTBEAT aktif | sysid=202 compid=191"
                )

    def _state_cb(self, msg: State):
        onceki = self.mavros_connected
        self.mavros_connected = bool(msg.connected)

        if self.mavros_connected and not onceki:
            self.get_logger().info(
                "✅ MAVROS/Cube bağlantısı aktif."
            )
        elif onceki and not self.mavros_connected:
            self.get_logger().error(
                "❌ MAVROS/Cube bağlantısı koptu."
            )

    # ------------------------------------------------------------------
    # Hedef renk
    # ------------------------------------------------------------------

    def _renk_yayinla(self, kod: int):
        msg = String()
        msg.data = ROS_RENKLER[kod]
        self.renk_pub.publish(msg)
        self.get_logger().info(
            f"📤 /carpilacakhedef -> {msg.data}"
        )

    def _renk_tekrar_timer(self):
        if self.kilitli_kod is not None:
            self._renk_yayinla(self.kilitli_kod)

    def _kaynak_dogrula(self, mav_msg) -> bool:
        src_sys = int(mav_msg.get_srcSystem())
        src_comp = int(mav_msg.get_srcComponent())

        if (
            self.beklenen_yki_sistem_id > 0
            and src_sys != self.beklenen_yki_sistem_id
        ):
            return False

        if (
            self.beklenen_yki_bilesen_id > 0
            and src_comp != self.beklenen_yki_bilesen_id
        ):
            return False

        return True

    @staticmethod
    def _komut_bu_aliciya_mi(mesaj) -> bool:
        hedef_sistem = int(
            getattr(mesaj, "target_system", 0)
        )
        hedef_bilesen = int(
            getattr(mesaj, "target_component", 0)
        )

        # RFD -> Cube -> USB/MAVROS hattinda paket broadcast (0/0),
        # Cube hedefli (1/1) veya Jetson hedefli (202/191) gelebilir.
        # Guvenlik yine MAV_CMD_USER_1 + protokol + imza ile saglanir.
        return (
            hedef_sistem in (0, 1, IDA_JETSON_SISTEM_ID)
            and hedef_bilesen in (0, 1, IDA_JETSON_BILESEN_ID)
        )

    def _ack_gonder(self, sonuc: int):
        """YKI'ye gercek MAVLink COMMAND_ACK cevabi gonderir."""
        ilerleme = (
            100
            if sonuc == mavutil.mavlink.MAV_RESULT_ACCEPTED
            else 0
        )

        try:
            ack = self.encoder.command_ack_encode(
                KOMUT_ID,
                int(sonuc),
                ilerleme,
                0,
                int(self.son_yki_sistem_id),
                int(self.son_yki_bilesen_id),
            )
        except TypeError:
            # Eski pymavlink surumlerinde hedef alanlari bulunmayabilir.
            # ACK yine broadcast olarak gecerli kalir; kaynak 202/191'dir.
            ack = self.encoder.command_ack_encode(
                KOMUT_ID,
                int(sonuc),
            )

        if self._mavlink_yayinla(ack, "COMMAND_ACK"):
            self.get_logger().info(
                "📤 COMMAND_ACK GONDERILDI | "
                f"sonuc={int(sonuc)} | "
                f"hedef={self.son_yki_sistem_id}/"
                f"{self.son_yki_bilesen_id}"
            )

    def _mavlink_from_cb(self, ros_mav_msg: Mavlink):
        # Sadece COMMAND_LONG decode et; diğer MAVLink trafiğini boşuna
        # Python decoder'a sokma.
        if int(ros_mav_msg.msgid) != int(
            mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG
        ):
            return

        try:
            paket = mavlink.convert_to_bytes(
                ros_mav_msg
            )
            mesaj = self.decoder.decode(paket)

        except Exception as e:
            self.get_logger().error(
                f"COMMAND_LONG decode hatası: "
                f"{type(e).__name__}: {e}",
                throttle_duration_sec=2.0,
            )
            return

        if mesaj is None:
            return

        self.get_logger().info(
            "🔎 COMMAND_LONG GORULDU | "
            f"cmd={int(getattr(mesaj, 'command', -1))} | "
            f"src={mesaj.get_srcSystem()}/{mesaj.get_srcComponent()} | "
            f"target={int(getattr(mesaj, 'target_system', 0))}/"
            f"{int(getattr(mesaj, 'target_component', 0))}"
        )

        if int(getattr(mesaj, "command", -1)) != KOMUT_ID:
            return

        if not self._komut_bu_aliciya_mi(mesaj):
            self.get_logger().warn(
                "⚠️ MAV_CMD_USER_1 hedef filtresinde reddedildi | "
                f"target={int(getattr(mesaj, 'target_system', 0))}/"
                f"{int(getattr(mesaj, 'target_component', 0))}"
            )
            return

        if not self._kaynak_dogrula(mesaj):
            self.get_logger().warn(
                "⚠️ Beklenmeyen MAVLink kaynağından hedef "
                "renk komutu geldi; yok sayıldı."
            )
            return

        # ACK, komutu gercekten gonderen YKI kimligine doner.
        self.son_yki_sistem_id = int(mesaj.get_srcSystem())
        self.son_yki_bilesen_id = int(mesaj.get_srcComponent())

        try:
            kod = int(round(float(mesaj.param1)))
            protokol = int(
                round(float(mesaj.param2))
            )
            islem_no = int(
                round(float(mesaj.param3))
            )
            imza = int(
                round(float(mesaj.param4))
            )

        except Exception as e:
            self.get_logger().error(
                f"Geçersiz COMMAND_LONG parametreleri: {e}"
            )
            self._ack_gonder(
                mavutil.mavlink.MAV_RESULT_DENIED
            )
            return

        self.get_logger().info(
            "📨 YKİ HEDEF KOMUTU ALINDI | "
            f"src={mesaj.get_srcSystem()}/"
            f"{mesaj.get_srcComponent()} | "
            f"kod={kod} | protokol={protokol} | "
            f"islem_no={islem_no} | imza={imza}"
        )

        # Protokol doğrulama.
        if (
            kod not in RENKLER
            or protokol != PROTOKOL_SURUMU
            or imza != PROTOKOL_IMZASI
        ):
            self.get_logger().error(
                "❌ GEÇERSİZ HEDEF KOMUTU: "
                f"kod={kod}, protokol={protokol}, "
                f"imza={imza}"
            )
            self._ack_gonder(
                mavutil.mavlink.MAV_RESULT_DENIED
            )
            return

        # Bir kere kilitlendikten sonra yarış sırasında farklı renk kabul etme.
        if (
            self.kilitli_kod is not None
            and kod != self.kilitli_kod
        ):
            self.get_logger().error(
                "❌ FARKLI RENK REDDEDİLDİ | "
                f"gelen={RENKLER[kod]} | "
                f"kilitli={RENKLER[self.kilitli_kod]}"
            )
            self._ack_gonder(
                mavutil.mavlink.MAV_RESULT_DENIED
            )
            return

        if self.kilitli_kod is None:
            hedef_verisi = {
                "durum": "KILITLI",
                "kod": kod,
                "renk": RENKLER[kod],
                "islem_no": islem_no,
                "protokol_surumu": PROTOKOL_SURUMU,
                "alindi_utc": datetime.now(
                    timezone.utc
                ).isoformat(),
                "kaynak_sistem": int(
                    mesaj.get_srcSystem()
                ),
                "kaynak_bilesen": int(
                    mesaj.get_srcComponent()
                ),
            }

            try:
                atomik_json_yaz(
                    self.kayit_yolu,
                    hedef_verisi,
                )
            except Exception as e:
                self.get_logger().error(
                    f"❌ Hedef renk JSON'a yazılamadı: {e}"
                )
                self._ack_gonder(
                    mavutil.mavlink.MAV_RESULT_FAILED
                )
                return

            # Dosyaya güvenli yazımdan SONRA kilitle ve ROS'a yayınla.
            self.kilitli_kod = kod
            self._renk_yayinla(kod)

            self.get_logger().info(
                "========================================"
            )
            self.get_logger().info(
                f"🎯 İDA HEDEF RENGİ KİLİTLENDİ: "
                f"{RENKLER[kod]} (KOD {kod})"
            )
            self.get_logger().info(
                f"✅ /carpilacakhedef -> "
                f"{ROS_RENKLER[kod]}"
            )
            self.get_logger().info(
                f"💾 Kayıt: {self.kayit_yolu}"
            )
            self.get_logger().info(
                "========================================"
            )
        else:
            # Aynı komut tekrar geldiyse topic'i tekrar tazele.
            self._renk_yayinla(kod)

        # Aynı komutun tekrarı idempotent: yine ACCEPTED.
        self._ack_gonder(
            mavutil.mavlink.MAV_RESULT_ACCEPTED
        )


def main(args=None):
    rclpy.init(args=args)
    node = IdaHedefRenkAlici()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
