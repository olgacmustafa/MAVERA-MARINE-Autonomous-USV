"""MAVERA YKI - IHA hedef rengini IDA'ya aktaran iki RFD'li kopru.

RFD aygıtları USB seri numarasına göre bulunabilir. Seri numaraları
MAVERA_IHA_RFD_SERIAL ve MAVERA_IDA_RFD_SERIAL ortam değişkenleriyle verilir.

Akis:
    IHA, NAMED_VALUE_INT/TGT_COLOR gonderir.
    YKI ayni renk kodunu art arda 3 kez dogrular.
    IDA hatti varsa YKI, IDA Jetson'a MAV_CMD_USER_1 komutu gonderir.
    IDA rengi diske yazinca COMMAND_ACK ile onay verir.
    Iki RFD'nin tum MAVLink trafigi ayri UDP kanallariyla Mission Planner'a
    cift yonlu aktarilir.

IDA RFD bagli degilse program kapanmaz. IHA-YKI hattini izlemeye devam eder ve
IDA RFD'yi arka planda yeniden arar. --sadece-iha secenegi kullanilirsa IDA
hatti hic aranmaz ve program yalnizca IHA-YKI canli baglanti testinde calisir.

Seri portlari yalnizca bu program acar. Mission Planner COM portlarini degil,
IHA icin UDP 14550'yi ve IDA icin UDP 14560'i kullanir.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import sys
import time

try:
    from pymavlink import mavutil
    from serial.tools import list_ports
except ImportError as hata:
    raise SystemExit(
        "Gerekli paket bulunamadi. CMD'de su komutu calistir:\n"
        "py -m pip install pymavlink pyserial"
    ) from hata


IHA_RFD_SERI_NO = os.environ.get("MAVERA_IHA_RFD_SERIAL", "").strip()
IDA_RFD_SERI_NO = os.environ.get("MAVERA_IDA_RFD_SERIAL", "").strip()
DENENECEK_BAUDLAR = (57600, 115200)
BAUD_DENEME_SURESI = 6.0
IDA_YENIDEN_DENEME_ARALIGI = 5.0
IHA_CANLI_DURUM_ARALIGI = 1.0

MISSION_PLANNER_IP = "127.0.0.1"
IHA_MP_DINLEME_PORTU = 14550
IHA_KOPRU_YEREL_PORTU = 14551
IDA_MP_DINLEME_PORTU = 14560
IDA_KOPRU_YEREL_PORTU = 14561
UDP_OKUMA_SINIRI = 100

YKI_SISTEM_ID = 201
YKI_BILESEN_ID = 190
IHA_JETSON_SISTEM_ID = 200
IHA_JETSON_BILESEN_ID = 191
IDA_JETSON_SISTEM_ID = 202
IDA_JETSON_BILESEN_ID = 191

RENK_DOGRULAMA_ADEDI = 3
ADAY_PAKET_ZAMAN_ASIMI = 2.0
KOMUT_GONDERIM_ARALIGI = 0.5
HEARTBEAT_ARALIGI = 1.0
PROTOKOL_SURUMU = 1.0
PROTOKOL_IMZASI = 2026.0

RENKLER = {
    1: "SIYAH",
    2: "KIRMIZI",
    3: "YESIL",
}

KOMUT_ID = mavutil.mavlink.MAV_CMD_USER_1
KAYIT_YOLU = Path.home() / "MaveraYKI" / "hedef_durumu.json"


class MissionPlannerUdpKanali:
    """Bir RFD seri hattini Mission Planner'a cift yonlu UDP ile baglar."""

    def __init__(self, ad, mp_portu, yerel_port):
        self.ad = ad
        self.mp_adresi = (MISSION_PLANNER_IP, mp_portu)
        self.yerel_port = yerel_port
        self.mission_planner_goruldu = False
        self.soket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.soket.setblocking(False)

        try:
            self.soket.bind((MISSION_PLANNER_IP, yerel_port))
        except OSError as hata:
            self.soket.close()
            raise SystemExit(
                f"HATA: {ad} UDP yerel portu {yerel_port} acilamadi. "
                "Koprunun eski bir kopyasi acik olabilir.\n"
                f"Ayrinti: {hata}"
            ) from hata

        print(
            f"{ad} Mission Planner kanali hazir | "
            f"MP UDP={mp_portu} | KOPRU UDP={yerel_port}"
        )

    def mavlink_mesajini_mpye_gonder(self, mesaj):
        if mesaj is None or mesaj.get_type() == "BAD_DATA":
            return

        ham_veri = mesaj.get_msgbuf()
        if not ham_veri:
            return

        try:
            self.soket.sendto(bytes(ham_veri), self.mp_adresi)
        except OSError as hata:
            # Windows, henuz dinleyici yokken UDP ICMP hatasi uretebilir.
            # Mission Planner daha sonra acildiginda yeni paketler aktarilir.
            if getattr(hata, "winerror", None) != 10054:
                raise

    def mpden_seriye_aktar(self, seri_baglanti):
        """UDP verisini seriye yollar; baglanti None ise guvenle bosaltir."""
        aktarilan = 0

        while aktarilan < UDP_OKUMA_SINIRI:
            try:
                ham_veri, kaynak = self.soket.recvfrom(65535)
            except BlockingIOError:
                break
            except ConnectionResetError:
                # Mission Planner kapaliyken Windows'ta gorulebilen UDP hatasi.
                break
            except OSError as hata:
                if getattr(hata, "winerror", None) == 10054:
                    break
                raise

            if not ham_veri:
                continue

            # Soket yalnizca loopback'e baglidir. Yine de kaynak adresini kontrol
            # ederek baska bir arayuzden komut kabul edilmesini engelliyoruz.
            if kaynak[0] != MISSION_PLANNER_IP:
                continue

            # IDA RFD takili degilken eski Mission Planner komutlarini kuyrukta
            # tutma. Baglanti sonradan gelirse bayat komutlar araca gitmesin.
            if seri_baglanti is None:
                aktarilan += 1
                continue

            seri_baglanti.write(ham_veri)
            aktarilan += 1

            if not self.mission_planner_goruldu:
                self.mission_planner_goruldu = True
                print(f"{self.ad}: Mission Planner cift yonlu baglandi.")

        return aktarilan

    def kapat(self):
        self.soket.close()


def mesaj_adini_duzelt(deger):
    if isinstance(deger, bytes):
        deger = deger.decode("ascii", errors="ignore")
    return str(deger).rstrip("\x00").strip()


def atomik_json_yaz(veri):
    KAYIT_YOLU.parent.mkdir(parents=True, exist_ok=True)
    gecici = KAYIT_YOLU.with_suffix(".tmp")
    gecici.write_text(
        json.dumps(veri, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    gecici.replace(KAYIT_YOLU)


def durum_kaydet(durum, kod=None, renk=None, ayrinti=None):
    atomik_json_yaz(
        {
            "durum": durum,
            "kod": kod,
            "renk": renk,
            "ayrinti": ayrinti,
            "zaman_utc": datetime.now(timezone.utc).isoformat(),
        }
    )


def seri_numarasini_oku(port_bilgisi):
    """PySerial bilgisinden USB seri numarasini guvenli sekilde alir."""
    seri_no = getattr(port_bilgisi, "serial_number", None)
    if seri_no:
        return str(seri_no).strip().upper()

    # Bazi Windows/PySerial surumlerinde serial_number alani bos olsa da
    # ayni bilgi HWID icinde "SER=..." olarak bulunur.
    for parca in str(getattr(port_bilgisi, "hwid", "")).split():
        if parca.upper().startswith("SER="):
            return parca.split("=", 1)[1].strip().upper()

    return ""


def rfd_portunu_seri_no_ile_bul(seri_no, ad):
    """COM numarasindan bagimsiz olarak dogru RFD portunu bulur."""
    hedef_seri = seri_no.strip().upper()
    eslesen_portlar = []
    usb_portlari = []

    for bilgi in list_ports.comports():
        bulunan_seri = seri_numarasini_oku(bilgi)
        if bulunan_seri:
            usb_portlari.append(f"{bilgi.device}/SER={bulunan_seri}")
        if bulunan_seri == hedef_seri:
            eslesen_portlar.append(str(bilgi.device).upper())

    if len(eslesen_portlar) == 1:
        port = eslesen_portlar[0]
        print(
            f"{ad}: otomatik port bulundu | "
            f"SERI={hedef_seri} | PORT={port}"
        )
        return port

    if len(eslesen_portlar) > 1:
        raise SystemExit(
            f"HATA: {ad} seri numarasi {hedef_seri} birden fazla portta "
            f"gorundu: {', '.join(eslesen_portlar)}"
        )

    gorulen = ", ".join(usb_portlari) if usb_portlari else "USB seri port yok"
    raise SystemExit(
        f"HATA: {ad} bulunamadi | Beklenen RFD SERI={hedef_seri}.\n"
        f"Gorulen USB portlar: {gorulen}\n"
        "RFD'nin USB kablosunu kontrol et ve programi yeniden baslat."
    )


def opsiyonel_rfd_portunu_bul(seri_no):
    """RFD yoksa hata vermeden None dondurur; sonradan tekrar denenebilir."""
    hedef_seri = seri_no.strip().upper()
    eslesen_portlar = []

    for bilgi in list_ports.comports():
        if seri_numarasini_oku(bilgi) == hedef_seri:
            eslesen_portlar.append(str(bilgi.device).upper())

    if len(eslesen_portlar) == 1:
        return eslesen_portlar[0]

    if len(eslesen_portlar) > 1:
        print(
            f"UYARI: IDA RFD SERI={hedef_seri} birden fazla portta gorundu: "
            f"{', '.join(eslesen_portlar)}. Tek RFD birakilana kadar bekleniyor."
        )

    return None


def port_listesi_uyarisi(port):
    mevcut = [bilgi.device for bilgi in list_ports.comports()]
    if port.upper() not in {deger.upper() for deger in mevcut}:
        liste = ", ".join(mevcut) if mevcut else "hic port listelenmedi"
        print(
            f"UYARI: {port} Python port listesinde gorunmedi ({liste}). "
            "Aygit Yoneticisi portu gosterdigi icin acmayi yine de deniyorum."
        )


def mavlink_mesaji_bekle(baglanti, sure):
    bitis = time.monotonic() + sure
    while time.monotonic() < bitis:
        mesaj = baglanti.recv_match(blocking=True, timeout=0.5)
        if mesaj is not None and mesaj.get_type() != "BAD_DATA":
            return mesaj
    return None


def baglanti_ac(port, ad, sabit_baud=None):
    port_listesi_uyarisi(port)
    baudlar = (sabit_baud,) if sabit_baud else DENENECEK_BAUDLAR
    son_hata = None

    for baud in baudlar:
        print(f"{ad}: {port} aciliyor | {baud} baud deneniyor...")
        try:
            baglanti = mavutil.mavlink_connection(
                port,
                baud=baud,
                source_system=YKI_SISTEM_ID,
                source_component=YKI_BILESEN_ID,
                autoreconnect=True,
            )
        except Exception as hata:
            son_hata = hata
            break

        ilk_mesaj = mavlink_mesaji_bekle(baglanti, BAUD_DENEME_SURESI)
        if ilk_mesaj is not None:
            print(f"{ad}: MAVLink bulundu | PORT={port} | BAUD={baud}")
            return baglanti, baud

        baglanti.close()

    if son_hata is not None:
        raise SystemExit(
            f"HATA: {ad} portu {port} acilamadi. Mission Planner veya "
            f"QGroundControl bu portu kullaniyorsa kapat.\nAyrinti: {son_hata}"
        )

    raise SystemExit(
        f"HATA: {ad} portu acildi fakat MAVLink verisi gelmedi. "
        "RFD hava modemi, anten, Cube ve telemetri ayarlari kontrol edilmeli."
    )


def opsiyonel_ida_baglantisi_ac(port, baud):
    """IDA yer RFD'sini MAVLink gelmesini beklemeden, hizli ve hatasiz dener."""
    port_listesi_uyarisi(port)
    try:
        baglanti = mavutil.mavlink_connection(
            port,
            baud=baud,
            source_system=YKI_SISTEM_ID,
            source_component=YKI_BILESEN_ID,
            autoreconnect=True,
        )
    except Exception as hata:
        return None, str(hata)

    return baglanti, None


def heartbeat_gonder(baglanti):
    baglanti.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def ida_komutu_gonder(baglanti, kod, islem_no, tekrar):
    baglanti.mav.command_long_send(
        IDA_JETSON_SISTEM_ID,
        IDA_JETSON_BILESEN_ID,
        KOMUT_ID,
        min(max(tekrar - 1, 0), 255),
        float(kod),
        PROTOKOL_SURUMU,
        float(islem_no),
        PROTOKOL_IMZASI,
        0.0,
        0.0,
        0.0,
    )


def cube_sysidini_oku(mesaj):
    if mesaj is None or mesaj.get_type() != "HEARTBEAT":
        return None
    if mesaj.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
        return None
    if int(mesaj.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return None
    return int(mesaj.get_srcSystem())


def main():
    ayrac = argparse.ArgumentParser(
        description=(
            "IHA hedef rengini izler; IDA hatti varsa otomatik aktarir."
        )
    )
    ayrac.add_argument(
        "--sadece-iha",
        action="store_true",
        help=(
            "IDA RFD'yi aramaz ve IDA'ya veri gondermez. Yalnizca "
            "IHA-YKI canli baglanti testi yapar."
        ),
    )
    ayrac.add_argument(
        "--iha-port",
        help=(
            "Normalde verilmez; IHA RFD otomatik bulunur. "
            "Sadece ariza testinde COM13 gibi elle port vermek icindir."
        ),
    )
    ayrac.add_argument(
        "--ida-port",
        help=(
            "Normalde verilmez; IDA RFD otomatik bulunur. "
            "Sadece ariza testinde COM12 gibi elle port vermek icindir."
        ),
    )
    ayrac.add_argument(
        "--iha-baud",
        type=int,
        help="Verilmezse 57600 ve 115200 otomatik denenir.",
    )
    ayrac.add_argument(
        "--ida-baud",
        type=int,
        help=(
            "IDA opsiyonel oldugu icin varsayilan 57600'dur. IDA RFD "
            "115200 ise --ida-baud 115200 ver."
        ),
    )
    ayrac.add_argument(
        "--iha-mp-port",
        type=int,
        default=IHA_MP_DINLEME_PORTU,
        help="Mission Planner'in IHA icin dinleyecegi UDP portu.",
    )
    ayrac.add_argument(
        "--ida-mp-port",
        type=int,
        default=IDA_MP_DINLEME_PORTU,
        help="Mission Planner'in IDA icin dinleyecegi UDP portu.",
    )
    ayrac.add_argument(
        "--iha-kopru-port",
        type=int,
        default=IHA_KOPRU_YEREL_PORTU,
        help="Koprunun IHA UDP donus trafigi icin yerel portu.",
    )
    ayrac.add_argument(
        "--ida-kopru-port",
        type=int,
        default=IDA_KOPRU_YEREL_PORTU,
        help="Koprunun IDA UDP donus trafigi icin yerel portu.",
    )
    ayarlar = ayrac.parse_args()

    if not ayarlar.iha_port and not IHA_RFD_SERI_NO:
        ayrac.error(
            "IHA RFD tanimsiz: MAVERA_IHA_RFD_SERIAL veya --iha-port belirtin."
        )
    if not ayarlar.sadece_iha and not ayarlar.ida_port and not IDA_RFD_SERI_NO:
        ayrac.error(
            "IDA RFD tanimsiz: MAVERA_IDA_RFD_SERIAL veya --ida-port belirtin."
        )

    iha_port = (
        ayarlar.iha_port.strip().upper()
        if ayarlar.iha_port
        else rfd_portunu_seri_no_ile_bul(IHA_RFD_SERI_NO, "IHA RFD")
    )
    ida_port = ayarlar.ida_port.strip().upper() if ayarlar.ida_port else None

    if ida_port is not None and iha_port == ida_port:
        raise SystemExit("HATA: IHA ve IDA icin ayni COM portu kullanilamaz.")

    udp_portlari = [ayarlar.iha_mp_port, ayarlar.iha_kopru_port]
    if not ayarlar.sadece_iha:
        udp_portlari.extend([ayarlar.ida_mp_port, ayarlar.ida_kopru_port])

    if len(set(udp_portlari)) != len(udp_portlari) or any(
        port < 1 or port > 65535 for port in udp_portlari
    ):
        raise SystemExit(
            "HATA: Kullanilan Mission Planner/Kopru UDP portlari farkli ve "
            "1-65535 araliginda olmalidir."
        )

    print("MAVERA YKI IHA -> IDA hedef renk koprusu basliyor.")
    if ayarlar.sadece_iha:
        print("MOD: SADECE IHA-YKI CANLI BAGLANTI TESTI")
        print(f"IHA girisi: {iha_port} | IDA hatti: KAPALI")
    else:
        print(
            f"IHA girisi: {iha_port} | "
            f"IDA cikisi: {ida_port or 'OTOMATIK BEKLENIYOR'}"
        )
    print(
        "Mission Planner RFD COM portlarini ACMAYACAK; "
        "bagli araclara UDP uzerinden baglanacak."
    )
    durum_kaydet(
        "SADECE_IHA_TESTI_BASLIYOR"
        if ayarlar.sadece_iha
        else "BAGLANTILAR_BEKLENIYOR"
    )

    iha_baglanti = None
    ida_baglanti = None
    iha_mp_kanali = None
    ida_mp_kanali = None

    try:
        # IHA hatti ana giristir ve zorunludur. IDA hatti artik opsiyoneldir.
        iha_baglanti, iha_baud = baglanti_ac(
            iha_port,
            "IHA RFD",
            ayarlar.iha_baud,
        )

        iha_mp_kanali = MissionPlannerUdpKanali(
            "IHA",
            ayarlar.iha_mp_port,
            ayarlar.iha_kopru_port,
        )
        if not ayarlar.sadece_iha:
            ida_mp_kanali = MissionPlannerUdpKanali(
                "IDA",
                ayarlar.ida_mp_port,
                ayarlar.ida_kopru_port,
            )

        print(f"IHA RFD HAZIR | {iha_port}/{iha_baud}")
        if ayarlar.sadece_iha:
            print(
                "MISSION PLANNER HAZIRLIK: "
                f"IHA=UDP {ayarlar.iha_mp_port}"
            )
        else:
            print(
                "MISSION PLANNER HAZIRLIK: "
                f"IHA=UDP {ayarlar.iha_mp_port} | "
                f"IDA=UDP {ayarlar.ida_mp_port}"
            )
            print(
                "IDA RFD su an zorunlu degil. Bagli degilse IHA izlenecek; "
                "takildiginda otomatik acilacak."
            )

        print("IHA'dan TGT_COLOR bekleniyor... Cikmak icin Ctrl+C")
        durum_kaydet(
            "SADECE_IHA_RENGI_BEKLENIYOR"
            if ayarlar.sadece_iha
            else "IHA_RENGI_BEKLENIYOR"
        )

        aday_kod = None
        aday_sayisi = 0
        son_aday_zamani = 0.0
        son_iha_canli_yazisi = 0.0
        kesin_kod = None
        islem_no = int(time.time()) % 10_000_000
        ida_onayladi = False
        gonderim_sayisi = 0
        son_komut_zamani = 0.0
        son_iha_heartbeat = 0.0
        son_ida_heartbeat = 0.0
        iha_cube_sysid = None
        ida_cube_sysid = None
        ida_hava_hatti_hazir = False
        sysid_kontrol_edildi = False
        sonraki_ida_denemesi = 0.0
        son_ida_durumu = None

        while True:
            simdi = time.monotonic()

            # Normal modda IDA yer RFD'si yoksa program kapanmaz. USB portu
            # belirli araliklarla aranir ve takildigi anda hizlica acilir.
            if (
                not ayarlar.sadece_iha
                and ida_baglanti is None
                and simdi >= sonraki_ida_denemesi
            ):
                aday_ida_port = (
                    ayarlar.ida_port.strip().upper()
                    if ayarlar.ida_port
                    else opsiyonel_rfd_portunu_bul(IDA_RFD_SERI_NO)
                )
                sonraki_ida_denemesi = (
                    simdi + IDA_YENIDEN_DENEME_ARALIGI
                )

                if aday_ida_port is None:
                    yeni_durum = (
                        "IDA RFD BAGLI DEGIL | IHA-YKI izlenmeye devam "
                        "ediliyor; IDA portu yeniden aranacak."
                    )
                    if yeni_durum != son_ida_durumu:
                        print(yeni_durum)
                        son_ida_durumu = yeni_durum
                elif aday_ida_port == iha_port:
                    yeni_durum = (
                        "IDA RFD icin IHA ile ayni COM portu bulundu; "
                        "IDA baglantisi bekleniyor."
                    )
                    if yeni_durum != son_ida_durumu:
                        print(f"UYARI: {yeni_durum}")
                        son_ida_durumu = yeni_durum
                else:
                    ida_baud = ayarlar.ida_baud or DENENECEK_BAUDLAR[0]
                    yeni_baglanti, ida_hatasi = opsiyonel_ida_baglantisi_ac(
                        aday_ida_port,
                        ida_baud,
                    )
                    if yeni_baglanti is None:
                        yeni_durum = (
                            f"IDA RFD acilamadi ({aday_ida_port}): "
                            f"{ida_hatasi} | IHA-YKI izleniyor."
                        )
                        if yeni_durum != son_ida_durumu:
                            print(f"UYARI: {yeni_durum}")
                            son_ida_durumu = yeni_durum
                    else:
                        ida_baglanti = yeni_baglanti
                        ida_port = aday_ida_port
                        ida_hava_hatti_hazir = False
                        ida_cube_sysid = None
                        sysid_kontrol_edildi = False
                        son_ida_heartbeat = 0.0
                        son_komut_zamani = 0.0
                        son_ida_durumu = "IDA_YER_RFD_ACIK"
                        print(
                            "IDA YER RFD ACILDI | "
                            f"PORT={ida_port} | BAUD={ida_baud} | "
                            "IDA hava MAVLink verisi bekleniyor."
                        )
                        durum_kaydet(
                            "IDA_HAVA_HATTI_BEKLENIYOR",
                            kesin_kod,
                            RENKLER.get(kesin_kod),
                        )

            if simdi - son_iha_heartbeat >= HEARTBEAT_ARALIGI:
                heartbeat_gonder(iha_baglanti)
                son_iha_heartbeat = simdi

            # Mission Planner'dan IHA'ya gelen komutlari aktar.
            iha_mp_kanali.mpden_seriye_aktar(iha_baglanti)

            # IHA hattindaki butun hazir mesajlari isle.
            while True:
                mesaj = iha_baglanti.recv_match(blocking=False)
                if mesaj is None:
                    break

                iha_mp_kanali.mavlink_mesajini_mpye_gonder(mesaj)
                bulunan_sysid = cube_sysidini_oku(mesaj)
                if bulunan_sysid is not None and iha_cube_sysid is None:
                    iha_cube_sysid = bulunan_sysid
                    print(f"IHA Cube SYSID bulundu: {iha_cube_sysid}")

                if mesaj.get_type() != "NAMED_VALUE_INT":
                    continue

                if mesaj_adini_duzelt(mesaj.name) != "TGT_COLOR":
                    continue

                if mesaj.get_srcSystem() != IHA_JETSON_SISTEM_ID:
                    continue

                if mesaj.get_srcComponent() != IHA_JETSON_BILESEN_ID:
                    continue

                kod = int(mesaj.value)
                if kod not in RENKLER:
                    continue

                if (
                    son_iha_canli_yazisi == 0.0
                    or simdi - son_iha_canli_yazisi
                    >= IHA_CANLI_DURUM_ARALIGI
                ):
                    print(
                        f"IHA-YKI CANLI | TGT_COLOR={RENKLER[kod]} "
                        f"(KOD {kod}) | baglanti aktif"
                    )
                    son_iha_canli_yazisi = simdi

                if kesin_kod is not None:
                    continue

                if (
                    kod == aday_kod
                    and simdi - son_aday_zamani <= ADAY_PAKET_ZAMAN_ASIMI
                ):
                    aday_sayisi += 1
                else:
                    aday_kod = kod
                    aday_sayisi = 1
                son_aday_zamani = simdi

                print(
                    f"IHA ADAY: {RENKLER[kod]} | "
                    f"dogrulama {aday_sayisi}/{RENK_DOGRULAMA_ADEDI}"
                )

                if aday_sayisi >= RENK_DOGRULAMA_ADEDI:
                    kesin_kod = kod
                    if ayarlar.sadece_iha:
                        print(
                            f"IHA RENGI KESIN: {RENKLER[kod]} | "
                            "SADECE-IHA testi tamam; IDA'ya gonderilmeyecek."
                        )
                        durum_kaydet(
                            "SADECE_IHA_TEST_RENGI_ALINDI",
                            kod,
                            RENKLER[kod],
                        )
                    elif ida_baglanti is None:
                        print(
                            f"IHA RENGI KESIN: {RENKLER[kod]} | "
                            "IDA bagli degil; renk bellekte tutuluyor ve "
                            "IDA baglaninca otomatik aktarilacak."
                        )
                        durum_kaydet(
                            "IDA_BAGLANTISI_BEKLENIYOR",
                            kod,
                            RENKLER[kod],
                        )
                    else:
                        print(
                            f"IHA RENGI KESIN: {RENKLER[kod]} | "
                            "IDA'ya aktarim icin hava hatti bekleniyor."
                        )
                        durum_kaydet(
                            "IDA_HAVA_HATTI_BEKLENIYOR",
                            kod,
                            RENKLER[kod],
                        )

            # IDA yokken UDP'de eski Mission Planner komutu biriktirme.
            if ida_baglanti is None:
                if ida_mp_kanali is not None:
                    ida_mp_kanali.mpden_seriye_aktar(None)
                time.sleep(0.02)
                continue

            # IDA seri hatti tamamen opsiyoneldir. Kablo cikarsa yalnizca bu
            # blok kapanir; IHA-YKI izlemesi kesintisiz surer.
            try:
                if simdi - son_ida_heartbeat >= HEARTBEAT_ARALIGI:
                    heartbeat_gonder(ida_baglanti)
                    son_ida_heartbeat = simdi

                ida_mp_kanali.mpden_seriye_aktar(ida_baglanti)

                # IDA hatti uzerinden telemetri ve ACK mesajlarini izle.
                while True:
                    mesaj = ida_baglanti.recv_match(blocking=False)
                    if mesaj is None:
                        break

                    if mesaj.get_type() == "BAD_DATA":
                        continue

                    ida_mp_kanali.mavlink_mesajini_mpye_gonder(mesaj)

                    if not ida_hava_hatti_hazir:
                        ida_hava_hatti_hazir = True
                        print("IDA HAVA HATTI AKTIF | MAVLink verisi alindi.")
                        durum_kaydet(
                            "IDA_ONAYI_BEKLENIYOR"
                            if kesin_kod is not None
                            else "IHA_RENGI_BEKLENIYOR",
                            kesin_kod,
                            RENKLER.get(kesin_kod),
                        )

                    bulunan_sysid = cube_sysidini_oku(mesaj)
                    if bulunan_sysid is not None and ida_cube_sysid is None:
                        ida_cube_sysid = bulunan_sysid
                        print(f"IDA Cube SYSID bulundu: {ida_cube_sysid}")

                    if mesaj.get_type() != "COMMAND_ACK":
                        continue

                    if int(mesaj.command) != KOMUT_ID:
                        continue

                    if mesaj.get_srcSystem() != IDA_JETSON_SISTEM_ID:
                        continue

                    if mesaj.get_srcComponent() != IDA_JETSON_BILESEN_ID:
                        continue

                    if ida_onayladi or kesin_kod is None:
                        continue

                    sonuc = int(mesaj.result)
                    if sonuc == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        ida_onayladi = True
                        print("\n========================================")
                        print(
                            f"IDA HEDEF RENGI YUKLEDI: "
                            f"{RENKLER[kesin_kod]} (KOD {kesin_kod})"
                        )
                        print("YKİ -> IDA renk aktarimi tamamlandi.")
                        print("========================================")
                        durum_kaydet(
                            "IDA_YUKLEDI",
                            kesin_kod,
                            RENKLER[kesin_kod],
                            f"{gonderim_sayisi} gonderimde onay alindi",
                        )
                    else:
                        print(
                            f"IDA KOMUTU REDDETTI | MAV_RESULT={sonuc}. "
                            "Aktarim tekrar denenecek."
                        )

                if (
                    not sysid_kontrol_edildi
                    and iha_cube_sysid is not None
                    and ida_cube_sysid is not None
                ):
                    sysid_kontrol_edildi = True
                    if iha_cube_sysid == ida_cube_sysid:
                        print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print(
                            "UYARI: IHA ve IDA Cube SYSID degerleri ayni: "
                            f"{iha_cube_sysid}"
                        )
                        print(
                            "Mission Planner'in iki araci ayirmasi icin "
                            "SYSID_THISMAV degerleri farkli yapilmalidir."
                        )
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
                    else:
                        print(
                            "CUBE SYSID KONTROLU TAMAM | "
                            f"IHA={iha_cube_sysid} | IDA={ida_cube_sysid}"
                        )

                if (
                    kesin_kod is not None
                    and ida_hava_hatti_hazir
                    and not ida_onayladi
                    and simdi - son_komut_zamani
                    >= KOMUT_GONDERIM_ARALIGI
                ):
                    gonderim_sayisi += 1
                    ida_komutu_gonder(
                        ida_baglanti,
                        kesin_kod,
                        islem_no,
                        gonderim_sayisi,
                    )
                    son_komut_zamani = simdi
                    print(
                        f"IDA'ya gonderiliyor: {RENKLER[kesin_kod]} | "
                        f"deneme {gonderim_sayisi}"
                    )

            except Exception as hata:
                print(
                    f"UYARI: IDA RFD hatti koptu: {hata} | "
                    "IHA-YKI izlenmeye devam ediliyor."
                )
                try:
                    ida_baglanti.close()
                except Exception:
                    pass
                ida_baglanti = None
                ida_port = None
                ida_hava_hatti_hazir = False
                ida_cube_sysid = None
                sysid_kontrol_edildi = False
                sonraki_ida_denemesi = (
                    time.monotonic() + IDA_YENIDEN_DENEME_ARALIGI
                )
                son_ida_durumu = None
                durum_kaydet(
                    "IDA_BAGLANTISI_BEKLENIYOR",
                    kesin_kod,
                    RENKLER.get(kesin_kod),
                    str(hata),
                )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nYKI hedef renk koprusu kapatildi.")
    finally:
        if iha_baglanti is not None:
            iha_baglanti.close()
        if ida_baglanti is not None:
            ida_baglanti.close()
        if iha_mp_kanali is not None:
            iha_mp_kanali.kapat()
        if ida_mp_kanali is not None:
            ida_mp_kanali.kapat()


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as hata:
        print(f"BEKLENMEYEN HATA: {hata}", file=sys.stderr)
        raise
