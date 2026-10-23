"""AR0234 kamerayla canlı hedef renk algılama ve MAVLink telemetri aktarımı.

Işık geçişi yönetimi:
    Ani golge/tam gunes gecisinde RAW donusumunu ve white-balance toparlanmasini
    guvenli yapar; goruntu dengelenirken yeni YOLO/HSV/RFD karari uretmez.
    GStreamer akisi basladiktan sonra gunes pozlamasini exposure=200 ve
    analogue_gain=100 olarak uygular ve geri okuyarak dogrular.

Algılama, kamera kontrolü ve telemetri bu betikte yönetilir.

Tuslar:
    Q : programdan cik
    R : kilitlenmis kesin karari sifirla
    S : ham ve isaretlenmis kamera goruntusunu kaydet

TX16S SG anahtari Cube'un RC7 kanalini kumanda eder:
    RC7 988/1500 : plaka arama kapali
    RC7 2012      : plaka arama acik

Guvenlik icin program her acildiginda once SG'nin kapali konumunu gormelidir.
Cube'dan RC verisi kesilirse plaka arama, renk kilidi ve RFD gonderimi durur.
40-pin basliktaki fiziksel Pin 11'e bagli durum LED'i kullanilir.
IHA hedef rengini YKI'ye gondermeye baslayinca LED 3 saniyede bir, YKI IDA'dan
COMMAND_ACK alinca saniyede bir yaklasik 150 ms yanip soner. LED'in eksi bacagi
GND Pin 14'e baglanir.

Program basladigi anda islenmis kamera goruntusunu MP4'e ve her kayit karesine
ait plaka/renk/RFD bilgilerini CSV'ye otomatik olarak kaydeder.
"""

from collections import Counter, deque
import csv
from datetime import datetime
from zoneinfo import ZoneInfo
import os
from pathlib import Path
import gc
import shutil
import subprocess
import time
import warnings

import cv2
import numpy as np
import torch

try:
    import Jetson.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    from pymavlink import mavutil
except ImportError as hata:
    raise SystemExit(
        "HATA: pymavlink bulunamadi. Sanal ortam acikken su komutu calistir:\n"
        "python -m pip install pymavlink==2.4.49"
    ) from hata

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError) as hata:
    raise SystemExit(
        "HATA: Python GStreamer paketi bulunamadi. Su komutu calistir:\n"
        "sudo apt install python3-gi gir1.2-gstreamer-1.0"
    ) from hata

# Kullanılmayan Matplotlib 3B bileşeninin uyarısını filtrele.
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

from ultralytics import YOLO


# ---------------------------------------------------------------------------
# AYARLAR
# ---------------------------------------------------------------------------

PROJE_KOKU = Path(__file__).resolve().parents[1]
MODEL_YOLU = PROJE_KOKU / "models" / "best.pt"
EKRAN_GORUNTUSU_KLASORU = PROJE_KOKU / "camera_captures"
YARIS_KAYIT_KLASORU = PROJE_KOKU / "kayitlar"
TURKIYE_SAATI = ZoneInfo("Europe/Istanbul")

# Görüntüleme masaüstünde açılabilir; MAVERA_EKRAN=0 ile başsız çalışmada
# önizleme kapatılır, algılama, kayıt ve telemetri gönderimi sürer.
EKRAN_AYARI = os.environ.get("MAVERA_EKRAN", "OTOMATIK").strip().upper()
if EKRAN_AYARI in {"1", "ACIK", "TRUE", "YES"}:
    EKRAN_GOSTER = True
elif EKRAN_AYARI in {"0", "KAPALI", "FALSE", "NO"}:
    EKRAN_GOSTER = False
else:
    EKRAN_GOSTER = bool(os.environ.get("DISPLAY"))

# AR0234 Jetvariety V4L2 ayarlari.
# Kamera /dev/video0 uzerinden Y16 icinde RAW10 veri verir. Gercek renk testi
# sonucunda dogru Bayer donusumu GB -> BGR olarak dogrulanmistir.
KAMERA_CIHAZ = "/dev/video0"
KAMERA_INDEX = 0
KAMERA_GENISLIK = 1280
KAMERA_YUKSEKLIK = 720

# Jetvariety sürücüsü otomatik exposure/ISP sağlamaz.
# Kontrol aralıkları: exposure=2..65535, analogue_gain=100..1200. RAW
# kare parlaklığından yazılımsal kapalı çevrim pozlama uygulanır.
# 1280x720 modu 120 FPS oldugu icin exposure ust siniri kare suresinin altinda
# tutulur. Once exposure kullanilir; yalnizca karanlikta analogue gain artar.
KAMERA_BASLANGIC_EXPOSURE = 200
KAMERA_BASLANGIC_ANALOG_GAIN = 100
KAMERA_KONTROL_DENEME_SAYISI = 4

AE_EXPOSURE_MIN = 20
AE_EXPOSURE_MAX = 6000
AE_GAIN_MIN = 100
AE_GAIN_MAX = 600
AE_GUNCELLEME_ARALIGI = 0.35
AE_PARLAKLIK_PERSENTILI = 99.5
AE_HEDEF_HAM_ORANI = 0.78
AE_DUSUK_ESIK_ORANI = 0.69
AE_YUKSEK_ESIK_ORANI = 0.87
AE_DOLU_PIKSEL_ESIGI = 65000
AE_DOLU_PIKSEL_ORANI = 0.003
AE_EN_AZ_CARPAN = 0.58
AE_EN_COK_CARPAN = 1.65
AE_LOG_ARALIGI = 1.0

# AR0234 RAW10, Y16 icinde sola yasli gelir (10 bit deger << 6). Bu nedenle
# ham beyaz seviyesi 1023 << 6 = 65472'dir. Her kareyi kendi en parlak
# pikseline gore esnetmek gunes gecisinde yalanci/termal renkler uretiyordu;
# artik sabit sensor beyaz seviyesine gore donusum yapilir.
ISP_RAW_BEYAZ_SEVIYE = float(1023 << 6)
ISP_RAW_BLACK_VARSAYILAN = float(64 << 6)
ISP_RAW_BLACK_MIN = float(32 << 6)
ISP_RAW_BLACK_MAX = float(128 << 6)

# Yazılımsal ISP parametreleri. White-balance, arama kapalıyken
# yavaşça güncellenir; RC7 ile plaka
# aramasi ACILINCA dondurulur. Boylece hedef plakanin kendi rengi WB'yi kaydirmaz.
ISP_BAYER_KODU = cv2.COLOR_BAYER_GB2BGR
ISP_GAMMA = 0.78
ISP_SATURATION = 1.25
ISP_PARLAKLIK = 1.00
ISP_WB_MIN = 0.65
ISP_WB_MAX = 1.80
ISP_WB_YUMUSATMA = 0.10

# Ani golge <-> tam gunes gecislerinde AR0234 RAW seviyesi birden degisebilir.
# Bu sirada eski white-balance kazanclariyla karar verilmesini engellemek icin
# degisimi goruntunun kenarlarindan izler, WB'yi hizli toparlar ve karar sistemini
# kisa sure bekletiriz. Sensore calisma aninda exposure/gain zorlanmaz.
ISP_ANI_ISIK_DEGISIM_ORANI = 0.22
ISP_ISIK_DENGELEME_KARESI = 20
ISP_WB_HIZLI_YUMUSATMA = 0.45
ISP_WB_NOTR_RENK_FARKI = 70.0
ISP_WB_MIN_PARLAKLIK = 25.0
ISP_WB_MAX_PARLAKLIK = 230.0

# Surekli yaris kaydi. Kamera 30 FPS calisir; islenmis goruntunun 10 karesi
# saniyede kaydedilir. Bu deger sartnamedeki en az 1 Hz sinirinin ustundedir.
VIDEO_KAYIT_FPS = 10.0
VIDEO_KODEKLERI = ("mp4v", "avc1")
MINIMUM_BOS_ALAN_GB = 3.0

# Jetson, Cube Orange+'a USB ile baglidir. Cube bu MAVLink mesajini TELEM1'e
# bagli IHA RFD'sine yonlendirir; YKI'daki RFD de ayni mesaji bilgisayara verir.
CUBE_BY_ID_KLASORU = Path("/dev/serial/by-id")
CUBE_PORT_DESENI = "usb-CubePilot_CubeOrange+_*-if00"
MAVLINK_BAUD = 115200
MAVLINK_KAYNAK_SISTEM = 200
MAVLINK_KAYNAK_BILESEN = 191
HEARTBEAT_BEKLEME_SURESI = 10
RFD_GONDERIM_ARALIGI = 0.5  # saniye; saniyede iki kez gonderir

# RFD durum LED'i: Jetson 40-pin baslik, BOARD numaralandirmasi.
# Fiziksel Pin 11 -> 330 ohm direnc -> LED uzun bacak (+)
# LED kisa bacak (-) -> fiziksel Pin 14 (GND)
RFD_LED_PIN = 11
RFD_LED_DARBE_SURESI = 0.15
RFD_LED_YKI_ARALIGI = 3.0   # IHA -> YKI asamasinda 3 saniyede bir
RFD_LED_IDA_ARALIGI = 1.0   # IDA ACK geldikten sonra saniyede bir

# YKI koprusunun IHA'ya geri bildirdigi asama mesaji.
YKI_SISTEM_ID = 201
YKI_BILESEN_ID = 190
YKI_DURUM_MESAJI = "TGT_STAGE"
YKI_SAAT_MESAJI = "YKI_TIME"
YKI_SAAT_BEKLEME_SURESI = 45.0
GECERLI_SAAT_MIN_UNIX = 1_735_689_600  # 2025-01-01 00:00:00 UTC
GECERLI_SAAT_MAX_UNIX = 2_051_222_400  # 2035-01-01 00:00:00 UTC
LED_DURUM_KAPALI = 0
LED_DURUM_YKI = 1
LED_DURUM_IDA = 2

# TX16S'te SG anahtari CH7'ye atanmistir. Mission Planner'da RC7 yaklasik
# 988 / 1500 / 2012 gorulur. Yalnizca 1700 ustu "arama acik" sayilir.
RC_ARAMA_KANALI = 7
RC_ARAMA_ACIK_ESIGI = 1700
RC_SINYAL_ZAMAN_ASIMI = 1.5
RC_MESAJ_HIZI_HZ = 10
RC_AKIS_YENILEME_ARALIGI = 2.0
RC_GECERLI_MIN = 800
RC_GECERLI_MAX = 2200

# Ucus guvenligi: Model guveni %70'in altindaki kutular karar sistemine girmez;
# dolayisiyla bu tespitlerden hedef rengi kilitlenmez ve RFD'ye gonderilmez.
YOLO_GUVEN_ESIGI = 0.70
YOLO_GIRIS_BOYUTU = 640

HSV_RENK_ORANI_ESIGI = 0.25

OY_PENCERESI = 10
GEREKLI_OY = 7

HEDEF_RENKLER = {"SIYAH", "KIRMIZI", "YESIL"}

# YKI tarafinda alinacak sayisal renk kodlari.
RENK_KODLARI = {
    "BELIRSIZ": 0,
    "SIYAH": 1,
    "KIRMIZI": 2,
    "YESIL": 3,
}

EKRAN_RENKLERI = {
    "SIYAH": (220, 220, 220),
    "KIRMIZI": (0, 0, 255),
    "YESIL": (0, 255, 0),
    "KARARSIZ": (0, 255, 255),
    "BEKLENIYOR": (255, 255, 255),
}


class ArducamKamera:
    """AR0234 RAW10 karelerini GStreamer/V4L2 ile alip BGR goruntuye cevirir.

    OpenCV yalnizca goruntu isleme icin kullanilir; kamera capture icin
    cv2.VideoCapture KULLANILMAZ. Bunun sebebi mavera .venv icindeki OpenCV
    paketinin V4L2 capture backend'ini kullanamamasidir.

    Islem zinciri:
        v4l2src Y16/RAW10 -> NumPy uint16 -> otomatik siyah/beyaz seviye
        -> GB Bayer demosaic -> yumusatilmis white balance
        -> saturation/parlaklik -> gamma
    """

    def __init__(self):
        self.pipeline = None
        self.appsink = None
        self.wb_gain = np.ones(3, dtype=np.float32)  # B, G, R
        self.wb_hazir = False
        self.ilk_kare_bilgisi_yazildi = False
        self.isik_referansi = None
        self.isik_dengeleme_kalan = ISP_ISIK_DENGELEME_KARESI
        self.isik_kararli = False
        self.isik_gecisi_sayisi = 0
        self.son_isik_log_zamani = 0.0
        self.ae_exposure = KAMERA_BASLANGIC_EXPOSURE
        self.ae_gain = KAMERA_BASLANGIC_ANALOG_GAIN
        self.ae_son_ayar_zamani = 0.0
        self.ae_son_log_zamani = 0.0
        self.ae_hata_log_zamani = 0.0
        self.ae_degisim_sayisi = 0

        self.gamma_lut = np.array(
            [
                ((i / 255.0) ** ISP_GAMMA) * 255.0
                for i in range(256)
            ],
            dtype=np.float32,
        ).clip(0, 255).astype(np.uint8)

    def _cihazi_bekle(self, zaman_asimi=10.0):
        son = time.monotonic() + zaman_asimi
        while time.monotonic() < son:
            if Path(KAMERA_CIHAZ).exists():
                return True
            time.sleep(0.25)
        return Path(KAMERA_CIHAZ).exists()

    def _sensor_kontrollerini_oku(self):
        """Surucunun gercek exposure/gain/FPS degerlerini degistirmeden okur."""
        komut = [
            "/usr/bin/v4l2-ctl",
            "-d",
            KAMERA_CIHAZ,
            "-C",
            "frame_rate,exposure,analogue_gain",
        ]
        try:
            sonuc = subprocess.run(
                komut,
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
        except Exception as hata:
            print(
                "UYARI: AR0234 surucu kontrolleri okunamadi; "
                f"kamera calismaya devam ediyor. Ayrinti: {hata}"
            )
            return None

        if sonuc.returncode != 0:
            ayrinti = (sonuc.stderr or sonuc.stdout or "").strip()
            print(
                "UYARI: AR0234 surucu kontrolleri okunamadi; "
                f"kamera calismaya devam ediyor. Ayrinti: {ayrinti}"
            )
            return None

        return " | ".join(
            satir.strip() for satir in sonuc.stdout.splitlines() if satir.strip()
        )

    def _sensor_pozlamasini_ayarla(self, exposure, gain, dogrula=False):
        """Exposure/gain uygular; acilista istenirse geri okuyarak dogrular."""
        exposure = int(np.clip(exposure, AE_EXPOSURE_MIN, AE_EXPOSURE_MAX))
        gain = int(np.clip(gain, AE_GAIN_MIN, AE_GAIN_MAX))
        komut = [
            "/usr/bin/v4l2-ctl",
            "-d",
            KAMERA_CIHAZ,
            "-c",
            f"exposure={exposure},analogue_gain={gain}",
        ]

        son_ayrinti = ""
        deneme_sayisi = KAMERA_KONTROL_DENEME_SAYISI if dogrula else 1
        for deneme in range(1, deneme_sayisi + 1):
            try:
                sonuc = subprocess.run(
                    komut,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            except Exception as hata:
                son_ayrinti = str(hata)
                time.sleep(0.25)
                continue

            if sonuc.returncode != 0:
                son_ayrinti = (sonuc.stderr or sonuc.stdout or "").strip()
                time.sleep(0.25)
                continue

            self.ae_exposure = exposure
            self.ae_gain = gain

            if not dogrula:
                return True

            time.sleep(0.15)
            okunan = self._sensor_kontrollerini_oku()
            if (
                okunan
                and f"exposure: {exposure}" in okunan
                and f"analogue_gain: {gain}" in okunan
            ):
                print(
                    "AR0234 OTOMATIK POZLAMA BASLANGICI DOGRULANDI | "
                    f"exposure={exposure} | gain={gain} | "
                    f"deneme={deneme}"
                )
                return True

            son_ayrinti = okunan or "kontroller geri okunamadi"
            time.sleep(0.25)

        print(
            "HATA: AR0234 sensor pozlamasi uygulanamadi | "
            f"Beklenen exposure={exposure}, gain={gain} | Ayrinti: {son_ayrinti}"
        )
        return False

    def _otomatik_pozlamayi_guncelle(self, ornek, black):
        """RAW parlakligindan exposure/gain'i golge ve gunese gore ayarlar."""
        simdi = time.monotonic()
        if simdi - self.ae_son_ayar_zamani < AE_GUNCELLEME_ARALIGI:
            return
        self.ae_son_ayar_zamani = simdi

        parlak = float(np.percentile(ornek, AE_PARLAKLIK_PERSENTILI))
        kullanilabilir_aralik = max(ISP_RAW_BEYAZ_SEVIYE - black, 1.0)
        ham_oran = float(
            np.clip((parlak - black) / kullanilabilir_aralik, 0.0, 1.5)
        )
        dolu_orani = float(np.mean(ornek >= AE_DOLU_PIKSEL_ESIGI))

        # Histerezis sayesinde ufak bulut/titresimlerde kontrol avlanmaz.
        if (
            AE_DUSUK_ESIK_ORANI <= ham_oran <= AE_YUKSEK_ESIK_ORANI
            and dolu_orani < AE_DOLU_PIKSEL_ORANI
        ):
            return

        if dolu_orani >= AE_DOLU_PIKSEL_ORANI:
            carpan = AE_EN_AZ_CARPAN
        else:
            carpan = AE_HEDEF_HAM_ORANI / max(ham_oran, 0.03)
            carpan = float(
                np.clip(carpan, AE_EN_AZ_CARPAN, AE_EN_COK_CARPAN)
            )

        # Toplam hassasiyet exposure * (gain / 100). Once gain'i minimumda
        # tutarak exposure ile ayarla; exposure tavana gelirse gain kullan.
        mevcut_hassasiyet = self.ae_exposure * (self.ae_gain / 100.0)
        hedef_hassasiyet = mevcut_hassasiyet * carpan

        if hedef_hassasiyet <= AE_EXPOSURE_MAX:
            yeni_exposure = int(round(hedef_hassasiyet))
            yeni_gain = AE_GAIN_MIN
        else:
            yeni_exposure = AE_EXPOSURE_MAX
            yeni_gain = int(
                round(100.0 * hedef_hassasiyet / AE_EXPOSURE_MAX)
            )

        yeni_exposure = int(
            np.clip(yeni_exposure, AE_EXPOSURE_MIN, AE_EXPOSURE_MAX)
        )
        yeni_gain = int(np.clip(yeni_gain, AE_GAIN_MIN, AE_GAIN_MAX))

        exposure_farki = abs(yeni_exposure - self.ae_exposure)
        gain_farki = abs(yeni_gain - self.ae_gain)
        if (
            exposure_farki < max(2, int(self.ae_exposure * 0.05))
            and gain_farki < 5
        ):
            return

        if not self._sensor_pozlamasini_ayarla(yeni_exposure, yeni_gain):
            if simdi - self.ae_hata_log_zamani >= AE_LOG_ARALIGI:
                print(
                    "UYARI: AR0234 otomatik pozlama sensor kontrolunu "
                    "yazamadi."
                )
                self.ae_hata_log_zamani = simdi
            return

        self.ae_degisim_sayisi += 1
        self.isik_dengeleme_kalan = ISP_ISIK_DENGELEME_KARESI
        self.isik_kararli = False

        if simdi - self.ae_son_log_zamani >= AE_LOG_ARALIGI:
            print(
                "AR0234 OTOMATIK POZLAMA | "
                f"ham=%{ham_oran * 100:.1f} | "
                f"dolu=%{dolu_orani * 100:.2f} | "
                f"exposure={yeni_exposure} | gain={yeni_gain}"
            )
            self.ae_son_log_zamani = simdi

    def _gst_hatti(self):
        # Arducam Y16, GStreamer tarafinda GRAY16_LE olarak temsil edilir.
        # Exposure/gain stream basladiktan sonra ayarlanir; caps'te zorlanmaz.
        return (
            f"v4l2src device={KAMERA_CIHAZ} ! "
            "video/x-raw,format=GRAY16_LE,"
            f"width=(int){KAMERA_GENISLIK},height=(int){KAMERA_YUKSEKLIK} ! "
            "appsink name=arducam_sink emit-signals=false "
            "drop=true max-buffers=1 sync=false"
        )

    def ac(self):
        if not self._cihazi_bekle():
            print(f"HATA: AR0234 V4L2 cihazi bulunamadi: {KAMERA_CIHAZ}")
            return False

        try:
            Gst.init(None)
            self.pipeline = Gst.parse_launch(self._gst_hatti())
            self.appsink = self.pipeline.get_by_name("arducam_sink")
            if self.appsink is None:
                print("HATA: GStreamer arducam_sink olusturulamadi.")
                return False

            sonuc = self.pipeline.set_state(Gst.State.PLAYING)
            if sonuc == Gst.StateChangeReturn.FAILURE:
                self.hata_yaz()
                print("HATA: AR0234 GStreamer pipeline baslatilamadi.")
                return False
        except Exception as hata:
            print(f"HATA: AR0234 GStreamer acilis hatasi: {hata}")
            return False

        # Surucu mode tablosu kontrolleri stream acilisinda varsayilana
        # dondurebildigi icin otomatik pozlamanin baslangic degeri mutlaka
        # PLAYING durumundan sonra uygulanir. Bundan sonra RAW geri bildirimi
        # exposure/gain'i golge ve gunese gore surekli ayarlar.
        self.pipeline.get_state(2 * Gst.SECOND)
        time.sleep(0.25)
        if not self._sensor_pozlamasini_ayarla(
            KAMERA_BASLANGIC_EXPOSURE,
            KAMERA_BASLANGIC_ANALOG_GAIN,
            dogrula=True,
        ):
            self.kapat()
            return False

        sensor_kontrolleri = self._sensor_kontrollerini_oku()

        print(
            "AR0234 KAMERA HAZIR (GStreamer/V4L2) | "
            f"{KAMERA_GENISLIK}x{KAMERA_YUKSEKLIK} | "
            "RAW=Y16 | Bayer=GB | OTOMATIK POZLAMA=ACIK"
        )
        if sensor_kontrolleri:
            print(f"AR0234 SURUCU KONTROLLERI | {sensor_kontrolleri}")
        return True

    def hata_yaz(self):
        if self.pipeline is None:
            return
        try:
            bus = self.pipeline.get_bus()
            mesaj = bus.timed_pop_filtered(0, Gst.MessageType.ERROR)
            if mesaj is not None:
                hata, ayrinti = mesaj.parse_error()
                print(f"GStreamer kamera hatasi: {hata}")
                if ayrinti:
                    print(f"GStreamer ayrintisi: {ayrinti}")
        except Exception as hata:
            print(f"GStreamer hata mesaji okunamadi: {hata}")

    def _isp_uygula(self, raw, wb_guncelle):
        # Percentile hesabini tum 921k pikselde yapmak gereksiz CPU harcar.
        ornek = raw[::8, ::8].astype(np.float32)
        black_olculen = float(np.percentile(ornek, 0.5))
        # Sahnedeki en koyu piksel sensorun siyah ofsetiyle ayni sey degildir.
        # Tam guneste butun sahne aydinlaninca p0.5 degeri cok yukseliyordu ve
        # onu cikarmak renk kanallarini ezip termal benzeri goruntu uretiyordu.
        # AR0234 RAW10 siyah ofseti 16-bit tasimada yaklasik 4096 civarindadir;
        # sicaklik/gurultu payi icin yalnizca 2048..8192 araliginda takip edilir.
        black = float(
            np.clip(black_olculen, ISP_RAW_BLACK_MIN, ISP_RAW_BLACK_MAX)
        )
        white = float(np.percentile(ornek, 99.7))

        # Jetvariety surucusu bu modelde exposure_auto sunmuyor. Telefon/webcam
        # gibi davranması için kapalı çevrim AE, RAW histogramından hesaplanır.
        self._otomatik_pozlamayi_guncelle(ornek, black)

        # Plaka genellikle goruntunun orta bolgesinde olur. Isik degisimini
        # kenarlardan olcmek, kirmizi/yesil plakanin kendisini ani gunes
        # degisimi sanma ihtimalini azaltir.
        oh, ow = ornek.shape
        ky = max(1, oh // 5)
        kx = max(1, ow // 5)
        isik_ornegi = np.concatenate(
            (
                ornek[:ky, :].ravel(),
                ornek[-ky:, :].ravel(),
                ornek[ky:-ky, :kx].ravel(),
                ornek[ky:-ky, -kx:].ravel(),
            )
        )
        isik_seviyesi = float(np.percentile(isik_ornegi, 90.0))

        ani_isik_degisimi = False
        if self.isik_referansi is None:
            self.isik_referansi = max(isik_seviyesi, 1.0)
        else:
            degisim_orani = abs(isik_seviyesi - self.isik_referansi) / max(
                self.isik_referansi,
                1.0,
            )
            if degisim_orani >= ISP_ANI_ISIK_DEGISIM_ORANI:
                ani_isik_degisimi = True
                self.isik_gecisi_sayisi += 1
                self.isik_dengeleme_kalan = ISP_ISIK_DENGELEME_KARESI
                self.isik_referansi = max(isik_seviyesi, 1.0)

                simdi = time.monotonic()
                if simdi - self.son_isik_log_zamani >= 1.0:
                    print(
                        "ANI ISIK DEGISIMI | "
                        f"oran=%{degisim_orani * 100:.0f} | "
                        "WB yeniden dengeleniyor; YOLO/RFD karari bekletiliyor."
                    )
                    self.son_isik_log_zamani = simdi
            else:
                # Yavas referans takibi; kucuk bulut/golge degisimleri yalanci
                # tetik uretmez ama yeni ortam seviyesi zamanla referans olur.
                self.isik_referansi = (
                    self.isik_referansi * 0.95 + isik_seviyesi * 0.05
                )

        if not ani_isik_degisimi and self.isik_dengeleme_kalan > 0:
            self.isik_dengeleme_kalan -= 1

        self.isik_kararli = self.isik_dengeleme_kalan == 0

        # Her karenin kendi %99.7 degerini beyaz yapmak sahne degisince renk ve
        # kontrasti ziplatiyordu. RAW10'un sabit fiziksel beyaz seviyesini kullan.
        raw_beyaz = max(ISP_RAW_BEYAZ_SEVIYE, black + 1.0)
        alpha = 255.0 / (raw_beyaz - black)

        # convertScaleAbs negatif degerlerin mutlak degerini alir; ani isik
        # gecisinde black seviyesinin altindaki pikselleri yanlislikla parlak
        # yaparak termal-kamera benzeri sahte renkler uretebilir. Burada gercek
        # ISP davranisi gibi once 0..255 araligina kirpip sonra uint8'e ceviririz.
        raw8 = np.clip(
            (raw.astype(np.float32) - black) * alpha,
            0.0,
            255.0,
        ).astype(np.uint8)

        # Kullanılan kamera düzenine karşılık gelen Bayer sıralaması: GB.
        img = cv2.cvtColor(raw8, ISP_BAYER_KODU)

        # Normalde WB arama kapaliyken adapte olur. Ani golge/gunes gecisinde
        # arama acik olsa bile kisa sure yeniden adapte edilir; bu sirada ana
        # dongu YOLO ve yeni RFD kararini zaten bekletir.
        if wb_guncelle or not self.wb_hazir or not self.isik_kararli:
            wb_ornek = img[::16, ::16].astype(np.float32)
            wh, ww = wb_ornek.shape[:2]
            wky = max(1, wh // 5)
            wkx = max(1, ww // 5)
            wb_kenar = np.concatenate(
                (
                    wb_ornek[:wky, :, :].reshape(-1, 3),
                    wb_ornek[-wky:, :, :].reshape(-1, 3),
                    wb_ornek[wky:-wky, :wkx, :].reshape(-1, 3),
                    wb_ornek[wky:-wky, -wkx:, :].reshape(-1, 3),
                ),
                axis=0,
            )

            # Guclu kirmizi/yesil nesneleri ve tamamen siyah/beyaz pikselleri
            # WB hesabindan cikar. Yeterli notr piksel yoksa kenar ortalamasina
            # geri don; plakanin merkezdeki rengi yine hesaba girmez.
            parlaklik = wb_kenar.mean(axis=1)
            renk_farki = wb_kenar.max(axis=1) - wb_kenar.min(axis=1)
            notr_maske = (
                (parlaklik >= ISP_WB_MIN_PARLAKLIK)
                & (parlaklik <= ISP_WB_MAX_PARLAKLIK)
                & (renk_farki <= ISP_WB_NOTR_RENK_FARKI)
            )
            wb_pikseller = wb_kenar[notr_maske]
            if wb_pikseller.shape[0] < 50:
                wb_pikseller = wb_kenar

            ort = wb_pikseller.mean(axis=0)
            hedef = float(np.mean(ort))
            yeni_gain = hedef / np.maximum(ort, 1.0)
            yeni_gain = np.clip(yeni_gain, ISP_WB_MIN, ISP_WB_MAX)

            if self.wb_hazir:
                a = (
                    ISP_WB_HIZLI_YUMUSATMA
                    if not self.isik_kararli
                    else ISP_WB_YUMUSATMA
                )
                self.wb_gain = (
                    self.wb_gain * (1.0 - a)
                    + yeni_gain.astype(np.float32) * a
                )
            else:
                self.wb_gain = yeni_gain.astype(np.float32)
                self.wb_hazir = True

        b, g, r = cv2.split(img)
        b = cv2.convertScaleAbs(b, alpha=float(self.wb_gain[0]))
        g = cv2.convertScaleAbs(g, alpha=float(self.wb_gain[1]))
        r = cv2.convertScaleAbs(r, alpha=float(self.wb_gain[2]))
        img = cv2.merge((b, g, r))

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] *= ISP_SATURATION
        hsv[:, :, 2] *= ISP_PARLAKLIK
        np.clip(hsv[:, :, 1], 0, 255, out=hsv[:, :, 1])
        np.clip(hsv[:, :, 2], 0, 255, out=hsv[:, :, 2])
        img = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        img = cv2.LUT(img, self.gamma_lut)

        if not self.ilk_kare_bilgisi_yazildi:
            print(
                "AR0234 ISP AKTIF | "
                f"RAW seviye={black_olculen:.0f}-{white:.0f} | "
                f"black={black:.0f} | "
                "WB(B,G,R)="
                f"{self.wb_gain[0]:.2f},"
                f"{self.wb_gain[1]:.2f},"
                f"{self.wb_gain[2]:.2f}"
            )
            self.ilk_kare_bilgisi_yazildi = True

        return img

    def oku(self, wb_guncelle=True):
        if self.appsink is None:
            return False, None

        sample = self.appsink.emit("try-pull-sample", 2 * Gst.SECOND)
        if sample is None:
            self.hata_yaz()
            return False, None

        caps = sample.get_caps()
        yapi = caps.get_structure(0)
        genislik = int(yapi.get_value("width"))
        yukseklik = int(yapi.get_value("height"))

        tampon = sample.get_buffer()
        basarili, harita = tampon.map(Gst.MapFlags.READ)
        if not basarili:
            return False, None

        try:
            veri = np.frombuffer(harita.data, dtype=np.uint8)

            if yukseklik <= 0 or veri.size % yukseklik != 0:
                print(
                    "HATA: AR0234 GStreamer tampon boyutu gecersiz | "
                    f"byte={veri.size} | h={yukseklik}"
                )
                return False, None

            satir_bayt = veri.size // yukseklik
            gerekli_bayt = genislik * 2
            if satir_bayt < gerekli_bayt:
                print(
                    "HATA: AR0234 Y16 satiri beklenenden kucuk | "
                    f"stride={satir_bayt} | gerekli={gerekli_bayt}"
                )
                return False, None

            # Olası stride/padding baytlarini at ve little-endian uint16 yap.
            satirlar = veri.reshape(yukseklik, satir_bayt)[:, :gerekli_bayt]
            raw = np.ascontiguousarray(satirlar).view("<u2").reshape(
                yukseklik,
                genislik,
            )
        finally:
            tampon.unmap(harita)

        if raw.shape != (KAMERA_YUKSEKLIK, KAMERA_GENISLIK):
            print(
                "HATA: AR0234 kare boyutu beklenenden farkli | "
                f"shape={raw.shape}"
            )
            return False, None

        return True, self._isp_uygula(raw, wb_guncelle)

    def kapat(self):
        if self.pipeline is not None:
            try:
                self.pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
        self.pipeline = None
        self.appsink = None


def mavlink_mesaj_adini_duzelt(deger):
    """MAVLink char[] alanini guvenli Python metnine cevirir."""
    if isinstance(deger, bytes):
        deger = deger.decode("ascii", errors="ignore")
    return str(deger).rstrip("\x00").strip()


def sistem_saati_gecerli():
    """Jetson saatinin 1970/sifirlanmis olmadigini kontrol eder."""
    return GECERLI_SAAT_MIN_UNIX <= time.time() < GECERLI_SAAT_MAX_UNIX


def cube_portunu_bul():
    """Cube'un kalici USB MAVLink yolunu bulur (if00)."""
    portlar = sorted(CUBE_BY_ID_KLASORU.glob(CUBE_PORT_DESENI))

    if not portlar:
        raise SystemExit(
            "HATA: Cube Orange+ USB MAVLink portu bulunamadi. "
            "Cube'un USB kablosunu kontrol et."
        )

    if len(portlar) > 1:
        liste = "\n".join(f"  - {port}" for port in portlar)
        raise SystemExit(
            "HATA: Birden fazla Cube if00 portu bulundu:\n" + liste
        )

    return str(portlar[0])


class RfdGonderimLedi:
    """IHA->YKI ve YKI->IDA durumunu tek LED'in yanip sonme hizi ile gosterir."""

    def __init__(self):
        self.hazir = False
        self.durum = LED_DURUM_KAPALI
        self.led_yaniyor = False
        self.sondurme_zamani = 0.0
        self.sonraki_darbe_zamani = 0.0

    def baslat(self):
        if GPIO is None:
            print(
                "UYARI: Jetson.GPIO bulunamadi; RFD durum LED'i "
                "devre disi. Ana tespit ve RFD sistemi calismaya devam eder."
            )
            return False

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(RFD_LED_PIN, GPIO.OUT, initial=GPIO.LOW)
        except Exception as hata:
            print(
                "UYARI: RFD durum LED'i baslatilamadi; ana sistem "
                f"calismaya devam eder. Ayrinti: {hata}"
            )
            return False

        self.hazir = True
        print(
            "RFD DURUM LEDI HAZIR | "
            f"Fiziksel Pin {RFD_LED_PIN} | Darbe "
            f"{RFD_LED_DARBE_SURESI * 1000:.0f} ms | "
            f"YKI={RFD_LED_YKI_ARALIGI:.0f} sn | "
            f"IDA={RFD_LED_IDA_ARALIGI:.0f} sn"
        )
        return True

    def durum_ayarla(self, yeni_durum):
        if yeni_durum not in {
            LED_DURUM_KAPALI,
            LED_DURUM_YKI,
            LED_DURUM_IDA,
        }:
            yeni_durum = LED_DURUM_KAPALI

        if self.durum == yeni_durum:
            return

        self.durum = yeni_durum
        self.sondurme_zamani = 0.0
        self.led_yaniyor = False
        self.sonraki_darbe_zamani = (
            time.monotonic() if yeni_durum != LED_DURUM_KAPALI else 0.0
        )

        if self.hazir:
            try:
                GPIO.output(RFD_LED_PIN, GPIO.LOW)
            except Exception as hata:
                self.hazir = False
                print(f"UYARI: RFD durum LED'i ayarlanamadi: {hata}")
                return

        durum_adi = {
            LED_DURUM_KAPALI: "KAPALI",
            LED_DURUM_YKI: "YKI'YE GONDERILIYOR (3 sn'de 1)",
            LED_DURUM_IDA: "IDA ONAYLADI (1 sn'de 1)",
        }[yeni_durum]
        print(f"LED DURUMU: {durum_adi}")

    def yki_gonderimi_basladi(self):
        # IDA onayi geldiyse tekrar YKI moduna geri dusme.
        if self.durum < LED_DURUM_YKI:
            self.durum_ayarla(LED_DURUM_YKI)

    def ida_onayi_alindi(self):
        self.durum_ayarla(LED_DURUM_IDA)

    def guncelle(self):
        """Ana donguyu bloklamadan periyodik 150 ms LED darbeleri uretir."""
        if not self.hazir:
            return

        simdi = time.monotonic()

        try:
            if self.led_yaniyor and simdi >= self.sondurme_zamani:
                GPIO.output(RFD_LED_PIN, GPIO.LOW)
                self.led_yaniyor = False
                self.sondurme_zamani = 0.0

            if self.durum == LED_DURUM_KAPALI or self.led_yaniyor:
                return

            if simdi < self.sonraki_darbe_zamani:
                return

            aralik = (
                RFD_LED_IDA_ARALIGI
                if self.durum == LED_DURUM_IDA
                else RFD_LED_YKI_ARALIGI
            )
            GPIO.output(RFD_LED_PIN, GPIO.HIGH)
            self.led_yaniyor = True
            self.sondurme_zamani = simdi + RFD_LED_DARBE_SURESI
            self.sonraki_darbe_zamani = simdi + aralik
        except Exception as hata:
            self.hazir = False
            print(f"UYARI: RFD durum LED'i surulemedi: {hata}")

    def kapat(self):
        if not self.hazir:
            return

        try:
            GPIO.output(RFD_LED_PIN, GPIO.LOW)
            GPIO.cleanup(RFD_LED_PIN)
        except Exception as hata:
            print(f"UYARI: RFD durum LED'i kapatma hatasi: {hata}")
        finally:
            self.hazir = False
            self.durum = LED_DURUM_KAPALI
            self.led_yaniyor = False
            self.sondurme_zamani = 0.0
            self.sonraki_darbe_zamani = 0.0


class CanliRenkGonderici:
    """Cube'dan RC7'yi okur ve kesin rengi RFD'ye MAVLink ile yollar."""

    def __init__(self, gonderim_ledi=None):
        self.port = cube_portunu_bul()
        self.baglanti = None
        self.gonderim_ledi = gonderim_ledi
        self.baslangic = time.monotonic()
        self.son_gonderim = 0.0
        self.son_renk = "HENUZ_YOK"
        self.son_kod = "-"
        self.gonderim_sayisi = 0
        self.rc7_raw = None
        self.son_rc_mesaj_zamani = 0.0
        self.son_rc_istegi = 0.0
        self.yki_asamasi = LED_DURUM_KAPALI
        self.saat_esitlendi = sistem_saati_gecerli()
        self.saat_yetki_hatasi_yazildi = False

    def baglan(self):
        print(f"Cube MAVLink portu aciliyor: {self.port}")

        try:
            self.baglanti = mavutil.mavlink_connection(
                self.port,
                baud=MAVLINK_BAUD,
                source_system=MAVLINK_KAYNAK_SISTEM,
                source_component=MAVLINK_KAYNAK_BILESEN,
                autoreconnect=True,
            )
        except Exception as hata:
            raise SystemExit(
                "HATA: Cube portu acilamadi. 'Permission denied' yaziyorsa "
                "mavera kullanicisini dialout grubuna ekle.\n"
                f"Ayrinti: {hata}"
            ) from hata

        print("Cube HEARTBEAT bekleniyor...")
        heartbeat = self.baglanti.wait_heartbeat(
            timeout=HEARTBEAT_BEKLEME_SURESI
        )

        if heartbeat is None:
            self.kapat()
            raise SystemExit(
                "HATA: Cube portu acildi fakat HEARTBEAT gelmedi."
            )

        print(
            "CUBE MAVLINK BAGLANDI | "
            f"sysid={self.baglanti.target_system} "
            f"compid={self.baglanti.target_component}"
        )

        self.rc_akisini_iste()

    def _yki_saatini_uygula(self, unix_saniye):
        """YKI'dan gelen dogrulanmis Unix saatini Jetson'a bir kez uygular."""
        if self.saat_esitlendi:
            return True

        if not GECERLI_SAAT_MIN_UNIX <= unix_saniye < GECERLI_SAAT_MAX_UNIX:
            return False

        # YKI_TIME tam saniye tasir. 0.5 saniye eklemek, paketin saniye icinde
        # hangi anda gonderildigi bilinmedigindeki ortalama hatayi azaltir.
        hedef_saat = float(unix_saniye) + 0.5
        try:
            time.clock_settime(time.CLOCK_REALTIME, hedef_saat)
        except (PermissionError, OSError) as hata:
            if not self.saat_yetki_hatasi_yazildi:
                self.saat_yetki_hatasi_yazildi = True
                print(
                    "HATA: YKI saati alindi fakat Jetson saati "
                    "degistirilemedi. mavera-iha.service icin "
                    f"CAP_SYS_TIME yetkisi eksik olabilir: {hata}"
                )
            return False

        self.saat_esitlendi = True
        self.saat_yetki_hatasi_yazildi = False
        print(
            "JETSON SAATI YKI'DAN ESITLENDI | "
            f"{datetime.now(TURKIYE_SAATI).isoformat(timespec='seconds')}"
        )
        return True

    def yki_saatini_bekle(self):
        """Saat gecersizse kayit baslamadan once YKI_TIME paketini bekler."""
        if sistem_saati_gecerli():
            self.saat_esitlendi = True
            print(
                "JETSON SISTEM SAATI ZATEN GECERLI | "
                f"{datetime.now(TURKIYE_SAATI).isoformat(timespec='seconds')}"
            )
            return True

        self.saat_esitlendi = False
        print(
            "JETSON SAATI GECERSIZ (1970 olabilir) | "
            f"YKI'dan {YKI_SAAT_MESAJI} bekleniyor..."
        )
        bitis = time.monotonic() + YKI_SAAT_BEKLEME_SURESI
        while time.monotonic() < bitis:
            self.mavlink_mesajlarini_isle()
            if self.saat_esitlendi:
                return True
            if sistem_saati_gecerli():
                self.saat_esitlendi = True
                print(
                    "JETSON SAATI HARICI KAYNAKTAN DUZELDI | "
                    f"{datetime.now(TURKIYE_SAATI).isoformat(timespec='seconds')}"
                )
                return True
            time.sleep(0.05)

        print(
            "HATA: YKI'dan gecerli saat alinamadi. MP4/CSV'nin 1970 "
            "zamanli olmasini engellemek icin kamera kaydi baslatilmadi."
        )
        return False

    def rc_akisini_iste(self):
        """Cube'dan RC kanal degerlerini 10 Hz hizla ister."""
        if self.baglanti is None:
            return

        try:
            self.baglanti.mav.request_data_stream_send(
                self.baglanti.target_system,
                self.baglanti.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                RC_MESAJ_HIZI_HZ,
                1,
            )
            self.son_rc_istegi = time.monotonic()
        except Exception as hata:
            print(f"RC7 AKIS ISTEGI HATASI: {hata}")

    def mavlink_mesajlarini_isle(self):
        """RC7'yi ve YKI/IDA durum geri bildirimini ayni MAVLink hattindan okur."""
        if self.baglanti is None:
            return

        while True:
            mesaj = self.baglanti.recv_match(blocking=False)
            if mesaj is None:
                break

            mesaj_tipi = mesaj.get_type()

            if mesaj_tipi in {"RC_CHANNELS", "RC_CHANNELS_RAW"}:
                rc_degeri = getattr(
                    mesaj,
                    f"chan{RC_ARAMA_KANALI}_raw",
                    None,
                )
                if (
                    rc_degeri is not None
                    and RC_GECERLI_MIN <= int(rc_degeri) <= RC_GECERLI_MAX
                ):
                    self.rc7_raw = int(rc_degeri)
                    self.son_rc_mesaj_zamani = time.monotonic()
                continue

            if mesaj_tipi != "NAMED_VALUE_INT":
                continue

            if mesaj.get_srcSystem() != YKI_SISTEM_ID:
                continue
            if mesaj.get_srcComponent() != YKI_BILESEN_ID:
                continue

            mesaj_adi = mavlink_mesaj_adini_duzelt(mesaj.name)
            if mesaj_adi == YKI_SAAT_MESAJI:
                self._yki_saatini_uygula(int(mesaj.value))
                continue

            if mesaj_adi != YKI_DURUM_MESAJI:
                continue

            # YKI degeri ASAMA*10 + RENK_KODU olarak yollar:
            # 11/12/13 = YKI rengi dogruladi
            # 21/22/23 = IDA COMMAND_ACK ile rengi kabul etti
            deger = int(mesaj.value)
            asama = deger // 10
            kod = deger % 10

            # Eski/baska renk icin gecikmis geri bildirimi dikkate alma.
            if self.son_kod in RENK_KODLARI.values() and kod != self.son_kod:
                continue

            if asama not in {LED_DURUM_YKI, LED_DURUM_IDA}:
                continue

            if asama != self.yki_asamasi:
                self.yki_asamasi = asama
                if asama == LED_DURUM_YKI:
                    print(
                        "YKI GERI BILDIRIMI: hedef rengi dogruladi | "
                        f"KOD={kod}"
                    )
                else:
                    print(
                        "IDA GERI BILDIRIMI: hedef rengi COMMAND_ACK ile "
                        f"kabul etti | KOD={kod}"
                    )

            if self.gonderim_ledi is not None:
                if asama == LED_DURUM_IDA:
                    self.gonderim_ledi.ida_onayi_alindi()
                else:
                    self.gonderim_ledi.yki_gonderimi_basladi()

    def rc_kanalini_oku(self):
        """Son RC7 degerini ve taze bir RC mesaji olup olmadigini dondurur."""
        self.mavlink_mesajlarini_isle()
        simdi = time.monotonic()
        rc_sinyali_var = (
            self.rc7_raw is not None
            and simdi - self.son_rc_mesaj_zamani <= RC_SINYAL_ZAMAN_ASIMI
        )

        if (
            not rc_sinyali_var
            and simdi - self.son_rc_istegi >= RC_AKIS_YENILEME_ARALIGI
        ):
            self.rc_akisini_iste()

        return self.rc7_raw, rc_sinyali_var

    def gonder(self, renk, zorla=False):
        """Son karari 2 Hz yollar; yeni kilitte beklemeden hemen yollar."""
        if self.baglanti is None:
            return False

        simdi = time.monotonic()
        if not zorla and simdi - self.son_gonderim < RFD_GONDERIM_ARALIGI:
            return True

        renk = renk if renk in RENK_KODLARI else "BELIRSIZ"
        kod = RENK_KODLARI[renk]
        uygulama_ms = int((simdi - self.baslangic) * 1000) & 0xFFFFFFFF

        try:
            # Tamponu bosaltirken RC7 mesajlarini da okuyup sakla.
            self.mavlink_mesajlarini_isle()

            self.baglanti.mav.named_value_int_send(
                uygulama_ms,
                b"TGT_COLOR",
                kod,
            )
        except Exception as hata:
            print(f"RFD GONDERIM HATASI: {hata}")
            return False

        self.son_gonderim = simdi
        self.son_renk = renk
        self.son_kod = kod
        self.gonderim_sayisi += 1

        # Gercek hedef rengi YKI'ye gonderilmeye baslandigi anda LED 3 sn'de
        # bir yanip sonmeye baslar. IDA geri bildirimi gelirse 1 Hz'e gececek.
        if self.gonderim_ledi is not None and kod in {1, 2, 3}:
            self.gonderim_ledi.yki_gonderimi_basladi()
        return True

    def kapat(self):
        if self.baglanti is not None:
            self.baglanti.close()
            self.baglanti = None


class RcAramaKontrolu:
    """SG/RC7 ile aramayi guvenli bicimde acip kapatan durum makinesi."""

    def __init__(self):
        self.kapali_konum_goruldu = False
        self.aktif = False

    def guncelle(self, rc7_raw, rc_sinyali_var):
        onceki_aktif = self.aktif

        if not rc_sinyali_var or rc7_raw is None:
            self.kapali_konum_goruldu = False
            self.aktif = False
            durum = "RC_SINYALI_YOK"
        elif rc7_raw <= RC_ARAMA_ACIK_ESIGI:
            self.kapali_konum_goruldu = True
            self.aktif = False
            durum = "ARAMA_KAPALI"
        elif not self.kapali_konum_goruldu:
            self.aktif = False
            durum = "ONCE_SG_KAPAT"
        else:
            self.aktif = True
            durum = "ARAMA_ACIK"

        yeni_acildi = self.aktif and not onceki_aktif
        yeni_kapandi = onceki_aktif and not self.aktif
        return self.aktif, yeni_acildi, yeni_kapandi, durum


def turkiye_zaman_bilgisi():
    """CSV ve goruntu icin ayni UTC+3 zaman damgasini uretir."""
    simdi = datetime.now(TURKIYE_SAATI)
    iso = simdi.isoformat(timespec="milliseconds")
    ekran = simdi.strftime("%Y-%m-%d %H:%M:%S")
    ekran += f".{simdi.microsecond // 1000:03d} UTC+3"
    return iso, ekran
    

class IhaKayitci:
    """Islenmis kamera videosunu ve ayni karelerin karar CSV'sini kaydeder."""

    CSV_ALANLARI = [
        "timestamp_utc3",
        "elapsed_s",
        "record_frame_no",
        "camera_frame_no",
        "rc7_raw",
        "rc_signal_ok",
        "search_state",
        "search_enabled",
        "plate_detected",
        "detection_count",
        "bbox_x1",
        "bbox_y1",
        "bbox_x2",
        "bbox_y2",
        "yolo_conf",
        "hsv_color",
        "hsv_conf",
        "instant_decision",
        "vote_count",
        "vote_window_fill",
        "locked_color",
        "rfd_packet_sent",
        "rfd_send_count",
        "rfd_last_color",
        "rfd_last_code",
    ]

    def __init__(self, genislik, yukseklik):
        YARIS_KAYIT_KLASORU.mkdir(parents=True, exist_ok=True)

        bos_bayt = shutil.disk_usage(YARIS_KAYIT_KLASORU).free
        bos_gb = bos_bayt / (1024 ** 3)
        if bos_gb < MINIMUM_BOS_ALAN_GB:
            raise SystemExit(
                "HATA: IHA kaydi icin yeterli bos alan yok. "
                f"Bos alan: {bos_gb:.1f} GB | "
                f"Gerekli: en az {MINIMUM_BOS_ALAN_GB:.1f} GB"
            )

        baslangic_tr = datetime.now(TURKIYE_SAATI)
        oturum_adi = baslangic_tr.strftime("%Y%m%d_%H%M%S")
        oturum_adi += f"_{baslangic_tr.microsecond // 1000:03d}_UTC3"
        self.oturum_klasoru = YARIS_KAYIT_KLASORU / oturum_adi
        self.oturum_klasoru.mkdir(parents=True, exist_ok=False)

        self.video_yolu = self.oturum_klasoru / "iha_plaka_islenmis.mp4"
        self.csv_yolu = self.oturum_klasoru / "iha_renk_karar.csv"
        self.video_yazici = None
        self.csv_dosyasi = None
        self.csv_yazici = None

        for kodek in VIDEO_KODEKLERI:
            fourcc = cv2.VideoWriter_fourcc(*kodek)
            aday = cv2.VideoWriter(
                str(self.video_yolu),
                fourcc,
                VIDEO_KAYIT_FPS,
                (genislik, yukseklik),
            )
            if aday.isOpened():
                self.video_yazici = aday
                self.video_kodeki = kodek
                break
            aday.release()

        if self.video_yazici is None:
            raise SystemExit(
                "HATA: MP4 kayit dosyasi acilamadi. "
                "OpenCV video kodekleri kontrol edilmeli."
            )

        try:
            self.csv_dosyasi = self.csv_yolu.open(
                "w",
                newline="",
                encoding="utf-8",
            )
        except Exception:
            self.video_yazici.release()
            self.video_yazici = None
            raise

        self.csv_yazici = csv.DictWriter(
            self.csv_dosyasi,
            fieldnames=self.CSV_ALANLARI,
        )
        self.csv_yazici.writeheader()
        self.csv_dosyasi.flush()

        self.baslangic_monotonic = time.monotonic()
        self.sonraki_kayit = self.baslangic_monotonic
        self.kayit_araligi = 1.0 / VIDEO_KAYIT_FPS
        self.son_csv_flush = self.baslangic_monotonic
        self.kayit_karesi = 0

        print("IHA OTOMATIK KAYIT BASLADI")
        print(f"Kayit klasoru: {self.oturum_klasoru}")
        print(
            f"Video: {self.video_yolu} | "
            f"{VIDEO_KAYIT_FPS:.0f} FPS | Kodek: {self.video_kodeki}"
        )
        print(f"Karar CSV: {self.csv_yolu}")
        print(f"Bos disk alani: {bos_gb:.1f} GB")

    def kaydet(
        self,
        kare,
        timestamp_utc3,
        kamera_kare_no,
        rc7_raw,
        rc_sinyali_var,
        arama_durumu,
        arama_aktif,
        tespit_sayisi,
        tespit_bilgisi,
        anlik_karar,
        oy_sayisi,
        oy_pencere_dolulugu,
        kilitli_karar,
        rfd_bu_kare_gonderildi,
        renk_gonderici,
    ):
        simdi = time.monotonic()
        if simdi < self.sonraki_kayit:
            return False

        # Isleme hizi gecici olarak duserse eski araliklari doldurmak icin ayni
        # kareyi tekrar yazmayiz; bir sonraki gercek kareden devam ederiz.
        while self.sonraki_kayit <= simdi:
            self.sonraki_kayit += self.kayit_araligi

        self.kayit_karesi += 1
        self.video_yazici.write(kare)

        bilgi = tespit_bilgisi or {}
        self.csv_yazici.writerow(
            {
                "timestamp_utc3": timestamp_utc3,
                "elapsed_s": f"{simdi - self.baslangic_monotonic:.3f}",
                "record_frame_no": self.kayit_karesi,
                "camera_frame_no": kamera_kare_no,
                "rc7_raw": rc7_raw if rc7_raw is not None else "",
                "rc_signal_ok": int(rc_sinyali_var),
                "search_state": arama_durumu,
                "search_enabled": int(arama_aktif),
                "plate_detected": int(tespit_sayisi > 0),
                "detection_count": tespit_sayisi,
                "bbox_x1": bilgi.get("x1", ""),
                "bbox_y1": bilgi.get("y1", ""),
                "bbox_x2": bilgi.get("x2", ""),
                "bbox_y2": bilgi.get("y2", ""),
                "yolo_conf": f"{bilgi.get('yolo_guven', 0.0):.4f}",
                "hsv_color": bilgi.get("hsv_renk", "BELIRSIZ"),
                "hsv_conf": f"{bilgi.get('hsv_oran', 0.0):.4f}",
                "instant_decision": anlik_karar,
                "vote_count": oy_sayisi,
                "vote_window_fill": oy_pencere_dolulugu,
                "locked_color": kilitli_karar or "",
                "rfd_packet_sent": int(rfd_bu_kare_gonderildi),
                "rfd_send_count": renk_gonderici.gonderim_sayisi,
                "rfd_last_color": renk_gonderici.son_renk,
                "rfd_last_code": renk_gonderici.son_kod,
            }
        )

        if simdi - self.son_csv_flush >= 1.0:
            self.csv_dosyasi.flush()
            self.son_csv_flush = simdi

        return True

    def kapat(self):
        if self.video_yazici is not None:
            self.video_yazici.release()
            self.video_yazici = None

        if self.csv_dosyasi is not None:
            self.csv_dosyasi.flush()
            self.csv_dosyasi.close()
            self.csv_dosyasi = None

        print("IHA KAYIT DOSYALARI KAPATILDI")
        print(f"Kaydedilen video karesi: {self.kayit_karesi}")
        print(f"Video: {self.video_yolu}")
        print(f"CSV: {self.csv_yolu}")


def hsv_renk_bul(goruntu):
    """Verilen goruntudeki siyah, kirmizi ve yesil piksel oranini olcer."""
    if goruntu is None or goruntu.size == 0:
        return "BELIRSIZ", 0.0

    yukseklik, genislik = goruntu.shape[:2]

    # Kutunun kenarindaki arka planin etkisini azalt.
    x_bosluk = int(genislik * 0.10)
    y_bosluk = int(yukseklik * 0.10)
    orta = goruntu[
        y_bosluk:max(y_bosluk + 1, yukseklik - y_bosluk),
        x_bosluk:max(x_bosluk + 1, genislik - x_bosluk),
    ]

    if orta.size == 0:
        return "BELIRSIZ", 0.0

    hsv = cv2.cvtColor(orta, cv2.COLOR_BGR2HSV)

    siyah = cv2.inRange(hsv, (0, 0, 0), (179, 255, 95))

    kirmizi_1 = cv2.inRange(hsv, (0, 70, 50), (12, 255, 255))
    kirmizi_2 = cv2.inRange(hsv, (168, 70, 50), (179, 255, 255))
    kirmizi = cv2.bitwise_or(kirmizi_1, kirmizi_2)

    yesil = cv2.inRange(hsv, (35, 50, 40), (95, 255, 255))

    toplam = float(hsv.shape[0] * hsv.shape[1])
    oranlar = {
        "SIYAH": cv2.countNonZero(siyah) / toplam,
        "KIRMIZI": cv2.countNonZero(kirmizi) / toplam,
        "YESIL": cv2.countNonZero(yesil) / toplam,
    }

    renk = max(oranlar, key=oranlar.get)
    oran = oranlar[renk]

    if oran < HSV_RENK_ORANI_ESIGI:
        return "BELIRSIZ", oran

    return renk, oran


class ZamansalKarar:
    """Son 10 anlik karardan en az 7 tanesi ayniysa kesin karar verir."""

    def __init__(self):
        self.gecmis = deque(maxlen=OY_PENCERESI)

    def sifirla(self):
        self.gecmis.clear()

    def ekle(self, anlik_karar):
        self.gecmis.append(anlik_karar)

        sayim = Counter(
            renk for renk in self.gecmis if renk in HEDEF_RENKLER
        )

        if not sayim:
            return None, 0

        renk, oy = sayim.most_common(1)[0]
        if len(self.gecmis) < OY_PENCERESI:
            return None, oy

        return (renk if oy >= GEREKLI_OY else None), oy


def kutuyu_sinirla(x1, y1, x2, y2, genislik, yukseklik):
    x1 = max(0, min(int(x1), genislik - 1))
    y1 = max(0, min(int(y1), yukseklik - 1))
    x2 = max(x1 + 1, min(int(x2), genislik))
    y2 = max(y1 + 1, min(int(y2), yukseklik))
    return x1, y1, x2, y2


def kareyi_isle(model, kare):
    """YOLO ile plakayi bulur; kutunun rengini yalnizca HSV ile belirler."""
    sonuc = model.predict(
        source=kare,
        device=0 if torch.cuda.is_available() else "cpu",
        conf=YOLO_GUVEN_ESIGI,
        imgsz=YOLO_GIRIS_BOYUTU,
        # Mevcut modelin renk siniflari bilerek filtrelenmez ve isimleri
        # kullanilmaz. Siniflardan herhangi biri, yalnizca "plaka bulundu"
        # anlamina gelir. Yeni tek sinifli PLATE modeli de ayni kodla calisir.
        classes=None,
        # Eski renk siniflarinin ayni plaka icin ust uste kutu uretmesini onler.
        agnostic_nms=True,
        max_det=10,
        verbose=False,
    )[0]

    anlik_karar = "KARARSIZ"
    en_iyi_guven = -1.0
    en_yuksek_yolo_guven = -1.0
    karar_tespiti = None
    en_iyi_tespit = None
    tespit_sayisi = 0
    yukseklik, genislik = kare.shape[:2]

    if sonuc.boxes is not None:
        tespit_sayisi = len(sonuc.boxes)

        for index in range(tespit_sayisi):
            yolo_guven = float(sonuc.boxes.conf[index].item())
            x1, y1, x2, y2 = sonuc.boxes.xyxy[index].tolist()
            x1, y1, x2, y2 = kutuyu_sinirla(
                x1, y1, x2, y2, genislik, yukseklik
            )

            plaka_bolgesi = kare[y1:y2, x1:x2]
            hsv_renk, hsv_oran = hsv_renk_bul(plaka_bolgesi)
            tespit_bilgisi = {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "yolo_guven": yolo_guven,
                "hsv_renk": hsv_renk,
                "hsv_oran": hsv_oran,
            }

            if yolo_guven > en_yuksek_yolo_guven:
                en_yuksek_yolo_guven = yolo_guven
                en_iyi_tespit = tespit_bilgisi

            # Renk kararinda YOLO sinifi kullanilmaz. YOLO sadece kutuyu,
            # HSV ise kutunun rengini verir.
            onaylandi = hsv_renk in HEDEF_RENKLER

            if onaylandi and yolo_guven > en_iyi_guven:
                anlik_karar = hsv_renk
                en_iyi_guven = yolo_guven
                karar_tespiti = tespit_bilgisi

            kutu_rengi = (0, 255, 0) if onaylandi else (0, 255, 255)
            etiket = (
                f"YOLO:PLAKA %{yolo_guven * 100:.0f} | "
                f"RENK(HSV):{hsv_renk} %{hsv_oran * 100:.0f}"
            )

            cv2.rectangle(kare, (x1, y1), (x2, y2), kutu_rengi, 3)
            cv2.putText(
                kare,
                etiket,
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                kutu_rengi,
                2,
            )

    # CSV'de karar veren kutu kaydedilir. Renk karari cikmadiysa tani icin en
    # yuksek YOLO guvenine sahip kutu yine de kayda yazilir.
    return anlik_karar, tespit_sayisi, karar_tespiti or en_iyi_tespit


def merkez_hsv_teshisi(kare):
    """YOLO'dan bagimsiz merkez renk olcumu; yalnizca tani bilgisidir."""
    yukseklik, genislik = kare.shape[:2]
    x1, x2 = int(genislik * 0.35), int(genislik * 0.65)
    y1, y2 = int(yukseklik * 0.30), int(yukseklik * 0.70)
    renk, oran = hsv_renk_bul(kare[y1:y2, x1:x2])
    return renk, oran, (x1, y1, x2, y2)


def zaman_damgasi_ciz(kare, zaman_metni):
    """Her kayit karesine okunabilir Turkiye (UTC+3) zamani ekler."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    olcek = 0.67
    kalinlik = 2
    (metin_genisligi, metin_yuksekligi), taban = cv2.getTextSize(
        zaman_metni,
        font,
        olcek,
        kalinlik,
    )
    x = 14
    y = kare.shape[0] - 16
    cv2.rectangle(
        kare,
        (x - 7, y - metin_yuksekligi - 8),
        (x + metin_genisligi + 7, y + taban + 7),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        kare,
        zaman_metni,
        (x, y),
        font,
        olcek,
        (255, 255, 255),
        kalinlik,
    )


def bilgi_paneli_ciz(
    kare,
    arama_durumu,
    arama_aktif,
    rc7_raw,
    rc_sinyali_var,
    anlik_karar,
    kilitli_karar,
    tespit_sayisi,
    merkez_renk,
    merkez_oran,
    fps,
    oy_sayisi,
    oy_doluluk,
    gonderilen_renk,
    gonderilen_kod,
    rfd_gonderim_sayisi,
):
    genislik = kare.shape[1]
    cv2.rectangle(kare, (0, 0), (genislik, 214), (0, 0, 0), -1)

    kesin_yazi = kilitli_karar or "BEKLENIYOR"
    yolo_yazi = (
        f"YOLO: {tespit_sayisi} PLAKA KUTUSU"
        if tespit_sayisi
        else "YOLO: PLAKA BULAMADI"
    )

    satirlar = [
        f"ARAMA: {'ACIK' if arama_aktif else 'KAPALI'} | "
        f"DURUM: {arama_durumu} | RC7: "
        f"{rc7_raw if rc7_raw is not None else '-'} | "
        f"RC: {'VAR' if rc_sinyali_var else 'YOK'}",
        f"{yolo_yazi} | Esik: {YOLO_GUVEN_ESIGI:.2f}",
        f"ANLIK: {anlik_karar} | OY: {oy_sayisi}/{OY_PENCERESI} | "
        f"Pencere: {oy_doluluk}/{OY_PENCERESI}",
        f"KESIN: {kesin_yazi} | FPS: {fps:.1f}",
        f"MERKEZ HSV (tani): {merkez_renk} %{merkez_oran * 100:.0f}",
        f"RFD GONDERILEN: {gonderilen_renk} | KOD: {gonderilen_kod} | "
        f"PAKET: {rfd_gonderim_sayisi}",
    ]

    for sira, yazi in enumerate(satirlar):
        cv2.putText(
            kare,
            yazi,
            (18, 28 + sira * 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
        )


def main():
    if not MODEL_YOLU.exists():
        raise SystemExit(f"HATA: Model bulunamadi: {MODEL_YOLU}")

    EKRAN_GORUNTUSU_KLASORU.mkdir(parents=True, exist_ok=True)

    print(f"Model yukleniyor: {MODEL_YOLU}")
    model = YOLO(str(MODEL_YOLU))

    renk_gonderici = CanliRenkGonderici()
    renk_gonderici.baglan()

    if not renk_gonderici.yki_saatini_bekle():
        renk_gonderici.kapat()
        raise SystemExit(
            "HATA: Jetson saati dogrulanmadan IHA kaydi baslatilmayacak."
        )

    print("AR0234 Global Shutter kamerasi aciliyor...")
    kamera = ArducamKamera()

    if not kamera.ac():
        raise SystemExit(
            "HATA: AR0234 kamerasi baslatilamadi."
        )

    rfd_ledi = RfdGonderimLedi()
    rfd_ledi.baslat()
    renk_gonderici.gonderim_ledi = rfd_ledi

    oylayici = ZamansalKarar()
    arama_kontrolu = RcAramaKontrolu()
    kilitli_karar = None
    onceki_zaman = time.perf_counter()
    fps = 0.0
    kamera_kare_no = 0
    kayitci = None
    onceki_isik_kararli = False

    print("Canli sistem basladi.")
    print("Q: cikis | R: karari sifirla | S: iki goruntu kaydet")
    print(f"EKRAN MODU: {'ACIK' if EKRAN_GOSTER else 'KAPALI (BASLIKSIZ)'}")
    print("PLAKA ARAMA BASLANGICTA KAPALI.")
    print("Once SG ile RC7=988/1500, aramak icin sonra RC7=2012 yap.")

    try:
        kayitci = IhaKayitci(KAMERA_GENISLIK, KAMERA_YUKSEKLIK)

        while True:
            rfd_ledi.guncelle()

            # RC7 durumunu kareyi islemeye baslamadan once bil. Arama acikken
            # white-balance dondurulur; plakanin kendi rengi ISP'yi kaydirmaz.
            rc7_raw, rc_sinyali_var = renk_gonderici.rc_kanalini_oku()
            (
                arama_aktif,
                arama_yeni_acildi,
                arama_yeni_kapandi,
                arama_durumu,
            ) = arama_kontrolu.guncelle(
                rc7_raw,
                rc_sinyali_var,
            )

            basarili, ham_kare = kamera.oku(
                wb_guncelle=not arama_aktif
            )

            if not basarili:
                print("HATA: AR0234 kameradan kare alinamadi.")
                break

            kamera_kare_no += 1

            isik_kararli = kamera.isik_kararli
            if arama_aktif and not isik_kararli:
                arama_durumu = "ISIK_DENGELENIYOR"

            if onceki_isik_kararli and not isik_kararli:
                # Henuz kesinlesmemis eski oylar farkli isik kosullariyla
                # karismasin. Daha once kilitlenmis dogru renk korunur.
                if kilitli_karar is None:
                    oylayici.sifirla()
                print(
                    "GORUNTU GECICI KARARSIZ | Yeni YOLO/HSV/RFD karari "
                    "isik dengelenene kadar durduruldu."
                )
            elif not onceki_isik_kararli and isik_kararli:
                print("ISIK DENGESI HAZIR | YOLO/HSV kararina izin verildi.")
            onceki_isik_kararli = isik_kararli

            # Kaydedilecek ham kareyi isaretleme yapmadan once koru.
            # Bu artik RAW degil; AR0234'ten ISP uygulanmis temiz BGR karedir.
            ham_kayit = ham_kare.copy()

            yeni_kilit = False
            if arama_yeni_acildi:
                oylayici.sifirla()
                kilitli_karar = None
                renk_gonderici.yki_asamasi = LED_DURUM_KAPALI
                rfd_ledi.durum_ayarla(LED_DURUM_KAPALI)
                print("PLAKA ARAMA ACILDI | YOLO ve renk karari aktif.")

            if arama_yeni_kapandi:
                oylayici.sifirla()
                kilitli_karar = None
                renk_gonderici.yki_asamasi = LED_DURUM_KAPALI
                rfd_ledi.durum_ayarla(LED_DURUM_KAPALI)
                print("PLAKA ARAMA KAPANDI | RFD renk gonderimi durdu.")

            if arama_aktif and isik_kararli:
                anlik_karar, tespit_sayisi, tespit_bilgisi = kareyi_isle(
                    model,
                    ham_kare,
                )
                kesin_karar, oy_sayisi = oylayici.ekle(anlik_karar)

                if kilitli_karar is None and kesin_karar is not None:
                    kilitli_karar = kesin_karar
                    yeni_kilit = True
                    print(
                        f"KESIN KARAR: {kilitli_karar} "
                        f"({oy_sayisi}/{OY_PENCERESI} oy)"
                    )
            else:
                anlik_karar = "KARARSIZ"
                tespit_sayisi = 0
                tespit_bilgisi = None
                oy_sayisi = 0

            # Kamera karari artik otomatik olarak Cube -> IHA RFD -> YKI RFD
            # hattina yalnizca arama acikken saniyede iki kez verilir.
            rfd_bu_kare_gonderildi = False
            if arama_aktif and kilitli_karar is not None:
                onceki_gonderim_sayisi = renk_gonderici.gonderim_sayisi
                renk_gonderici.gonder(
                    kilitli_karar,
                    zorla=yeni_kilit,
                )
                rfd_bu_kare_gonderildi = (
                    renk_gonderici.gonderim_sayisi
                    > onceki_gonderim_sayisi
                )

            merkez_renk, merkez_oran, merkez_kutu = merkez_hsv_teshisi(
                ham_kayit
            )
            mx1, my1, mx2, my2 = merkez_kutu
            cv2.rectangle(ham_kare, (mx1, my1), (mx2, my2), (255, 255, 0), 1)

            simdi = time.perf_counter()
            gecen = max(simdi - onceki_zaman, 1e-6)
            onceki_zaman = simdi
            anlik_fps = 1.0 / gecen
            fps = anlik_fps if fps == 0 else fps * 0.90 + anlik_fps * 0.10

            timestamp_utc3, zaman_metni = turkiye_zaman_bilgisi()

            bilgi_paneli_ciz(
                ham_kare,
                arama_durumu,
                arama_aktif,
                rc7_raw,
                rc_sinyali_var,
                anlik_karar,
                kilitli_karar,
                tespit_sayisi,
                merkez_renk,
                merkez_oran,
                fps,
                oy_sayisi,
                len(oylayici.gecmis),
                renk_gonderici.son_renk,
                renk_gonderici.son_kod,
                renk_gonderici.gonderim_sayisi,
            )
            zaman_damgasi_ciz(ham_kare, zaman_metni)

            kayitci.kaydet(
                ham_kare,
                timestamp_utc3,
                kamera_kare_no,
                rc7_raw,
                rc_sinyali_var,
                arama_durumu,
                arama_aktif,
                tespit_sayisi,
                tespit_bilgisi,
                anlik_karar,
                oy_sayisi,
                len(oylayici.gecmis),
                kilitli_karar,
                rfd_bu_kare_gonderildi,
                renk_gonderici,
            )

            tus = 255
            if EKRAN_GOSTER:
                cv2.imshow("MAVERA IHA - Canli Plaka Tespiti", ham_kare)
                tus = cv2.waitKey(1) & 0xFF

            if tus == ord("q"):
                break

            if tus == ord("r"):
                oylayici.sifirla()
                kilitli_karar = None
                renk_gonderici.yki_asamasi = LED_DURUM_KAPALI
                rfd_ledi.durum_ayarla(LED_DURUM_KAPALI)
                renk_gonderici.gonder("BELIRSIZ", zorla=True)
                print("Karar sifirlandi.")

            if tus == ord("s"):
                zaman = int(time.time())
                ham_yol = EKRAN_GORUNTUSU_KLASORU / f"ham_{zaman}.jpg"
                sonuc_yol = (
                    EKRAN_GORUNTUSU_KLASORU / f"sonuc_{zaman}.jpg"
                )
                cv2.imwrite(str(ham_yol), ham_kayit)
                cv2.imwrite(str(sonuc_yol), ham_kare)
                print(f"Ham goruntu: {ham_yol}")
                print(f"Isaretli goruntu: {sonuc_yol}")

    finally:
        rfd_ledi.kapat()
        if kayitci is not None:
            kayitci.kapat()
        kamera.kapat()
        renk_gonderici.kapat()
        if EKRAN_GOSTER:
            cv2.destroyAllWindows()

        if torch.cuda.is_available():
            torch.cuda.synchronize()

        del model
        gc.collect()

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        print("AR0234 kamera ve GPU temiz sekilde kapatildi.")


if __name__ == "__main__":
    main()
