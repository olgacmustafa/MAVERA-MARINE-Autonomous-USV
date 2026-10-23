# EGM MAVERA MARINE | Otonom İnsansız Deniz Aracı (MAVERA-T)

**TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması — Türkiye Birincisi**

Bu depo, **EGM MAVERA MARINE** ekibinin MAVERA-T insansız deniz aracı projesinde geliştirdiği yazılım bileşenlerini içerir. Proje; **İHA destekli hedef renk tespiti**, **yer kontrol istasyonu (YKİ) üzerinden haberleşme** ve **ROS 2 tabanlı İDA otonomisini** bir araya getirir.

[English README](README_EN.md) · [Sistem mimarisi](docs/ARCHITECTURE.md) · [Test ve güvenlik](docs/TESTING.md) · [Sürüm notları](docs/RELEASE_NOTES.md)

> **Yayın durumu:** Bu depo, yarışma döneminde kullanılan kaynaklardan hazırlanmış ve GitHub'da paylaşılmak üzere düzenlenmiş bir sürümdür. Kodların sözdizimi ve temel bileşen uyumluluğu kontrol edilmiştir; düzenlenmiş paket gerçek araç üzerinde yeniden uçtan uca test edilmemiştir.

## Proje hakkında

MAVERA-T sisteminde hava ve deniz araçları aynı görev akışına katkıda bulunur. İHA'nın kamerasından algılanan hedef plaka rengi yer kontrol istasyonuna iletilir. Yer kontrol istasyonu gelen bilgiyi doğrulayarak İDA'ya gönderir. Deniz aracı, ROS 2 düğümleri üzerinden navigasyon, algılama, kontrol ve görev yönetimini yürütür.

### Sistem akışı

```text
İHA — Kamera (AR0234) → YOLO + HSV → Hedef renk (MAVLink)
                                              │ RFD telemetri
                                              ▼
Yer Kontrol İstasyonu — Renk doğrulama + İHA/İDA haberleşme köprüsü
                                              │ RFD / MAV_CMD_USER_1
                                              ▼
İDA — Cube Orange+ → MAVROS → Hedef renk alıcısı → /carpilacakhedef
                         │
             ZED 2 görüntü/derinlik + ROS 2
                         │
          LOS navigasyon → Engelden kaçınma → PID kontrol
                         │
                Hareket komutları ve kayıt
```

Normal rota takibi, hedef renk bilgisinin gelmesini beklemek zorunda değildir. Hedef rengin kullanılması gereken görev aşamasında bu veri değerlendirilir.

## Kullanılan teknolojiler

| Alan | Teknolojiler |
| --- | --- |
| Otonomi ve robotik | ROS 2 Humble, MAVROS, Python |
| Görüntü işleme | OpenCV, YOLO (Ultralytics), HSV, ZED 2 |
| Navigasyon ve kontrol | LOS (Line-of-Sight), PID, engelden kaçınma |
| Haberleşme | MAVLink, PyMAVLink, RFD telemetri, UDP, Mission Planner |
| Gömülü sistemler | NVIDIA Jetson, Cube Orange+ |

## Klasör yapısı

| Yol | İçerik |
| --- | --- |
| [`iha/`](iha/README.md) | İHA kamerasından plaka rengi algılama ve MAVLink ile gönderim |
| [`yki/`](yki/README.md) | İki RFD üzerinden İHA–YKİ–İDA veri aktarımı ve Mission Planner UDP köprüsü |
| [`ida_ws/`](ida_ws/README.md) | İDA için ROS 2 çalışma alanı, otonomi düğümleri ve yapılandırmalar |
| [`docs/`](docs/ARCHITECTURE.md) | Sistem mimarisi, protokol, test ve yayın notları |
| [`tests/`](tests/test_repository.py) | Donanım gerektirmeyen statik testler |

Her bileşen kendi çalışma ortamında başlatılır; tüm projeyi tek bir Python komutuyla çalıştırmak mümkün değildir. Ayrıntılı kurulum için **[İHA](iha/README.md)**, **[YKİ](yki/README.md)** ve **[İDA](ida_ws/README.md)** kılavuzlarına bakın.

## Hedef renk haberleşmesi

1. İHA, `NAMED_VALUE_INT` mesajındaki `TGT_COLOR` alanıyla hedef rengi gönderir.
2. YKİ, aynı renk bilgisini tekrar alarak doğrular ve `MAV_CMD_USER_1` komutunu İDA'ya iletir.
3. İDA alıcısı komutu kontrol eder, kabul ettiği rengi kaydeder, `COMMAND_ACK` ile yanıt verir ve ROS 2 üzerinde `/carpilacakhedef` konusuna (`std_msgs/String`) yayınlar.

| Kod | Hedef rengi | ROS 2 mesajı |
| --- | --- | --- |
| `1` | Siyah | `siyah` |
| `2` | Kırmızı | `kirmizi` |
| `3` | Yeşil | `yesil` |

Bileşenlerin MAVLink kimlikleri, mesaj adları ve protokol parametreleri birbiriyle tutarlı kalmalıdır. Daha fazla bilgi: [Sistem mimarisi](docs/ARCHITECTURE.md).

## Kurulum ve çalıştırma

### İHA — Plaka rengi algılama

İHA üzerinde Jetson, uyumlu AR0234 kamera sürücüsü ve **ayrı eğitilmiş hedef plaka modeli** gereklidir. Modelinizi `iha/models/best.pt` konumuna yerleştirdikten ve ortamı kurduktan sonra:

```bash
python3 iha/src/canli_plaka_tespit.py
```

### YKİ — İHA/İDA köprüsü

```bash
python -m pip install -r yki/requirements.txt
python yki/yki_iha_ida_koprusu.py --help
```

RFD cihazlarının USB seri numaraları ortam değişkenlerinden alınabilir veya portlar komut satırından belirtilebilir. Örnekler için [YKİ kılavuzunu](yki/README.md) inceleyin.

### İDA — ROS 2 otonomi

```bash
cd ida_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select usv_navigation
source install/setup.bash
ros2 launch usv_navigation usv_otonom.launch.py fcu_url:=serial:///dev/ttyACM0:115200
```

Buradaki `/dev/ttyACM0` **örnek bir aygıt yoludur**; kendi donanımınıza göre düzenleyin. Tam bağımlılıklar, kamera ayarları ve donanım bağlantıları için [İDA kılavuzuna](ida_ws/README.md) bakın.

## Kontroller

Donanım olmadan aşağıdaki kontroller çalıştırılabilir:

```bash
python -m unittest discover -s tests -v
python -m compileall -q iha yki ida_ws/src/usv_navigation
```

Bu kontroller **gerçek deniz aracı davranışını, E-stop devresini veya uçtan uca haberleşmeyi doğrulamaz**. Donanım entegrasyonu öncesinde [test ve güvenlik kontrol listesini](docs/TESTING.md) uygulayın.

## Teknik notlar

- Bu yayındaki İDA launch dosyası ZED 2 görüntü/derinlik akışını kullanır; aktif bir **LiDAR sürücüsü veya LiDAR tabanlı SLAM** başlatmaz. Arşivde bulunan üçüncü taraf LiDAR ve ZED kaynak kopyaları depoya dahil edilmemiştir.
- İDA arşivindeki `best.pt` tek kopya olarak `ida_ws/src/usv_navigation/best.pt` yolunda tutulmuştur. **İHA'nın ayrı plaka sınıflandırma modeli dosyalar arasında bulunmadığı için** `iha/models/best.pt` bu pakete eklenmemiştir. İDA modeli, İHA modelinin yerine doğrudan kullanılabilir varsayılmamalıdır.
- Kodun donanıma özgü GPIO, kamera, port, TF, PID ve güvenlik parametreleri gerçek sistem üzerinde doğrulanmalıdır.
- Kaynakların nasıl düzenlendiği ve hangi değişikliklerin yapıldığı [sürüm notlarında](docs/RELEASE_NOTES.md) belgelenmiştir.

## Lisans ve kullanım

Gönderilen ROS paketinin meta verilerinde `Proprietary` ibaresi bulunmaktadır. Bu depoda ayrıca bir açık kaynak lisansı verilmediğinden, GitHub'da herkese açık yayımlanması üçüncü kişilere otomatik yeniden kullanım veya dağıtım izni vermez. Takım açık kaynak paylaşımı planlıyorsa uygun bir lisans belirlemelidir. Model ağırlıklarının ve üçüncü taraf bileşenlerin yayımlanma hakları da kontrol edilmelidir.

---

**EGM MAVERA MARINE — TEKNOFEST 2026 Türkiye Birincisi**
