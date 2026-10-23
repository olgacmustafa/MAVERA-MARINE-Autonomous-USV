# EGM–MAVERA MARINE | İnsansız Deniz Aracı Yazılımı

TEKNOFEST 2026 İnsansız Deniz Aracı Yarışması'nda Türkiye birinciliği elde eden **EGM–MAVERA MARINE** takımının İHA algılama, yer kontrol istasyonu (YKİ) haberleşme ve İDA otonomi bileşenleri.

**Durum:** GitHub'da yayımlanmak üzere sadeleştirilmiş kaynak arşivi. Bu düzenleme, yarışta yeniden çalıştırılmış ve saha testlerinden geçirilmiş bir yazılım sürümü anlamına gelmez.

## İçerik

| Klasör | Açıklama |
| --- | --- |
| [`iha/`](iha/README.md) | AR0234 kamera, YOLO/HSV ile hedef renk tespiti, RC7 arama anahtarı ve MAVLink gönderimi |
| [`yki/`](yki/README.md) | İki RFD bağlantısıyla İHA'dan alınan hedef rengin İDA'ya aktarımı ve Mission Planner UDP köprüsü |
| [`ida_ws/src/usv_navigation/`](ida_ws/README.md) | ROS 2 Humble otonomi, ZED görüntü/depth, LOS, engelden kaçınma, PID, E-stop ve kayıt |
| [`docs/`](docs/ARCHITECTURE.md) | Mimari, haberleşme ve yayın notları |
| [`tests/`](tests/test_repository.py) | Statik bütünlük kontrolleri |

## Çalışma mantığı

1. İHA, plaka görüntüsünden hedef rengi çıkarır ve MAVLink ile YKİ'ye gönderir.
2. YKİ, alınan rengi doğrular ve MAVLink komutuyla İDA'ya aktarır.
3. İDA, hedef rengi `/carpilacakhedef` başlığında yayınlar. Normal waypoint seyri hedef rengin gelmesini beklemeden ilerleyebilir; hedefe çarpma safhasında bu bilgi gerekir.
4. ZED 2 görüntü/derinlik verileriyle engeller değerlendirilir; LOS ve PID düğümleri hareket komutlarını üretir. Kamera tabanlı yerel harita ve telemetri kayıtları oluşturulur.

**Renk kodları:** `1 = siyah`, `2 = kırmızı`, `3 = yeşil`. Bu üç bileşenin MAVLink kimlikleri, sürümü ve mesaj adları birbiriyle uyumlu olmalıdır.

## Kurulum ve çalıştırma

Bu depo tek bir bilgisayarda çalışan monolitik bir program değildir. Her bileşen kendi ortamına kurulur:

- **İHA:** [iha/README.md](iha/README.md). Jetson, AR0234, GStreamer, CUDA uyumlu Torch ve **ayrı eğitilmiş plaka modeli** gerekir.
- **YKİ:** [yki/README.md](yki/README.md). Windows/diğer uygun ortamda PyMAVLink + PySerial, USB seri numarası ya da `--iha-port` / `--ida-port` ayarı gerekir.
- **İDA:** [ida_ws/README.md](ida_ws/README.md). ROS 2 Humble, MAVROS, ZED ve araç donanımı gerektirir. Cube USB portunu launch argümanıyla belirtin.

```bash
python -m unittest discover -s tests -v
python -m compileall -q iha yki ida_ws/src/usv_navigation
```

## Önemli teknik notlar

- Orijinal ZIP'teki `sllidar_ros2`, ZED kaynak kopyaları, iç içe `.git` klasörleri, eski yedekler ve `__pycache__` dosyaları yayına dahil edilmedi. Mevcut İDA launch dosyası aktif LiDAR/SLAM başlatmıyor.
- İDA arşivindeki `best.pt` İDA ROS paketinde **tek kopya** olarak tutuldu. İHA için ayrı plaka modeli gönderilmedi; `iha/models/best.pt` yolu bilinçli olarak boş. İDA modelinin İHA'da uyumlu olduğu varsayılmamalıdır.
- RFD seri numaraları, tekneye özel Cube USB kimliği ve sabit kullanıcı ev klasörü yolu genel kullanım için parametreleştirildi.
- Gerçek araçtaki E-stop çıkış polaritesi, PID ayarları, konum/TF ofsetleri, kamera sürücüsü ve MAVLink akışının uçtan uca davranışı **yeniden fiziksel olarak doğrulanmalıdır**. [Test kontrol listesi](docs/TESTING.md).

## Lisans

Gönderilen ROS paketinde `Proprietary` ibaresi bulunuyordu; takım tarafından açık kaynak lisansı belirtilmedi. Bu nedenle depoyu kamuya açık yapmak, üçüncü kişilere otomatik kullanım/dağıtım izni vermez. Açık kaynak paylaşım hedefleniyorsa lisans ayrıca seçilmelidir. Model ağırlıklarının yayımlama hakları da kontrol edilmelidir.
