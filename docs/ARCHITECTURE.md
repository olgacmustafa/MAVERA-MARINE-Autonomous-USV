# Sistem mimarisi ve arayüzler

## Bileşenler ve bağlantılar

- **İHA:** AR0234 RAW10 kamera (V4L2/GStreamer), YOLO + HSV, Jetson, Cube/RC7, MAVLink ile veri gönderimi.
- **Yer kontrol istasyonu (YKİ):** İki USB RFD modem, PyMAVLink seri bağlantısı, Mission Planner için UDP köprüsü ve hedef renk doğrulama mantığı.
- **İnsansız deniz aracı (İDA):** Cube bağlantısını MAVROS yönetir. ROS 2 üzerindeki `haberlesme` düğümü MAVROS raw mesajlarını alır, hedef rengi yayınlar ve `COMMAND_ACK` yanıtını MAVROS üzerinden gönderir.

## MAVLink haberleşme protokolü

| Alan | Kaynak kodlarda kullanılan değer |
| --- | --- |
| İHA renk bildirimi | `NAMED_VALUE_INT`, mesaj adı `TGT_COLOR` |
| YKİ → İDA komutu | `MAV_CMD_USER_1` (alıcıda `31010` yedeği) |
| Protokol sürümü | `1` |
| Uygulama işareti | `2026` (kriptografik imza değildir) |
| Hedef renk 1 | `SIYAH` → `siyah` |
| Hedef renk 2 | `KIRMIZI` → `kirmizi` |
| Hedef renk 3 | `YESIL` → `yesil` |
| YKİ MAVLink kaynak kimliği | `201/190` |
| İDA Jetson kaynak kimliği | `202/191` |
| İHA Jetson kaynak kimliği | `200/191` |
| İDA hedef renk konusu | `/carpilacakhedef` (`std_msgs/String`) |
| İDA MAVROS alış/gönderiş | `/mavros/mavlink/from`, `/mavros/mavlink/to` |

YKİ, arka arkaya gelen üç aynı renk bildirimini doğrular. İDA, kabul ettiği hedef rengi kilitler. **Uygulama işareti bir kimlik doğrulama veya şifreleme mekanizması değildir**; gerçek sistemde güvenilir telemetri bağlantısı ve uygun erişim kontrolleri kullanılmalıdır.

## ROS 2 düğümleri

| Düğüm | Sorumluluk |
| --- | --- |
| `haberlesme` | MAVLink komutunu alma, doğrulama, onaylama ve hedef renk yayınlama |
| `los_node` | Waypoint takibi ve yönlendirme |
| `obstacle_avoidance_node` | ZED görüntü/derinlik ile YOLO tabanlı engel değerlendirme, yön ve hız yönetimi |
| `pid_node` | Yönlendirme komutlarını MAVROS setpoint mesajlarına dönüştürme |
| `estop` | RC kanal 9 üzerinden yazılımsal acil durdurma ve GPIO yönetimi |
| `data_logger_node` | Video, derinlik/yerel harita ve telemetri kaydı |

Verilen launch dosyası LiDAR sürücüsü veya `slam_toolbox` başlatmaz. `/camera_local_costmap` çıktısı, kameraya göre oluşturulan **yerel** doluluk temsilidir; kalıcı bir küresel SLAM haritası değildir.
