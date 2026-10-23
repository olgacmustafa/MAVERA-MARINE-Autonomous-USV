# GitHub yayın notları

Bu depo dört kullanıcı dosyasından düzenlendi: İHA kamera yazılımı, YKİ haberleşme köprüsü, güncellenmiş bağımsız İDA MAVROS hedef renk alıcısı ve İDA ROS 2 ZIP arşivi.

## Yayına hazırlık ve entegrasyon değişiklikleri

1. Kaynaklar `iha/`, `yki/` ve `ida_ws/src/usv_navigation/` altında düzenlendi; genel kullanıma uygun dosya adları verildi.
2. Üçüncü taraf Stereolabs ve Slamtec kaynak kopyaları, iç içe Git geçmişleri, eski yedekler, geçici önbellekler ve proje içi notlar çıkarıldı. Bu bağımlılıklar gerektiğinde ayrıca kurulur.
3. İDA'ya ait YOLO modelinin yinelenen kopyası kaldırıldı; tek model `ida_ws/src/usv_navigation/best.pt` yolunda tutuldu.
4. Ayrı gönderilen güncel renk alıcısı, `usv_navigation/haberlesme.py` olarak entegre edildi ve ROS 2 launch/setuptools tarafından kullanılan sürüm haline getirildi. Bu **davranışı etkileyebilecek bir entegrasyon değişikliğidir**; masa testi yapılmalıdır.
5. İDA launch dosyasındaki cihaza özgü Cube USB kimliği ve sabit ev dizini yolu kaldırıldı. Örnek port `/dev/ttyACM0`, renk kaydı için kullanıcıya göre belirlenen varsayılan yol kullanıldı.
6. YKİ RFD tespiti, sabit kişisel USB seri numaraları yerine ortam değişkenleri ve isteğe bağlı elle COM portu parametreleriyle yapılandırılabilir hale getirildi.
7. İHA yazılımındaki kişiye özel anlatım ve geçici öğretici yorumlar sadeleştirildi; algılama, pozlama, uçuşta etkinleştirme ve mesajlaşma eşiklerine dokunulmadı.
8. Türkçe ana README, İngilizce README, bileşen kılavuzları, mimari/protokol belgeleri, statik testler, CI ve `.gitignore` hazırlandı.
9. Paket yöneticisindeki örnek iletişim adresi `maintainer@example.com` olarak değiştirildi. Yayımlamadan önce ekipçe uygun bir iletişim adresi seçilebilir.

## Bilinçli olarak değiştirilmemiş noktalar

- İDA hareket kontrol algoritmaları, ROS konuları, parametreler, waypoint mantığı, PID katsayıları, kamera eşikleri ve E-stop GPIO polaritesi.
- İHA RC7 arama anahtarı, model güven eşikleri, renk kilitleme ve MAVLink mesajlaşma davranışı.
- YKİ komut tekrar/ACK mantığı ile Mission Planner UDP portları; yalnızca radyo kimliklerini yapılandırma yöntemi değiştirildi.

## Pakette bulunmayanlar

- Jetson servis/systemd tanımları ve cihaza özgü işletim sistemi ayarları.
- İHA plaka algılama modeli: proje sahibinin `iha/models/best.pt` yoluna ayrıca yerleştirmesi gerekir.
- Üçüncü taraf SDK kaynak kopyaları ve derlenmiş ROS çalışma alanı çıktıları.
