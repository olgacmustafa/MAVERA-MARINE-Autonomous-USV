# Test, donanım entegrasyonu ve güvenlik

## Bu yayın için yapılan kontroller

- Yayımlanan Python kaynakları için sözdizimi ve AST kontrolü.
- Uygun ayrıştırıcı bulunduğunda XML ve YAML dosyalarının kontrolü.
- ROS 2 `setup.py` giriş noktaları ile kaynak modüllerin eşleşmesi.
- İHA, YKİ ve İDA hedef renk protokol kimliklerinin uyumu.
- İç içe üçüncü taraf Git depoları, eski yedekler ve derlenmiş Python önbelleğinin pakette bulunmaması.

Bu statik kontroller, otonom deniz aracının **donanım güvenliğini veya gerçek koşullarda doğru çalıştığını kanıtlamaz**.

## Kontrollü entegrasyon kontrol listesi

- [ ] İHA AR0234 ve İDA ZED 2 kamera donanımlarını, sürücüleri ve YOLO sınıflarını doğrulayın; eksik İHA `best.pt` modelini sağlayın.
- [ ] USB aygıtlarını ve MAVROS `fcu_url` adresini yapılandırın; aynı seri bağlantıyı iki sürecin açmadığını kontrol edin. Motorlar kapalıyken mesaj yönlendirmesi ve ACK akışını test edin.
- [ ] `MAV_CMD_USER_1`, `TGT_COLOR`, üç renk kodu ve `/carpilacakhedef` iletişimini; gecikmiş hedef, yinelenen paket ve bağlantı kopması senaryolarıyla sınayın.
- [ ] RFD bağlantılarını, RC alıcısını, ZED akışını ve MAVROS'u ayrı ayrı keserek zaman aşımı ve güvenli duruş davranışını doğrulayın.
- [ ] **E-stop fiziksel devresinin çıkış polaritesini doğrulayın.** Mevcut GPIO/RC mantığı korunmuştur; RC zaman aşımında ROS duruşu üretmesi fiziksel çıkışın güvenli olduğunu tek başına garanti etmez.
- [ ] Kamera/GPS TF ofsetlerini gerçek araç üzerinde ölçün.
- [ ] Su testinden önce pervane yönlerini, setpoint maskelerini ve PID parametrelerini emniyetli, kısıtlı bir test düzeneğinde kontrol edin.
- [ ] Model ağırlıklarının paylaşım haklarını, bağımlılık lisanslarını ve ilgili güvenlik/yarışma kurallarını değerlendirin.
- [ ] Kayıtları, logları, görev bitirme durumlarını önce kontrollü kuru testte, ardından gözetimli su testinde doğrulayın.

## Sınırlamalar

Kaynaklar paylaşılmıştır ancak birlikte gerçek donanım günlükleri ve yeniden üretilebilir bir çalıştırma ortamı sağlanmamıştır. Bu nedenle depo **yayına hazırlanmış tarihsel proje kaynağı ve eğitim/inceleme referansı** niteliğindedir; hazır ve sertifikalı bir otonom kontrol sistemi değildir. ZIP arşivindeki eski alıcı yerine daha güncel bağımsız İDA alıcısı kullanıldığı için entegrasyonun yeniden masa testi gerekir.
