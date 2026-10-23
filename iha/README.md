# İHA | Hedef plaka rengi algılama

`src/canli_plaka_tespit.py`, AR0234 global shutter kameradan V4L2/GStreamer üzerinden aldığı görüntüyü YOLO ve HSV yöntemleriyle işler. RC kanalı üzerinden etkinleştirilen algılama sonucunda hedef renk MAVLink ile yer kontrol istasyonuna gönderilir.

## Gereksinimler

- AR0234 kamera, uygun sürücü ve GB Bayer düzeni; kaynak dosyada varsayılan aygıt `/dev/video0`.
- NVIDIA Jetson ortamına uygun CUDA/PyTorch, Ultralytics, OpenCV, NumPy, PyMAVLink, GStreamer Python bağları ve `v4l2-ctl`. GPIO kullanımı için Jetson.GPIO.
- İHA'ya ait **ayrı eğitilmiş plaka tespit modeli**: `iha/models/best.pt`. Bu model gönderilen dosyalar arasında yoktur; İDA paketindeki modelin aynı işi yapacağı varsayılmamalıdır.
- Cube/telemetri MAVLink bağlantısı ve kaynakta tanımlanmış RC7 arama anahtarı.

Bağımlılık sürümleri Jetson/JetPack ve kullanılan kamera sürücüsüne bağlıdır; `requirements.txt` dosyası tek başına tüm sistem kurulumunu tamamlamaz.

## Başlatma

Depo kök klasöründeyken:

```bash
python3 iha/src/canli_plaka_tespit.py
```

Ekransız kullanım için `MAVERA_EKRAN=0` ayarlanabilir. Kamera kayıtları ve yakalanan görüntüler `iha/` altında oluşturulur; bu çalışma çıktıları `.gitignore` kapsamındadır.

**Güvenlik:** Kamera pozlama, GPIO, RC7 ve algılama eşikleri kullanılan donanıma özgüdür. Gerçek uçuş öncesinde tahrik sistemleri devre dışıyken haberleşme ve karar mekanizması doğrulanmalıdır.
