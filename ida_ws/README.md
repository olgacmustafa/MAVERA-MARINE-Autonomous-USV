# İDA | ROS 2 otonomi çalışma alanı

Bu dizin, ekibin `usv_navigation` adlı ROS 2 Python paketini içerir. İDA'ya ait `best.pt` model dosyası paketle birlikte tutulmuştur. ZED SDK, ZED ROS 2 wrapper ve MAVROS gibi üçüncü taraf bağımlılıklar bu depoya kopyalanmamıştır.

## Gereksinimler

- Ubuntu ve ROS 2 Humble; çalışan MAVROS kurulumu.
- ZED 2 kamera, uyumlu ZED SDK ve ROS 2 `zed_wrapper` paketi.
- `rclpy`, `mavros_msgs`, `numpy`, `opencv`, `pymavlink`, `ultralytics`, `torch` ve E-stop GPIO kullanılıyorsa `Jetson.GPIO` gibi paketler.
- Cube Orange+ veya uyumlu bir MAVLink uçuş kontrolcüsü ve ayrı doğrulanmış fiziksel emniyet devresi.

ROS/MAVROS, Jetson ve CUDA bağımlılıklarının kurulumu kullanılan sisteme göre farklılık gösterebilir. İHA'ya ait `requirements.txt` dosyası İDA çalışma alanının tüm bağımlılıklarını kurmaz.

## Derleme ve başlatma

```bash
cd ida_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select usv_navigation
source install/setup.bash
ros2 launch usv_navigation usv_otonom.launch.py fcu_url:=serial:///dev/ttyACM0:115200
```

`/dev/ttyACM0` yalnızca **örnek porttur**. Mümkünse `ls -l /dev/serial/by-id/` komutuyla bağlı cihazınızı doğrulayın ve `fcu_url` parametresini uyarlayın.

Launch dosyası MAVROS ve ZED ile birlikte `haberlesme`, `estop`, `los_node`, `obstacle_avoidance_node`, `pid_node` ve `data_logger_node` düğümlerini başlatır. Model dosyası `setup.py` aracılığıyla `share/usv_navigation/models/best.pt` konumuna kurulur.

Hedef renk durumunun JSON olarak kaydedileceği yol, `haberlesme` düğümündeki `kayit_yolu` ROS parametresiyle değiştirilebilir; varsayılan yol `~/mavera_ida/data/` altındadır.

**Önemli:** TF ofsetleri, GPIO çıkış mantığı, setpoint biçimleri, PID ayarları ve kapalı çevrim yönlendirme davranışı bu yayın hazırlığında gerçek donanımla yeniden doğrulanmamıştır.
