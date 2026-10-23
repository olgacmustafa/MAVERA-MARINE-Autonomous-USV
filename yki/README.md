# YKİ | İHA–İDA MAVLink haberleşme köprüsü

`yki_iha_ida_koprusu.py`, iki RFD seri bağlantısını yönetir; İHA'dan alınan hedef renk mesajlarını tekrar doğrular ve `MAV_CMD_USER_1` komutuyla İDA'ya aktarır. İDA'dan `COMMAND_ACK` yanıtını izler. Mission Planner trafiğini ayrı UDP kanallarından çift yönlü iletir.

## Kurulum

```bash
python -m pip install -r yki/requirements.txt
```

## RFD bağlantılarını yapılandırma

Windows PowerShell üzerinde USB seri numaralarını ortam değişkeniyle tanımlayabilirsiniz:

```powershell
$env:MAVERA_IHA_RFD_SERIAL = "IHA_RFD_USB_SERI_NUMARASI"
$env:MAVERA_IDA_RFD_SERIAL = "IDA_RFD_USB_SERI_NUMARASI"
python yki/yki_iha_ida_koprusu.py
```

Alternatif olarak portları doğrudan belirtin (yalnızca örnektir):

```powershell
python yki/yki_iha_ida_koprusu.py --iha-port COM13 --ida-port COM12 --iha-baud 57600 --ida-baud 57600
```

`--sadece-iha` seçeneğiyle İDA hattını açmadan İHA–YKİ bağlantısını test edebilirsiniz. Tüm seçenekler için `--help` kullanın.

## Mission Planner

RFD seri portlarının sahibi bu köprü programıdır. Mission Planner'da aynı COM portlarını açmak yerine **UDP bağlantısını** kullanın. Varsayılan olarak İHA için `14550/14551`, İDA için `14560/14561` portları kullanılır (Mission Planner dinleme / köprü yerel portu). Çalışma ortamınıza göre düzenleyebilirsiniz.

**Not:** Gerçek araçta komut gönderilmeden önce seri port, MAVLink yönlendirmesi, hedef renk doğrulaması ve ACK akışı kontrol edilmelidir.
