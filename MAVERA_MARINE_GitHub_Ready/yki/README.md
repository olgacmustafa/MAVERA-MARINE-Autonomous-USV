# Ground station: UAV → USV MAVLink relay

`yki_iha_ida_koprusu.py` handles two RFD serial links, forwards Mission Planner traffic over separate loopback UDP endpoints, validates repeated incoming colour messages, and sends a MAVLink `MAV_CMD_USER_1` to the USV until it receives an acknowledgement.

## Install

```bash
python -m pip install -r yki/requirements.txt
```

## Configure

For automated USB detection, set serial numbers locally (examples only):

```powershell
$env:MAVERA_IHA_RFD_SERIAL = "YOUR_UAV_RADIO_USB_SERIAL"
$env:MAVERA_IDA_RFD_SERIAL = "YOUR_USV_RADIO_USB_SERIAL"
python yki/yki_iha_ida_koprusu.py
```

Alternatively, supply ports explicitly (Windows example):

```powershell
python yki/yki_iha_ida_koprusu.py --iha-port COM13 --ida-port COM12 --iha-baud 57600 --ida-baud 57600
```

`--sadece-iha` tests the UAV link without opening the USV radio. Find all supported options with `--help`.

**Mission Planner:** connect by UDP instead of opening a COM port that the bridge already owns. Default loopback channels are UAV `14550/14551` and USV `14560/14561` (Mission Planner listener / bridge local port). Adapt ports to your setup.
