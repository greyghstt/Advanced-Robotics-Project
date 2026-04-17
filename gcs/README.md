# Robjut GCS

Dashboard Python untuk membaca telemetry ESP32 dan mengirim tuning PID.

Ada dua aplikasi:

```text
gcs.py      -> Bluetooth Serial GCS
gcs_udp.py  -> WiFi UDP GCS
```

## Jalankan Bluetooth GCS

```powershell
cd C:\vscode\Project-Robjut\gcs
python gcs.py
```

Alur Bluetooth:

1. Upload firmware ESP32.
2. Pair Bluetooth Windows dengan device `Robjut-GCS`.
3. Cek COM port Bluetooth yang muncul di Device Manager.
4. Buka `python gcs.py`.
5. Pilih COM port Bluetooth.
6. Klik `Connect`.

## Jalankan UDP GCS

Pastikan ESP32 dan laptop ada di jaringan WiFi yang sama, lalu isi SSID/password di:

```text
include/Gcs_config.h
```

Jalankan:

```powershell
cd C:\vscode\Project-Robjut\gcs
python gcs_udp.py
```

Alur UDP:

1. Upload firmware ESP32 dengan `ENABLE_UDP_GCS 1`.
2. Buka Serial Monitor untuk melihat IP ESP32.
3. Buka `python gcs_udp.py`.
4. Isi host `robjut.local` atau IP ESP32.
5. Remote port default `4210`.
6. Local port default `4211`.
7. Klik `Start UDP`.

GCS akan mengirim `HELLO` ke ESP32. Setelah ESP32 menerima `HELLO`, telemetry mulai dikirim ke dashboard.

## Command Firmware

```text
HELLO
HEADER
GET CSV
GET JSON
PID
SET K_PITCH 4.8000
SET K_PITCH_RATE 1.3500
```

Secara default firmware menolak update PID saat `arm=1`. Disarm dulu sebelum mengirim PID dari GCS.

## Pilih Mode Firmware

Mode dipilih di:

```text
include/Gcs_config.h
```

Contoh mode UDP saja:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 0
#define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1
```

Karena mode memakai `#define`, setiap ganti mode perlu build dan upload ulang ke ESP32.
