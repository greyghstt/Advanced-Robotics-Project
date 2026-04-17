# Proyek Robotika Lanjut GCS

Dashboard Python untuk membaca telemetry ESP32 dan mengirim tuning PID.
Dashboard dibuat untuk dua jalur komunikasi: Bluetooth Serial dan WiFi UDP.
Nama Robjut dipakai sebagai singkatan dari Robotika Lanjut.

## File

| File | Fungsi |
| --- | --- |
| `gcs.py` | Dashboard Bluetooth Serial Robjut |
| `gcs_udp.py` | Dashboard WiFi UDP Robjut |
| `requirements.txt` | Dependency Python dashboard |

## Dependency

Gunakan Python yang sudah memiliki PySide6 dan pyserial.

```powershell
cd C:\vscode\Project-Robjut\gcs
python -m pip install -r requirements.txt
```

## UDP GCS

UDP adalah mode yang aktif saat ini di firmware. Pastikan konfigurasi firmware
di `include/Gcs_config.h` seperti ini:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 0
#define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1
```

Jalankan dashboard:

```powershell
cd C:\vscode\Project-Robjut\gcs
python gcs_udp.py
```

Alur koneksi:

1. Nyalakan hotspot atau WiFi yang sama dengan ESP32.
2. Upload firmware ke ESP32.
3. Buka Serial Monitor untuk melihat IP ESP32.
4. Jalankan `python gcs_udp.py`.
5. Isi host dengan `robjut.local` atau IP ESP32.
6. Remote port default `4210`.
7. Local port default `4211`.
8. Klik `Start UDP`.

Dashboard akan mengirim `HELLO` ke ESP32. Setelah ESP32 menerima `HELLO`,
telemetry mulai dikirim ke dashboard.

## Bluetooth GCS

Aktifkan mode Bluetooth di `include/Gcs_config.h`:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 0
#define ENABLE_BT_GCS 1
#define ENABLE_UDP_GCS 0
```

Jalankan dashboard:

```powershell
cd C:\vscode\Project-Robjut\gcs
python gcs.py
```

Alur koneksi:

1. Upload firmware ESP32.
2. Pair Bluetooth Windows dengan device `Robjut-GCS`.
3. Cek COM port Bluetooth di Device Manager.
4. Jalankan `python gcs.py`.
5. Pilih COM port Bluetooth.
6. Klik `Connect`.

## Format Telemetry

Firmware dapat mengirim CSV dan JSON. Dashboard memakai data status seperti:

- radio/failsafe/arm/mode
- input PWM remote
- output PWM motor
- roll/pitch/yaw
- gyro
- nilai PID aktif
- trim

## Command Firmware

Command yang didukung:

```text
HELLO
PING
HEADER
GET CSV
GET JSON
PID
SET K_PITCH 4.8000
SET K_PITCH_RATE 1.3500
SET TRIM_PITCH -2.0000
```

Secara default firmware menolak update PID saat `arm=1`. Disarm dulu sebelum
mengirim PID dari dashboard.

## Catatan Mode

Mode telemetry memakai `#define`, jadi setiap ganti mode perlu build dan upload
ulang firmware ke ESP32.
