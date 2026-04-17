# Project-Robjut

Firmware ESP32 untuk quadcopter Robjut, berisi pembacaan SBUS receiver,
kontrol motor ESC, kontrol attitude, dan telemetry/GCS melalui WiFi atau
Bluetooth.

## Status Singkat

- Board: ESP32 DOIT DevKit V1.
- Framework: Arduino melalui PlatformIO.
- Receiver: SBUS pada GPIO 35.
- Motor ESC: output PWM LEDC 50 Hz.
- Mode telemetry aktif saat ini: UDP GCS.
- Dashboard GCS tersedia di folder `gcs`.

## Struktur Folder

```text
Project-Robjut/
|-- include/              Header firmware dan konfigurasi utama
|-- src/                  Entry point firmware ESP32
|-- gcs/                  Aplikasi dashboard Python
|-- lib/                  Library lokal atau vendored dependency
|-- test/                 Area test PlatformIO
|-- platformio.ini        Konfigurasi build/upload PlatformIO
`-- README.md            Dokumentasi utama project
```

## Hardware Utama

| Fungsi | Pin |
| --- | --- |
| SBUS RX | GPIO 35 |
| Motor 1 | GPIO 33 |
| Motor 2 | GPIO 25 |
| Motor 3 | GPIO 26 |
| Motor 4 | GPIO 27 |

Layout motor yang dipakai di mixer adalah diagonal/X:

| Motor | Posisi | Arah putar yang disarankan |
| --- | --- | --- |
| M1 | Front-left | CW |
| M2 | Front-right | CCW |
| M3 | Rear-right | CW |
| M4 | Rear-left | CCW |

## Persiapan

1. Install Visual Studio Code.
2. Install extension PlatformIO.
3. Buka folder project:

```text
C:\vscode\Project-Robjut
```

4. Pastikan ESP32 terhubung melalui USB.
5. Build atau upload melalui PlatformIO.

## Build dan Upload

Dari PlatformIO UI:

1. Buka PlatformIO.
2. Pilih environment `esp32doit-devkit-v1`.
3. Klik `Build` untuk kompilasi.
4. Klik `Upload` untuk upload ke ESP32.

Dari terminal:

```powershell
cd C:\vscode\Project-Robjut
platformio run -e esp32doit-devkit-v1
platformio run -e esp32doit-devkit-v1 -t upload
```

Serial Monitor:

```powershell
platformio device monitor -b 115200
```

## Pilihan Mode Telemetry

Mode dipilih di `include/Gcs_config.h`.

Mode UDP GCS aktif saat ini:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 0
#define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1
```

Setiap perubahan mode perlu build dan upload ulang ke ESP32.

## UDP GCS

UDP GCS dipakai untuk dashboard WiFi dengan latency rendah. Alurnya:

1. Nyalakan hotspot atau WiFi yang sama dengan konfigurasi firmware.
2. Upload firmware dengan `ENABLE_UDP_GCS 1`.
3. Buka Serial Monitor dan catat IP ESP32.
4. Jalankan dashboard:

```powershell
cd C:\vscode\Project-Robjut\gcs
python gcs_udp.py
```

5. Isi host dengan `robjut.local` atau IP ESP32.
6. Remote port default adalah `4210`.
7. Klik `Start UDP`.

Dashboard akan mengirim `HELLO` ke ESP32. Setelah itu ESP32 mengirim data
telemetry ke dashboard.

## Bluetooth GCS

Bluetooth GCS dapat dipakai jika mode Bluetooth diaktifkan:

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

Pair Windows dengan device `Robjut-GCS`, pilih COM port, lalu klik
`Connect`.

## WiFi HTTP Telemetry

HTTP telemetry dapat dipakai jika mode ini diaktifkan:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 1
#define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 0
```

Setelah ESP32 terhubung WiFi, buka:

```text
http://robjut.local:8080
```

Jika mDNS tidak terbaca, gunakan IP ESP32 dari Serial Monitor.

## Tuning PID

Nilai default PID ada di `include/Copter_control.h`. Dashboard GCS dapat
membaca nilai PID dan mengirim command tuning.

Secara default update PID saat drone `arm=1` diblokir untuk keamanan. Disarm
dulu sebelum mengirim nilai PID baru dari dashboard.

Contoh command:

```text
PID
SET K_PITCH 4.8000
SET K_PITCH_RATE 1.3500
```

## Catatan Keamanan

- Lepas propeller saat upload, test motor, dan tuning awal.
- Pastikan arah motor dan posisi motor sesuai tabel hardware.
- Pastikan ESC sudah terkalibrasi dan menerima ground yang sama dengan ESP32.
- Jangan ubah mode telemetry saat motor sedang armed.
- Simpan SSID/password WiFi hanya di repo private atau ubah sebelum publish.

## Dokumentasi Lanjutan

- `include/README` menjelaskan fungsi header firmware.
- `gcs/README.md` menjelaskan dashboard Python.
- `lib/README` menjelaskan folder library lokal.
- `test/README` menjelaskan folder test PlatformIO.
