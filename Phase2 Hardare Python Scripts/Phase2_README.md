# Phase 2 — Real Hardware Deployment

This phase deploys the full sensor fusion pipeline on a physical RC car platform. A Jetson Orin Nano runs perception and fusion; a Raspberry Pi 5 handles radar processing; an Arduino Uno R4 WiFi bridges motor control via UDP. The system performs reactive obstacle avoidance driven by live LiDAR zone analysis and TTC-based speed decisions.

---

## Folder Structure

```
Phase2 Hardware Python Scripts/
├── jetson_fusion.py                    # Main fusion pipeline (runs on Jetson Orin Nano)
├── radar_send.py                       # Radar streamer (runs on Raspberry Pi 5)
│
├── initial_Test_Debug_Scripts/         # Bench testing and sensor validation scripts
│   ├── camera_test.py                  # IMX477 camera basic capture test
│   ├── Front_obj.py                    # Early forward obstacle detection
│   ├── Front_obj_GUI.py                # GUI version of forward obstacle detection
│   ├── jetson_receive.py               # Early UDP receive test on Jetson
│   ├── lidar_distance.py               # LD06 distance measurement script
│   ├── lidar_gauge.py                  # LiDAR proximity gauge display
│   ├── lidar_GUI.py                    # LiDAR bird's-eye GUI (Tkinter + Matplotlib)
│   ├── lidar_test.py                   # Basic LD06 packet parser test
│   ├── radar.py                        # Standalone radar pipeline test
│   ├── run_example_range_doppler.py    # Range-Doppler heatmap example
│   ├── udp_streaming_azimuth.py        # Radar azimuth UDP stream test
│   ├── udp_streaming_range_doppler.py  # Radar range-Doppler UDP stream test
│   ├── udp_streaming_tracking.py       # Radar tracking UDP stream test
│   ├── visualization_range_doppler.py  # Range-Doppler visualisation
│   └── yolo_test.py                    # YOLOv8 inference test on Jetson
│
├── Motor scripts/
│   ├── car_control.py                  # High-level car control (keyboard / UDP command input)
│   ├── motor_controller.py             # Motor command mapper (TTC level to PWM values)
│   ├── arduino_motor_controller/
│   │   └── arduino_motor_controller.ino   # Arduino firmware (ESC + steering servo via UDP)
│   └── offline_motor_control_only/
│       ├── motor.py                    # Offline motor test (no fusion)
│       └── sketch_mar12a.ino           # Early Arduino sketch
│
└── Radar_Base_Configurations/
    ├── radar_config/                   # BGT60TR13C hardware configuration files
    │   ├── config_3rx_2m/              # Short range (2m) config
    │   ├── config_3rx_5m/              # Medium range (5m) config — used in deployment
    │   ├── config_3rx_10m/             # Long range (10m) config
    │   └── config_track/               # Tracking-optimised config
    └── utility/                        # BGT60TR13C SDK utility modules
        ├── BGT60TR13C.py               # Main radar driver
        ├── BGT60TR13C_CONST.py         # Radar constants
        ├── FFTW.py                     # FFT processing wrapper
        ├── helper.py                   # Shared helper functions
        ├── mmw_cube_proc_v0.py         # Millimetre-wave cube processing
        ├── udp_real_time_vis.py        # UDP real-time visualisation
        └── udp_streaming.py            # UDP streaming utility
```

---

## System Architecture

```
Raspberry Pi 5                    Jetson Orin Nano
  BGT60TR13C radar (SPI)    -->   jetson_fusion.py
  radar_send.py                     |-- LD06 LiDAR (serial)
  UDP port 9576             -->     |-- IMX477 camera (CSI / GStreamer)
                                    |-- Fusion verdict (SAFE/CAUTION/IMMINENT)
                                    |
                                    v UDP port 5005
                              Arduino Uno R4 WiFi
                                |-- ESC (pin 9, MSC-25RC)
                                |-- Steering servo (pin 10)
```

---

## Setup and Requirements

### Jetson Orin Nano

**System:** JetPack 6, Python 3.10

```bash
pip install pyqtgraph PyQt5 numpy scipy pyserial
```

**PyTorch (NVIDIA JetPack wheel):**

```bash
pip install cusparselt
pip install torch-2.5.0a0+...  # Use the NVIDIA JetPack 6.1/6.2 wheel
```

**OpenCV fix (use system apt build inside venv):**

```bash
ln -s /usr/lib/python3/dist-packages/cv2/python-3.10/cv2.cpython-310-aarch64-linux-gnu.so \
      $VIRTUAL_ENV/lib/python3.10/site-packages/cv2.so
```

**YOLOv8:**

```bash
pip install ultralytics
```

Note: torchvision is mocked on Jetson — a pure-PyTorch NMS stub is injected via `sys.modules` at runtime. No separate torchvision install is needed.

### Raspberry Pi 5

**Python 3.11**

```bash
pip install numpy scipy ifxdaq  # ifxdaq = Infineon DreamHAT+ SDK
```

The BGT60TR13C SDK (`utility/`) must be on the Python path. Copy the `utility/` folder to the same directory as `radar_send.py` or add it to `PYTHONPATH`:

```bash
export PYTHONPATH=$PYTHONPATH:/path/to/Radar_Base_Configurations/utility
```

### Arduino Uno R4 WiFi

Open `Motor scripts/arduino_motor_controller/arduino_motor_controller.ino` in the Arduino IDE.

Update the following before flashing:

```cpp
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

The Arduino listens on **UDP port 5005**. It controls:
- Pin 9: ESC (MSC-25RC) — forward = 1700 µs, slow forward = 1570 µs, stop = 1500 µs
- Pin 10: Steering servo — centre = 1500 µs, left/right adjusted per avoidance direction

**Note:** The Jetson's GPIO outputs 3.3 V, which is insufficient for the MSC-25RC ESC. The Arduino provides the required 5 V signal level. The ESC's BEC powers the servo rail.

**Known issue:** `WiFi.localIP()` may return `0.0.0.0` after connecting. To find the Arduino's IP, check your router's DHCP table or run `arp -a` on the host machine after the Arduino connects.

---

## Running the System

### Step 1 — Flash the Arduino

Flash `arduino_motor_controller.ino` via the Arduino IDE. Open Serial Monitor at 115200 baud to confirm WiFi connection and IP address.

### Step 2 — Start the radar streamer on the Pi

```bash
python radar_send.py
```

This reads the BGT60TR13C over SPI, runs the full pipeline (Range FFT, Doppler FFT, MTI clutter suppression, CFAR detection, nearest-neighbour tracking), and streams confirmed tracks as JSON to the Jetson on port 9576.

### Step 3 — Start the fusion pipeline on the Jetson

```bash
python jetson_fusion.py
```

This receives radar tracks on UDP 9576, reads the LD06 LiDAR on serial, runs YOLOv8 on the IMX477 camera, fuses all three sensors, and sends motor commands to the Arduino on UDP port 5005.

The fusion GUI (PyQt5 + pyqtgraph) shows:
- Left panel: LiDAR bird's-eye scatter (zones: red = centre, blue = left, green = right)
- Centre panel: Radar range-velocity plot with TTC annotations
- Right panel: TTC bar chart and track table
- Bottom bar: System health status

### Step 4 — (Optional) Manual control test

```bash
python "Motor scripts/car_control.py"
```

Sends keyboard-controlled UDP commands to the Arduino for pre-integration testing.

---

## Sensor Configuration

### LD06 LiDAR
- Baud rate: 230400
- FOV: ±20° forward only (configurable in `jetson_fusion.py`)
- Min distance floor: 20 cm
- Persistence filter: 3 scans to raise alert, 4 scans to clear (asymmetric hysteresis)

### BGT60TR13C Radar
- Config used: `config_3rx_5m` (5 m range)
- MTI clutter suppression memory coefficient: 0.92
- CFAR threshold: 16 dB
- Tracker: nearest-neighbour, 3 confirmed frames to report, 3 misses to delete
- TTC thresholds: IMMINENT = < 1.5 s or < 0.5 m; CAUTION = < 3 s or < 1.2 m
- Only detects moving targets (MTI design — static obstacles are LiDAR's responsibility)

### IMX477 Camera
- Interface: CSI (GStreamer pipeline via `nvarguscamerasrc`)
- Input resolution for YOLO: 416 x 416 (hardware-resized via `nvvidconv`)
- Inference: YOLOv8n on Jetson CUDA, every 2nd frame
- Forward zone: centre 40% of frame width
- Camera is cross-validation only — it can suppress a LiDAR alert with no visual backing, but cannot trigger alerts independently

### Fusion Logic
- Final verdict = worst case across all active sensors
- No sensor can vote a result downward
- If both LiDAR and camera are simultaneously impaired, output floors at CAUTION (never silently returns SAFE when blind on 2 of 3 sensors)

---

## TTC to Motor Mapping

| Fusion Level | ESC Command | Behaviour |
|---|---|---|
| SAFE | 1700 µs | Full forward |
| CAUTION | 1570 µs | Slow forward, steer away from obstacle |
| IMMINENT | 1500 µs | Full stop |
