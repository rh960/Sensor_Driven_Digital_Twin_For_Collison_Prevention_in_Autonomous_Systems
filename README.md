# Sensor-Driven Digital Twin Framework for Collision Prevention in Autonomous Systems

**Raffay Hassan** | BEng Computer Systems Engineering | Middlesex University London | 2026

[![LinkedIn](https://img.shields.io/badge/LinkedIn-raffay--hassan-blue)](https://www.linkedin.com/in/raffay-hassan)
[![Blog](https://img.shields.io/badge/Blog-autonomous--systems-green)](https://raffayhassan772.wixsite.com/autonomous-systems)

---

## Project Overview

This project develops a low-cost, distributed edge-computing system for real-time collision prevention in autonomous vehicles. Rather than targeting full autonomy, the focus is on predictive Time-to-Collision (TTC) estimation using multi-sensor fusion across LiDAR, radar, and camera — building a software-layer Digital Twin that continuously models obstacle positions, velocities, and risk levels.

The Digital Twin in this project is **not** a simulation environment. It is the internal world model maintained by the fusion pipeline, updated in real time from live sensor data.

---

## Repository Structure

```
├── Phase1 Carla Project Python Scripts/   # Simulation baseline (CARLA)
├── Phase2 Hardware Python Scripts/        # Real hardware deployment
├── Project_Documentation/                 # Report, poster, presentation
└── README.md
```

---

## Project Phases

### Phase 1 — CARLA Simulation
Establishes a simulation baseline using the CARLA autonomous driving simulator. Implements YOLOv8n object detection, LiDAR and radar sensor fusion, TTC-based emergency braking, and adverse weather testing in Town04 scenarios.

### Phase 2 — Real Hardware
Deploys the full sensor stack on a Maverick RC car chassis. A Jetson Orin Nano handles perception and fusion; a Raspberry Pi 5 runs the radar pipeline; an Arduino Uno R4 WiFi bridges motor control. Reactive obstacle avoidance is driven by live LiDAR zone analysis and TTC fusion output.

---

## Hardware Platform (Phase 2)

| Component | Details |
|---|---|
| Chassis | Maverick Monster Truck RC car |
| Main compute | NVIDIA Jetson Orin Nano |
| Co-processor | Raspberry Pi 5 |
| LiDAR | LD06 (230400 baud, ±20° FOV) |
| Radar | BGT60TR13C on DreamHAT+ (SPI, Pi 5) |
| Camera | Arducam IMX477 HQ (CSI, Jetson) |
| Motor controller | Arduino Uno R4 WiFi (UDP port 5005) |
| ESC | MSC-25RC (5V signal, pin 9) |
| Steering servo | Pin 10, BEC-powered |

---

## Key Technical Contributions

- Asymmetric hysteresis persistence filter on LiDAR (3 scans to raise alert, 4 to clear)
- MTI clutter suppression on radar (memory coefficient 0.92) with CFAR detection at 16 dB
- Monocular distance estimation from YOLOv8 bounding box height using projective geometry
- Worst-case fusion verdict across all active sensors; floors at CAUTION if two sensors are simultaneously impaired
- Reactive navigation: TTC level maps directly to ESC throttle (SAFE = forward, CAUTION = slow, IMMINENT = stop)

---

## Documentation

| File | Description |
|---|---|
| `Project_Documentation/Raffay Hassan Final_Project_Report.docx` | Full written report |
| `Project_Documentation/Raffay Hassan Project Poster.pdf` | Conference-style poster |
| `Project_Documentation/Raffay_Hassan_FYP_Final4 (2).pptx` | Final presentation slides |

---

## Contact

- Email: raffayhassan772@gmail.com
- University email: rh960@live.mdx.ac.uk
- LinkedIn: [linkedin.com/in/raffay-hassan](https://www.linkedin.com/in/raffay-hassan)
- Blog: [raffayhassan772.wixsite.com/autonomous-systems](https://raffayhassan772.wixsite.com/autonomous-systems)
