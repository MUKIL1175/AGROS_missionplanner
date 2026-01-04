# AGROS Mission Command: The Future of Precision Agriculture

> **Nidar (Visionary Precision)** – Empowering farmers with intelligent drone-based crop management.

AGROS Mission Command is a unified Ground Control Station (GCS) designed specifically for autonomous agricultural spraying missions. By integrating real-time computer vision with ArduPilot-based mission planning, AGROS enables precise, efficient, and effortless target detection and treatment.

---

## Key Features

- **Intelligent Scanning**: Real-time target detection using optimized HSV color-space algorithms.
- **Unified Control**: A single, premium dashboard for scanning, reviewing, and uploading missions.
- **Mission Planner Sync**: One-click launch and link to ArduPilot Mission Planner.
- **Dynamic Parameters**: Configure takeoff altitude, spray height, and loiter time on the fly.
- **Universal Support**: Persistent settings and configurable execution paths for any Windows PC.

---

## The Procedure: How it Works

1. **Scan**: Launch the **Live Vision System** and start the scanning drone. AGROS will automatically identify targets (e.g., weeds or infected plants) and capture their GPS coordinates.
2. **Review**: Go to the **Plan & Generate** tab. Review the captured images, confidence scores, and timestamps. Filter or manually select the targets you want to treat.
3. **Generate**: Click **Generate Mission File** to create a standard `.waypoints` file perfectly compatible with **Mission Planner**.
4. **Link**: Use the **LINK MISSION PLANNER** button to open your control software and establish a MAVLink bridge.
5. **Treat**: Deploy your spray drone by uploading the generated mission directly from the AGROS UI.

---

## Installation & Setup

1. **Clone the Repo**:
   ```bash
   git clone https://github.com/MUKIL1175/AGROS_missionplanner.git
   cd nidar_agri_gcs
   ```
2. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
3. **Launch the System**:
   ```bash
   python main.py
   ```

---

## Mission Planner Integration
In the **Settings** tab, configure your `MissionPlanner.exe` path. Once saved, you can use the integrated launch button to sync your AGROS detections with your primary flight control software instantly.

---

## License
This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## Contact & Support
For queries, custom integrations, or business inquiries, reach out to the developer:

- **Email**: [mukil11ss@gmail.com](mailto:mukil11ss@gmail.com)
- **Subject**: "AGROS GCS Query"

---
*Developed for the future of farming.*
