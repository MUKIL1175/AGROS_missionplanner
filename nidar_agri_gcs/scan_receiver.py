import cv2
import numpy as np
import time
import json
import os
import threading
from datetime import datetime
from pymavlink import mavutil

DETECTION_DIR = "detections"
SCAN_RESULTS_FILE = "scan_results.json"
os.makedirs(DETECTION_DIR, exist_ok=True)

class Scanner:
    def __init__(self, connection_string='udp:127.0.0.1:14550', baudrate=115200):
        self.running = False
        self.cap = None
        self.master = None
        self.current_frame = None
        self.display_frame = None
        self.frame_id = 0
        self.detection_id = 1
        self.enable_osd = True  # OSD Toggle
        
        self.connection_string = connection_string
        self.baudrate = baudrate

        
        self.scan_data = {
            "mission_id": "AGRI_MISSION_001",
            "scan_drone": {
                "vehicle_id": "SCAN_DRONE_01",
                "mission_time": datetime.utcnow().isoformat() + "Z",
                "detections": []
            }
        }
        self.scan_data_lock = threading.Lock()
        self.mavlink_connected = False
        self.last_heartbeat = 0
        
        # Initial GPS state to prevent AttributeError before first update
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_rel = 0.0
        self.gps_fix = 0      # 0-1: No Fix, 2: 2D, 3: 3D, etc.
        self.sat_count = 0
        
        # Battery Status
        self.battery_voltage = 0.0
        self.battery_remaining = 0
        
        # HSV Thresholds (Default Yellow)
        self.hsv_min = np.array([20, 80, 80])
        self.hsv_max = np.array([35, 255, 255])
        
        # Load existing data and check for date reset
        self._load_and_check_reset()

    def _load_and_check_reset(self):
        if os.path.exists(SCAN_RESULTS_FILE):
             try:
                with open(SCAN_RESULTS_FILE, 'r') as f:
                    data = json.load(f)
                    
                    # Check if session is from a previous day
                    mission_time_str = data.get("scan_drone", {}).get("mission_time", "")
                    if mission_time_str:
                        session_date = mission_time_str.split('T')[0]
                        current_date = datetime.utcnow().strftime("%Y-%m-%d")
                        
                        if session_date != current_date:
                            print(f"[INFO] New day detected ({current_date}). Archiving session from {session_date}...")
                            self.archive_session(data)
                            # After archiving old data, save the fresh scan_data initialized in __init__
                            with open(SCAN_RESULTS_FILE, "w") as f:
                                json.dump(self.scan_data, f, indent=2)
                            return
                    
                    self.scan_data = data
                    # Update ID to max + 1
                    dets = data.get("scan_drone", {}).get("detections", [])
                    if dets:
                        try:
                           ids = [int(d['id'].split('_')[1]) for d in dets if 'PLANT_' in d['id']]
                           if ids:
                               self.detection_id = max(ids) + 1
                        except:
                           pass
             except Exception as e:
                 print(f"[ERROR] Failed to load existing scan data: {e}")

    def archive_session(self, data_to_archive=None):
        """
        Deep archive: Move JSON and images to logs/YYYY-MM-DD/site_HHMMSS/
        Uses CURRENT UTC time for folder naming.
        """
        with self.scan_data_lock:
            target_data = data_to_archive if data_to_archive else self.scan_data
            
            if not target_data["scan_drone"]["detections"]:
                return None

            try:
                # 1. Prepare Paths based on CURRENT time
                now = datetime.utcnow()
                date_str = now.strftime("%Y-%m-%d")
                time_str = now.strftime("%H%M%S")
                
                log_dir = os.path.join("logs", date_str, time_str)
                os.makedirs(log_dir, exist_ok=True)
                
                # 2. Copy Images to archive
                log_det_dir = os.path.join(log_dir, "detections")
                os.makedirs(log_det_dir, exist_ok=True)
                
                import shutil
                new_detections = []
                for det in target_data["scan_drone"]["detections"]:
                    old_path = det.get("image", "")
                    if old_path and os.path.exists(old_path):
                        new_path = os.path.join(log_det_dir, os.path.basename(old_path))
                        shutil.copy2(old_path, new_path)
                        det["image"] = new_path 
                    new_detections.append(det)
                
                target_data["scan_drone"]["detections"] = new_detections

                # 3. Save Archived JSON
                archive_file = os.path.join(log_dir, "scan_results.json")
                with open(archive_file, "w") as f:
                    json.dump(target_data, f, indent=2)
                
                # 4. Cleanup and Reset if it was the current session
                if not data_to_archive:
                    # Remove images from current detections dir
                    if os.path.exists(DETECTION_DIR):
                        for img_file in os.listdir(DETECTION_DIR):
                            try:
                                os.remove(os.path.join(DETECTION_DIR, img_file))
                            except: pass
                    
                    # Reset Data with FRESH mission_time
                    self.detection_id = 1
                    self.scan_data = {
                        "mission_id": "AGRI_MISSION_001",
                        "scan_drone": {
                            "vehicle_id": "SCAN_DRONE_01",
                            "mission_time": datetime.utcnow().isoformat() + "Z",
                            "detections": []
                        }
                    }
                    with open(SCAN_RESULTS_FILE, "w") as f:
                        json.dump(self.scan_data, f, indent=2)
                
                print(f"[INFO] Data archived to {log_dir}")
                return log_dir
            except Exception as e:
                print(f"[ERROR] Archiving failed: {e}")
                return None

    def clear_data(self):
        self.archive_session()
        print("[INFO] Scanner data cleared and archived.")

    def set_connection_string(self, conn_str, baudrate=115200):
        self.connection_string = conn_str
        self.baudrate = baudrate

        # Logic to reconnect could go here if we want dynamic switching

    def connect_mavlink(self, force=False):
        if (self.mavlink_connected and not force) or getattr(self, 'connecting', False):
            return
            
        self.connecting = True
        try:
            # Force MAVLink 2.0
            os.environ['MAVLINK20'] = '1'
            
            print(f"[INFO] Scanner connecting to MAVLink at {self.connection_string} (Baud: {self.baudrate})...")
            # Close existing if any
            if self.master:
                try: self.master.close()
                except: pass
                
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            # Wait for heartbeat with more feedback
            self.master.wait_heartbeat(timeout=10) 
            print("[INFO] Scanner MAVLink Heartbeat Received")
            self.mavlink_connected = True
            self.last_heartbeat = time.time()
            
            # Start GPS thread if not alive or restarted
            if not hasattr(self, 'gps_thread') or self.gps_thread is None or not self.gps_thread.is_alive():
                def _gps_update_loop():
                    print("[INFO] Scanner GPS/Telemetry Loop Started")
                    while True:
                        if not self.master: break
                        try:
                            # Use non-blocking to keep loop alive for heartbeat checks
                            msg = self.master.recv_match(type=['GLOBAL_POSITION_INT', 'HEARTBEAT', 'GPS_RAW_INT', 'SYS_STATUS'], blocking=True, timeout=0.1)
                            if msg:
                                if msg.get_type() == 'HEARTBEAT':
                                    self.last_heartbeat = time.time()
                                    self.mavlink_connected = True
                                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                                    self.latitude = msg.lat / 1e7
                                    self.longitude = msg.lon / 1e7
                                    self.altitude_rel = msg.relative_alt / 1000.0
                                    # Positional data also counts as heartbeat of sorts
                                    self.last_heartbeat = time.time()
                                    self.mavlink_connected = True
                                elif msg.get_type() == 'GPS_RAW_INT':
                                    self.gps_fix = msg.fix_type
                                    self.sat_count = msg.satellites_visible
                                elif msg.get_type() == 'SYS_STATUS':
                                    self.battery_voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else 0.0
                                    self.battery_remaining = msg.battery_remaining if msg.battery_remaining != -1 else 0
                            
                            # Check connection status
                            if time.time() - self.last_heartbeat > 7.0:
                                if self.mavlink_connected:
                                    print("[WARN] MAVLink Heartbeat Lost")
                                    self.mavlink_connected = False
                        except Exception as e:
                            print(f"[DEBUG] Telemetry Loop Exception: {e}")
                            self.mavlink_connected = False
                        time.sleep(0.01)
                
                self.gps_thread = threading.Thread(target=_gps_update_loop, daemon=True)
                self.gps_thread.start()

        except Exception as e:
            print(f"[WARN] Scanner MAVLink connection failed or timed out: {e}")
            self.mavlink_connected = False
        finally:
            self.connecting = False

    def get_gps(self):
        if not self.mavlink_connected or not self.master:
            return None

        return {
            "lat": self.latitude,
            "lon": self.longitude,
            "alt": self.altitude_rel
        }

    def detect_color(self, frame, hsv_min, hsv_max):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        
        yellow_pixels = cv2.countNonZero(mask)
        total_pixels = frame.shape[0] * frame.shape[1]
        confidence = yellow_pixels / total_pixels
        detected = confidence > 0.01
        return detected, confidence, mask

    def start(self, camera_index=None):
        if self.running:
            return
            
        if camera_index is not None:
            self.camera_index = camera_index
            
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
        # Connect mavlink in bg
        mav_thread = threading.Thread(target=self.connect_mavlink, daemon=True)
        mav_thread.start()

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()
            self.cap = None

    def _run_loop(self):
        # Allow explicit index or default
        idx = getattr(self, 'camera_index', 0)
        print(f"[INFO] Opening Camera {idx}...")
        
        self.cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera {idx}. Trying fallback search...")
            self.cap.release()
            
            found = False
            for i in range(5):
                 if i == idx: continue
                 temp = cv2.VideoCapture(i, cv2.CAP_DSHOW)
                 if temp.isOpened():
                     self.cap = temp
                     print(f"[INFO] Found alternative camera at {i}")
                     found = True
                     break
            
            if not found:
                print("[ERROR] No camera found.")
                self.running = False
                return

        print("[INFO] Scanner Loop Started")
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            # Use current color thresholds
            detected, confidence, mask = self.detect_color(frame, self.hsv_min, self.hsv_max)
            
            # Determine Color Label based on thresholds (simple heuristic or passed from GUI)
            # For now, let's assume the GUI sets a 'current_label' or we infer it.
            # To make it robust, we'll let the GUI pass the label if it wants.
            color_label = getattr(self, 'current_color_label', "TARGET")

            if detected:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(c)
                    
                    # Draw Color-coded boxes?
                    box_color = (0, 255, 0) # Green for all by default
                    if "YELLOW" in color_label.upper(): box_color = (0, 255, 255)
                    elif "RED" in color_label.upper(): box_color = (0, 0, 255)
                    elif "GREEN" in color_label.upper(): box_color = (0, 255, 0)

                    if self.enable_osd:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                        label_str = f"{color_label}: {self.detection_id:03d}"
                        cv2.putText(frame, label_str, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
            
            try:
                h, w = frame.shape[:2]
                disp_w, disp_h = 640, 480
                scale = min(disp_w/w, disp_h/h)
                new_w, new_h = int(w*scale), int(h*scale)
                resized = cv2.resize(frame, (new_w, new_h))
                self.display_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                self.frame_id += 1
            except Exception as e:
                print(f"[WARN] Frame conversion failed: {e}")

            self.current_frame = frame 
            
            if detected:
                gps = self.get_gps()
                if not gps:
                    gps = {"lat": 0.0, "lon": 0.0, "alt": 0.0}

                det_id = f"{color_label}_{self.detection_id:03d}"
                img_path = os.path.join(DETECTION_DIR, f"{det_id}.jpg")
                
                if hasattr(self, 'last_save_time') and (time.time() - self.last_save_time < 2.0):
                    pass 
                else:
                    cv2.imwrite(img_path, frame)
                    
                    detection_entry = {
                        "id": det_id,
                        "type": color_label, # New field
                        "latitude": gps["lat"],
                        "longitude": gps["lon"],
                        "altitude_rel": gps["alt"],
                        "confidence": round(confidence, 3),
                        "timestamp": datetime.utcnow().isoformat() + "Z",
                        "image": img_path
                    }
                    
                    with self.scan_data_lock:
                        self.scan_data["scan_drone"]["detections"].append(detection_entry)
                        with open(SCAN_RESULTS_FILE, "w") as f:
                            json.dump(self.scan_data, f, indent=2)

                    print(f"[SCANNER] Detected {det_id}")
                    self.detection_id += 1
                    self.last_save_time = time.time()
            
            time.sleep(0.03) 

# Scanner module for AGROS Ground Control System.
# Import and use Scanner class.
