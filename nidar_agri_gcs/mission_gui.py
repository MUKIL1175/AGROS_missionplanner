import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, font, filedialog
import json
import os
import cv2
import mission_generator
from threading import Thread
import time
from PIL import Image, ImageTk

# Import Scanner
try:
    from scan_receiver import Scanner
    SCANNER_AVAILABLE = True
except ImportError:
    SCANNER_AVAILABLE = False
    print("Warning: scan_receiver.py not found or invalid.")

# Import i18n for multi-language support
try:
    import i18n
    I18N_AVAILABLE = True
except ImportError:
    I18N_AVAILABLE = False
    print("Warning: i18n.py not found. Language support disabled.")

# Config
SCAN_RESULTS_FILE = "scan_results.json"
SETTINGS_FILE = "settings.json"

# Helper function for translation
def _(text):
    """Translation wrapper - returns translated text if i18n is available."""
    if I18N_AVAILABLE:
        return i18n._(text)
    return text

class MissionAGROSGUI:
    def __init__(self, root):
        self.root = root
        self.root.title(_("AGROS Mission Control & Live Scan Station"))
        self.root.geometry("1150x850")
        
        self.raw_detections = []    # Loaded from JSON
        self.selection_state = {}   # ID -> Boolean
        self.geofence = []          # List of (lat, lon)
        self.geofence_breached = False
        self.breach_alert_active = False
        
        # Load Settings
        settings = self.load_settings()
        self.connection_string_var = tk.StringVar(value=settings.get("connection_string", "udp:127.0.0.1:14550"))
        self.mp_path_var = tk.StringVar(value=settings.get("mp_path", self.auto_detect_mp()))
        self.language_var = tk.StringVar(value=settings.get("language", "System"))
        
        # Set language if i18n is available
        if I18N_AVAILABLE:
            i18n.set_language(self.language_var.get())

        # Scanner Instance
        self.scanner = None
        if SCANNER_AVAILABLE:
            self.scanner = Scanner()
            if self.scanner:
                self.scanner.set_connection_string(self.connection_string_var.get())
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')
        
        self.last_frame_id = -1
        
        # Color Presets
        self.presets = {
            "Yellow (Pigments)": (20, 80, 80, 35, 255, 255),
            "Green (Plants)": (35, 40, 40, 85, 255, 255),
            "Red (Generic)": (0, 100, 100, 10, 255, 255), # Red has two ranges, this is one. 
            "Custom": None
        }
        
        # === Header ===
        header_frame = ttk.Frame(root, padding="10")
        header_frame.pack(fill=tk.X)
        ttk.Label(header_frame, text="AGROS Mission Command", font=("Helvetica", 16, "bold")).pack(side=tk.LEFT)
        
        # Connection Status in Header
        status_frame = ttk.Frame(header_frame)
        status_frame.pack(side=tk.RIGHT)
        ttk.Label(status_frame, text=_("MAVLink Status:")).pack(side=tk.LEFT, padx=5)
        self.lbl_mav_status = ttk.Label(status_frame, text=_("CONNECTED") if (self.scanner and self.scanner.mavlink_connected) else _("DISCONNECTED"), 
                                       font=("Arial", 10, "bold"), foreground="red")
        self.lbl_mav_status.pack(side=tk.LEFT, padx=5)

        self.btn_mav_connect = ttk.Button(status_frame, text=_("🔌 CONNECT"), command=self.manual_mavlink_connect, width=12)
        self.btn_mav_connect.pack(side=tk.LEFT, padx=5)

        ttk.Label(status_frame, text=" | GPS:").pack(side=tk.LEFT, padx=(5, 0))
        self.lbl_gps_status = ttk.Label(status_frame, text="OFFLINE", 
                                       font=("Arial", 10, "bold"), foreground="gray")
        self.lbl_gps_status.pack(side=tk.LEFT, padx=5)

        # LINK MISSION PLANNER BUTTON
        link_btn = ttk.Button(header_frame, text="🚀 LINK MISSION PLANNER", command=self.launch_mission_planner)
        link_btn.pack(side=tk.RIGHT, padx=20)
        
        # === Controls Pane (Notebook for Tabbed View) ===
        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # TAB 1: MISSION PLANNING
        self.frame_plan = ttk.Frame(notebook)
        notebook.add(self.frame_plan, text="Plan & Generate")
        self._init_planning_tab(self.frame_plan)
        
        # TAB 2: LIVE SCANNING
        self.frame_scan = ttk.Frame(notebook)
        notebook.add(self.frame_scan, text="Live Vision System")
        self._init_scanning_tab(self.frame_scan)

        # TAB 3: SETTINGS
        self.frame_settings = ttk.Frame(notebook)
        notebook.add(self.frame_settings, text=_("Settings"))
        self._init_settings_tab(self.frame_settings)
        
        self.notebook = notebook 
        
        # === Log Window (Bottom) ===
        log_frame = ttk.LabelFrame(root, text="System Log", padding="5")
        log_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Clear Log Button
        clear_btn = ttk.Button(log_frame, text="Clear Log", command=self.clear_log, width=10)
        clear_btn.pack(side=tk.RIGHT, padx=5, anchor="n")
        
        self.log_area = scrolledtext.ScrolledText(log_frame, height=5, state='disabled')
        self.log_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Start Video Loop
        self.video_update_loop()

        # Initial Load
        self.load_data()

    def clear_log(self):
        self.log_area.config(state='normal')
        self.log_area.delete('1.0', tk.END)
        self.log_area.config(state='disabled')

    def log(self, message):
        self.log_area.config(state='normal')
        self.log_area.insert(tk.END, message + "\n")
        self.log_area.see(tk.END)
        self.log_area.config(state='disabled')
        print(message)

    def manual_mavlink_connect(self):
        if self.scanner:
            conn_str = self.connection_string_var.get()
            self.log(f"[INFO] Manual MAVLink reconnection triggered to {conn_str}...")
            self.scanner.set_connection_string(conn_str)
            self.lbl_mav_status.config(text="CONNECTING...", foreground="orange")
            # Start connection in thread to avoid freezing UI
            Thread(target=self.scanner.connect_mavlink, args=(True,), daemon=True).start()
        else:
            self.log("[ERROR] Scanner not available to connect.")

    def apply_settings(self):
        conn_str = self.connection_string_var.get()
        mp_path = self.mp_path_var.get()
        language = self.language_var.get()
        
        self.log(f"[CONFIG] Updating connection to: {conn_str}")
        self.log(f"[CONFIG] MP Path: {mp_path}")
        self.log(f"[CONFIG] Language: {language}")
        
        if self.scanner:
            self.scanner.set_connection_string(conn_str)
        
        # Update language if i18n is available
        if I18N_AVAILABLE:
            i18n.set_language(language)
        
        self.save_settings()
        messagebox.showinfo("Settings", "Configuration Saved Successfully!\n\nNote: Language changes will take effect after restarting the application.")

    def load_settings(self):
        if os.path.exists(SETTINGS_FILE):
            try:
                with open(SETTINGS_FILE, 'r') as f:
                    return json.load(f)
            except: pass
        return {}

    def save_settings(self):
        settings = {
            "connection_string": self.connection_string_var.get(),
            "mp_path": self.mp_path_var.get(),
            "language": self.language_var.get()
        }
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(settings, f, indent=2)

    def auto_detect_mp(self):
        paths = [
            r"C:\Program Files (x86)\Mission Planner\MissionPlanner.exe",
            r"C:\Program Files\Mission Planner\MissionPlanner.exe",
            os.path.join(os.path.expanduser("~"), "Desktop", "Mission Planner", "MissionPlanner.exe")
        ]
        for p in paths:
            if os.path.exists(p): return p
        return ""

    def browse_mp_path(self):
        path = filedialog.askopenfilename(title="Select MissionPlanner.exe", filetypes=[("Executable", "*.exe")])
        if path:
            self.mp_path_var.set(path)

    def launch_mission_planner(self):
        path = self.mp_path_var.get()
        if not path or not os.path.exists(path):
            messagebox.showerror("Launcher", "Mission Planner Executable not found!\nPlease configure path in Settings.")
            return
        
        try:
            self.log(f"[LAUNCH] Starting Mission Planner: {path}")
            os.startfile(path)
        except Exception as e:
            messagebox.showerror("Launcher", f"Failed to launch MP: {e}")

    # --- SETTINGS TAB ---
    def _init_settings_tab(self, parent):
        # Container
        container = ttk.Frame(parent, padding="10")
        container.pack(fill=tk.BOTH, expand=True)

        # MAVLink Config
        frame = ttk.LabelFrame(container, text="Communication Configuration", padding="10")
        frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(frame, text="MAVLink Connection String:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=5)
        ttk.Entry(frame, textvariable=self.connection_string_var, width=40).grid(row=0, column=1, padx=5, pady=5)
        
        info_lbl = ttk.Label(frame, text="Examples: udp:127.0.0.1:14551, com4", foreground="gray")
        info_lbl.grid(row=1, column=1, sticky="w", padx=5)

        # MP Integration
        mp_frame = ttk.LabelFrame(container, text="Mission Planner Integration (Universal)", padding="10")
        mp_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(mp_frame, text="Mission Planner Path:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=5)
        ttk.Entry(mp_frame, textvariable=self.mp_path_var, width=60).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(mp_frame, text="Browse...", command=self.browse_mp_path).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Label(mp_frame, text="Tip: Configure this once and it will be saved for future sessions.", 
                  foreground="gray").grid(row=1, column=1, sticky="w", padx=5)
        
        # Language Settings
        lang_frame = ttk.LabelFrame(container, text="Language Settings", padding="10")
        lang_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(lang_frame, text="UI Language:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=5)
        
        languages = ["System", "English (en)", "हिन्दी (hi)", "தமிழ் (ta)", "తెలుగు (te)", "മലയാളം (ml)"]
        lang_combo = ttk.Combobox(lang_frame, textvariable=self.language_var, values=languages, state="readonly", width=25)
        lang_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(lang_frame, text="Note: Language change will take effect after restart.", 
                  foreground="gray").grid(row=1, column=1, sticky="w", padx=5)

        # Save Button at Bottom
        save_btn = ttk.Button(container, text="💾 SAVE ALL SETTINGS", style="Accent.TButton", command=self.apply_settings)
        save_btn.pack(pady=20)

    # --- PLANNING TAB LOGIC ---
    def _init_planning_tab(self, parent):
        # Parameters Frame
        param_frame = ttk.LabelFrame(parent, text="Mission Parameters", padding="10")
        param_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(param_frame, text="Takeoff Alt (m):").grid(row=0, column=0, padx=5)
        self.takeoff_alt_var = tk.DoubleVar(value=10.0)
        ttk.Entry(param_frame, textvariable=self.takeoff_alt_var, width=8).grid(row=0, column=1, padx=5)
        
        ttk.Label(param_frame, text="Spray Alt (m):").grid(row=0, column=2, padx=5)
        self.spray_alt_var = tk.DoubleVar(value=3.0)
        ttk.Entry(param_frame, textvariable=self.spray_alt_var, width=8).grid(row=0, column=3, padx=5)
        
        ttk.Label(param_frame, text="Loiter Time (s):").grid(row=0, column=4, padx=5)
        self.loiter_time_var = tk.IntVar(value=6)
        ttk.Entry(param_frame, textvariable=self.loiter_time_var, width=8).grid(row=0, column=5, padx=5)
        
        ttk.Button(param_frame, text="Refresh", command=self.load_data).grid(row=0, column=6, padx=10)
        ttk.Button(param_frame, text="History", command=self.open_history_browser).grid(row=0, column=7, padx=5)
        ttk.Button(param_frame, text="Clear All Detections", command=self.clear_detections).grid(row=0, column=8, padx=5)

        # Split Pane
        content_pane = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        content_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Left: Treeview
        list_frame = ttk.Labelframe(content_pane, text="Detections (Check to include)", padding="5")
        content_pane.add(list_frame, weight=2)
        
        columns = ("select", "id", "type", "conf", "lat", "lon", "time")
        self.tree = ttk.Treeview(list_frame, columns=columns, show="headings", height=15)
        
        self.tree.heading("select", text="[x]")
        self.tree.heading("id", text="ID")
        self.tree.heading("type", text="Type")
        self.tree.heading("conf", text="Conf")
        self.tree.heading("lat", text="Lat")
        self.tree.heading("lon", text="Lon")
        self.tree.heading("time", text="Time")
        
        self.tree.column("select", width=40, anchor="center")
        self.tree.column("id", width=80)
        self.tree.column("type", width=80)
        self.tree.column("conf", width=50)
        self.tree.column("lat", width=80)
        self.tree.column("lon", width=80)
        self.tree.column("time", width=140)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Right: Image Preview
        preview_frame = ttk.Labelframe(content_pane, text="Target Preview", padding="5")
        content_pane.add(preview_frame, weight=1)
        
        self.img_label = ttk.Label(preview_frame, text="Select a row to preview")
        self.img_label.pack(fill=tk.BOTH, expand=True, anchor="center")
        
        # Action Buttons
        action_frame = ttk.Frame(parent, padding="10")
        action_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(action_frame, text="GENERATE MISSION FILE", command=self.generate_file).pack(side=tk.LEFT, padx=5)
        ttk.Label(action_frame, text=" | ").pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="UPLOAD KML (Geofence)", command=self.start_upload_kml_thread).pack(side=tk.LEFT, padx=5)
        ttk.Button(action_frame, text="UPLOAD FROM FILE", command=self.start_upload_file_thread).pack(side=tk.RIGHT, padx=5)

        # Binding
        self.tree.bind('<Button-1>', self.on_click)
        self.tree.bind('<<TreeviewSelect>>', self.on_select)

    # --- SCANNING TAB LOGIC ---
    def _init_scanning_tab(self, parent):
        # Top Controls
        ctrl_frame = ttk.Frame(parent, padding="10")
        ctrl_frame.pack(fill=tk.X)
        
        # Camera Selector
        ttk.Label(ctrl_frame, text="Cam Input:").pack(side=tk.LEFT, padx=5)
        self.cam_combo = ttk.Combobox(ctrl_frame, values=["0", "1", "2", "3", "4"], width=3, state="readonly")
        self.cam_combo.current(0)
        self.cam_combo.pack(side=tk.LEFT, padx=5)
        
        self.btn_start = ttk.Button(ctrl_frame, text="START SCAN", command=self.start_scan)
        self.btn_start.pack(side=tk.LEFT, padx=10)
        
        self.btn_stop = ttk.Button(ctrl_frame, text="STOP", command=self.stop_scan, state='disabled')
        self.btn_stop.pack(side=tk.LEFT, padx=5)
        
        self.lbl_status = ttk.Label(ctrl_frame, text="Status: IDLE", font=("Arial", 10, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=20)
        
        # Split Pane for Video + Settings
        scan_pane = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        scan_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Left: Video Feed
        video_frame = ttk.LabelFrame(scan_pane, text="Drone Feed (Live)", padding="5")
        scan_pane.add(video_frame, weight=3)
        
        self.live_video_label = ttk.Label(video_frame, text="Camera Offline", background="black", foreground="white", anchor="center")
        self.live_video_label.pack(fill=tk.BOTH, expand=True)

        # Right: Calibration Controls
        calib_frame = ttk.LabelFrame(scan_pane, text="HSV Calibration", padding="10")
        scan_pane.add(calib_frame, weight=1)

        # Presets Selection
        ttk.Label(calib_frame, text="Color Preset:", font=("Arial", 9, "bold")).pack(fill=tk.X, pady=(0, 5))
        self.preset_combo = ttk.Combobox(calib_frame, values=list(self.presets.keys()), state="readonly")
        self.preset_combo.current(0) # Default Yellow
        self.preset_combo.pack(fill=tk.X, pady=(0, 10))
        self.preset_combo.bind("<<ComboboxSelected>>", self.apply_preset)
        
        # Sliders
        self.h_min = self._create_slider(calib_frame, "H Min", 0, 179, 20)
        self.s_min = self._create_slider(calib_frame, "S Min", 0, 255, 80)
        self.v_min = self._create_slider(calib_frame, "V Min", 0, 255, 80)
        
        ttk.Separator(calib_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        self.h_max = self._create_slider(calib_frame, "H Max", 0, 179, 35)
        self.s_max = self._create_slider(calib_frame, "S Max", 0, 255, 255)
        self.v_max = self._create_slider(calib_frame, "V Max", 0, 255, 255)

    def apply_preset(self, event=None):
        name = self.preset_combo.get()
        vals = self.presets.get(name)
        if not vals: return

        self.h_min.set(vals[0])
        self.s_min.set(vals[1])
        self.v_min.set(vals[2])
        self.h_max.set(vals[3])
        self.s_max.set(vals[4])
        self.v_max.set(vals[5])
        self.update_hsv()

    def _sync_scanner_label(self):
         if self.scanner:
            preset_name = self.preset_combo.get()
            label = "TARGET"
            if "Yellow" in preset_name: label = "YELLOW"
            elif "Green" in preset_name: label = "GREEN"
            elif "Red" in preset_name: label = "RED"
            self.scanner.current_color_label = label

    def _create_slider(self, parent, label, min_val, max_val, default):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=2)
        ttk.Label(frame, text=label, width=6).pack(side=tk.LEFT)
        scale = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL, command=self.update_hsv)
        scale.set(default)
        scale.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        return scale

    def update_hsv(self, _=None):
        self._sync_scanner_label()
        if self.scanner:
            import numpy as np
            h_min = self.h_min.get()
            s_min = self.s_min.get()
            v_min = self.v_min.get()
            
            h_max = self.h_max.get()
            s_max = self.s_max.get()
            v_max = self.v_max.get()
            
            self.scanner.hsv_min = np.array([h_min, s_min, v_min])
            self.scanner.hsv_max = np.array([h_max, s_max, v_max])

    def start_scan(self):
        if not self.scanner:
            self.log("[ERROR] Scanner module unavailable.")
            return
        
        try:
            full_str = self.cam_combo.get() # "0"
            idx = int(full_str)
        except:
            idx = 0
            
        # Ensure connection string is up to date
        if self.connection_string_var:
             self.scanner.set_connection_string(self.connection_string_var.get())

        self.log(f"[INFO] Starting Scanner on Camera {idx}...")
        self.scanner.start(camera_index=idx)
        
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')
        self.lbl_status.config(text="Status: SCANNING", foreground="green")

    def stop_scan(self):
        if not self.scanner:
            return
            
        self.log("[INFO] Stopping Scanner...")
        self.scanner.stop()
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')
        self.lbl_status.config(text="Status: IDLE", foreground="black")
        self.live_video_label.config(image='')
        self.live_video_label.config(text="Camera Offline")

    def video_update_loop(self):
        # Update MAVLink Status
        if self.scanner:
            if self.scanner.mavlink_connected:
                self.lbl_mav_status.config(text="CONNECTED", foreground="green")
                # Update GPS
                fix = getattr(self.scanner, 'gps_fix', 0)
                sats = getattr(self.scanner, 'sat_count', 0)
                fix_map = {0: "NO FIX", 1: "NO FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK-F", 6: "RTK-X"}
                fix_name = fix_map.get(fix, f"F:{fix}")
                self.lbl_gps_status.config(text=f"{fix_name} ({sats} Sats)", foreground="blue" if fix >= 3 else "orange")
                
                # Geofence Breach Check
                if self.geofence:
                    lat = getattr(self.scanner, 'latitude', 0.0)
                    lon = getattr(self.scanner, 'longitude', 0.0)
                    if lat != 0.0 and lon != 0.0:
                        is_inside = mission_generator.is_point_in_polygon(lat, lon, self.geofence)
                        if not is_inside:
                            if not self.geofence_breached:
                                self.geofence_breached = True
                                self.show_geofence_alert(lat, lon)
                        else:
                            self.geofence_breached = False
            else:
                self.lbl_mav_status.config(text="DISCONNECTED", foreground="red")
                self.lbl_gps_status.config(text="OFFLINE", foreground="gray")

        if self.scanner and self.scanner.running:
             # Sync with background processing
             current_id = getattr(self.scanner, 'frame_id', 0)
             if current_id != self.last_frame_id:
                  frame_rgb = getattr(self.scanner, 'display_frame', None)
                  if frame_rgb is not None:
                       img = Image.fromarray(frame_rgb)
                       photo = ImageTk.PhotoImage(image=img)
                       self.live_video_label.config(image=photo, text="")
                       self.live_video_label.image = photo
                       self.last_frame_id = current_id

        # Periodic refresh loop - 25ms (~40 FPS)
        self.root.after(25, self.video_update_loop)
    def load_data(self):
        # Trigger reconnection if disconnected
        if self.scanner and not self.scanner.mavlink_connected:
             self.log("[INFO] Refresh triggered MAVLink reconnection attempt...")
             Thread(target=self.scanner.connect_mavlink, daemon=True).start()

        # 1. Try Loading proper Scan Results (Method 3 flow)
        data_source = self.raw_detections 
        
        if self.scanner and self.scanner.scan_data:
             data_source = self.scanner.scan_data.get("scan_drone", {}).get("detections", [])
        elif os.path.exists(SCAN_RESULTS_FILE):
             try:
                 with open(SCAN_RESULTS_FILE, 'r') as f:
                    data = json.load(f)
                    data_source = data.get("scan_drone", {}).get("detections", [])
             except:
                 pass
        
        if len(data_source) == len(self.raw_detections) and len(data_source) > 0:
             pass
             
        self.raw_detections = data_source
        self.log(f"[INFO] Loaded {len(self.raw_detections)} detections.")
        self._populate_tree()

    def _populate_tree(self):
        # Populate Tree
        self.tree.delete(*self.tree.get_children())
        self.selection_state = {}
        
        for i, det in enumerate(self.raw_detections):
            conf = det.get('confidence', 0.0)
            timestamp = det.get('timestamp', '?')
            det_type = det.get('type', 'Unknown')
            
            is_selected = (conf >= 0.90)
            self.selection_state[i] = is_selected
            
            check_mark = "✔" if is_selected else "☐"
            self.tree.insert("", tk.END, values=(
                check_mark, det['id'], det_type, f"{conf:.2f}",
                f"{det['latitude']:.6f}", f"{det['longitude']:.6f}",
                timestamp
            ), iid=str(i))

    def open_history_browser(self):
        if not os.path.exists("logs"):
            messagebox.showinfo("History", "No logs found yet.")
            return
            
        # Simple Date Picker Dialog
        history_win = tk.Toplevel(self.root)
        history_win.title("Detection History")
        history_win.geometry("400x500")
        
        ttk.Label(history_win, text="Select a date to view logs:", font=("Arial", 10, "bold")).pack(pady=10)
        
        list_frame = ttk.Frame(history_win, padding=10)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        lb = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, font=("Arial", 10))
        lb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=lb.yview)
        
        # List dates
        dates = sorted([d for d in os.listdir("logs") if os.path.isdir(os.path.join("logs", d))], reverse=True)
        for d in dates:
            lb.insert(tk.END, d)
            
        def on_date_select():
            selection = lb.curselection()
            if not selection: return
            date_str = lb.get(selection[0])
            self._show_day_sessions(date_str)
            history_win.destroy()
            
        ttk.Button(history_win, text="OPEN DATE", command=on_date_select).pack(pady=10)

    def _show_day_sessions(self, date_str):
        day_path = os.path.join("logs", date_str)
        sessions = sorted([s for s in os.listdir(day_path) if os.path.isdir(os.path.join(day_path, s))], reverse=True)
        
        if len(sessions) == 1:
            self._load_archived_session(os.path.join(day_path, sessions[0]))
            return
            
        # Picker for multiple sessions in a day
        session_win = tk.Toplevel(self.root)
        session_win.title(f"Sessions for {date_str}")
        session_win.geometry("300x400")
        
        lb = tk.Listbox(session_win, font=("Arial", 10))
        lb.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        for s in sessions:
            lb.insert(tk.END, s)
            
        def on_session_select():
            selection = lb.curselection()
            if not selection: return
            session_str = lb.get(selection[0])
            self._load_archived_session(os.path.join(day_path, session_str))
            session_win.destroy()
            
        ttk.Button(session_win, text="LOAD SESSION", command=on_session_select).pack(pady=10)

    def _load_archived_session(self, session_path):
        json_path = os.path.join(session_path, "scan_results.json")
        if not os.path.exists(json_path):
            messagebox.showerror("Error", "Session data not found!")
            return
            
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
                self.raw_detections = data.get("scan_drone", {}).get("detections", [])
                self.log(f"[HISTORY] Loaded {len(self.raw_detections)} detections from {session_path}")
                self._populate_tree()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load history: {e}")

    def on_click(self, event):
        region = self.tree.identify("region", event.x, event.y)
        if region == "cell":
            col = self.tree.identify_column(event.x)
            item_id = self.tree.identify_row(event.y)
            if col == "#1" and item_id:
                try:
                    idx = int(item_id)
                    current_state = self.selection_state.get(idx, False)
                    new_state = not current_state
                    self.selection_state[idx] = new_state
                    char = "✔" if new_state else "☐"
                    self.tree.set(item_id, "select", char)
                except ValueError:
                    pass

    def on_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items:
            return
        try:
            item_id = selected_items[0]
            idx = int(item_id)
            if 0 <= idx < len(self.raw_detections):
                det = self.raw_detections[idx]
                if det and det.get('image') and os.path.exists(det['image']):
                    self.show_image(det['image'])
                else:
                    self.img_label.config(image='', text="No Image Available")
        except ValueError:
            pass

    def show_image(self, path):
        try:
            cv_img = cv2.imread(path)
            if cv_img is None: raise Exception("CV2 Load Failed")
            h, w = cv_img.shape[:2]
            scale = min(300/w, 250/h)
            new_w, new_h = int(w*scale), int(h*scale)
            resized = cv2.resize(cv_img, (new_w, new_h))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            _, buffer = cv2.imencode('.ppm', rgb)
            data = buffer.tobytes()
            photo = tk.PhotoImage(data=data)
            self.img_label.config(image=photo, text="")
            self.img_label.image = photo 
        except Exception as e:
            self.img_label.config(text="Image Error")

    def get_selected_targets(self):
        targets = []
        spray_alt = self.spray_alt_var.get()
        loiter_time = int(self.loiter_time_var.get())
        
        for i, det in enumerate(self.raw_detections):
            if self.selection_state.get(i, False):
                t = det.copy()
                t['spray_altitude'] = spray_alt
                t['loiter_time'] = loiter_time
                targets.append(t)
        return targets

    def show_geofence_alert(self, lat, lon):
        if self.breach_alert_active:
            return
            
        self.breach_alert_active = True
        self.log(f"[WARNING] GEOFENCE BREACH DETECTED at {lat:.6f}, {lon:.6f}!")
        
        # Bring window to front
        self.root.lift()
        self.root.attributes('-topmost', True)
        self.root.attributes('-topmost', False)
        
        alert_win = tk.Toplevel(self.root)
        alert_win.title("⚠️ GEOFENCE BREACH ⚠️")
        alert_win.geometry("400x300")
        alert_win.configure(bg="#ffebee") # Light red
        
        ttk.Label(alert_win, text="GEOFENCE BREACH!", font=("Helvetica", 16, "bold"), foreground="red", background="#ffebee").pack(pady=20)
        ttk.Label(alert_win, text=f"Drone is outside the geofence!\nLocation: {lat:.6f}, {lon:.6f}", justify="center", background="#ffebee").pack(pady=10)
        
        btn_frame = ttk.Frame(alert_win, padding=10)
        btn_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        def on_rtl():
            master = self.scanner.master if self.scanner else None
            mission_generator.send_rtl_command(self.connection_string_var.get(), master=master)
            self.breach_alert_active = False
            alert_win.destroy()
            
        def on_land():
            master = self.scanner.master if self.scanner else None
            mission_generator.send_land_command(self.connection_string_var.get(), master=master)
            self.breach_alert_active = False
            alert_win.destroy()
            
        def on_continue():
            self.log("[INFO] User chose to CONTINUE MISSION despite geofence breach.")
            self.breach_alert_active = False
            alert_win.destroy()

        ttk.Button(btn_frame, text="RETURN TO LAUNCH (RTL)", command=on_rtl).pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="LAND IMMEDIATELY", command=on_land).pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="CONTINUE MISSION", command=on_continue).pack(fill=tk.X, pady=5)
        
        # Prevent closing without action
        alert_win.protocol("WM_DELETE_WINDOW", lambda: None)
        alert_win.grab_set() # Modal


    def clear_detections(self):
        if not messagebox.askyesno("Confirm", "Are you sure you want to archive and clear ALL detections?"):
            return
            
        if self.scanner:
            archived_path = self.scanner.archive_session()
            if archived_path:
                self.log(f"[INFO] Detections archived to: {archived_path}")
            else:
                self.log("[WARN] No detections found to archive, but workspace reset.")
        else:
             dummy = {
                "mission_id": "AGRI_MISSION_001",
                "scan_drone": {
                    "vehicle_id": "SCAN_DRONE_01",
                    "mission_time": datetime.utcnow().isoformat() + "Z",
                    "detections": []
                }
            }
             try:
                 with open(SCAN_RESULTS_FILE, 'w') as f:
                     json.dump(dummy, f, indent=2)
             except: pass
        
        self.raw_detections = []
        self.tree.delete(*self.tree.get_children())
        self.selection_state = {}
        self.img_label.config(image='', text="No Data")
        self.log("[INFO] Detections workspace cleared.")

    def generate_file(self):
        targets = self.get_selected_targets()
        
        # Filter by geofence if available
        if self.geofence:
            filtered_targets = []
            for t in targets:
                if mission_generator.is_point_in_polygon(t['latitude'], t['longitude'], self.geofence):
                    filtered_targets.append(t)
            
            diff = len(targets) - len(filtered_targets)
            if diff > 0:
                self.log(f"[INFO] Geofence filtered out {diff} targets.")
            targets = filtered_targets

        if not targets:
            messagebox.showwarning("Warning", "No targets selected or all filtered by geofence!")
            return
        
        # Show Mission Configuration Dialog
        config_dialog = tk.Toplevel(self.root)
        config_dialog.title("Mission Configuration")
        config_dialog.geometry("400x250")
        config_dialog.resizable(False, False)
        config_dialog.configure(bg="#f0f0f0")
        
        # Center the dialog
        config_dialog.transient(self.root)
        config_dialog.grab_set()
        
        # Variables to store user choices
        auto_takeoff_var = tk.BooleanVar(value=True)
        completion_action_var = tk.StringVar(value="RTL")
        user_confirmed = tk.BooleanVar(value=False)
        
        # Title
        title_label = ttk.Label(config_dialog, text="Configure Mission Parameters", 
                                font=("Arial", 12, "bold"), background="#f0f0f0")
        title_label.pack(pady=15)
        
        # Auto Takeoff Frame
        takeoff_frame = ttk.LabelFrame(config_dialog, text="Takeoff Configuration", padding="10")
        takeoff_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Checkbutton(takeoff_frame, text="Auto Takeoff (Include takeoff command)", 
                       variable=auto_takeoff_var).pack(anchor="w")
        
        # Completion Action Frame
        completion_frame = ttk.LabelFrame(config_dialog, text="Mission Completion Action", padding="10")
        completion_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Radiobutton(completion_frame, text="Return to Launch (RTL)", 
                       variable=completion_action_var, value="RTL").pack(anchor="w", pady=2)
        ttk.Radiobutton(completion_frame, text="Land at Final Position", 
                       variable=completion_action_var, value="LAND").pack(anchor="w", pady=2)
        
        # Buttons
        btn_frame = ttk.Frame(config_dialog)
        btn_frame.pack(fill=tk.X, padx=20, pady=15)
        
        def on_confirm():
            user_confirmed.set(True)
            config_dialog.destroy()
        
        def on_cancel():
            user_confirmed.set(False)
            config_dialog.destroy()
        
        ttk.Button(btn_frame, text="Generate Mission", command=on_confirm).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=5)
        
        # Wait for user to close dialog
        self.root.wait_window(config_dialog)
        
        # If user cancelled, return
        if not user_confirmed.get():
            self.log("[INFO] Mission generation cancelled by user.")
            return
        
        # Get user choices
        auto_takeoff = auto_takeoff_var.get()
        completion_action = completion_action_var.get()
        
        self.log(f"[INFO] Mission Config - Auto Takeoff: {auto_takeoff}, Completion: {completion_action}")
            
        # 1. Ask User for Path
        initial_file = "mission.waypoints"
        path = filedialog.asksaveasfilename(
            defaultextension=".waypoints",
            initialfile=initial_file,
            filetypes=[("Waypoints", "*.waypoints"), ("Mission Plan", "*.plan"), ("All Files", "*.*")],
            title="Save Mission File"
        )
        
        if not path:
            return 
            
        try:
            # Generate waypoints with user configuration
            content = mission_generator.generate_waypoints_content(
                targets, 
                auto_takeoff=auto_takeoff, 
                completion_action=completion_action
            )
            
            with open(path, 'w') as f:
                f.write(content)
                
            self.log(f"[SUCCESS] Saved mission to {path}")
            messagebox.showinfo("Success", f"Saved mission to:\n{path}\n\nYou can now upload this file using 'Upload From File'.")
            
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def start_upload_kml_thread(self):
        path = filedialog.askopenfilename(
            title="Select KML Geofence File",
            filetypes=[("KML Files", "*.kml"), ("All Files", "*.*")]
        )
        if not path:
            return
        t = Thread(target=self.upload_kml_logic, args=(path,))
        t.start()
        
    def upload_kml_logic(self, path):
        try:
            self.log(f"[INFO] Parsing KML: {path}...")
            poly = mission_generator.parse_kml_polygon(path)
            if not poly:
                raise Exception("No polygon/coordinates found in KML or invalid format.")
            
            self.geofence = poly
            self.log(f"[SUCCESS] Loaded geofence with {len(poly)} points.")
            
            # Upload to MAVLink Fence
            try:
                master = self.scanner.master if self.scanner else None
                mission_generator.upload_fence_mavlink(self.geofence, self.connection_string_var.get(), master=master)
                self.log(f"[INFO] MAVLink Fence uploaded ({len(self.geofence)} points).")
            except Exception as e:
                self.log(f"[WARN] Failed to upload MAVLink fence: {e}")

            messagebox.showinfo("KML Upload", f"Loaded geofence with {len(poly)} points.\nMAVLink Fence enabled and visibility should be updated in Mission Planner.\nMission will be filtered by this area.")
        except Exception as e:
            self.log(f"[ERROR] KML Load Failed: {e}")
            messagebox.showerror("KML Error", str(e))

    def start_upload_file_thread(self):
        path = filedialog.askopenfilename(
            title="Select Mission File to Upload",
            filetypes=[("Waypoints", "*.waypoints"), ("Mission Plan", "*.plan"), ("JSON", "*.json"), ("All Files", "*.*")]
        )
        if not path:
            return
            
        t = Thread(target=self.upload_file_logic, args=(path,))
        t.start()

    def upload_file_logic(self, path):
        try:
             conn_str = self.connection_string_var.get()
             self.log(f"[INFO] Uploading file: {path} to {conn_str}...")
             master = self.scanner.master if self.scanner else None
             mission_generator.upload_mission_from_file(path, connection_string=conn_str, master=master)
             messagebox.showinfo("Upload", "File Upload Successful!")
             self.log("[SUCCESS] File Upload Complete.")
        except Exception as e:
             self.log(f"[ERROR] File Upload Failed: {e}")
             messagebox.showerror("Upload Error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = MissionAGROSGUI(root)
    root.mainloop()
