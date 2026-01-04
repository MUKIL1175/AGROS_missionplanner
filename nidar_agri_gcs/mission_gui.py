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

# Import Map View
try:
    import tkintermapview
    MAP_AVAILABLE = True
except ImportError:
    MAP_AVAILABLE = False
    print("Warning: tkintermapview not found. Map features disabled.")



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
        self.fence_alt_max_var = tk.DoubleVar(value=settings.get("fence_alt_max", 80.0))
        self.fence_margin_var = tk.DoubleVar(value=settings.get("fence_margin", 2.0))
        self.fence_type_var = tk.StringVar(value=settings.get("fence_type", "Polygon + Alt (7)"))
        self.fence_action_var = tk.StringVar(value=settings.get("fence_action", "RTL (1)"))

        # Connection Variables (UDP Only)
        self.udp_host_var = tk.StringVar(value=settings.get("udp_host", "127.0.0.1"))
        self.udp_port_var = tk.StringVar(value=settings.get("udp_port", "14550"))





        
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

        ttk.Label(status_frame, text=" | GPS:").pack(side=tk.LEFT, padx=(5, 0))
        self.lbl_gps_status = ttk.Label(status_frame, text="OFFLINE", 
                                       font=("Arial", 10, "bold"), foreground="gray")
        self.lbl_gps_status.pack(side=tk.LEFT, padx=5)

        
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

    def _get_connection_params(self):
        """Helper to build UDP connection string from current UI state."""
        conn_str = f"udp:{self.udp_host_var.get()}:{self.udp_port_var.get()}"
        return conn_str, 115200 # Baud not used for UDP but kept for API compatibility


    def manual_mavlink_connect(self):
        if self.scanner:
            conn_str, baud = self._get_connection_params()
            # Update the stored variable as well
            self.connection_string_var.set(conn_str)
            
            self.log(f"[INFO] Manual MAVLink reconnection triggered to {conn_str} at {baud}...")
            self.scanner.set_connection_string(conn_str, baudrate=baud)
            self.lbl_mav_status.config(text="CONNECTING...", foreground="orange")
            # Start connection in thread to avoid freezing UI
            Thread(target=self.scanner.connect_mavlink, args=(True,), daemon=True).start()
        else:
            self.log("[ERROR] Scanner not available to connect.")


    def apply_settings(self):
        conn_str, baud = self._get_connection_params()
        self.connection_string_var.set(conn_str)
        
        mp_path = self.mp_path_var.get()
        language = self.language_var.get()
        
        self.log(f"[CONFIG] UDP Sync: {conn_str}")
        self.log(f"[CONFIG] MP Path: {mp_path}")
        self.log(f"[CONFIG] Language: {language}")
        
        if self.scanner:
            self.scanner.set_connection_string(conn_str, baudrate=baud)



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
            "language": self.language_var.get(),
            "mp_path": self.mp_path_var.get(),
            "fence_alt_max": self.fence_alt_max_var.get(),
            "fence_margin": self.fence_margin_var.get(),
            "fence_type": self.fence_type_var.get(),
            "fence_action": self.fence_action_var.get(),
            "udp_host": self.udp_host_var.get(),
            "udp_port": self.udp_port_var.get()
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

        # MAVLink Config (UDP ONLY)
        frame = ttk.LabelFrame(container, text="MAVLink Communication (UDP Network)", padding="15")
        frame.pack(fill=tk.X, pady=5)
        
        row1 = ttk.Frame(frame)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="UDP Host:", font=("Arial", 10, "bold"), width=15).pack(side=tk.LEFT)
        ttk.Entry(row1, textvariable=self.udp_host_var, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Label(row1, text="(e.g. 127.0.0.1)", foreground="gray").pack(side=tk.LEFT)

        row2 = ttk.Frame(frame)
        row2.pack(fill=tk.X, pady=2)
        ttk.Label(row2, text="UDP Port:", font=("Arial", 10, "bold"), width=15).pack(side=tk.LEFT)
        ttk.Entry(row2, textvariable=self.udp_port_var, width=25).pack(side=tk.LEFT, padx=5)
        ttk.Label(row2, text="(typically 14550 or 14551)", foreground="gray").pack(side=tk.LEFT)

        ttk.Label(frame, text="Note: Serial functionality has been disabled as per requirements.", 
                  foreground="#cc0000", font=("Arial", 8, "italic")).pack(anchor="w", pady=(10, 0))

        # Relocated Action Buttons
        btn_row = ttk.Frame(frame)
        btn_row.pack(pady=10, fill=tk.X)
        self.btn_mav_connect = ttk.Button(btn_row, text=_("🔌 CONNECT MAVLINK"), command=self.manual_mavlink_connect, padding=5)
        self.btn_mav_connect.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))
        ttk.Button(btn_row, text="🚀 LINK MISSION PLANNER", command=self.launch_mission_planner, padding=5).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5, 0))


        # MP Integration
        mp_frame = ttk.LabelFrame(container, text="Mission Planner Integration (Universal)", padding="10")
        mp_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(mp_frame, text="Mission Planner Path:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=5)
        ttk.Entry(mp_frame, textvariable=self.mp_path_var, width=60).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(mp_frame, text="Browse...", command=self.browse_mp_path).grid(row=0, column=2, padx=5, pady=5)


        ttk.Label(mp_frame, text="Tip: Configure this once and it will be saved for future sessions.", 
                  foreground="gray").grid(row=2, column=1, sticky="w", padx=5)

        
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
        # Data Management Frame
        ctrl_frame = ttk.LabelFrame(parent, text="Data Management", padding="10")
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(ctrl_frame, text="Refresh List", command=self.load_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="View History", command=self.open_history_browser).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_frame, text="Clear All Detections", command=self.clear_detections).pack(side=tk.LEFT, padx=5)

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




        # Binding
        self.tree.bind('<Button-1>', self.on_click)
        self.tree.bind('<<TreeviewSelect>>', self.on_select)

    # --- SCANNING TAB LOGIC ---
    def _init_scanning_tab(self, parent):
        ctrl_frame = ttk.Frame(parent, padding="10")
        ctrl_frame.pack(fill=tk.X)
        
        # GPS & Battery Status (Right Side)
        status_frame = ttk.Frame(ctrl_frame)
        status_frame.pack(side=tk.RIGHT, padx=5)
        
        self.lbl_mav_status = ttk.Label(status_frame, text="MAV: WAIT", font=("Arial", 9, "bold"), foreground="gray")
        self.lbl_mav_status.pack(side=tk.LEFT, padx=5)
        
        self.lbl_gps_status = ttk.Label(status_frame, text="GPS: Wait", font=("Arial", 9), foreground="gray")
        self.lbl_gps_status.pack(side=tk.LEFT, padx=5)
        
        self.lbl_bat_status = ttk.Label(status_frame, text="Bat: N/A", font=("Arial", 9, "bold"), foreground="gray")
        self.lbl_bat_status.pack(side=tk.LEFT, padx=5)
        
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
        
        # OSD Toggle
        self.osd_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl_frame, text="Show OSD", variable=self.osd_var, command=self.toggle_osd).pack(side=tk.LEFT, padx=5)
        
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

    def toggle_osd(self):
        if self.scanner:
            self.scanner.enable_osd = self.osd_var.get()

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
                
                # Update Battery
                volts = getattr(self.scanner, 'battery_voltage', 0.0)
                pct = getattr(self.scanner, 'battery_remaining', 0)
                
                # Determine color based on pct
                bat_color = "green"
                if pct < 20: bat_color = "red"
                elif pct < 40: bat_color = "orange"
                
                self.lbl_bat_status.config(text=f"Bat: {volts:.1f}V ({pct}%)", foreground=bat_color)
                
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
                self.lbl_bat_status.config(text="Bat: N/A", foreground="gray")

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
        # Altitudes/Timings specific to targets are now handled globally in the Generator Config
        # But if we wanted per-target overrides, we'd keep them here.
        # For now, we strip them out or just return the raw dict copy.
        
        for i, det in enumerate(self.raw_detections):
            if self.selection_state.get(i, False):
                targets.append(det.copy())
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
        
        # Filter by geofence logic removed as per user request
        # Geofence is only for MAVLink upload now.


        if not targets:
            messagebox.showwarning("Warning", "No targets selected or all filtered by geofence!")
            return
        
        # Show Mission Configuration Dialog
        config_dialog = tk.Toplevel(self.root)
        config_dialog.title("Mission Configuration")
        config_dialog.geometry("450x450")
        config_dialog.resizable(True, True)
        config_dialog.configure(bg="#f0f0f0")
        
        # Center the dialog
        config_dialog.transient(self.root)
        config_dialog.grab_set()
        
        # Variables (Defaults)
        auto_takeoff_var = tk.BooleanVar(value=True)
        completion_action_var = tk.StringVar(value="RTL")
        
        travel_alt_var = tk.DoubleVar(value=10.0)
        spray_alt_var = tk.DoubleVar(value=3.0)
        loiter_time_var = tk.DoubleVar(value=6.0)
        
        user_confirmed = tk.BooleanVar(value=False)
        
        # Title
        title_label = ttk.Label(config_dialog, text="Configure Mission Parameters", 
                                font=("Arial", 12, "bold"), background="#f0f0f0")
        title_label.pack(pady=10)
        
        # 1. Altitude & Timing Params
        param_frame = ttk.LabelFrame(config_dialog, text="Flight Parameters", padding="10")
        param_frame.pack(fill=tk.X, padx=20, pady=5)
        
        ttk.Label(param_frame, text="Point-to-Point Altitude (m):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=travel_alt_var, width=10).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(param_frame, text="Waypoint Loiter Altitude (m):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=spray_alt_var, width=10).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(param_frame, text="Spray/Loiter Time (s):").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=loiter_time_var, width=10).grid(row=2, column=1, padx=5, pady=5)

        # 2. Auto Takeoff
        takeoff_frame = ttk.LabelFrame(config_dialog, text="Takeoff Configuration", padding="10")
        takeoff_frame.pack(fill=tk.X, padx=20, pady=5)
        
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
        travel_alt = travel_alt_var.get()
        spray_alt = spray_alt_var.get()
        loiter_time = loiter_time_var.get()
        
        self.log(f"[INFO] Config - Travel Alt: {travel_alt}, Spray Alt: {spray_alt}, Time: {loiter_time}")
            
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
                completion_action=completion_action,
                travel_alt=travel_alt,
                spray_alt=spray_alt,
                loiter_time=loiter_time
            )
            
            with open(path, 'w') as f:
                f.write(content)
                
            self.log(f"[SUCCESS] Saved mission to {path}")
            messagebox.showinfo("Success", f"Saved mission to:\n{path}\n\nYou can now upload this file using 'Upload From File'.")
            
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def start_upload_kml_thread(self):
        """Step 1: Load, Simplify, and Show Preview."""
        path = filedialog.askopenfilename(
            title="Select KML Geofence File",
            filetypes=[("KML Files", "*.kml"), ("All Files", "*.*")]
        )
        if not path:
            return
            
        try:
            self.log(f"[INFO] Parsing KML: {path}...")
            poly = mission_generator.parse_kml_polygon(path)
            if not poly:
                raise Exception("No polygon/coordinates found in KML or invalid format.")
            
            # Professional Setup: Simplify and Validate
            original_count = len(poly)
            if original_count > 100:
                self.log(f"[INFO] Simplifying complex polygon ({original_count} points)...")
                poly = mission_generator.simplify_polygon(poly, epsilon=0.00005) # ~5m tolerance
                self.log(f"[INFO] Simplified to {len(poly)} points.")

            ok, msg = mission_generator.validate_fence(poly)
            if not ok:
                raise Exception(f"Fence Validation Failed: {msg}")
            
            self.geofence = poly
            self.log(f"[SUCCESS] Loaded geofence with {len(poly)} points. Opening preview.")
            
            # Show preview popup (Manual Push)
            self.show_geofence_popup(poly, path)
            
        except Exception as e:
            self.log(f"[ERROR] Geofence Load Failed: {e}")
            messagebox.showerror("Geofence Error", str(e))


    def show_geofence_popup(self, polygon, path):
        """Shows a popup with preview and adjustable fence configurations."""
        win = tk.Toplevel(self.root)
        win.title("Geofence Preview & Config")
        win.geometry("1100x800")
        win.grab_set() 
        
        # Main Layout: Left (Map) and Right (Config)
        main_pane = ttk.PanedWindow(win, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # --- LEFT: MAP PREVIEW ---
        map_container = ttk.LabelFrame(main_pane, text=f"Map Preview: {os.path.basename(path)}", padding="5")
        main_pane.add(map_container, weight=3)
        
        if MAP_AVAILABLE:
            map_widget = tkintermapview.TkinterMapView(map_container, corner_radius=0)
            map_widget.pack(fill=tk.BOTH, expand=True)
            map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
            
            # Center on polygon
            lats = [p[0] for p in polygon]
            lons = [p[1] for p in polygon]
            center_lat, center_lon = sum(lats)/len(lats), sum(lons)/len(lons)
            map_widget.set_position(center_lat, center_lon)
            map_widget.set_zoom(17)
            map_widget.set_polygon(polygon, outline_color="red", border_width=2, fill_color=None)
            for i, p in enumerate(polygon):
                map_widget.set_marker(p[0], p[1], text=f"P{i+1}")
        else:
            ttk.Label(map_container, text="Map View Unavailable").pack(expand=True)

        # --- RIGHT: CONFIG PANEL ---
        config_frame = ttk.Frame(main_pane, padding="10")
        main_pane.add(config_frame, weight=1)

        ttk.Label(config_frame, text="Sync Parameters", font=("Arial", 11, "bold")).pack(anchor="w", pady=(0, 10))

        # Re-use class variables but place in this UI
        box = ttk.LabelFrame(config_frame, text="Hard Geofence Settings", padding="10")
        box.pack(fill=tk.X, pady=5)

        ttk.Label(box, text="Max Altitude (m):", font=("Arial", 9, "bold")).pack(anchor="w", pady=(5,0))
        ttk.Entry(box, textvariable=self.fence_alt_max_var, width=15).pack(anchor="w", pady=2)

        ttk.Label(box, text="Fence Margin (m):", font=("Arial", 9, "bold")).pack(anchor="w", pady=(5,0))
        ttk.Entry(box, textvariable=self.fence_margin_var, width=15).pack(anchor="w", pady=2)

        ttk.Label(box, text="Fence Type:", font=("Arial", 9, "bold")).pack(anchor="w", pady=(5,0))
        type_opts = ["Polygon (4)", "Altitude + Polygon (5)", "Circle + Polygon (6)", "Polygon + Alt (7)", "All (7)"]
        ttk.Combobox(box, textvariable=self.fence_type_var, values=type_opts, state="readonly", width=25).pack(anchor="w", pady=2)

        ttk.Label(box, text="Fence Action:", font=("Arial", 9, "bold")).pack(anchor="w", pady=(5,0))
        act_opts = ["RTL (1)", "Hold (2)", "Land (5)", "Brake (4)", "SmartRTL (3)", "None (0)"]
        ttk.Combobox(box, textvariable=self.fence_action_var, values=act_opts, state="readonly", width=25).pack(anchor="w", pady=2)

        ttk.Label(config_frame, text="Status:", font=("Arial", 10, "bold")).pack(anchor="w", pady=(20, 5))
        status_lbl = ttk.Label(config_frame, text="Ready to Sync", foreground="blue", font=("Arial", 10, "italic"))
        status_lbl.pack(anchor="w", pady=5)

        def on_push():
            try:
                import re
                def get_int_val(s):
                    match = re.search(r'\((\d+)\)', s)
                    return int(match.group(1)) if match else 1

                # Connection Checks
                master = self.scanner.master if self.scanner else None
                if not master or not getattr(master, 'target_system', 0):
                     status_lbl.config(text="❌ FAILED: Drone Not Connected", foreground="red")
                     self.log("[ERROR] Geofence Push Failed: No Active MAVLink Connection.")
                     return

                # Use centralized params
                conn_str, baud = self._get_connection_params()

                max_alt = self.fence_alt_max_var.get()
                margin = self.fence_margin_var.get()
                f_type = get_int_val(self.fence_type_var.get())
                f_action = get_int_val(self.fence_action_var.get())
                
                status_lbl.config(text="⏱ Syncing...", foreground="orange")
                win.update_idletasks()

                self.log(f"[INFO] Pushing geofence to {conn_str}...")
                mission_generator.upload_fence_mavlink(
                    polygon, conn_str, baudrate=baud, master=master, 
                    fence_alt_max=max_alt, fence_margin=margin, 
                    fence_type=f_type, fence_action=f_action
                )
                mission_generator.save_fence_file(polygon, "mission.fence")
                
                self.log("[SUCCESS] Geofence pushed successfully.")
                status_lbl.config(text="✅ SYNC SUCCESSFUL", foreground="green")
                messagebox.showinfo("Success", "Geofence pushed successfully!")
                win.destroy()
            except Exception as e:
                self.log(f"[ERROR] Push Failed: {e}")
                status_lbl.config(text=f"❌ FAILED: {str(e)[:25]}...", foreground="red")
                messagebox.showerror("Push Error", str(e))

        ttk.Button(config_frame, text="🚀 PUSH TO DRONE", command=on_push, padding=5).pack(fill=tk.X, pady=10)
        ttk.Button(config_frame, text="CANCEL", command=win.destroy).pack(fill=tk.X, pady=5)




    def start_upload_file_thread(self):
        """Standard method to upload any mission/fence file."""
        path = filedialog.askopenfilename(
            title="Select Mission or Fence File",
            filetypes=[("ArduPilot Files", "*.plan *.waypoints *.fence *.json"), ("All Files", "*.*")]
        )
        if not path:
            return
            
        t = Thread(target=self.upload_file_logic, args=(path,))
        t.start()

    def upload_file_logic(self, path):
        try:
             conn_str = self.connection_string_var.get()
             baud = self.baudrate_var.get()
             self.log(f"[INFO] Uploading file: {path} to {conn_str} at {baud}...")
             master = self.scanner.master if self.scanner else None
             mission_generator.upload_mission_from_file(path, connection_string=conn_str, baudrate=baud, master=master)

             messagebox.showinfo("Upload", "File Upload Successful!")
             self.log("[SUCCESS] File Upload Complete.")
        except Exception as e:
             self.log(f"[ERROR] File Upload Failed: {e}")
             messagebox.showerror("Upload Error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = MissionAGROSGUI(root)
    root.mainloop()
