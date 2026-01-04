import json
import os
import sys
import xml.etree.ElementTree as ET
from pymavlink import mavutil

def parse_kml_polygon(kml_path):
    """
    Parses a KML file to extract the first polygon found.
    Returns a list of (lat, lon) tuples.
    """
    try:
        tree = ET.parse(kml_path)
        root = tree.getroot()
        
        # KML namespaces can be tricky, we'll try to find coordinates anywhere
        # Simple extraction for <coordinates> tag
        coords_text = ""
        for coord_node in root.iter():
            if coord_node.tag.endswith('coordinates'):
                coords_text = coord_node.text.strip()
                break
        
        if not coords_text:
            return []

        # KML coords are lon,lat,alt
        polygon = []
        for line in coords_text.split():
            parts = line.split(',')
            if len(parts) >= 2:
                lon = float(parts[0])
                lat = float(parts[1])
                polygon.append((lat, lon))
        
        return polygon
    except Exception as e:
        print(f"[ERROR] failed to parse KML: {e}")
        return []

def is_point_in_polygon(lat, lon, polygon):
    """
    Ray-casting algorithm to check if a point is inside a polygon.
    polygon: list of (lat, lon) tuples.
    """
    if not polygon:
        return True # Default to True if no geofence
        
    num = len(polygon)
    j = num - 1
    c = False
    for i in range(num):
        if ((polygon[i][1] > lon) != (polygon[j][1] > lon)) and \
           (lat < (polygon[j][0] - polygon[i][0]) * (lon - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]):
            c = not c
        j = i
    return c

SELECTED_TARGETS_FILE = "selected_targets.json"
MISSION_PLAN_FILE = "mission.plan"  # File extension .plan is standard for QGC/MP JSON plans

def create_mission_item(command, params, frame=3, auto_continue=True, do_jump_id=0):
    """
    Helper to create a Mission Planner JSON item
    Frame 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
    """
    # Ensure params are floats to avoid compatibility issues
    float_params = [float(p) for p in params]
    
    return {
        "autoContinue": auto_continue,
        "command": command,
        "doJumpId": do_jump_id,
        "frame": frame,
        "params": float_params,
        "type": "SimpleItem"
    }

def generate_mp_json(targets):
    """
    Generates a Mission Planner compatible JSON structure.
    """
    items = []
    
    # 1. Takeoff (Command 22), Alt 10m
    items.append(create_mission_item(22, [0, 0, 0, 0, 0, 0, 10], do_jump_id=1))

    jump_id = 2
    for target in targets:
        lat = target['latitude']
        lon = target['longitude']
        alt = target.get('spray_altitude', 3)
        loiter_time = target.get('loiter_time', 6)

        # 2. Fly to Target (NAV_WAYPOINT = 16)
        items.append(create_mission_item(16, [0, 0, 0, 0, lat, lon, alt], do_jump_id=jump_id))
        jump_id += 1
        
        # 3. Loiter/Spray (NAV_LOITER_TIME = 19)
        items.append(create_mission_item(19, [loiter_time, 0, 0, 0, lat, lon, alt], do_jump_id=jump_id))
        jump_id += 1

    # 4. RTL (NAV_RETURN_TO_LAUNCH = 20)
    items.append(create_mission_item(20, [0, 0, 0, 0, 0, 0, 0], do_jump_id=jump_id))

    mission_plan = {
        "fileType": "Plan",
        "geoFence": {
            "polygons": [],
            "version": 2
        },
        "groundStation": "MissionPlanner",
        "mission": {
            "cruiseSpeed": 5,
            "firmwareType": 12, # 12 = ArduPilot standard in MP
            "hoverSpeed": 3,
            "items": items,
            "plannedHomePosition": [float(targets[0]['latitude']), float(targets[0]['longitude']), 0.0] if targets else [0.0, 0.0, 0.0],
            "vehicleType": 2, # Quadcopter
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        },
        "version": 1
    }
    return mission_plan

def upload_mission_mavlink(targets, connection_string='udp:127.0.0.1:14550'):
    """
    Uploads the mission directly to the drone via MAVLink.
    """
    print(f"[INFO] Connecting to drone at {connection_string}...")
    # Connection string - adjust if needed (e.g., com port or udp)
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("[INFO] Connected. Clearing mission...")

    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    
    # Calculate total items: 1 (Home) + 1 (Takeoff) + 2*N (Fly+Spray) + 1 (RTL) = 3 + 2N
    # Wait, pymavlink Mission protocol includes a "home" (seq 0) usually?
    # Yes, Seq 0 is usually home/current location.
    
    count = 3 + 2 * len(targets)
    master.mav.mission_count_send(master.target_system, master.target_component, count)
    
    seq = 0
    
    # 0. Home (Current location / Dummy)
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0, 0, 0, 0
    )
    seq += 1
    
    # 1. Takeoff
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 1, 0, 0, 0, 0, 0, 0, 10 # 10m alt
    )
    seq += 1
    
    for target in targets:
        lat = int(target['latitude'] * 1e7)
        lon = int(target['longitude'] * 1e7)
        alt = target.get('spray_altitude', 3)
        loiter_time = target.get('loiter_time', 6)
        
        # Fly to Target
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt
        )
        seq += 1
        
        # Spray (Loiter Time)
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            0, 1, loiter_time, 0, 0, 0,
            lat, lon, alt
        )
        seq += 1
        
    # RTL
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 1, 0, 0, 0, 0, 0, 0, 0
    )
    seq += 1
    
    print("[INFO] MAVLink Mission Uploaded Successfully!")

def send_rtl_command(connection_string='udp:127.0.0.1:14550', master=None):
    """Sends a direct MAVLink command to Return To Launch."""
    if master is None:
        print(f"[INFO] Sending RTL command to {connection_string}...")
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
    else:
        print("[INFO] Reusing existing MAVLink connection for RTL...")
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[SUCCESS] RTL Command Sent.")

def send_land_command(connection_string='udp:127.0.0.1:14550', master=None):
    """Sends a direct MAVLink command to Land immediately."""
    if master is None:
        print(f"[INFO] Sending Land command to {connection_string}...")
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
    else:
        print("[INFO] Reusing existing MAVLink connection for Land...")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[SUCCESS] Land Command Sent.")

def upload_fence_mavlink(polygon, connection_string='udp:127.0.0.1:14550', master=None):
    """
    Uploads a geofence (exclusion/inclusion) via MAVLink.
    Follows ArduPilot FENCE_POINT protocol:
     - Point 0 is the 'Return Point' (safe area inside).
     - Points 1..N are the polygon vertices.
    """
    if not polygon:
        return
    
    # ArduPilot expects first point to be Return Point, then vertices.
    # Total count = len(polygon) + 1
    count = len(polygon) + 1
    
    if master is None:
        print(f"[INFO] Uploading {count} fence points to {connection_string}...")
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
    else:
        print(f"[INFO] Reusing existing MAVLink connection for {count} fence points...")

    # 1. Send Return Point (Index 0) - we use the first polygon point as return point
    ret_lat, ret_lon = polygon[0]
    master.mav.fence_point_send(
        master.target_system, master.target_component,
        0, count, ret_lat, ret_lon
    )
    
    # 2. Send Polygon Vertices (Index 1..N)
    for i, (lat, lon) in enumerate(polygon):
        master.mav.fence_point_send(
            master.target_system, master.target_component,
            i + 1, count, lat, lon
        )
    
    # 3. Enable Fence and set type
    # Param 1=Enable, 2=Polygon Type, 4=Altitude? Let's ensure Polygon is bitwise 2.
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
        0, 1, 0, 0, 0, 0, 0, 0 # 1 = Enable
    )
    
    # 4. Set Parameters for visibility and action
    def set_param(name, value):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('utf-8'), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    set_param('FENCE_ENABLE', 1)
    set_param('FENCE_TYPE', 2)    # 2 = Polygon
    set_param('FENCE_ACTION', 1)  # 1 = RTL
    set_param('FENCE_TOTAL', count)
    
    print(f"[SUCCESS] Geofence ({count} points) Uploaded and Parameters set.")
    print("[TIP] In Mission Planner, go to 'Flight Plan' and ensure 'Fence' is visible, or click 'Get' in the Fence tab.")

def generate_waypoints_content(targets, auto_takeoff=True, completion_action="RTL"):
    """
    Generates QGC WPL 110 text format content.
    
    Args:
        targets: List of target dictionaries
        auto_takeoff: Boolean - whether to include auto takeoff command
        completion_action: String - "RTL" or "LAND"
    """
    lines = ["QGC WPL 110"]
    # Columns: Index CurrentWP CoordFrame Command P1 P2 P3 P4 Lat Lon Alt AutoContinue
    
    # 0. Home 
    h_lat = float(targets[0]['latitude']) if targets else 0.0
    h_lon = float(targets[0]['longitude']) if targets else 0.0
    lines.append(f"0 1 0 16 0 0 0 0 {h_lat:.7f} {h_lon:.7f} 0.000000 1")

    seq = 1
    
    # 1. Takeoff (Cmd 22) - only if auto_takeoff is True
    if auto_takeoff:
        lines.append(f"{seq} 0 3 22 0 0 0 0 0 0 10.000000 1")
        seq += 1
    
    for t in targets:
        lat = float(t['latitude'])
        lon = float(t['longitude'])
        alt = float(t.get('spray_altitude', 3))
        loiter = float(t.get('loiter_time', 6))
        
        # Fly to (Cmd 16)
        lines.append(f"{seq} 0 3 16 0 0 0 0 {lat:.7f} {lon:.7f} {alt:.6f} 1")
        seq += 1
        
        # Loiter (Cmd 19) - Param 1 is time
        lines.append(f"{seq} 0 3 19 {loiter:.2f} 0 0 0 {lat:.7f} {lon:.7f} {alt:.6f} 1")
        seq += 1
    
    # Mission completion action
    if completion_action == "LAND":
        # NAV_LAND (Cmd 21) - land at last target position
        if targets:
            last_lat = float(targets[-1]['latitude'])
            last_lon = float(targets[-1]['longitude'])
        else:
            last_lat, last_lon = 0.0, 0.0
        lines.append(f"{seq} 0 3 21 0 0 0 0 {last_lat:.7f} {last_lon:.7f} 0.000000 1")
    else:  # RTL (default)
        # NAV_RETURN_TO_LAUNCH (Cmd 20)
        lines.append(f"{seq} 0 3 20 0 0 0 0 0 0 0 1")
    
    return "\n".join(lines)

def parse_waypoints_file(filepath):
    """
    Parses a QGC WPL 110 file into list of MAVLink items
    """
    items = []
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    if not lines or "QGC WPL 110" not in lines[0]:
        raise ValueError("Invalid Waypoints File (Header missing)")
        
    # Skip header
    for line in lines[1:]:
        parts = line.strip().split()
        if len(parts) < 12: continue
        
        # Format: Index Curr Frame Cmd P1 P2 P3 P4 X Y Z Auto
        # We need to extract: Frame, Cmd, Params(P1..P4, X,Y,Z)
        frame = int(parts[2])
        cmd = int(parts[3])
        p1 = float(parts[4])
        p2 = float(parts[5])
        p3 = float(parts[6])
        p4 = float(parts[7])
        x = float(parts[8])
        y = float(parts[9])
        z = float(parts[10])
        
        items.append({
            'command': cmd,
            'frame': frame,
            'params': [p1, p2, p3, p4, x, y, z]
        })
    return items

def upload_mission_from_file(filepath, connection_string='udp:127.0.0.1:14551'):
    """
    Uploads a Mission Planner .plan/.json OR .waypoints file.
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")

    items = []
    
    # Check if text format
    is_text = False
    with open(filepath, 'r') as f:
        first_line = f.readline()
        if "QGC WPL 110" in first_line:
            is_text = True
            
    if is_text:
        raw_items = parse_waypoints_file(filepath)
        # Convert internal dict to structure loop expects below? 
        # Actually parse_waypoints_file returns dict with 'command', 'frame', 'params'.
        # The JSON 'items' list also has 'command', 'frame', 'params'.
        # So compatible!
        items = raw_items
    else:
        # JSON fallback
        with open(filepath, 'r') as f:
            plan = json.load(f)
        items = plan.get('mission', {}).get('items', [])

    if not items:
        raise ValueError("No mission items found in file.")

    print(f"[INFO] Uploading {len(items)} items from {filepath}...")
    print(f"[INFO] Connecting to drone at {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    
    # Count: If text file included Home (Seq 0), we use length.
    # If JSON list included Home? 
    # Usually JSON "items" starts at 1? 
    # Let's check our JSON generator: we put Home in "plannedHomePosition" but not in items.
    # Our Text generator: Line 0 IS Home.
    
    # If Text: Item 0 is Home.
    # If JSON: Item 0 is usually Takeoff (Seq 1).
    
    # We need to construct the mission sequence properly.
    if is_text:
        # Text file usually contains Seq 0.
        # So we send all items.
        # But MAVLink protocol: 
        # Seq 0 is Home.
        # mission_count should be len(items).
        pass
    else:
        # JSON usually doesn't have Seq 0 in 'items'.
        # We need to insert a dummy home or use plannedHome?
        # For this fix, let's assume we insert dummy home if JSON.
        pass
    
    count = len(items) 
    # If JSON, we need +1 for Home? 
    # Wait, my previous JSON uploader added Home manually.
    # If text has home, count is fine.
    # Let's adjust logic:
    
    final_items = []
    if is_text:
        final_items = items # contains seq 0
    else:
        # Add Home
        final_items.append({
            'command': 16, # Waypoint
            'frame': 0, # Global
            'params': [0,0,0,0,0,0,0] # Dummy
        })
        final_items.extend(items)
        
    master.mav.mission_count_send(master.target_system, master.target_component, len(final_items))
    
    seq = 0
    for item in final_items:
        cmd = item['command']
        params = item['params']
        frame = item.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        
        master.mav.mission_item_send(
            master.target_system, master.target_component, seq,
            frame, cmd, 0, 1,
            params[0], params[1], params[2], params[3],
            params[4], params[5], params[6]
        )
        seq += 1
        
    print("[INFO] File Mission Uploaded Successfully!")

def run_mission_generation(upload=False):
    if not os.path.exists(SELECTED_TARGETS_FILE):
        print(f"[ERROR] {SELECTED_TARGETS_FILE} not found. Run Target Selector first.")
        return

    with open(SELECTED_TARGETS_FILE, 'r') as f:
        data = json.load(f)
        targets = data.get("selected_targets", [])

    if not targets:
        print("[WARN] No targets selected.")
        return

    # Method 1: Generate Mission Planner File
    mp_json = generate_mp_json(targets)
    with open(MISSION_PLAN_FILE, 'w') as f:
        json.dump(mp_json, f, indent=2)
    print(f"[SUCCESS] Mission Planner file generated: {MISSION_PLAN_FILE}")

    # Method 2: Automatic Upload
    if upload:
        upload_mission_mavlink(targets)
    else:
        print("[INFO] Skipping MAVLink upload. Use --upload CLI arg or select in Menu to upload.")

if __name__ == "__main__":
    upload_arg = "--upload" in sys.argv
    run_mission_generation(upload=upload_arg)
