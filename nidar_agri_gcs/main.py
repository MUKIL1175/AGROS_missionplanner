import sys
import os

def main():
    print("--- AGROS Ground Control System ---")
    print("[INFO] Launching Unified Mission Control...")
    # Launch GUI directly
    os.system(f"{sys.executable} mission_gui.py")

if __name__ == "__main__":
    main()
