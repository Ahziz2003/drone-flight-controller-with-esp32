#!/usr/bin/env python3
"""

GUI:
- Real-time attitude visualization
- PID tuning (cascaded: angle + rate)
- Motor monitoring
- Arm/Disarm control

"""

import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

# CONFIGURATION

DRONE_IP = "192.168.4.1"
DRONE_PORT = 4210
LOCAL_PORT = 4211

TELEMETRY_TIMEOUT = 1.0  # seconds
COMMAND_RATE = 50  # Hz

# DATA STRUCTURES

@dataclass
class TelemetryData:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    throttle: float = 0.0
    armed: bool = False
    motor_fl: int = 0
    motor_fr: int = 0
    motor_rl: int = 0
    motor_rr: int = 0

@dataclass
class PIDGains:
    kP: float = 0.0
    kI: float = 0.0
    kD: float = 0.0

# MAIN APPLICATION

class DroneGCS:
    def __init__(self, root):
        self.root = root
        self.root.title("Quadcopter Ground Control Station")
        self.root.geometry("1200x800")
        self.root.configure(bg='#1a1a2e')
        
        # Data
        self.telemetry = TelemetryData()
        self.connected = False
        self.last_telemetry_time = 0
        self.running = True
        
        # PID gains storage
        self.roll_angle_pid = PIDGains(4.0, 0.0, 0.0)
        self.pitch_angle_pid = PIDGains(4.0, 0.0, 0.0)
        self.roll_rate_pid = PIDGains(0.7, 0.3, 0.01)
        self.pitch_rate_pid = PIDGains(0.7, 0.3, 0.01)
        
        # Control values
        self.throttle_var = tk.DoubleVar(value=0.0)
        self.roll_var = tk.DoubleVar(value=0.0)
        self.pitch_var = tk.DoubleVar(value=0.0)
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', LOCAL_PORT))
        self.sock.settimeout(0.1)
        
        # History for graphs
        self.roll_history = deque(maxlen=200)
        self.pitch_history = deque(maxlen=200)
        
        # Build UI
        self.setup_styles()
        self.create_ui()
        
        # Start threads
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()
        
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()
        
        # Update UI
        self.update_ui()
        
        # Cleanup on close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure colors
        style.configure('TFrame', background='#1a1a2e')
        style.configure('TLabel', background='#1a1a2e', foreground='#eee')
        style.configure('TLabelframe', background='#1a1a2e', foreground='#00d4ff')
        style.configure('TLabelframe.Label', background='#1a1a2e', foreground='#00d4ff', font=('Helvetica', 10, 'bold'))
        style.configure('TButton', font=('Helvetica', 10))
        style.configure('TScale', background='#1a1a2e')
        
        # Custom button styles
        style.configure('Arm.TButton', background='#00ff88', foreground='black')
        style.configure('Disarm.TButton', background='#ff4444', foreground='white')
        style.configure('Send.TButton', background='#00d4ff', foreground='black')
    
    def create_ui(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Top row: Status + Attitude
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.create_status_panel(top_frame)
        self.create_attitude_panel(top_frame)
        
        # Middle row: PID Tuning
        self.create_pid_panel(main_frame)
        
        # Bottom row: Controls + Motors
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        
        self.create_control_panel(bottom_frame)
        self.create_motor_panel(bottom_frame)
    
    def create_status_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="📡 Status", padding=10)
        frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        
        # Connection status
        self.conn_label = ttk.Label(frame, text="● DISCONNECTED", foreground='#ff4444', font=('Helvetica', 12, 'bold'))
        self.conn_label.pack(anchor=tk.W)
        
        # Armed status
        self.arm_label = ttk.Label(frame, text="● DISARMED", foreground='#888', font=('Helvetica', 12, 'bold'))
        self.arm_label.pack(anchor=tk.W, pady=(5, 10))
        
        # Control buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X)
        
        ttk.Button(btn_frame, text="ARM", style='Arm.TButton', command=self.arm_drone).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="DISARM", style='Disarm.TButton', command=self.disarm_drone).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="CALIBRATE", command=self.calibrate_imu).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="GET PID", command=self.get_pid_values).pack(fill=tk.X, pady=2)
    
    def create_attitude_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="🎯 Attitude", padding=10)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Canvas for attitude indicator
        self.attitude_canvas = tk.Canvas(frame, width=300, height=200, bg='#0f0f1a', highlightthickness=0)
        self.attitude_canvas.pack(side=tk.LEFT, padx=(0, 20))
        
        # Numeric values
        values_frame = ttk.Frame(frame)
        values_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        # Roll
        ttk.Label(values_frame, text="Roll:", font=('Helvetica', 10)).grid(row=0, column=0, sticky=tk.W)
        self.roll_value = ttk.Label(values_frame, text="0.0°", font=('Courier', 14, 'bold'), foreground='#00ff88')
        self.roll_value.grid(row=0, column=1, sticky=tk.E, padx=(10, 0))
        
        # Pitch
        ttk.Label(values_frame, text="Pitch:", font=('Helvetica', 10)).grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        self.pitch_value = ttk.Label(values_frame, text="0.0°", font=('Courier', 14, 'bold'), foreground='#00d4ff')
        self.pitch_value.grid(row=1, column=1, sticky=tk.E, padx=(10, 0), pady=(5, 0))
        
        # Yaw
        ttk.Label(values_frame, text="Yaw:", font=('Helvetica', 10)).grid(row=2, column=0, sticky=tk.W, pady=(5, 0))
        self.yaw_value = ttk.Label(values_frame, text="0.0°", font=('Courier', 14, 'bold'), foreground='#ffaa00')
        self.yaw_value.grid(row=2, column=1, sticky=tk.E, padx=(10, 0), pady=(5, 0))
        
        # Rates
        ttk.Label(values_frame, text="─────────", foreground='#444').grid(row=3, column=0, columnspan=2, pady=5)
        
        ttk.Label(values_frame, text="Roll Rate:", font=('Helvetica', 9)).grid(row=4, column=0, sticky=tk.W)
        self.roll_rate_value = ttk.Label(values_frame, text="0 °/s", font=('Courier', 11), foreground='#888')
        self.roll_rate_value.grid(row=4, column=1, sticky=tk.E)
        
        ttk.Label(values_frame, text="Pitch Rate:", font=('Helvetica', 9)).grid(row=5, column=0, sticky=tk.W)
        self.pitch_rate_value = ttk.Label(values_frame, text="0 °/s", font=('Courier', 11), foreground='#888')
        self.pitch_rate_value.grid(row=5, column=1, sticky=tk.E)
    
    def create_pid_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="⚙️ PID Tuning (Cascaded)", padding=10)
        frame.pack(fill=tk.X)
        
        # Create PID entries storage
        self.pid_entries = {}
        
        # Headers
        headers_frame = ttk.Frame(frame)
        headers_frame.pack(fill=tk.X)
        
        ttk.Label(headers_frame, text="", width=15).pack(side=tk.LEFT)
        for label in ["P", "I", "D"]:
            ttk.Label(headers_frame, text=label, width=10, font=('Helvetica', 10, 'bold'), foreground='#00d4ff').pack(side=tk.LEFT, padx=5)
        ttk.Label(headers_frame, text="", width=10).pack(side=tk.LEFT)  # Button space
        
        # PID rows
        pid_configs = [
            ("Roll Angle", "RA", self.roll_angle_pid),
            ("Pitch Angle", "PA", self.pitch_angle_pid),
            ("Roll Rate", "RR", self.roll_rate_pid),
            ("Pitch Rate", "PR", self.pitch_rate_pid),
        ]
        
        for name, code, gains in pid_configs:
            self.create_pid_row(frame, name, code, gains)
    
    def create_pid_row(self, parent, name: str, code: str, gains: PIDGains):
        row_frame = ttk.Frame(parent)
        row_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(row_frame, text=name, width=15, anchor=tk.W).pack(side=tk.LEFT)
        
        entries = {}
        for param, value in [("P", gains.kP), ("I", gains.kI), ("D", gains.kD)]:
            entry = ttk.Entry(row_frame, width=10)
            entry.insert(0, f"{value:.3f}")
            entry.pack(side=tk.LEFT, padx=5)
            entries[param] = entry
        
        self.pid_entries[code] = entries
        
        ttk.Button(row_frame, text="Send", style='Send.TButton', 
                   command=lambda c=code: self.send_pid(c)).pack(side=tk.LEFT, padx=5)
    
    def create_control_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="🎮 Control", padding=10)
        frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # Throttle slider
        throttle_frame = ttk.Frame(frame)
        throttle_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(throttle_frame, text="Throttle:").pack(side=tk.LEFT)
        self.throttle_slider = ttk.Scale(throttle_frame, from_=0, to=1, orient=tk.HORIZONTAL, 
                                         variable=self.throttle_var, length=200)
        self.throttle_slider.pack(side=tk.LEFT, padx=10)
        self.throttle_display = ttk.Label(throttle_frame, text="0%", width=6)
        self.throttle_display.pack(side=tk.LEFT)
        
        # Roll slider
        roll_frame = ttk.Frame(frame)
        roll_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(roll_frame, text="Roll Target:").pack(side=tk.LEFT)
        self.roll_slider = ttk.Scale(roll_frame, from_=-30, to=30, orient=tk.HORIZONTAL, 
                                     variable=self.roll_var, length=200)
        self.roll_slider.pack(side=tk.LEFT, padx=10)
        self.roll_target_display = ttk.Label(roll_frame, text="0°", width=6)
        self.roll_target_display.pack(side=tk.LEFT)
        
        # Pitch slider
        pitch_frame = ttk.Frame(frame)
        pitch_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(pitch_frame, text="Pitch Target:").pack(side=tk.LEFT)
        self.pitch_slider = ttk.Scale(pitch_frame, from_=-30, to=30, orient=tk.HORIZONTAL, 
                                      variable=self.pitch_var, length=200)
        self.pitch_slider.pack(side=tk.LEFT, padx=10)
        self.pitch_target_display = ttk.Label(pitch_frame, text="0°", width=6)
        self.pitch_target_display.pack(side=tk.LEFT)
        
        # Reset button
        ttk.Button(frame, text="Reset to Zero", command=self.reset_controls).pack(pady=10)
        
        # Emergency stop
        self.emergency_btn = tk.Button(frame, text="⚠️ EMERGENCY STOP ⚠️", bg='#ff0000', fg='white',
                                       font=('Helvetica', 12, 'bold'), command=self.emergency_stop)
        self.emergency_btn.pack(fill=tk.X, pady=10)
    
    def create_motor_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="🔧 Motors", padding=10)
        frame.pack(side=tk.LEFT, fill=tk.BOTH)
        
        # Motor visualization canvas
        self.motor_canvas = tk.Canvas(frame, width=200, height=200, bg='#0f0f1a', highlightthickness=0)
        self.motor_canvas.pack()
        
        # Draw motor layout
        self.draw_motor_layout()
    

    # DRAWING FUNCTIONS
    
    
    def draw_attitude_indicator(self):
        canvas = self.attitude_canvas
        canvas.delete("all")
        
        w, h = 300, 200
        cx, cy = w // 2, h // 2
        
        # Draw horizon
        roll_rad = math.radians(self.telemetry.roll)
        pitch_offset = self.telemetry.pitch * 2  # Scale pitch for visibility
        
        # Sky and ground
        canvas.create_rectangle(0, 0, w, h, fill='#1a3a5c')  # Sky
        
        # Calculate horizon line with roll and pitch
        horizon_y = cy + pitch_offset
        
        # Draw ground (tilted)
        dx = math.tan(roll_rad) * (h / 2)
        points = [
            0, horizon_y + dx,
            w, horizon_y - dx,
            w, h,
            0, h
        ]
        canvas.create_polygon(points, fill='#5a3a1a')
        
        # Horizon line
        canvas.create_line(0, horizon_y + dx, w, horizon_y - dx, fill='white', width=2)
        
        # Center reference (aircraft symbol)
        canvas.create_line(cx - 40, cy, cx - 15, cy, fill='#ffaa00', width=3)
        canvas.create_line(cx + 15, cy, cx + 40, cy, fill='#ffaa00', width=3)
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, outline='#ffaa00', width=2)
        
        # Roll indicator arc
        canvas.create_arc(cx - 80, cy - 80, cx + 80, cy + 80, 
                          start=30, extent=120, style=tk.ARC, outline='white', width=1)
        
        # Roll tick marks
        for angle in [-60, -45, -30, -15, 0, 15, 30, 45, 60]:
            rad = math.radians(90 - angle)
            x1 = cx + 70 * math.cos(rad)
            y1 = cy - 70 * math.sin(rad)
            x2 = cx + 80 * math.cos(rad)
            y2 = cy - 80 * math.sin(rad)
            canvas.create_line(x1, y1, x2, y2, fill='white', width=1)
        
        # Roll pointer
        roll_pointer_rad = math.radians(90 + self.telemetry.roll)
        px = cx + 65 * math.cos(roll_pointer_rad)
        py = cy - 65 * math.sin(roll_pointer_rad)
        canvas.create_polygon(px, py - 8, px - 5, py + 2, px + 5, py + 2, fill='#00ff88')
    
    def draw_motor_layout(self):
        canvas = self.motor_canvas
        canvas.delete("all")
        
        w, h = 200, 200
        cx, cy = w // 2, h // 2
        
        # Drone frame
        canvas.create_line(cx - 50, cy - 50, cx + 50, cy + 50, fill='#444', width=3)
        canvas.create_line(cx + 50, cy - 50, cx - 50, cy + 50, fill='#444', width=3)
        
        # Center body
        canvas.create_oval(cx - 15, cy - 15, cx + 15, cy + 15, fill='#333', outline='#666')
        
        # Direction arrow
        canvas.create_polygon(cx, cy - 12, cx - 5, cy - 2, cx + 5, cy - 2, fill='#ff4444')
        
        # Motor positions
        motor_pos = [
            (cx - 50, cy - 50, "FL"),  # Front Left
            (cx + 50, cy - 50, "FR"),  # Front Right
            (cx - 50, cy + 50, "RL"),  # Rear Left
            (cx + 50, cy + 50, "RR"),  # Rear Right
        ]
        
        motor_values = [
            self.telemetry.motor_fl,
            self.telemetry.motor_fr,
            self.telemetry.motor_rl,
            self.telemetry.motor_rr,
        ]
        
        for (x, y, label), value in zip(motor_pos, motor_values):
            # Calculate color based on throttle
            intensity = min(255, int(value / 2000 * 255))
            color = f'#{intensity:02x}{max(0, 255-intensity):02x}00'
            
            # Motor circle
            r = 20
            canvas.create_oval(x - r, y - r, x + r, y + r, fill=color, outline='white', width=2)
            
            # Label
            canvas.create_text(x, y, text=label, fill='white', font=('Helvetica', 8, 'bold'))
            
            # Value
            canvas.create_text(x, y + 30, text=str(value), fill='#888', font=('Courier', 8))
    
    # COMMUNICATION
    
    def send_command(self, cmd: str):
        try:
            self.sock.sendto(cmd.encode(), (DRONE_IP, DRONE_PORT))
        except Exception as e:
            print(f"Send error: {e}")
    
    def receive_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                self.parse_telemetry(data.decode())
                self.last_telemetry_time = time.time()
            except socket.timeout:
                pass
            except Exception as e:
                print(f"Receive error: {e}")
    
    def send_loop(self):
        interval = 1.0 / COMMAND_RATE
        while self.running:
            if self.telemetry.armed:
                cmd = f"CTRL,{self.throttle_var.get():.2f},{self.roll_var.get():.1f},{self.pitch_var.get():.1f}"
                self.send_command(cmd)
            time.sleep(interval)
    
    def parse_telemetry(self, data: str):
        try:
            if data.startswith("TEL,"):
                parts = data.split(",")
                if len(parts) >= 13:
                    self.telemetry.roll = float(parts[1])
                    self.telemetry.pitch = float(parts[2])
                    self.telemetry.yaw = float(parts[3])
                    self.telemetry.roll_rate = float(parts[4])
                    self.telemetry.pitch_rate = float(parts[5])
                    self.telemetry.yaw_rate = float(parts[6])
                    self.telemetry.throttle = float(parts[7])
                    self.telemetry.armed = parts[8] == "1"
                    self.telemetry.motor_fl = int(parts[9])
                    self.telemetry.motor_fr = int(parts[10])
                    self.telemetry.motor_rl = int(parts[11])
                    self.telemetry.motor_rr = int(parts[12])
                    
                    # Update history
                    self.roll_history.append(self.telemetry.roll)
                    self.pitch_history.append(self.telemetry.pitch)
            
            elif data.startswith("PIDS,"):
                parts = data.split(",")
                if len(parts) >= 13:
                    # Update entries with received values
                    self.pid_entries["RA"]["P"].delete(0, tk.END)
                    self.pid_entries["RA"]["P"].insert(0, parts[1])
                    self.pid_entries["RA"]["I"].delete(0, tk.END)
                    self.pid_entries["RA"]["I"].insert(0, parts[2])
                    self.pid_entries["RA"]["D"].delete(0, tk.END)
                    self.pid_entries["RA"]["D"].insert(0, parts[3])
                    
                    self.pid_entries["PA"]["P"].delete(0, tk.END)
                    self.pid_entries["PA"]["P"].insert(0, parts[4])
                    self.pid_entries["PA"]["I"].delete(0, tk.END)
                    self.pid_entries["PA"]["I"].insert(0, parts[5])
                    self.pid_entries["PA"]["D"].delete(0, tk.END)
                    self.pid_entries["PA"]["D"].insert(0, parts[6])
                    
                    self.pid_entries["RR"]["P"].delete(0, tk.END)
                    self.pid_entries["RR"]["P"].insert(0, parts[7])
                    self.pid_entries["RR"]["I"].delete(0, tk.END)
                    self.pid_entries["RR"]["I"].insert(0, parts[8])
                    self.pid_entries["RR"]["D"].delete(0, tk.END)
                    self.pid_entries["RR"]["D"].insert(0, parts[9])
                    
                    self.pid_entries["PR"]["P"].delete(0, tk.END)
                    self.pid_entries["PR"]["P"].insert(0, parts[10])
                    self.pid_entries["PR"]["I"].delete(0, tk.END)
                    self.pid_entries["PR"]["I"].insert(0, parts[11])
                    self.pid_entries["PR"]["D"].delete(0, tk.END)
                    self.pid_entries["PR"]["D"].insert(0, parts[12])
                    
                    print("PID values received from drone")
                    
        except Exception as e:
            print(f"Parse error: {e}")
    
    # BUTTON HANDLERS
    
    def arm_drone(self):
        self.send_command("ARM")
    
    def disarm_drone(self):
        self.send_command("DISARM")
        self.reset_controls()
    
    def calibrate_imu(self):
        self.send_command("CAL")
    
    def get_pid_values(self):
        self.send_command("GETPID")
    
    def send_pid(self, code: str):
        try:
            p = float(self.pid_entries[code]["P"].get())
            i = float(self.pid_entries[code]["I"].get())
            d = float(self.pid_entries[code]["D"].get())
            cmd = f"PID,{code},{p},{i},{d}"
            self.send_command(cmd)
            print(f"Sent: {cmd}")
        except ValueError:
            messagebox.showerror("Error", "Invalid PID values")
    
    def reset_controls(self):
        self.throttle_var.set(0)
        self.roll_var.set(0)
        self.pitch_var.set(0)
    
    def emergency_stop(self):
        self.send_command("DISARM")
        self.reset_controls()
        for _ in range(5):  # Send multiple times for reliability
            self.send_command("DISARM")
    
    # UI UPDATe
    
    def update_ui(self):
        if not self.running:
            return
        
        # Check connection status
        connected = (time.time() - self.last_telemetry_time) < TELEMETRY_TIMEOUT
        
        if connected != self.connected:
            self.connected = connected
            if connected:
                self.conn_label.config(text="● CONNECTED", foreground='#00ff88')
            else:
                self.conn_label.config(text="● DISCONNECTED", foreground='#ff4444')
        
        # Update armed status
        if self.telemetry.armed:
            self.arm_label.config(text="● ARMED", foreground='#ff4444')
        else:
            self.arm_label.config(text="● DISARMED", foreground='#888')
        
        # Update attitude values
        self.roll_value.config(text=f"{self.telemetry.roll:+.1f}°")
        self.pitch_value.config(text=f"{self.telemetry.pitch:+.1f}°")
        self.yaw_value.config(text=f"{self.telemetry.yaw:+.1f}°")
        self.roll_rate_value.config(text=f"{self.telemetry.roll_rate:+.0f} °/s")
        self.pitch_rate_value.config(text=f"{self.telemetry.pitch_rate:+.0f} °/s")
        
        # Update control displays
        self.throttle_display.config(text=f"{self.throttle_var.get()*100:.0f}%")
        self.roll_target_display.config(text=f"{self.roll_var.get():.0f}°")
        self.pitch_target_display.config(text=f"{self.pitch_var.get():.0f}°")
        
        # Draw visualizations
        self.draw_attitude_indicator()
        self.draw_motor_layout()
        
        # Schedule next update
        self.root.after(50, self.update_ui)  # 20 Hz UI update
    
    def on_close(self):
        self.running = False
        self.send_command("DISARM")
        time.sleep(0.1)
        self.sock.close()
        self.root.destroy()

# ENTRY POINT

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneGCS(root)
    root.mainloop()
