#!/usr/bin/env python3
import sys
import time
import math
import re
import argparse
import threading
from collections import deque

# Check and prompt for required packages
try:
    import serial
    import serial.tools.list_ports
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.animation as animation
except ImportError as e:
    print(f"Missing dependency: {e.name}")
    print("Please install them using: pip install pyserial matplotlib")
    sys.exit(1)

# Regex to parse the [TELE] telemetry lines
# Example: [TELE] pos:0.012,-0.045,245.320 vel:0.005,-0.010,45.120 q:0.999,0.002,-0.001,0.015
tele_pattern = re.compile(
    r"\[TELE\] pos:([\d\.-]+),([\d\.-]+),([\d\.-]+) "
    r"vel:([\d\.-]+),([\d\.-]+),([\d\.-]+) "
    r"q:([\d\.-]+),([\d\.-]+),([\d\.-]+),([\d\.-]+)"
)

# Shared data structures for the plotting thread
data_lock = threading.Lock()
pos_history = deque(maxlen=500)  # Store last 500 points for trail
latest_pos = [0.0, 0.0, 0.0]
latest_vel = [0.0, 0.0, 0.0]
latest_q = [1.0, 0.0, 0.0, 0.0]
is_running = True

# Helper: Quaternion to Rotation Matrix
def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    r11 = 1.0 - 2.0 * (qy**2 + qz**2)
    r12 = 2.0 * (qx*qy - qw*qz)
    r13 = 2.0 * (qx*qz + qw*qy)

    r21 = 2.0 * (qx*qy + qw*qz)
    r22 = 1.0 - 2.0 * (qx**2 + qz**2)
    r23 = 2.0 * (qy*qz - qw*qx)

    r31 = 2.0 * (qx*qz - qw*qy)
    r32 = 2.0 * (qy*qz + qw*qx)
    r33 = 1.0 - 2.0 * (qx**2 + qy**2)
    return [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]

# -------------------------------------------------------------------------
# Serial Reader Thread
# -------------------------------------------------------------------------
def serial_reader(port_name, baud_rate):
    global latest_pos, latest_vel, latest_q, is_running
    print(f"[Serial] Opening port {port_name} at {baud_rate} baud...")
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=1.0)
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[Serial] Failed to open port: {e}")
        return

    print("[Serial] Port successfully opened. Listening for EKF Telemetry...")
    while is_running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            # Print raw serial data containing raw IMU, EKF playbacks, and quaternions
            print(line)

            match = tele_pattern.search(line)
            if match:
                px, py, pz = map(float, match.groups()[0:3])
                vx, vy, vz = map(float, match.groups()[3:6])
                qw, qx, qy, qz = map(float, match.groups()[6:10])

                with data_lock:
                    latest_pos = [px, py, pz]
                    latest_vel = [vx, vy, vz]
                    latest_q = [qw, qx, qy, qz]
                    pos_history.append((px, py, pz))
        except Exception as e:
            print(f"[Serial] Read error: {e}")
            break

    ser.close()
    print("[Serial] Port closed.")

# -------------------------------------------------------------------------
# Simulation Mode (Fallback for testing without board)
# -------------------------------------------------------------------------
def simulation_worker():
    global latest_pos, latest_vel, latest_q, is_running
    print("[Simulation] Launching EKF telemetry simulation mode...")
    
    # Rocket state parameters
    t = 0.0
    dt = 0.02  # 50 Hz updates
    
    px, py, pz = 0.0, 0.0, 0.0
    vx, vy, vz = 0.0, 0.0, 0.0
    pitch, roll, yaw = 0.0, 0.0, 0.0
    
    state = "LAUNCHPAD" # LAUNCHPAD, BURN, COAST, APOGEE, RECOVERY, LANDED
    burn_time = 3.5
    apogee_time = 9.0
    recovery_time = 35.0

    while is_running:
        t += dt
        
        # State machine for rocket flight dynamics
        if state == "LAUNCHPAD":
            if t > 2.0:
                state = "BURN"
                print("\n🚀 IGNITION! Rocket has launched!")
        elif state == "BURN":
            # High vertical thrust + slight horizontal wind drift
            acc_z = 35.0 - 9.8  # ~3.5g net acceleration
            acc_x = 0.8         # Wind push east
            acc_y = -0.4        # Wind push north
            
            vx += acc_x * dt
            vy += acc_y * dt
            vz += acc_z * dt
            
            # Tilt slightly during ascent
            pitch = 5.0 * (t - 2.0) / burn_time  # Tilts up to 5 degrees
            yaw = 15.0 * (t - 2.0) / burn_time
            
            if t > (2.0 + burn_time):
                state = "COAST"
                print("\n🔥 MOTOR BURNOUT! Entering coast phase.")
        elif state == "COAST":
            # Gravity only
            vx += 0.0 * dt
            vy += 0.0 * dt
            vz += -9.80665 * dt
            
            # Rocket begins tumbling slightly due to aerodynamic drag
            pitch += 2.0 * dt
            yaw += 3.0 * dt
            
            if vz <= 0.0 and state != "APOGEE":
                state = "APOGEE"
                print(f"\n🎈 APOGEE REACHED! Max altitude: {pz:.2f}m. Deploying Drogue Parachute.")
        elif state == "APOGEE":
            state = "RECOVERY"
        elif state == "RECOVERY":
            # Descent under parachute (terminal velocity ~ -6.0 m/s)
            vz = -6.0 + 0.5 * math.sin(t)  # Parachute swing oscillation
            vx = 1.5 * math.sin(t * 0.5)   # Drifting horizontally
            vy = 1.0 * math.cos(t * 0.5)
            
            # Parachute swinging tilt
            pitch = 15.0 * math.sin(t * 0.8)
            roll = 10.0 * math.cos(t * 0.6)
            
            if pz <= 0.2:
                state = "LANDED"
                print("\n🏡 LANDED! Recovery successful.")
        elif state == "LANDED":
            vx, vy, vz = 0.0, 0.0, 0.0
            pz = 0.0
            pitch, roll, yaw = 0.0, 0.0, 0.0

        # Update position
        px += vx * dt
        py += vy * dt
        pz += vz * dt
        if pz < 0.0:
            pz = 0.0

        # Calculate quaternion from Euler angles (yaw, pitch, roll)
        cy = math.cos(math.radians(yaw) * 0.5)
        sy = math.sin(math.radians(yaw) * 0.5)
        cp = math.cos(math.radians(pitch) * 0.5)
        sp = math.sin(math.radians(pitch) * 0.5)
        cr = math.cos(math.radians(roll) * 0.5)
        sr = math.sin(math.radians(roll) * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        with data_lock:
            latest_pos = [px, py, pz]
            latest_vel = [vx, vy, vz]
            latest_q = [qw, qx, qy, qz]
            pos_history.append((px, py, pz))

        # Output mock telemetry string to terminal (matching board stdout)
        sys.stdout.write(
            f"\r[TELE] pos:{px:.3f},{py:.3f},{pz:.3f} "
            f"vel:{vx:.3f},{vy:.3f},{vz:.3f} "
            f"q:{qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f}"
        )
        sys.stdout.flush()
        
        time.sleep(dt)

# -------------------------------------------------------------------------
# Dynamic 3D Plotting
# -------------------------------------------------------------------------
def run_visualization():
    fig = plt.figure(figsize=(12, 8))
    fig.canvas.manager.set_window_title("RocketCup Avionics 3D EKF Live Tracker")
    
    # 3D Trajectory Axes
    ax_3d = fig.add_subplot(121, projection='3d')
    ax_3d.view_init(elev=20, azim=45)
    
    # 2D Attitude / Orientation vectors Axes
    ax_att = fig.add_subplot(122, projection='3d')
    ax_att.view_init(elev=25, azim=30)
    
    # Text readouts
    text_info = fig.text(0.02, 0.9, "", fontsize=10, family='monospace',
                         bbox=dict(facecolor='black', alpha=0.1, boxstyle='round,pad=0.5'))

    # Plot initializations
    trail_line, = ax_3d.plot([], [], [], 'cyan', lw=2, label="Flight Trajectory")
    rocket_dot, = ax_3d.plot([], [], [], 'ro', ms=8, label="Rocket EKF Pos")
    
    def init_plots():
        ax_3d.set_xlabel("East (X) [meters]")
        ax_3d.set_ylabel("North (Y) [meters]")
        ax_3d.set_zlabel("Altitude (Z) [meters]")
        ax_3d.grid(True)
        ax_3d.legend(loc="upper left")
        
        ax_att.set_xlim3d([-1.2, 1.2])
        ax_att.set_ylim3d([-1.2, 1.2])
        ax_att.set_zlim3d([-1.2, 1.2])
        ax_att.set_title("Rocket Attitude (Body Frame Vectors)")
        ax_att.set_xlabel("X (Right)")
        ax_att.set_ylabel("Y (Forward)")
        ax_att.set_zlabel("Z (Up / Longitudinal)")
        
        return trail_line, rocket_dot

    def update_frame(frame):
        with data_lock:
            hist = list(pos_history)
            curr_pos = list(latest_pos)
            curr_vel = list(latest_vel)
            curr_q = list(latest_q)
            
        if not hist:
            return trail_line, rocket_dot

        # 1. Update 3D flight trajectory
        xs = [p[0] for p in hist]
        ys = [p[1] for p in hist]
        zs = [p[2] for p in hist]

        trail_line.set_data(xs, ys)
        trail_line.set_3d_properties(zs)
        rocket_dot.set_data([curr_pos[0]], [curr_pos[1]])
        rocket_dot.set_3d_properties([curr_pos[2]])

        # Dynamically scale 3D bounds to keep flight trail in view
        max_range = max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs), 10.0)
        mid_x = (max(xs)+min(xs))/2.0
        mid_y = (max(ys)+min(ys))/2.0
        mid_z = (max(zs)+min(zs))/2.0

        ax_3d.set_xlim3d([mid_x - max_range/2, mid_x + max_range/2])
        ax_3d.set_ylim3d([mid_y - max_range/2, mid_y + max_range/2])
        ax_3d.set_zlim3d([0, max(max(zs) + 5.0, 10.0)])

        # 2. Update Attitude Vectors on right subplot
        ax_att.cla()
        ax_att.set_xlim3d([-1.2, 1.2])
        ax_att.set_ylim3d([-1.2, 1.2])
        ax_att.set_zlim3d([-1.2, 1.2])
        ax_att.set_xlabel("X (Right)")
        ax_att.set_ylabel("Y (Forward)")
        ax_att.set_zlabel("Z (Up / Longitudinal)")
        ax_att.set_title("Rocket Attitude (Body Frame Orientation)")

        # Rotate base unit vectors [1,0,0], [0,1,0], [0,0,1] using current attitude rotation matrix
        R = quaternion_to_rotation_matrix(curr_q)
        
        # Red vector = Body X (Right)
        ax_att.quiver(0, 0, 0, R[0][0], R[1][0], R[2][0], color='r', length=1.0, arrow_length_ratio=0.2, label="Body-X (Right)")
        # Green vector = Body Y (Forward)
        ax_att.quiver(0, 0, 0, R[0][1], R[1][1], R[2][1], color='g', length=1.0, arrow_length_ratio=0.2, label="Body-Y (Forward)")
        # Blue vector = Body Z (Up / longitudinal pointer)
        ax_att.quiver(0, 0, 0, R[0][2], R[1][2], R[2][2], color='b', length=1.2, arrow_length_ratio=0.2, label="Body-Z (Up)")
        ax_att.legend(loc="upper right")

        # 3. Update Text Info Box
        vel_mag = math.sqrt(curr_vel[0]**2 + curr_vel[1]**2 + curr_vel[2]**2)
        text_info.set_text(
            f"=== 🚀 LIVE EKF ESTIMATION ===\n"
            f"Altitude : {curr_pos[2]:7.2f} m\n"
            f"Vertical V: {curr_vel[2]:7.2f} m/s\n"
            f"Total Vel: {vel_mag:7.2f} m/s\n"
            f"Horizontal: ({curr_pos[0]:.2f}, {curr_pos[1]:.2f}) m\n"
            f"Quaternion: [{curr_q[0]:.3f}, {curr_q[1]:.3f}, {curr_q[2]:.3f}, {curr_q[3]:.3f}]\n"
        )
        
        return trail_line, rocket_dot

    # Set up matplotlib animation
    ani = animation.FuncAnimation(
        fig, update_frame, init_func=init_plots, interval=50, blit=False, cache_frame_data=False
    )
    plt.tight_layout()
    plt.show()

# -------------------------------------------------------------------------
# Main Launcher
# -------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Avionics 3D EKF Real-Time Serial Visualizer")
    parser.add_argument("--port", type=str, default="AUTO", help="Serial port (e.g. /dev/tty.usbmodem10)")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default: 460800)")
    parser.add_argument("--sim", action="store_true", help="Run in offline simulation mode instead of reading serial")
    
    args = parser.parse_args()

    # Launch Simulation or Serial thread
    if args.sim:
        sim_thread = threading.Thread(target=simulation_worker, daemon=True)
        sim_thread.start()
    else:
        # Auto-detect serial port if not specified
        port = args.port
        if port.upper() == "AUTO":
            import os
            # On macOS, prioritize the known working cu.usbserial-110 bridge
            if os.path.exists("/dev/cu.usbserial-110"):
                port = "/dev/cu.usbserial-110"
            else:
                ports = list(serial.tools.list_ports.comports())
                usb_ports = []
                for p in ports:
                    dev = p.device
                    # Prioritize actual USB-to-UART bridges, avoiding Bluetooth devices
                    if any(kw in dev.lower() for kw in ["usb", "ch34", "cp210", "ftdi", "uart"]):
                        if sys.platform == "darwin" and "tty." in dev:
                            dev = dev.replace("tty.", "cu.")
                        usb_ports.append(dev)
                
                if usb_ports:
                    port = usb_ports[0]
                elif ports:
                    port = ports[0].device
                    if sys.platform == "darwin" and "tty." in port:
                        port = port.replace("tty.", "cu.")
                else:
                    print("[Error] No active COM/Serial ports detected!")
                    print("Please plug in the STM32 board or run simulation mode: python ekf_visualizer.py --sim")
                    sys.exit(1)
            
            print(f"[COM] Auto-selected active serial port: {port}")

        serial_thread = threading.Thread(target=serial_reader, args=(port, args.baud), daemon=True)
        serial_thread.start()

    # Open visualization GUI on the main thread
    try:
        run_visualization()
    except KeyboardInterrupt:
        pass
    finally:
        is_running = False
        print("\nExiting. Thank you!")
