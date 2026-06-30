import math, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.transforms as transforms
import requests
from collections import deque
from matplotlib.widgets import Button, TextBox
import csv
import datetime

# ------------- CONFIG -------------
ROBOT_IP = "http://192.168.4.1"
USE_HARDWARE = False  
INVERT_MOTOR = True
INVERT_STEER = True

WHEEL_RADIUS = 0.032465  
GEAR_RATIO = 1.0

AREA_W = 10.0
AREA_H = 6.0
WHEELBASE = 0.2365        
BODY_LEN = 0.38415        
BODY_WID = 0.09622        
WHEEL_LEN = 0.06493       
WHEEL_WID = 0.033         

V_BASE = 0.8
DT = 0.05
LOOKAHEAD = 0.8          
MAX_STEER_DEG = 60.0
MAX_STEER_RATE = math.radians(60.0)
SERVO_CENTER = 75
SERVO_LEFT_LIMIT = 55
SERVO_RIGHT_LIMIT = 110
MOTOR_PWM_MAX = 180
GOAL_TOL = 0.12
WAYPOINT_TOL = 0.18
MAX_SEG_LOOK = 2

# --- SEED FILLER CONFIG ---
PLANT_SPACING = 0.8
MIN_STRAIGHT_LEN = 1.2
CORNER_AVOID_DIST = 0.6
STEPPER_MAX_STEPS = 400
STEPPER_DOWN_TIME = 0.5
STEPPER_UP_TIME = 0.4
STEPPER_HOLD_TIME = 0.15
STEPPER_WAIT_TIME = 0.2

# --- MAX TRAJECTORY POINTS (memory management) ---
MAX_TRAJ_POINTS = 5000

# --- SEED FILLER STATES ---
SF_MOVING = "MOVING"
SF_APPROACHING = "APPROACHING"
SF_STOPPING = "STOPPING"
SF_STEPPER_DOWN = "STEPPER_DOWN"
SF_HOLDING = "HOLDING"
SF_STEPPER_UP = "STEPPER_UP"
SF_WAITING = "WAITING"
SF_DONE = "DONE"
SF_IDLE = "IDLE"

HISTORY_LEN = 100
history_v = deque([0]*HISTORY_LEN, maxlen=HISTORY_LEN)
history_w = deque([0]*HISTORY_LEN, maxlen=HISTORY_LEN)
history_servo = deque([0]*HISTORY_LEN, maxlen=HISTORY_LEN)
history_stepper = deque([0]*HISTORY_LEN, maxlen=HISTORY_LEN)

# --- GLOBAL FLAGS ---
is_running = False
robot_online = False
robot_mode = "IDLE" 
record_session = False
session_data = [] 

# --- SEED FILLER STATE ---
sf_state = SF_IDLE
plant_points = []
current_plant_idx = 0
planted_points = []
plant_count = 0
total_plant_points = 0
stepper_pos = 0.0
stepper_timer = 0.0
stop_timer = 0.0
current_seg = 0
current_pwm = 0
servo_deg = SERVO_CENTER
motor_dir = 0
last_look_point = None  # FIX: Track lookahead point for visualization

# --- NETWORK SESSION ---
http_session = requests.Session()

# ----- Helper Functions -----
def send_to_esp(command_string):
    if USE_HARDWARE:
        try:
            http_session.get(f"{ROBOT_IP}/cmd", params={'c': command_string}, timeout=0.15)
        except:
            pass

def get_odometry():
    global robot_online
    if not USE_HARDWARE:
        robot_online = False
        return 0.0, 0.0, 75.0, 0.0
    
    try:
        r = http_session.get(f"{ROBOT_IP}/data", timeout=0.15)
        if r.status_code == 200:
            data = r.json()
            rpm1 = int(data.get('rpm1', 0))
            rpm2 = int(data.get('rpm2', 0))
            servo_current = int(data.get('servo', 75))
            stepper_raw = int(data.get('stepper', 0))
            
            v_left = (rpm1 * 2 * math.pi * WHEEL_RADIUS) / (60 * GEAR_RATIO)
            v_right = (rpm2 * 2 * math.pi * WHEEL_RADIUS) / (60 * GEAR_RATIO)
            v_linear = (v_left + v_right) / 2.0
            w_angular = (v_right - v_left) / WHEELBASE
            
            stepper_norm = min(1.0, max(0.0, stepper_raw / STEPPER_MAX_STEPS)) if STEPPER_MAX_STEPS > 0 else 0.0
            
            robot_online = True
            return v_linear, w_angular, servo_current, stepper_norm
        else:
            robot_online = False
    except Exception as e:
        robot_online = False
    return 0.0, 0.0, 75.0, 0.0

def generate_seed_filler_path():
    pts = []
    rows_y = [0.5, 1.5, 2.5, 3.5, 4.5, 5.5]
    for i, y in enumerate(rows_y):
        if i % 2 == 0:
            pts.append((0.5, y))
            pts.append((9.5, y))
        else:
            pts.append((9.5, y))
            pts.append((0.5, y))
    return np.array(pts, dtype=float)

def calculate_turn_angles(path):
    n = len(path)
    angles = []
    for i in range(n - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        angles.append(math.atan2(dy, dx))
    
    turn_angles = [0.0]
    for i in range(1, len(angles)):
        diff = angles[i] - angles[i-1]
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        turn_angles.append(abs(diff))
    turn_angles.append(0.0)
    return turn_angles

def is_near_corner(path_idx, turn_angles, path, threshold):
    n = len(path)
    for i in range(max(0, path_idx - 2), min(n, path_idx + 3)):
        if i < len(turn_angles) and turn_angles[i] > 0.1:
            dist = math.hypot(path[path_idx][0] - path[i][0], path[path_idx][1] - path[i][1])
            if dist < threshold:
                return True
    return False

def generate_plant_points_on_path(path, spacing, corner_avoid):
    turn_angles = calculate_turn_angles(path)
    plant_pts = []
    n = len(path)
    
    for i in range(n - 1):
        x1, y1 = path[i]
        x2, y2 = path[i+1]
        seg_len = math.hypot(x2-x1, y2-y1)
        
        if seg_len < MIN_STRAIGHT_LEN:
            continue
        
        dx, dy = x2-x1, y2-y1
        margin = corner_avoid
        
        if margin * 2 >= seg_len:
            continue
            
        effective_len = seg_len - (margin * 2)
        num_plants = int(effective_len / spacing)
        if num_plants <= 0:
            continue
        
        for j in range(num_plants):
            t = (margin + (j + 0.5) * spacing) / seg_len
            px, py = x1 + t*dx, y1 + t*dy
            plant_pts.append((px, py))
            
    return plant_pts

# Generate path and plant points
path = generate_seed_filler_path()
plant_points = generate_plant_points_on_path(path, PLANT_SPACING, CORNER_AVOID_DIST)
total_plant_points = len(plant_points)

def find_lookahead_point_from(path, pos, lookahead, start_seg, heading, max_seg_look=10):
    px, py = pos
    hx, hy = heading
    n = len(path)
    candidates = []
    end_seg = min(n - 1, start_seg + max_seg_look)
    
    if start_seg >= n - 1:
        return np.array(path[-1]), n - 2, 1.0

    for i in range(start_seg, end_seg):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        a = dx * dx + dy * dy
        if a < 1e-12: continue
        b = 2 * (dx * (x1 - px) + dy * (y1 - py))
        c = (x1 - px) ** 2 + (y1 - py) ** 2 - lookahead ** 2
        disc = b * b - 4 * a * c
        if disc < 0: continue
        sqrt_disc = math.sqrt(disc)
        t_candidates = [(-b - sqrt_disc) / (2 * a), (-b + sqrt_disc) / (2 * a)]
        for t in t_candidates:
            if 0.0 <= t <= 1.0:
                lx, ly = x1 + t * dx, y1 + t * dy
                if (lx - px) * hx + (ly - py) * hy > 1e-9:
                    candidates.append((i, t, np.array([lx, ly])))
    if candidates:
        i_best, t_best, p_best = min(candidates, key=lambda k: (k[0], k[1]))
        return p_best, i_best, t_best
        
    try:
        if start_seg < n - 1:
            x1, y1 = path[start_seg]
            x2, y2 = path[start_seg + 1]
            dx, dy = x2 - x1, y2 - y1
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-12:
                return find_lookahead_point_from(path, pos, lookahead, start_seg + 1, heading, max_seg_look)
            seg_len = math.sqrt(seg_len2)
            t0 = ((px - x1) * dx + (py - y1) * dy) / seg_len2
            t_tar = min(1.0, max(0.0, t0) + lookahead / seg_len)
            vx, vy = x1 + t_tar * dx, y1 + t_tar * dy
            if (vx - px) * hx + (vy - py) * hy <= 0.0:
                eps = 1e-3
                t_tar = min(1.0, t_tar + eps)
                vx, vy = x1 + t_tar * dx, y1 + t_tar * dy
            return np.array([vx, vy]), start_seg, t_tar
    except:
        pass 
        
    return np.array(path[-1]), n - 2, 1.0

def pure_pursuit_steer_from(path, rear_pos, yaw, lookahead, wheelbase, max_steer_rad, start_seg):
    global last_look_point  # FIX: Track for visualization
    try:
        heading = (math.cos(yaw), math.sin(yaw))
        look_pt, seg_idx, seg_t = find_lookahead_point_from(path, rear_pos, lookahead, start_seg, heading)
        last_look_point = look_pt  # FIX: Store for visualization
        
        dx = look_pt[0] - rear_pos[0]
        dy = look_pt[1] - rear_pos[1]
        x_f =  math.cos(yaw)*dx + math.sin(yaw)*dy
        y_l = -math.sin(yaw)*dx + math.cos(yaw)*dy
        
        if not (math.isfinite(x_f) and math.isfinite(y_l) and math.isfinite(dx) and math.isfinite(dy)):
            return 0.0, look_pt, seg_idx

        if x_f <= 0.0:
            look_pt, seg_idx, seg_t = find_lookahead_point_from(
                path, rear_pos, lookahead, min(start_seg+1, len(path)-2), heading)
            last_look_point = look_pt  # FIX: Update stored point
            dx = look_pt[0] - rear_pos[0]
            dy = look_pt[1] - rear_pos[1]
            x_f =  math.cos(yaw)*dx + math.sin(yaw)*dy
            y_l = -math.sin(yaw)*dx + math.cos(yaw)*dy

        if abs(x_f) < 1e-9 and abs(y_l) < 1e-9:
            return 0.0, look_pt, seg_idx

        curvature = 2.0 * y_l / (lookahead**2)
        steer = math.atan(curvature * wheelbase)
        steer = max(-max_steer_rad, min(max_steer_rad, steer))
        return steer, look_pt, seg_idx
    except Exception as e:
        print(f"PP Error: {e}")
        # FIX: Use rear_pos instead of global variables
        return 0.0, np.array([rear_pos[0], rear_pos[1]]), start_seg

def map_steering_to_servo(steer_rad, max_steer_rad):
    norm_steer = steer_rad / max_steer_rad if max_steer_rad != 0 else 0
    if INVERT_STEER:
        norm_steer = -norm_steer
    if norm_steer < 0:
        angle = SERVO_CENTER + (norm_steer * (SERVO_CENTER - SERVO_LEFT_LIMIT))
    else:
        angle = SERVO_CENTER + (norm_steer * (SERVO_RIGHT_LIMIT - SERVO_CENTER))
    return int(max(SERVO_LEFT_LIMIT, min(SERVO_RIGHT_LIMIT, angle)))

rear_x, rear_y, yaw = 0.5, 0.5, 0.0
steer = 0.0
prev_steer = 0.0
trajectory = [(rear_x, rear_y)]
path_done = False

# ----- Plotting Setup -----
fig = plt.figure(figsize=(14, 8))

ax = fig.add_axes([0.02, 0.05, 0.55, 0.9]) 
ax.set_aspect('equal')
ax.set_xlim(-1, AREA_W + 1)
ax.set_ylim(-1, AREA_H + 1)
ax.set_title("Seed Filler Navigation Map")

for gx in np.arange(0, AREA_W+1, 1.0): ax.axvline(gx, color='lightgray', linewidth=0.6)
for gy in np.arange(0, AREA_H+1, 1.0): ax.axhline(gy, color='lightgray', linewidth=0.6)
ax.add_patch(patches.Rectangle((0,0), AREA_W, AREA_H, fill=False, linestyle='--', color='gray'))

# Path dan titik tanam
ax.plot(path[:,0], path[:,1], 'c-', linewidth=2, label='Path', alpha=0.6)
ax.scatter(path[:,0], path[:,1], c='cyan', s=30, alpha=0.8, zorder=3)

if plant_points:
    pp = np.array(plant_points)
    plant_scatter = ax.scatter(pp[:,0], pp[:,1], c='orange', s=60, marker='o', alpha=0.6, 
                               label=f'Plant Points ({len(plant_points)})', zorder=4)
else:
    plant_scatter = ax.scatter([], [], c='orange', s=60, marker='o', alpha=0.6, zorder=4)

planted_scatter = ax.scatter([], [], c='green', s=100, marker='v', alpha=0.9, label='Planted', zorder=5)
current_target_marker, = ax.plot([], [], 'r*', markersize=18, label='Current Target', zorder=6)

traj_line, = ax.plot([], [], 'b-', linewidth=2, label='Trajectory', alpha=0.7)
look_circle = patches.Circle((rear_x, rear_y), LOOKAHEAD, fill=False, linestyle=':', edgecolor='blue', alpha=0.4)
ax.add_patch(look_circle)
look_dot, = ax.plot([], [], 'ro', ms=5, label='Lookahead')  # FIX: Added label
heading_line, = ax.plot([], [], 'k-', lw=2)

half_w = BODY_WID/2.0
rear_ext = 0.05 
body_local = np.array([[-rear_ext, half_w], [BODY_LEN-rear_ext, half_w], 
                       [BODY_LEN-rear_ext, -half_w], [-rear_ext, -half_w]])
body_patch = patches.Polygon(body_local, closed=True, facecolor='gray', alpha=0.7) 
ax.add_patch(body_patch)

wheel_local = np.array([[-WHEEL_LEN/2, -WHEEL_WID/2], [WHEEL_LEN/2, -WHEEL_WID/2], 
                        [WHEEL_LEN/2, WHEEL_WID/2], [-WHEEL_LEN/2, WHEEL_WID/2]])
wheel_centers = [np.array([0.0, half_w]), np.array([0.0, -half_w]), 
                 np.array([WHEELBASE, half_w]), np.array([WHEELBASE, -half_w])]
wheel_patches = [patches.Polygon(wheel_local.copy(), closed=True, facecolor='black') for _ in wheel_centers]
for wp in wheel_patches: ax.add_patch(wp)

# Visual stepper
stepper_vis_line, = ax.plot([], [], 'm-', linewidth=4, alpha=0.8, zorder=8)
stepper_vis_head = ax.scatter([], [], c='magenta', s=100, marker='s', alpha=0.8, zorder=9, edgecolors='white')

ax.legend(loc='upper right', fontsize=7, ncol=2)

# --- UI CONTROLS ---
status_label_text = fig.text(0.05, 0.97, "ROBOT STATUS: IDLE", fontsize=11, fontweight='bold', va='center', color='black')

ax_ip = plt.axes([0.05, 0.92, 0.15, 0.03])
text_box = TextBox(ax_ip, 'IP: ', initial=ROBOT_IP)

def submit_ip(text):
    global ROBOT_IP, http_session
    ROBOT_IP = text
    http_session = requests.Session()
    print(f"IP set to: {ROBOT_IP}")

text_box.on_submit(submit_ip)

ax_set_ip = plt.axes([0.22, 0.92, 0.08, 0.03])
btn_set_ip = Button(ax_set_ip, 'Set IP')
btn_set_ip.on_clicked(lambda event: submit_ip(text_box.text))

ax_hw = plt.axes([0.05, 0.85, 0.10, 0.04])
btn_hw = Button(ax_hw, 'START HW', color='lightgreen', hovercolor='lime')

ax_sim = plt.axes([0.16, 0.85, 0.10, 0.04])
btn_sim = Button(ax_sim, 'START SIM', color='lightblue', hovercolor='skyblue')

ax_stop = plt.axes([0.27, 0.85, 0.08, 0.04])
btn_stop = Button(ax_stop, 'STOP', color='salmon', hovercolor='red')

ax_reset = plt.axes([0.36, 0.85, 0.08, 0.04])
btn_reset = Button(ax_reset, 'RESET', color='orange', hovercolor='yellow')

ax_log = plt.axes([0.45, 0.85, 0.08, 0.04])
btn_log = Button(ax_log, 'LOG: OFF', color='lightgray', hovercolor='white')

# Callbacks
def start_hw(event):
    global is_running, USE_HARDWARE, robot_mode, session_data, sf_state
    is_running = True
    USE_HARDWARE = True
    robot_mode = "HW"
    if sf_state == SF_IDLE:
        sf_state = SF_MOVING
    if record_session:
        session_data = [] 
    print("Mode: HARDWARE")

def start_sim(event):
    global is_running, USE_HARDWARE, robot_mode, session_data, sf_state
    is_running = True
    USE_HARDWARE = False
    robot_mode = "SIM"
    if sf_state == SF_IDLE:
        sf_state = SF_MOVING
    if record_session:
        session_data = [] 
    print("Mode: SIMULATION")

def stop_sys(event):
    global is_running, robot_mode, sf_state
    is_running = False
    robot_mode = "IDLE"
    sf_state = SF_IDLE
    send_to_esp("STOP")
    send_to_esp("STEPPER,0")
    if record_session:
        save_session_data()

def reset_sim(event):
    global is_running, rear_x, rear_y, yaw, steer, prev_steer, trajectory, path_done, robot_mode
    global sf_state, current_plant_idx, planted_points, plant_count, stepper_pos, stepper_timer
    global stop_timer, current_seg, current_pwm, servo_deg, motor_dir, last_look_point
    
    is_running = False
    send_to_esp("STOP")
    send_to_esp("STEPPER,0")
    
    rear_x = 0.5; rear_y = 0.5; yaw = 0.0
    steer = 0.0; prev_steer = 0.0
    path_done = False; current_seg = 0
    trajectory = [(rear_x, rear_y)]
    robot_mode = "IDLE"
    
    sf_state = SF_IDLE; current_plant_idx = 0; planted_points = []
    plant_count = 0; stepper_pos = 0.0; stepper_timer = 0.0
    stop_timer = 0.0; current_pwm = 0; servo_deg = SERVO_CENTER; motor_dir = 0
    last_look_point = None  # FIX: Reset lookahead point
    
    planted_scatter.set_offsets(np.empty((0, 2)))
    current_target_marker.set_data([], [])
    look_dot.set_data([], [])  # FIX: Clear lookahead dot
    
    print("System Reset.")

def toggle_log(event):
    global record_session
    record_session = not record_session
    if record_session:
        btn_log.label.set_text("LOG: ON"); btn_log.color = 'red'
    else:
        btn_log.label.set_text("LOG: OFF"); btn_log.color = 'lightgray'

def save_session_data():
    global record_session, session_data
    if not session_data:
        return
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fname_csv = f"filler_{timestamp}.csv"
    try:
        with open(fname_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["TimeStep", "Vel", "AngVel", "Steer", "Stepper", "State"])
            for i, row in enumerate(session_data):
                w.writerow([i, row[0], row[1], row[2], row[3], row[4]])
        print(f"Saved: {fname_csv}")
    except Exception as e:
        print(f"Error: {e}")
    record_session = False
    btn_log.label.set_text('LOG: OFF')
    btn_log.color = 'lightgray'
    session_data = []

btn_hw.on_clicked(start_hw)
btn_sim.on_clicked(start_sim)
btn_stop.on_clicked(stop_sys)
btn_reset.on_clicked(reset_sim)
btn_log.on_clicked(toggle_log)

# --- PANEL KANAN ---
term_ax = fig.add_axes([0.60, 0.58, 0.37, 0.36])
term_ax.axis('off')
terminal_text = term_ax.text(0, 1.0, "", va='top', ha='left', fontsize=10, family='monospace', 
                             transform=term_ax.transAxes)

ax_vel = fig.add_axes([0.60, 0.465, 0.37, 0.09])
ax_vel.set_title("Linear Velocity (m/s)", fontsize=9)
ax_vel.grid(True, linestyle='--', alpha=0.6)
ax_vel.set_xlim(0, HISTORY_LEN)
ax_vel.set_xticklabels([])
ax_vel.tick_params(axis='both', which='major', labelsize=7)
line_v, = ax_vel.plot([], [], 'b-', lw=1.5)

ax_ang = fig.add_axes([0.60, 0.355, 0.37, 0.09])
ax_ang.set_title("Angular Velocity (rad/s)", fontsize=9)
ax_ang.grid(True, linestyle='--', alpha=0.6)
ax_ang.set_xlim(0, HISTORY_LEN)
ax_ang.set_xticklabels([])
ax_ang.tick_params(axis='both', which='major', labelsize=7)
line_w, = ax_ang.plot([], [], 'r-', lw=1.5)

ax_steer = fig.add_axes([0.60, 0.245, 0.37, 0.09])
ax_steer.set_title("Steering Angle (deg)", fontsize=9)
ax_steer.grid(True, linestyle='--', alpha=0.6)
ax_steer.set_xlim(0, HISTORY_LEN)
ax_steer.set_xticklabels([])
ax_steer.tick_params(axis='both', which='major', labelsize=7)
line_s, = ax_steer.plot([], [], 'g-', lw=1.5)

ax_step = fig.add_axes([0.60, 0.135, 0.37, 0.09])
ax_step.set_title("Stepper Pos (0=Up 1=Down)", fontsize=9)
ax_step.grid(True, linestyle='--', alpha=0.6)
ax_step.set_xlim(0, HISTORY_LEN)
ax_step.set_ylim(-0.1, 1.1)
ax_step.set_xticklabels([])
ax_step.tick_params(axis='both', which='major', labelsize=7)
line_step, = ax_step.plot([], [], 'm-', lw=1.5)

# ----- SEED FILLER STATE MACHINE -----
def update_seed_filler():
    global sf_state, stepper_pos, stepper_timer, stop_timer, current_seg
    global current_plant_idx, planted_points, plant_count
    global rear_x, rear_y, yaw, is_running, path_done
    
    if current_plant_idx >= total_plant_points:
        sf_state = SF_DONE; path_done = True; is_running = False
        send_to_esp("STOP"); send_to_esp("STEPPER,0")
        return 0.0, 0.0, False
    
    tx, ty = plant_points[current_plant_idx]
    dist = math.hypot(tx - rear_x, ty - rear_y)
    
    if sf_state == SF_MOVING:
        if dist < 0.6:
            sf_state = SF_APPROACHING
        desired, look_pt, seg = pure_pursuit_steer_from(
            path, (rear_x, rear_y), yaw, LOOKAHEAD, WHEELBASE, max_steer, current_seg)
        current_seg = max(current_seg, seg)
        return V_BASE, desired, True
        
    elif sf_state == SF_APPROACHING:
        desired, look_pt, seg = pure_pursuit_steer_from(
            path, (rear_x, rear_y), yaw, LOOKAHEAD, WHEELBASE, max_steer, current_seg)
        current_seg = max(current_seg, seg)
        v_slow = V_BASE * max(0.2, dist / 0.6)
        if dist < 0.12:
            sf_state = SF_STOPPING; stop_timer = 0
            return 0.0, 0.0, True
        return v_slow, desired, True
        
    elif sf_state == SF_STOPPING:
        stop_timer += DT
        if stop_timer > 0.2:
            sf_state = SF_STEPPER_DOWN; stepper_timer = 0
            if USE_HARDWARE: send_to_esp(f"STEPPER,{STEPPER_MAX_STEPS}")
        return 0.0, 0.0, True
        
    elif sf_state == SF_STEPPER_DOWN:
        stepper_timer += DT
        stepper_pos = min(1.0, stepper_timer / STEPPER_DOWN_TIME)
        if stepper_timer >= STEPPER_DOWN_TIME:
            stepper_pos = 1.0; sf_state = SF_HOLDING; stepper_timer = 0
            if USE_HARDWARE: send_to_esp("SEED_DROP")
        return 0.0, 0.0, True
        
    elif sf_state == SF_HOLDING:
        stepper_timer += DT
        if stepper_timer >= STEPPER_HOLD_TIME:
            sf_state = SF_STEPPER_UP; stepper_timer = 0
            if USE_HARDWARE: send_to_esp("STEPPER,0")
        return 0.0, 0.0, True
        
    elif sf_state == SF_STEPPER_UP:
        stepper_timer += DT
        stepper_pos = max(0.0, 1.0 - stepper_timer / STEPPER_UP_TIME)
        if stepper_timer >= STEPPER_UP_TIME:
            stepper_pos = 0.0; sf_state = SF_WAITING; stepper_timer = 0
        return 0.0, 0.0, True
        
    elif sf_state == SF_WAITING:
        stepper_timer += DT
        if stepper_timer >= STEPPER_WAIT_TIME:
            planted_points.append((tx, ty))
            plant_count = len(planted_points)
            current_plant_idx += 1
            sf_state = SF_MOVING
        return 0.0, 0.0, True
    
    return 0.0, 0.0, False

# ----- Animation Update -----
max_steer = math.radians(MAX_STEER_DEG)

def update(frame):
    global rear_x, rear_y, yaw, steer, prev_steer, path_done, current_seg
    global stepper_pos, current_pwm, servo_deg, motor_dir, trajectory

    # Status display
    if sf_state == SF_DONE:
        body_patch.set_facecolor('green')
        status_label_text.set_text("FILLER: COMPLETE ✓")
        status_label_text.set_color('green')
    elif sf_state in [SF_STEPPER_DOWN, SF_HOLDING, SF_STEPPER_UP]:
        body_patch.set_facecolor('magenta')
        status_label_text.set_text(f"FILLER: {sf_state}")
        status_label_text.set_color('darkviolet')
    elif sf_state in [SF_STOPPING, SF_WAITING, SF_APPROACHING]:
        body_patch.set_facecolor('orange')
        status_label_text.set_text(f"FILLER: {sf_state}")
        status_label_text.set_color('darkorange')
    elif sf_state == SF_MOVING:
        body_patch.set_facecolor('gold')
        status_label_text.set_text(f"FILLER: {sf_state}")
        status_label_text.set_color('goldenrod')
    elif robot_mode == "SIM":
        body_patch.set_facecolor('dodgerblue')
        status_label_text.set_text("ROBOT STATUS: SIM READY")
        status_label_text.set_color('dodgerblue')
    elif robot_mode == "HW":
        if robot_online:
            body_patch.set_facecolor('lime')
            status_label_text.set_text("ROBOT STATUS: HW ONLINE")
            status_label_text.set_color('green')
        else:
            body_patch.set_facecolor('red')
            status_label_text.set_text("ROBOT STATUS: HW OFFLINE")
            status_label_text.set_color('red')
    else: 
        body_patch.set_facecolor('gray')
        status_label_text.set_text("ROBOT STATUS: IDLE")
        status_label_text.set_color('black')

    if not is_running:
        terminal_text.set_text("SYSTEM STOPPED\nPress START SIM or START HW.")
        return (traj_line, look_circle, look_dot, heading_line, body_patch, 
                *wheel_patches, terminal_text, line_v, line_w, line_s, line_step,
                plant_scatter, planted_scatter, current_target_marker, stepper_vis_line, stepper_vis_head)

    if path_done:
        send_to_esp("STOP")
        if record_session: save_session_data()
        return (traj_line, look_circle, look_dot, heading_line, body_patch,
                *wheel_patches, terminal_text, line_v, line_w, line_s, line_step,
                plant_scatter, planted_scatter, current_target_marker, stepper_vis_line, stepper_vis_head)

    try:
        real_v, real_w, real_servo, real_stepper = get_odometry()
        
        # Update stepper pos dari hardware
        if USE_HARDWARE and robot_online:
            stepper_pos = real_stepper

        history_v.append(real_v)
        history_w.append(real_w)
        history_servo.append(real_servo - SERVO_CENTER)
        history_stepper.append(stepper_pos)

        if is_running and record_session:
            session_data.append([real_v, real_w, real_servo - SERVO_CENTER, stepper_pos, sf_state])

        x_data = np.arange(len(history_v))
        line_v.set_data(x_data, history_v)
        ax_vel.set_ylim(min(history_v)-0.1, max(max(history_v)+0.1, 0.5))
        line_w.set_data(x_data, history_w)
        ax_ang.set_ylim(min(history_w)-0.5, max(max(history_w)+0.5, 0.5))
        line_s.set_data(x_data, history_servo)
        ax_steer.set_ylim(min(history_servo)-10, max(max(history_servo)+10, 10))
        line_step.set_data(x_data, history_stepper)

        # State machine
        v_cmd, steer_cmd, motor_on = update_seed_filler()
        
        # Rate limit steering
        max_ds = MAX_STEER_RATE * DT
        ds = steer_cmd - prev_steer
        if ds > max_ds: steer_cmd = prev_steer + max_ds
        elif ds < -max_ds: steer_cmd = prev_steer - max_ds
        prev_steer = steer_cmd
        steer = steer_cmd
        
        # Hitung perintah motor
        servo_deg = map_steering_to_servo(steer, max_steer)
        steer_factor = max(0.3, 1.0 - (abs(steer) / max_steer) * 0.6) if max_steer != 0 else 1.0
        current_pwm = int(MOTOR_PWM_MAX * steer_factor * (v_cmd / V_BASE)) if V_BASE != 0 else 0
        motor_dir = 0 if INVERT_MOTOR else 1
        
        # Kirim ke hardware
        if USE_HARDWARE and motor_on and v_cmd > 0.01:
            send_to_esp(f"SERVO,{servo_deg}")
            send_to_esp(f"MOTOR,{current_pwm},{motor_dir},{current_pwm},{motor_dir}")
        elif USE_HARDWARE:
            send_to_esp("STOP")
        
        # Simulasi posisi
        if not USE_HARDWARE and motor_on and v_cmd > 0.01:
            omega = (v_cmd / WHEELBASE) * math.tan(steer) if WHEELBASE != 0 else 0
            if not (math.isnan(omega) or math.isinf(omega)):
                rear_x += v_cmd * math.cos(yaw) * DT
                rear_y += v_cmd * math.sin(yaw) * DT
                yaw += omega * DT
        
        trajectory.append((rear_x, rear_y))
        
        # FIX: Limit trajectory length to prevent memory issues
        if len(trajectory) > MAX_TRAJ_POINTS:
            trajectory = trajectory[-MAX_TRAJ_POINTS:]
        
        # Update visual tanaman
        if planted_points:
            planted_scatter.set_offsets(np.array(planted_points))
        if current_plant_idx < total_plant_points:
            tx, ty = plant_points[current_plant_idx]
            current_target_marker.set_data([tx], [ty])
        else:
            current_target_marker.set_data([], [])
        
        # FIX: Update lookahead point visualization
        if last_look_point is not None:
            look_dot.set_data([last_look_point[0]], [last_look_point[1]])
        else:
            look_dot.set_data([], [])
        
        # Update visual stepper
        sb_x = rear_x + BODY_LEN*0.6*math.cos(yaw)
        sb_y = rear_y + BODY_LEN*0.6*math.sin(yaw)
        ext = 0.4 * stepper_pos
        st_x = sb_x - ext*math.sin(yaw)
        st_y = sb_y + ext*math.cos(yaw)
        stepper_vis_line.set_data([sb_x, st_x], [sb_y, st_y])
        stepper_vis_head.set_offsets([[st_x, st_y]])

        # Update graphics robot
        tr = np.array(trajectory)
        traj_line.set_data(tr[:,0], tr[:,1])
        look_circle.center = (rear_x, rear_y)
        
        head_len = 0.2
        hx = rear_x + head_len * math.cos(yaw)
        hy = rear_y + head_len * math.sin(yaw)
        heading_line.set_data([rear_x, hx], [rear_y, hy])
        
        t_body = transforms.Affine2D().rotate(yaw).translate(rear_x, rear_y)
        body_patch.set_transform(t_body + ax.transData)

        for idx, wpatch in enumerate(wheel_patches):
            center = wheel_centers[idx]
            cx = rear_x + math.cos(yaw)*center[0] - math.sin(yaw)*center[1]
            cy = rear_y + math.sin(yaw)*center[0] + math.cos(yaw)*center[1]
            if idx < 2: rot = yaw
            else: rot = yaw + steer
            R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
            corners = (wheel_local @ R.T) + np.array([cx, cy])
            wpatch.set_xy(corners)

        # Terminal text
        log_status = "ON" if record_session else "OFF"
        progress = 100 * plant_count / max(1, total_plant_points)
        
        # FIX: Better formatted progress bar
        bar_len = 20
        filled = int(bar_len * plant_count / max(1, total_plant_points))
        bar = '█' * filled + '░' * (bar_len - filled)
        
        terminal_str = (
            f"MODE: SEED FILLER\n"
            f"STATE: {sf_state}\n"
            f"IP: {ROBOT_IP}\n"
            f"LOGGING: {log_status}\n"
            f"{'─'*35}\n"
            f"[FILLER STATUS]\n"
            f"Planted  : {plant_count}/{total_plant_points}\n"
            f"[{bar}] {progress:.1f}%\n"
            f"\n"
            f"[STEPPER]\n"
            f"Position : {stepper_pos*100:.0f}% ({'DOWN' if stepper_pos>0.5 else 'UP'})\n"
            f"Steps    : {int(stepper_pos*STEPPER_MAX_STEPS)}/{STEPPER_MAX_STEPS}\n"
            f"Timer    : {stepper_timer:.2f}s\n"
            f"\n"
            f"[COMMAND SENT]\n"
            f"Servo    : {servo_deg}°\n"
            f"Motor PWM: {current_pwm}\n"
            f"Direction: {'FWD' if motor_dir==1 else 'REV'}\n"
            f"\n"
            f"[ODOMETRY]\n"
            f"Lin. Vel : {real_v:.2f} m/s\n"
            f"Ang. Vel : {real_w:.2f} rad/s\n"
            f"Servo Pos: {real_servo}°\n"
            f"\n"
            f"[POSITION]\n"
            f"X, Y     : ({rear_x:.2f}, {rear_y:.2f})\n"
            f"Yaw      : {math.degrees(yaw):.1f}°\n"
            f"Path Seg : {current_seg}"
        )
        terminal_text.set_text(terminal_str)
        
    except Exception as e:
        print(f"ERROR: {e}")
    
    return (traj_line, look_circle, look_dot, heading_line, body_patch,
            *wheel_patches, terminal_text, line_v, line_w, line_s, line_step,
            plant_scatter, planted_scatter, current_target_marker, stepper_vis_line, stepper_vis_head)

print(f"\n{'='*45}")
print("  SEED FILLER NAVIGATION SYSTEM")
print(f"{'='*45}")
print(f"  Total Plant Points : {total_plant_points}")
print(f"  Path Waypoints     : {len(path)}")
print(f"  Max Traj Points    : {MAX_TRAJ_POINTS}")
print(f"{'='*45}\n")

ani = animation.FuncAnimation(fig, update, frames=10000, interval=int(DT*1000), blit=False, repeat=False)
plt.show()
