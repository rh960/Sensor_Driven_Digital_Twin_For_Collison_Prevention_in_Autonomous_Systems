"""
LIVE SENSOR VISUALIZATION
Student: Raffay Hassan (M00944822)

Displays real-time visualization during scenario:
- Left: Camera feed (RGB image)
- Right: LiDAR point cloud (top-down view with color by height)
- Bottom: Sensor fusion data (distance, TTC, action)

Shows exactly what sensors are detecting in real-time!
"""

import carla
import time
import os
import csv
import numpy as np
from datetime import datetime
import cv2
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HOST = "localhost"
PORT = 2000
TOWN = "Town04"
FIXED_DT = 0.05
DURATION_SEC = 70

CAM_W, CAM_H = 1920, 1080
SENSOR_TICK = 0.05

BRAKE_DISTANCE = 8.0
AVOID_DISTANCE = 15.0
TTC_CRITICAL = 2.0
ASSOC_GATE_M = 12.0
SENTINEL_DIST = 999.0

def make_run_dir(tag):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = f"/home/rh960/carla_env/dt_logs/{ts}_{tag}"
    os.makedirs(run_dir, exist_ok=True)
    return run_dir

def get_speed(v):
    vel = v.get_velocity()
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

def process_lidar(meas):
    if meas is None or meas.raw_data is None or len(meas.raw_data) == 0:
        return SENTINEL_DIST, 0.0, None

    pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
    mask = (pts[:, 0] > 2.0) & (np.abs(pts[:, 1]) < 2.0) & (pts[:, 0] < 120.0)
    
    if not mask.any():
        return SENTINEL_DIST, 0.0, pts

    valid = pts[mask]
    distances = np.sqrt(valid[:, 0]**2 + valid[:, 1]**2)
    i = int(np.argmin(distances))
    return float(distances[i]), float(valid[i, 1]), pts

def calculate_ttc(distance, relative_velocity):
    if relative_velocity <= 0.1:
        return SENTINEL_DIST
    return distance / relative_velocity

def visualize_lidar(pts, ax, fused_dist, action):
    """Create top-down LiDAR visualization"""
    ax.clear()
    
    if pts is None or len(pts) == 0:
        ax.text(0.5, 0.5, 'No LiDAR Data', ha='center', va='center', fontsize=20)
        return
    
    # Top-down view: X (forward), Y (left/right)
    x = pts[:, 0]  # Forward
    y = pts[:, 1]  # Lateral
    z = pts[:, 2]  # Height
    
    # Color by height
    colors = plt.cm.viridis((z - z.min()) / (z.max() - z.min() + 0.001))
    
    # Plot points
    ax.scatter(x, y, c=colors, s=1, alpha=0.6)
    
    # Draw ego vehicle (rectangle at origin)
    ego_rect = Rectangle((-2, -1), 4, 2, fill=True, color='white', alpha=0.8, 
                         edgecolor='blue', linewidth=2)
    ax.add_patch(ego_rect)
    ax.text(0, 0, 'EGO', ha='center', va='center', fontsize=8, fontweight='bold')
    
    # Draw detection zone (front ROI)
    roi_rect = Rectangle((2, -2), 118, 4, fill=False, edgecolor='yellow', 
                         linewidth=1, linestyle='--', alpha=0.5)
    ax.add_patch(roi_rect)
    
    # Draw detected obstacle distance
    if fused_dist < SENTINEL_DIST:
        # Draw line to detected obstacle
        ax.plot([0, fused_dist], [0, 0], 'r-', linewidth=3, label=f'Detected: {fused_dist:.1f}m')
        # Draw circle at obstacle
        circle = plt.Circle((fused_dist, 0), 2, color='red', fill=False, linewidth=2)
        ax.add_patch(circle)
        
        # Safety zones
        if fused_dist < BRAKE_DISTANCE:
            zone_color = 'red'
            zone_label = 'CRITICAL'
        elif fused_dist < AVOID_DISTANCE:
            zone_color = 'orange'
            zone_label = 'WARNING'
        else:
            zone_color = 'green'
            zone_label = 'SAFE'
        
        ax.text(fused_dist, 3, f'{zone_label}\n{fused_dist:.1f}m', 
               ha='center', fontsize=10, color=zone_color, fontweight='bold',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Action text
    action_color = {'EMERGENCY_BRAKE': 'red', 'LANE_CHANGE': 'orange', 
                   'AVOIDING': 'yellow', 'DRIVING': 'green', 
                   'STOPPED': 'red', 'STRAIGHTENING': 'blue', 'RESUMED': 'green'}
    
    ax.text(60, -8, f'ACTION: {action}', ha='center', fontsize=12, 
           fontweight='bold', color=action_color.get(action, 'black'),
           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    
    ax.set_xlim(-5, 120)
    ax.set_ylim(-10, 10)
    ax.set_xlabel('Forward (m)', fontsize=10)
    ax.set_ylabel('Lateral (m)', fontsize=10)
    ax.set_title('LiDAR Point Cloud (Top-Down View)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

def main():
    run_dir = make_run_dir("live_viz")
    
    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)
    world = client.get_world()
    
    if world.get_map().name != f"Carla/Maps/{TOWN}":
        world = client.load_world(TOWN)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    bp_lib = world.get_blueprint_library()
    actors = []
    
    # Setup matplotlib figure
    plt.ion()  # Interactive mode
    fig = plt.figure(figsize=(16, 6))
    
    # Camera view (left)
    ax_cam = fig.add_subplot(1, 2, 1)
    ax_cam.set_title('Camera Feed', fontsize=14, fontweight='bold')
    ax_cam.axis('off')
    
    # LiDAR view (right)
    ax_lidar = fig.add_subplot(1, 2, 2)
    
    plt.tight_layout()
    
    try:
        spawn_points = world.get_map().get_spawn_points()
        print(f"🔍 Finding spawn with clear path...")

        ego_bp = bp_lib.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("color", "255,255,255")
        ego = None

        for idx in list(range(0, min(len(spawn_points), 140), 10)):
            try:
                test_ego = world.spawn_actor(ego_bp, spawn_points[idx])
                actors.append(test_ego)
                for _ in range(10):
                    world.tick()
                test_ego.apply_control(carla.VehicleControl(throttle=0.35, brake=0.0, steer=0.0))
                for _ in range(25):
                    world.tick()
                if get_speed(test_ego) > 1.0:
                    ego = test_ego
                    print(f"✅ Found clear spawn at index {idx}\n")
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
                    for _ in range(25):
                        world.tick()
                    break
                else:
                    test_ego.destroy()
                    actors.pop()
            except RuntimeError:
                continue

        if ego is None:
            ego = world.spawn_actor(ego_bp, spawn_points[0])
            actors.append(ego)
            for _ in range(20):
                world.tick()

        # Spawn obstacle
        obstacle_distance_ahead = 80.0
        ego_wp = world.get_map().get_waypoint(ego.get_location(), project_to_road=True)
        next_wps = ego_wp.next(obstacle_distance_ahead) or ego_wp.next(50.0)
        obs_tf = next_wps[0].transform
        obs_tf.location.z += 0.5

        obs_bp = bp_lib.filter("vehicle.dodge.charger_2020")[0]
        obs_bp.set_attribute("color", "255,0,0")
        obstacle = world.spawn_actor(obs_bp, obs_tf)
        actors.append(obstacle)

        for _ in range(20):
            world.tick()

        print("\n" + "="*70)
        print("🎥 LIVE SENSOR VISUALIZATION")
        print("="*70)
        print("  Left:  Camera RGB feed")
        print("  Right: LiDAR point cloud (top-down)")
        print("  Bottom: Real-time sensor data")
        print("="*70 + "\n")

        # Sensors
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "960")  # Smaller for display
        cam_bp.set_attribute("image_size_y", "540")
        cam_bp.set_attribute("fov", "110")
        cam_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        cam = world.spawn_actor(cam_bp, carla.Transform(
            carla.Location(x=2.5, z=1.5), carla.Rotation(pitch=0)
        ), attach_to=ego)
        actors.append(cam)

        lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", "120")
        lidar_bp.set_attribute("channels", "32")
        lidar_bp.set_attribute("points_per_second", "56000")
        lidar_bp.set_attribute("rotation_frequency", "10")
        lidar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=2.5)), attach_to=ego)
        actors.append(lidar)

        radar_bp = bp_lib.find("sensor.other.radar")
        radar_bp.set_attribute("horizontal_fov", "30")
        radar_bp.set_attribute("range", "120")
        radar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=ego)
        actors.append(radar)

        sensor_data = {
            "camera_img": None,
            "lidar_pts": None,
            "radar_dist": SENTINEL_DIST,
            "radar_vel": 0.0,
            "lidar_dist": SENTINEL_DIST
        }

        def on_radar(meas):
            if len(meas) == 0:
                sensor_data["radar_dist"] = SENTINEL_DIST
                sensor_data["radar_vel"] = 0.0
                return
            front_dets = [(float(d.depth), abs(float(d.velocity)))
                         for d in meas if abs(float(d.azimuth)) < 20]
            if front_dets:
                depth, vel = min(front_dets, key=lambda x: x[0])
                sensor_data["radar_dist"] = depth
                sensor_data["radar_vel"] = vel

        def on_lidar(meas):
            dist, lateral, pts = process_lidar(meas)
            sensor_data["lidar_dist"] = dist
            sensor_data["lidar_pts"] = pts

        def on_camera(img):
            array = np.frombuffer(img.raw_data, dtype=np.uint8)
            array = array.reshape((img.height, img.width, 4))[:, :, :3]
            sensor_data["camera_img"] = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)

        radar.listen(on_radar)
        lidar.listen(on_lidar)
        cam.listen(on_camera)

        print("🎬 Starting visualization (close plot window to stop)...\n")

        start = time.time()
        frame = 0
        braked = False
        lane_change_complete = False
        resuming = False
        frames_since_stop = 0

        while time.time() - start < DURATION_SEC:
            world.tick()
            elapsed = time.time() - start

            ego_speed = get_speed(ego)
            ego_pos = ego.get_location()
            ego_tf = ego.get_transform()

            obstacle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))

            obs_pos = obstacle.get_location()
            true_dist = math.sqrt((ego_pos.x - obs_pos.x)**2 + (ego_pos.y - obs_pos.y)**2)

            radar_dist = sensor_data["radar_dist"]
            radar_vel = sensor_data["radar_vel"]
            lidar_dist = sensor_data["lidar_dist"]

            # Gated fusion
            candidates = []
            if lidar_dist < SENTINEL_DIST and abs(lidar_dist - true_dist) < ASSOC_GATE_M:
                candidates.append(lidar_dist)
            if radar_dist < SENTINEL_DIST and abs(radar_dist - true_dist) < ASSOC_GATE_M:
                candidates.append(radar_dist)
            fused_dist = min(candidates) if candidates else SENTINEL_DIST

            relative_vel = max(radar_vel, ego_speed)
            ttc = calculate_ttc(fused_dist, relative_vel) if fused_dist < SENTINEL_DIST else SENTINEL_DIST

            # Lane following steering
            current_wp = world.get_map().get_waypoint(ego_pos, project_to_road=True)
            lookahead_dist = 5.0 + ego_speed * 0.5
            next_wps = current_wp.next(lookahead_dist)
            if next_wps:
                target_vec = next_wps[0].transform.location - ego_pos
                target_yaw = math.degrees(math.atan2(target_vec.y, target_vec.x))
                angle_diff = (target_yaw - ego_tf.rotation.yaw + 180) % 360 - 180
                lane_steer = max(-1.0, min(1.0, angle_diff / 30.0))
            else:
                lane_steer = 0.0

            # Control logic (same as before)
            ego_ctrl = carla.VehicleControl()
            ego_ctrl.hand_brake = False
            brake_trigger = (fused_dist < BRAKE_DISTANCE) or (fused_dist < SENTINEL_DIST and ttc < TTC_CRITICAL)

            if brake_trigger and not braked:
                ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.0, 1.0, 0.0
                action = "EMERGENCY_BRAKE"
                braked = True
                frames_since_stop = 0
            elif braked and not lane_change_complete:
                frames_since_stop += 1
                if frames_since_stop < 40:
                    ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.0, 1.0, 0.0
                    action = "STOPPED"
                elif frames_since_stop < 100:
                    ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.35, 0.0, -0.25
                    action = "LANE_CHANGE"
                elif frames_since_stop < 140:
                    blend_factor = (frames_since_stop - 100) / 40.0
                    ego_ctrl.throttle = 0.45
                    ego_ctrl.brake = 0.0
                    ego_ctrl.steer = (1.0 - blend_factor) * 0.0 + blend_factor * lane_steer
                    action = "STRAIGHTENING"
                else:
                    lane_change_complete = True
                    resuming = True
                    ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.5, 0.0, lane_steer
                    action = "RESUMED"
            elif fused_dist < AVOID_DISTANCE and not braked:
                ego_ctrl.throttle = 0.5
                ego_ctrl.brake = 0.0
                ego_ctrl.steer = 0.6 * (-0.4) + 0.4 * lane_steer
                action = "AVOIDING"
            else:
                ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.5, 0.0, lane_steer
                action = "DRIVING"

            ego.apply_control(ego_ctrl)

            # UPDATE VISUALIZATION every 5 frames (smoother display)
            if frame % 5 == 0 and plt.fignum_exists(fig.number):
                # Camera view
                if sensor_data["camera_img"] is not None:
                    ax_cam.clear()
                    ax_cam.imshow(sensor_data["camera_img"])
                    
                    # Overlay sensor data
                    ax_cam.text(10, 30, f'Speed: {ego_speed:.1f} m/s', color='white', 
                               fontsize=12, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
                    ax_cam.text(10, 60, f'Fused Dist: {fused_dist:.1f} m', color='cyan', 
                               fontsize=12, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
                    ax_cam.text(10, 90, f'TTC: {ttc:.1f} s', color='yellow', 
                               fontsize=12, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
                    
                    action_color_rgb = {'EMERGENCY_BRAKE': 'red', 'LANE_CHANGE': 'orange',
                                       'AVOIDING': 'yellow', 'DRIVING': 'lime',
                                       'STOPPED': 'red', 'STRAIGHTENING': 'cyan', 'RESUMED': 'lime'}
                    ax_cam.text(10, 120, f'Action: {action}', 
                               color=action_color_rgb.get(action, 'white'),
                               fontsize=14, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
                    
                    ax_cam.set_title(f'Camera Feed - Time: {elapsed:.1f}s', 
                                    fontsize=14, fontweight='bold')
                    ax_cam.axis('off')
                
                # LiDAR view
                visualize_lidar(sensor_data["lidar_pts"], ax_lidar, fused_dist, action)
                
                plt.pause(0.001)
            
            if frame % 60 == 0:
                print(f"{elapsed:.1f}s: v={ego_speed:.1f}m/s | fused={fused_dist:.1f}m | TTC={ttc:.1f}s | {action}")

            frame += 1
            
            # Check if window closed
            if not plt.fignum_exists(fig.number):
                print("\n🛑 Visualization window closed - stopping simulation")
                break

        print(f"\n✅ Visualization complete!\n")

        radar.stop()
        lidar.stop()
        cam.stop()
        plt.close('all')

    finally:
        for a in actors:
            try:
                a.destroy()
            except:
                pass
        settings.synchronous_mode = False
        world.apply_settings(settings)
        plt.close('all')

if __name__ == "__main__":
    main()