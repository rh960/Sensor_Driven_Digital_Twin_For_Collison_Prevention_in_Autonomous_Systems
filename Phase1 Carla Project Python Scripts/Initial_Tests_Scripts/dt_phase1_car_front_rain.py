"""
AUTONOMOUS COLLISION AVOIDANCE SYSTEM - EXTREME RAIN CONDITIONS
Student: Raffay Hassan (M00944822)

SCENARIO CONFIGURATION:
- Map: Town04 (longer stretches)
- Weather: ☔ EXTREME RAIN (heavy precipitation, wet roads, reduced visibility)
- Obstacle: 80m ahead (stationary red car)
- Duration: 70s
- Steering: Waypoint-based lane following

FEATURES:
1) EXTREME WEATHER: Heavy rain, wet roads, puddles, fog
2) Obstacle spawned ON road using waypoints
3) Gated fusion: ignore LiDAR/Radar hits that don't match obstacle distance
4) TTC only computed when fused_dist is valid
5) LANE-FOLLOWING STEERING: Tracks waypoints to stay on road

Scenario:
- Your car drives forward in HEAVY RAIN
- Approaches a stationary obstacle car (reduced visibility!)
- Decision logic:
  * Too close (< 8m) → EMERGENCY BRAKE
  * After stopping → LANE CHANGE maneuver around obstacle
  * Resume normal driving after passing obstacle

Multi-sensor fusion: Camera + LiDAR + Radar
Tests autonomous system in adverse weather conditions!
"""

import carla
import time
import os
import csv
import numpy as np
from datetime import datetime
import cv2
import math

HOST = "localhost"
PORT = 2000

TOWN = "Town04"
FIXED_DT = 0.05
DURATION_SEC = 70

CAM_W, CAM_H = 1920, 1080
CAM_FOV = 110
SENSOR_TICK = 0.05

# Safety thresholds
BRAKE_DISTANCE = 8.0
AVOID_DISTANCE = 15.0
TTC_CRITICAL = 2.0

# Association gate
ASSOC_GATE_M = 12.0
SENTINEL_DIST = 999.0

def make_run_dir(tag):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = f"/home/rh960/carla_env/dt_logs/{ts}_{tag}"
    os.makedirs(os.path.join(run_dir, "camera"), exist_ok=True)
    os.makedirs(os.path.join(run_dir, "lidar"), exist_ok=True)
    return run_dir

def get_speed(v):
    vel = v.get_velocity()
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

def save_lidar_ply(path, data):
    pts = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for x, y, z in pts:
            f.write(f"{x} {y} {z}\n")

def process_lidar(meas):
    if meas is None or meas.raw_data is None or len(meas.raw_data) == 0:
        return SENTINEL_DIST, 0.0

    pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
    mask = (pts[:, 0] > 2.0) & (np.abs(pts[:, 1]) < 2.0) & (pts[:, 0] < 120.0)
    
    if not mask.any():
        return SENTINEL_DIST, 0.0

    valid = pts[mask]
    distances = np.sqrt(valid[:, 0]**2 + valid[:, 1]**2)
    i = int(np.argmin(distances))
    return float(distances[i]), float(valid[i, 1])

def calculate_ttc(distance, relative_velocity):
    if relative_velocity <= 0.1:
        return SENTINEL_DIST
    return distance / relative_velocity

def create_video(run_dir, fps=20):
    frames = sorted([f for f in os.listdir(os.path.join(run_dir, "camera")) if f.endswith(".jpg")])
    if not frames:
        return

    first = cv2.imread(os.path.join(run_dir, "camera", frames[0]))
    h, w, _ = first.shape
    video = cv2.VideoWriter(
        os.path.join(run_dir, "output.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (w, h)
    )

    for f in frames:
        img = cv2.imread(os.path.join(run_dir, "camera", f))
        if img is not None:
            video.write(img)

    video.release()
    print(f"✅ Video created: {os.path.join(run_dir, 'output.mp4')}")

def main():
    run_dir = make_run_dir("rainy_collision_avoidance")

    radar_f = open(os.path.join(run_dir, "radar_data.csv"), "w", newline="")
    radar_w = csv.writer(radar_f)
    radar_w.writerow(["timestamp", "frame", "distance_m", "velocity_mps", "azimuth_deg", "altitude_deg"])

    lidar_f = open(os.path.join(run_dir, "lidar_data.csv"), "w", newline="")
    lidar_w = csv.writer(lidar_f)
    lidar_w.writerow(["timestamp", "frame", "min_distance_m", "lateral_offset_m", "point_count"])

    state_f = open(os.path.join(run_dir, "vehicle_state.csv"), "w", newline="")
    state_w = csv.writer(state_f)
    state_w.writerow(["time_s", "ego_speed_mps", "ego_x", "ego_y", "ego_yaw",
                      "throttle", "brake", "steer", "action", "decision_reason"])

    fusion_f = open(os.path.join(run_dir, "sensor_fusion.csv"), "w", newline="")
    fusion_w = csv.writer(fusion_f)
    fusion_w.writerow(["time_s", "true_dist_m", "radar_dist_m", "lidar_dist_m", "fused_dist_m",
                       "relative_vel_mps", "ttc_s", "safety_level"])

    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)
    world = client.get_world()

    if world.get_map().name != f"Carla/Maps/{TOWN}":
        world = client.load_world(TOWN)

    # === SET EXTREME RAIN WEATHER ===
    print("\n" + "="*70)
    print("🌧️  SETTING EXTREME RAIN CONDITIONS")
    print("="*70)
    
    weather = carla.WeatherParameters(
        cloudiness=100.0,              # Maximum clouds
        precipitation=100.0,           # HEAVY rain
        precipitation_deposits=100.0,  # Wet roads with puddles
        wind_intensity=80.0,           # Strong wind
        sun_azimuth_angle=70.0,        # Sun angle
        sun_altitude_angle=-30.0,      # Dark/overcast (negative = below horizon)
        fog_density=40.0,              # Moderate fog for realism
        fog_distance=30.0,             # Fog starts at 30m
        wetness=100.0,                 # Maximum road wetness
        fog_falloff=0.2                # Fog falloff
    )
    world.set_weather(weather)
    
    print("  ☔ Precipitation: 100% (Heavy rain)")
    print("  💧 Road wetness: 100% (Puddles everywhere)")
    print("  💨 Wind: 80% (Strong gusts)")
    print("  🌫️  Fog: 40% density, 30m distance")
    print("  ☁️  Cloudiness: 100% (Completely overcast)")
    print("  🌑 Sun altitude: -30° (Dark conditions)")
    print("="*70 + "\n")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    bp_lib = world.get_blueprint_library()
    actors = []

    try:
        spawn_points = world.get_map().get_spawn_points()
        print(f"🔍 Finding spawn with clear path from {len(spawn_points)} points...")

        ego_bp = bp_lib.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("color", "255,255,255")

        ego = None

        candidate_indices = list(range(0, min(len(spawn_points), 140), 10))
        if 0 not in candidate_indices:
            candidate_indices.insert(0, 0)

        for idx in candidate_indices:
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
                    print(f"✅ Found clear spawn at index {idx} (car moving)")
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
                    
                    # Enable headlights
                    print("💡 Turning on headlights...")
                    vehicle_lights = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position
                    ego.set_light_state(carla.VehicleLightState(vehicle_lights))
                    print("✅ Headlights ON\n")
                    
                    for _ in range(25):
                        world.tick()
                    break
                else:
                    print(f"   Spawn {idx}: Blocked (not moving)")
                    test_ego.destroy()
                    actors.pop()
            except RuntimeError:
                continue

        if ego is None:
            print("❌ No clear spawn found! Using spawn 0 (best effort)")
            ego = world.spawn_actor(ego_bp, spawn_points[0])
            actors.append(ego)
            for _ in range(20):
                world.tick()

        # Enable headlights for rainy conditions
        print("💡 Turning on headlights...")
        vehicle_lights = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position
        ego.set_light_state(carla.VehicleLightState(vehicle_lights))
        print("✅ Headlights ON\n")

        # Obstacle on-lane via waypoint
        obstacle_distance_ahead = 80.0
        ego_wp = world.get_map().get_waypoint(
            ego.get_location(),
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        next_wps = ego_wp.next(obstacle_distance_ahead) or ego_wp.next(50.0)
        target_wp = next_wps[0]
        obs_tf = target_wp.transform
        obs_tf.location.z += 0.5

        obs_bp = bp_lib.filter("vehicle.dodge.charger_2020")[0]
        obs_bp.set_attribute("color", "255,0,0")
        obstacle = world.spawn_actor(obs_bp, obs_tf)
        actors.append(obstacle)

        for _ in range(20):
            world.tick()

        ego_pos = ego.get_location()
        obs_pos = obstacle.get_location()
        initial_sep = math.sqrt((ego_pos.x - obs_pos.x)**2 + (ego_pos.y - obs_pos.y)**2)

        print("\n" + "="*70)
        print("🚗 AUTONOMOUS COLLISION AVOIDANCE - RAINY CONDITIONS")
        print("="*70)
        print(f"Student: Raffay Hassan (M00944822)")
        print(f"\n🌧️  WEATHER: Extreme rain, wet roads, reduced visibility")
        print(f"🗺️  MAP: {TOWN}")
        print(f"🎮 STEERING: Waypoint-based lane following")
        print(f"\n✅ YOUR CAR (White Tesla)")
        print(f"   Position: ({ego_pos.x:.1f}, {ego_pos.y:.1f})")
        print(f"\n✅ OBSTACLE CAR (Red Charger) - STATIONARY")
        print(f"   Position: ({obs_pos.x:.1f}, {obs_pos.y:.1f})")
        print(f"   Distance ahead: {initial_sep:.1f}m")
        print("="*70 + "\n")

        # Sensors
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(CAM_W))
        cam_bp.set_attribute("image_size_y", str(CAM_H))
        cam_bp.set_attribute("fov", str(CAM_FOV))
        cam_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        cam = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=2.5, z=1.5), carla.Rotation(pitch=0)),
            attach_to=ego
        )
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
        radar_bp.set_attribute("vertical_fov", "10")
        radar_bp.set_attribute("range", "120")
        radar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=ego)
        actors.append(radar)

        sensor_data = {
            "radar_dist": SENTINEL_DIST,
            "radar_vel": 0.0,
            "lidar_dist": SENTINEL_DIST,
            "lidar_lateral": 0.0
        }
        files_open = True

        def on_radar(meas):
            if not files_open:
                return

            if meas is None or len(meas) == 0:
                sensor_data["radar_dist"] = SENTINEL_DIST
                sensor_data["radar_vel"] = 0.0
                return

            for d in meas:
                radar_w.writerow([
                    meas.timestamp,
                    meas.frame,
                    float(d.depth),
                    float(d.velocity),
                    float(d.azimuth),
                    float(d.altitude)
                ])

            front_dets = [(float(d.depth), abs(float(d.velocity)))
                          for d in meas if abs(float(d.azimuth)) < 20]

            if front_dets:
                depth, vel = min(front_dets, key=lambda x: x[0])
                sensor_data["radar_dist"] = depth
                sensor_data["radar_vel"] = vel
            else:
                sensor_data["radar_dist"] = SENTINEL_DIST
                sensor_data["radar_vel"] = 0.0

        def on_lidar(meas):
            if not files_open:
                return

            save_lidar_ply(os.path.join(run_dir, "lidar", f"{meas.frame:06d}.ply"), meas)

            dist, lateral = process_lidar(meas)
            sensor_data["lidar_dist"] = dist
            sensor_data["lidar_lateral"] = lateral

            pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)
            lidar_w.writerow([meas.timestamp, meas.frame, dist, lateral, len(pts)])

        def on_camera(img):
            if files_open:
                img.save_to_disk(os.path.join(run_dir, "camera", f"{img.frame:06d}.jpg"))

        radar.listen(on_radar)
        lidar.listen(on_lidar)
        cam.listen(on_camera)

        print("="*70)
        print("🎬 SCENARIO START - EXTREME RAIN")
        print("="*70)
        print("  Phase 1: Driving in heavy rain")
        print("  Phase 2: Approaching obstacle (reduced visibility)")
        print("  Phase 3: Emergency brake when too close (< 8m)")
        print("  Phase 4: Lane change around obstacle on wet road")
        print("  Phase 5: Resume driving")
        print("\n  Testing: Sensor performance in adverse weather!")
        print("="*70 + "\n")

        start = time.time()
        frame = 0
        avoided = False
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

            # Keep obstacle stationary
            obstacle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))

            # True obstacle distance
            obs_pos = obstacle.get_location()
            true_dist = math.sqrt((ego_pos.x - obs_pos.x)**2 + (ego_pos.y - obs_pos.y)**2)

            # Sensor readings
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

            # Relative velocity
            relative_vel = max(radar_vel, ego_speed)

            # TTC
            if fused_dist >= SENTINEL_DIST:
                ttc = SENTINEL_DIST
            else:
                ttc = calculate_ttc(fused_dist, relative_vel)

            # Safety level
            if (fused_dist < BRAKE_DISTANCE) or (fused_dist < SENTINEL_DIST and ttc < TTC_CRITICAL):
                safety = "CRITICAL"
            elif fused_dist < AVOID_DISTANCE:
                safety = "WARNING"
            else:
                safety = "SAFE"

            fusion_w.writerow([elapsed, true_dist, radar_dist, lidar_dist, fused_dist, relative_vel, ttc, safety])

            # === LANE-FOLLOWING STEERING ===
            current_wp = world.get_map().get_waypoint(
                ego_pos,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            
            lookahead_dist = 5.0 + ego_speed * 0.5
            next_wps = current_wp.next(lookahead_dist)
            
            if next_wps:
                target_wp = next_wps[0]
                target_vec = target_wp.transform.location - ego_pos
                target_yaw = math.degrees(math.atan2(target_vec.y, target_vec.x))
                angle_diff = (target_yaw - ego_tf.rotation.yaw + 180) % 360 - 180
                lane_steer = max(-1.0, min(1.0, angle_diff / 30.0))
            else:
                lane_steer = 0.0

            # === EGO CONTROL ===
            ego_ctrl = carla.VehicleControl()
            ego_ctrl.hand_brake = False

            brake_trigger = (fused_dist < BRAKE_DISTANCE) or (fused_dist < SENTINEL_DIST and ttc < TTC_CRITICAL)

            if brake_trigger and not braked:
                ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.0, 1.0, 0.0
                action = "EMERGENCY_BRAKE"
                reason = f"FusedDist={fused_dist:.1f} TTC={ttc:.2f}"
                
                braked = True
                frames_since_stop = 0
                print(f"\n🛑 {elapsed:.1f}s: EMERGENCY BRAKE! (Rainy conditions)")
                print(f"   Fused Distance: {fused_dist:.1f}m | True Distance: {true_dist:.1f}m")
                print(f"   TTC: {ttc:.2f}s")
                print(f"   Speed: {ego_speed:.1f} m/s → STOPPING\n")

            elif braked and not lane_change_complete:
                frames_since_stop += 1
                
                if frames_since_stop < 40:
                    ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.0, 1.0, 0.0
                    action = "STOPPED"
                    reason = "Waiting before lane change"
                    
                elif frames_since_stop < 100:
                    if frames_since_stop == 40:
                        print(f"\n🔀 {elapsed:.1f}s: LANE CHANGE ON WET ROAD!")
                        print(f"   Maneuvering around obstacle...\n")
                    
                    ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.35, 0.0, -0.25
                    action = "LANE_CHANGE"
                    reason = "Steering around obstacle"
                    
                elif frames_since_stop < 140:
                    if frames_since_stop == 100:
                        print(f"\n↪️  {elapsed:.1f}s: STRAIGHTENING OUT\n")
                    
                    ego_ctrl.throttle = 0.45
                    ego_ctrl.brake = 0.0
                    blend_factor = (frames_since_stop - 100) / 40.0
                    ego_ctrl.steer = (1.0 - blend_factor) * 0.0 + blend_factor * lane_steer
                    action = "STRAIGHTENING"
                    reason = "Completing lane change"
                    
                else:
                    lane_change_complete = True
                    resuming = True
                    print(f"\n✅ {elapsed:.1f}s: LANE CHANGE COMPLETE!")
                    print(f"   Resuming normal driving in rain\n")
                    
                    ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.5, 0.0, lane_steer
                    action = "RESUMED"
                    reason = "Normal driving"

            elif fused_dist < AVOID_DISTANCE and not braked:
                ego_ctrl.throttle = 0.5
                ego_ctrl.brake = 0.0
                avoid_steer = -0.4
                ego_ctrl.steer = 0.6 * avoid_steer + 0.4 * lane_steer
                action = "AVOIDING"
                reason = f"FusedDist={fused_dist:.1f}m"

                if not avoided:
                    avoided = True
                    print(f"\n🔀 {elapsed:.1f}s: PRE-EMPTIVE AVOIDANCE!")
                    print(f"   Fused Distance: {fused_dist:.1f}m | True Distance: {true_dist:.1f}m\n")

            else:
                ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer = 0.5, 0.0, lane_steer
                action = "DRIVING"
                reason = "Path clear" if not resuming else "Post-lane-change driving"

            ego.apply_control(ego_ctrl)

            state_w.writerow([
                elapsed, ego_speed, ego_pos.x, ego_pos.y, ego_tf.rotation.yaw,
                ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer,
                action, reason
            ])

            if frame % 60 == 0:
                print(f"{elapsed:.1f}s: v={ego_speed:.1f}m/s | fused={fused_dist:.1f}m | true={true_dist:.1f}m | {action} [RAIN]")

            frame += 1

        print(f"\n{'='*70}")
        print(f"✅ RAINY SCENARIO COMPLETE - {frame} frames recorded")
        print(f"{'='*70}\n")

        radar.stop()
        lidar.stop()
        cam.stop()

        time.sleep(0.5)
        files_open = False

        radar_f.close()
        lidar_f.close()
        state_f.close()
        fusion_f.close()

        create_video(run_dir, fps=20)

        print(f"\n📁 ALL DATA SAVED TO: {run_dir}")
        print("\nFiles created:")
        print("   • radar_data.csv")
        print("   • lidar_data.csv")
        print("   • vehicle_state.csv")
        print("   • sensor_fusion.csv")
        print("   • lidar/*.ply (point clouds)")
        print("   • camera/*.jpg (frames)")
        print("   • output.mp4 (video)\n")
        print("🌧️  Weather challenge: Sensors tested in extreme rain!\n")

    finally:
        for a in actors:
            try:
                a.destroy()
            except:
                pass
        settings.synchronous_mode = False
        world.apply_settings(settings)

if __name__ == "__main__":
    main()