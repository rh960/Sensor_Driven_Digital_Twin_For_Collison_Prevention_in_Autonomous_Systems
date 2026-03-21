"""
AUTONOMOUS COLLISION AVOIDANCE SYSTEM (TARGET-ASSOCIATED FIX)
Student: Raffay Hassan (M00944822)

Key fixes:
1) No more instant EMERGENCY BRAKE at t=0:
   - TTC is only considered when ego_speed > 0.5 m/s
   - Radar/LiDAR detections are ASSOCIATED to the obstacle car (direction + distance gate)

2) No more random wall distance (e.g., 4.3m) when obstacle is far:
   - LiDAR/Radar are filtered to ONLY track the obstacle car "target"

Scenario:
- Ego drives forward
- Approaches stationary obstacle
- Decisions:
  < 8m -> brake
  8-15m -> steer
  > 15m -> drive
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
TOWN = "Town01"
FIXED_DT = 0.05
DURATION_SEC = 40

CAM_W, CAM_H = 1920, 1080
CAM_FOV = 110
SENSOR_TICK = 0.05

BRAKE_DISTANCE = 8.0
AVOID_DISTANCE = 15.0
TTC_CRITICAL = 2.0

SENTINEL_DIST = 999.0

# Association / gating parameters
ASSOC_AZIMUTH_DEG = 8.0     # how close radar azimuth must be to obstacle bearing
ASSOC_DEPTH_TOL = 8.0       # meters: radar depth must be within +/- this of true obstacle distance
LIDAR_DEPTH_TOL = 8.0       # meters: lidar min distance must be within +/- this of true obstacle distance
LIDAR_WIDTH = 2.5           # meters: lateral window in ego frame

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

def world_to_ego_frame(ego_tf, world_loc):
    """
    Convert a world location to ego local coordinates (x forward, y right, z up).
    """
    ex = ego_tf.location.x
    ey = ego_tf.location.y
    ez = ego_tf.location.z

    dx = world_loc.x - ex
    dy = world_loc.y - ey
    dz = world_loc.z - ez

    yaw = math.radians(ego_tf.rotation.yaw)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # Rotate world delta into ego frame (inverse yaw rotation)
    x_local =  cy * dx + sy * dy
    y_local = -sy * dx + cy * dy
    z_local = dz
    return x_local, y_local, z_local

def compute_obstacle_bearing_and_distance(ego, obstacle):
    """
    Returns:
      true_dist (m), true_azimuth_rad (bearing of obstacle in ego frame), x_local, y_local
    """
    ego_tf = ego.get_transform()
    obs_loc = obstacle.get_location()
    x_local, y_local, _ = world_to_ego_frame(ego_tf, obs_loc)

    true_dist = math.sqrt(x_local**2 + y_local**2)
    true_azimuth = math.atan2(y_local, x_local)  # rad, 0 = straight ahead
    return true_dist, true_azimuth, x_local, y_local

def calculate_ttc(distance, closing_speed):
    if closing_speed <= 0.1:
        return SENTINEL_DIST
    return distance / closing_speed

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
        (w, h),
    )
    for f in frames:
        img = cv2.imread(os.path.join(run_dir, "camera", f))
        if img is not None:
            video.write(img)
    video.release()
    print(f"✅ Video created: {os.path.join(run_dir, 'output.mp4')}")

def main():
    run_dir = make_run_dir("collision_avoidance")

    radar_f = open(os.path.join(run_dir, "radar_data.csv"), "w", newline="")
    radar_w = csv.writer(radar_f)
    radar_w.writerow(["timestamp", "frame", "depth_m", "velocity_mps", "azimuth", "altitude"])

    lidar_f = open(os.path.join(run_dir, "lidar_data.csv"), "w", newline="")
    lidar_w = csv.writer(lidar_f)
    lidar_w.writerow(["timestamp", "frame", "min_dist_m", "lateral_m", "point_count", "accepted"])

    state_f = open(os.path.join(run_dir, "vehicle_state.csv"), "w", newline="")
    state_w = csv.writer(state_f)
    state_w.writerow([
        "time_s", "ego_speed_mps", "ego_x", "ego_y", "ego_yaw",
        "throttle", "brake", "steer", "action", "reason"
    ])

    fusion_f = open(os.path.join(run_dir, "sensor_fusion.csv"), "w", newline="")
    fusion_w = csv.writer(fusion_f)
    fusion_w.writerow([
        "time_s",
        "true_dist_m", "true_azimuth_deg",
        "radar_dist_m", "radar_closing_mps", "radar_accepted",
        "lidar_dist_m", "lidar_lateral_m", "lidar_accepted",
        "fused_dist_m", "closing_mps", "ttc_s", "safety"
    ])

    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    world = client.get_world()

    if world.get_map().name != f"Carla/Maps/{TOWN}":
        world = client.load_world(TOWN)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    bp_lib = world.get_blueprint_library()
    actors = []

    try:
        spawn_points = world.get_map().get_spawn_points()

        ego_bp = bp_lib.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("color", "255,255,255")

        ego = None
        for idx in [70, 80, 90, 100, 110, 60, 50, 40, 30, 20, 10]:
            if idx >= len(spawn_points):
                continue
            try:
                test_ego = world.spawn_actor(ego_bp, spawn_points[idx])
                actors.append(test_ego)

                for _ in range(10):
                    world.tick()

                test_ego.apply_control(carla.VehicleControl(throttle=0.3))
                for _ in range(20):
                    world.tick()

                if get_speed(test_ego) > 1.0:
                    ego = test_ego
                    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                    for _ in range(20):
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

        start_loc = ego.get_location()

        # Spawn obstacle 40m ahead
        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        obs_loc = carla.Location(
            x=ego_tf.location.x + fwd.x * 40,
            y=ego_tf.location.y + fwd.y * 40,
            z=ego_tf.location.z + 0.5
        )
        obs_bp = bp_lib.filter("vehicle.dodge.charger_2020")[0]
        obs_bp.set_attribute("color", "255,0,0")
        obstacle = world.spawn_actor(obs_bp, carla.Transform(obs_loc, ego_tf.rotation))
        actors.append(obstacle)

        for _ in range(20):
            world.tick()

        # Sensors
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(CAM_W))
        cam_bp.set_attribute("image_size_y", str(CAM_H))
        cam_bp.set_attribute("fov", str(CAM_FOV))
        cam_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        cam = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=2.5, z=1.5)),
            attach_to=ego
        )
        actors.append(cam)

        lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", "100")
        lidar_bp.set_attribute("channels", "32")
        lidar_bp.set_attribute("points_per_second", "56000")
        lidar_bp.set_attribute("rotation_frequency", "10")
        lidar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=2.5)), attach_to=ego)
        actors.append(lidar)

        radar_bp = bp_lib.find("sensor.other.radar")
        radar_bp.set_attribute("horizontal_fov", "30")
        radar_bp.set_attribute("vertical_fov", "10")
        radar_bp.set_attribute("range", "100")
        radar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=ego)
        actors.append(radar)

        files_open = True

        # Latest associated measurements
        sensor_data = {
            "radar_dist": SENTINEL_DIST,
            "radar_closing": 0.0,
            "radar_ok": 0,

            "lidar_dist": SENTINEL_DIST,
            "lidar_lat": 0.0,
            "lidar_ok": 0,

            # store latest obstacle bearing/distance for association inside callbacks
            "true_dist": SENTINEL_DIST,
            "true_azimuth": 0.0,
        }

        def on_camera(img):
            if files_open:
                img.save_to_disk(os.path.join(run_dir, "camera", f"{img.frame:06d}.jpg"))

        def on_radar(meas):
            if not files_open:
                return

            # Default reset
            sensor_data["radar_dist"] = SENTINEL_DIST
            sensor_data["radar_closing"] = 0.0
            sensor_data["radar_ok"] = 0

            if meas is None or len(meas) == 0:
                return

            # Log all detections
            for d in meas:
                radar_w.writerow([meas.timestamp, meas.frame,
                                  float(d.depth), float(d.velocity),
                                  float(d.azimuth), float(d.altitude)])

            true_dist = sensor_data["true_dist"]
            true_az = sensor_data["true_azimuth"]
            if true_dist >= SENTINEL_DIST:
                return

            # Associate: choose detection closest to obstacle direction and depth
            best = None
            best_cost = 1e9

            az_gate = math.radians(ASSOC_AZIMUTH_DEG)

            for d in meas:
                depth = float(d.depth)
                az = float(d.azimuth)

                if abs(az - true_az) > az_gate:
                    continue
                if abs(depth - true_dist) > ASSOC_DEPTH_TOL:
                    continue

                # cost = depth error + weighted az error
                cost = abs(depth - true_dist) + 3.0 * abs(az - true_az)
                if cost < best_cost:
                    best_cost = cost
                    best = d

            if best is None:
                return

            depth = float(best.depth)
            vrel = float(best.velocity)

            # Closing speed: guard noise.
            # If sign is opposite in your build, flip the sign here.
            closing = max(0.0, -vrel)

            # Clamp to reasonable range (prevents 0.00 TTC at startup from radar spikes)
            closing = min(closing, 40.0)

            sensor_data["radar_dist"] = depth
            sensor_data["radar_closing"] = closing
            sensor_data["radar_ok"] = 1

        def on_lidar(meas):
            if not files_open:
                return

            sensor_data["lidar_dist"] = SENTINEL_DIST
            sensor_data["lidar_lat"] = 0.0
            sensor_data["lidar_ok"] = 0

            if meas is None or meas.raw_data is None or len(meas.raw_data) == 0:
                return

            save_lidar_ply(os.path.join(run_dir, "lidar", f"{meas.frame:06d}.ply"), meas)

            pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]

            # LiDAR in sensor frame is aligned with ego: x forward, y right
            # Only consider points near the obstacle distance and near centerline
            true_dist = sensor_data["true_dist"]
            if true_dist >= SENTINEL_DIST:
                return

            # forward range gate around expected obstacle distance
            # Using radial distance gate improves association
            radial = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)

            mask = (
                (pts[:, 0] > 2.0) &
                (np.abs(pts[:, 1]) < LIDAR_WIDTH) &
                (radial > max(0.0, true_dist - LIDAR_DEPTH_TOL)) &
                (radial < (true_dist + LIDAR_DEPTH_TOL)) &
                (pts[:, 0] < 80.0)
            )

            accepted = 1 if mask.any() else 0

            if not mask.any():
                lidar_w.writerow([meas.timestamp, meas.frame, SENTINEL_DIST, 0.0, len(pts), 0])
                return

            valid = pts[mask]
            distances = np.sqrt(valid[:, 0]**2 + valid[:, 1]**2)
            i = int(np.argmin(distances))
            dmin = float(distances[i])
            lat = float(valid[i, 1])

            sensor_data["lidar_dist"] = dmin
            sensor_data["lidar_lat"] = lat
            sensor_data["lidar_ok"] = 1

            lidar_w.writerow([meas.timestamp, meas.frame, dmin, lat, len(pts), accepted])

        cam.listen(on_camera)
        radar.listen(on_radar)
        lidar.listen(on_lidar)

        print("=" * 70)
        print("🎬 SCENARIO START")
        print("=" * 70)
        print("  Phase 1 (0-30s):  Your car drives forward")
        print("  Phase 2 (30s+):   Approaches obstacle")
        print("  Decision:")
        print("    • Distance < 8m  → EMERGENCY BRAKE")
        print("    • Distance 8-15m → STEER AROUND (avoid)")
        print("    • Distance > 15m → Continue")
        print("=" * 70 + "\n")

        start = time.time()
        frame = 0
        avoided = False
        braked = False

        while time.time() - start < DURATION_SEC:
            world.tick()
            elapsed = time.time() - start

            ego_speed = get_speed(ego)
            ego_pos = ego.get_location()
            ego_tf = ego.get_transform()

            # keep obstacle stopped
            obstacle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))

            # Update true obstacle geometry (for association)
            true_dist, true_az, x_loc, y_loc = compute_obstacle_bearing_and_distance(ego, obstacle)
            sensor_data["true_dist"] = true_dist
            sensor_data["true_azimuth"] = true_az

            # Read associated sensor values
            radar_dist = sensor_data["radar_dist"]
            radar_close = sensor_data["radar_closing"]
            radar_ok = sensor_data["radar_ok"]

            lidar_dist = sensor_data["lidar_dist"]
            lidar_lat = sensor_data["lidar_lat"]
            lidar_ok = sensor_data["lidar_ok"]

            # Fused distance: prefer LiDAR if it matched target; else radar if matched target; else sentinel
            if lidar_ok == 1:
                fused_dist = lidar_dist
            elif radar_ok == 1:
                fused_dist = radar_dist
            else:
                fused_dist = SENTINEL_DIST

            # TTC: only when ego is actually moving (prevents startup TTC spikes)
            if fused_dist >= SENTINEL_DIST or ego_speed <= 0.5:
                closing = 0.0
                ttc = SENTINEL_DIST
            else:
                # use radar closing if valid; otherwise approximate with ego speed
                closing = radar_close if (radar_ok == 1 and radar_close > 0.1) else ego_speed
                # clamp closing to not exceed ego_speed by crazy margin
                closing = min(closing, ego_speed + 5.0)
                ttc = calculate_ttc(fused_dist, closing)

            # Safety
            if (fused_dist < BRAKE_DISTANCE) or (ttc < TTC_CRITICAL):
                safety = "CRITICAL"
            elif fused_dist < AVOID_DISTANCE:
                safety = "WARNING"
            else:
                safety = "SAFE"

            fusion_w.writerow([
                elapsed,
                true_dist, math.degrees(true_az),
                radar_dist, radar_close, radar_ok,
                lidar_dist, lidar_lat, lidar_ok,
                fused_dist, closing, ttc, safety
            ])

            # Decision
            ego_ctrl = carla.VehicleControl()
            ego_ctrl.hand_brake = False

            if fused_dist < BRAKE_DISTANCE or ttc < TTC_CRITICAL:
                ego_ctrl.throttle = 0.0
                ego_ctrl.brake = 1.0
                ego_ctrl.steer = 0.0
                action = "EMERGENCY_BRAKE"

                if fused_dist < BRAKE_DISTANCE:
                    reason = f"Dist={fused_dist:.1f}m < {BRAKE_DISTANCE}m"
                else:
                    reason = f"TTC={ttc:.2f}s < {TTC_CRITICAL}s (Dist={fused_dist:.1f}m, v={closing:.1f})"

                if not braked:
                    braked = True
                    print(f"\n🛑 {elapsed:.1f}s: EMERGENCY BRAKE!")
                    print(f"   Distance: {fused_dist:.1f}m")
                    print(f"   TTC: {ttc:.2f}s")
                    print(f"   Speed: {ego_speed:.1f} m/s → STOPPING")
                    print(f"   Reason: {reason}\n")

            elif fused_dist < AVOID_DISTANCE and not braked:
                ego_ctrl.throttle = 0.5
                ego_ctrl.brake = 0.0
                ego_ctrl.steer = -0.4
                action = "AVOIDING"
                reason = f"Dist={fused_dist:.1f}m in avoid band (lat={lidar_lat:.2f}m)"

                if not avoided:
                    avoided = True
                    print(f"\n🔀 {elapsed:.1f}s: STEERING TO AVOID!")
                    print(f"   Distance: {fused_dist:.1f}m ({BRAKE_DISTANCE}-{AVOID_DISTANCE}m)")
                    print(f"   Maneuver: Steering LEFT around obstacle")
                    print(f"   Reason: {reason}\n")

            elif braked:
                ego_ctrl.throttle = 0.0
                ego_ctrl.brake = 1.0
                ego_ctrl.steer = 0.0
                action = "STOPPED"
                reason = "Collision prevented"

            else:
                ego_ctrl.throttle = 0.5
                ego_ctrl.brake = 0.0
                ego_ctrl.steer = 0.0
                action = "DRIVING"
                reason = "Path clear"

            ego.apply_control(ego_ctrl)

            state_w.writerow([
                elapsed, ego_speed,
                ego_pos.x, ego_pos.y, ego_tf.rotation.yaw,
                ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer,
                action, reason
            ])

            if frame % 60 == 0:
                print(f"{elapsed:.1f}s: Speed={ego_speed:.1f}m/s, Dist={fused_dist:.1f}m, TTC={ttc:.1f}s, {action}")

            frame += 1

        print(f"\n{'=' * 70}")
        print(f"✅ SCENARIO COMPLETE - {frame} frames recorded")
        print(f"{'=' * 70}\n")

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
        print("   • lidar/*.ply")
        print("   • camera/*.jpg")
        print("   • output.mp4\n")

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
