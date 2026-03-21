"""
Pedestrian Crossing Scenario with Sensors and Logging
- Town07
- Ego Tesla drives forward
- One pedestrian crosses 50 m ahead; ego stops, waits, then resumes
- Sensors: RGB camera, LiDAR, Radar (same as previous scenario)
- Logs: camera frames, lidar PLY, radar CSV, lidar CSV, vehicle_state CSV, sensor_fusion CSV, output.mp4
"""

import carla
import time
import os
import csv
import math
import numpy as np
import cv2
from datetime import datetime

HOST = "localhost"
PORT = 2000
TOWN = "Town07"
FIXED_DT = 0.05
DURATION_SEC = 30

CAM_W, CAM_H = 1920, 1080
CAM_FOV = 110
SENSOR_TICK = 0.05

BRAKE_DISTANCE = 8.0
AVOID_DISTANCE = 15.0
TTC_CRITICAL = 2.0
ASSOC_GATE_M = 12.0
SENTINEL_DIST = 999.0


def make_run_dir(tag):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(os.getcwd(), "dt_logs")
    run_dir = os.path.join(base, f"{ts}_{tag}")
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
        cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
    )
    for f in frames:
        img = cv2.imread(os.path.join(run_dir, "camera", f))
        if img is not None:
            video.write(img)
    video.release()


def main():
    run_dir = make_run_dir("ped_crossing")

    # Open CSVs
    radar_f = open(os.path.join(run_dir, "radar_data.csv"), "w", newline="")
    radar_w = csv.writer(radar_f)
    radar_w.writerow(["timestamp", "frame", "distance_m", "velocity_mps", "azimuth_deg", "altitude_deg"])

    lidar_f = open(os.path.join(run_dir, "lidar_data.csv"), "w", newline="")
    lidar_w = csv.writer(lidar_f)
    lidar_w.writerow(["timestamp", "frame", "min_distance_m", "lateral_offset_m", "point_count"])

    state_f = open(os.path.join(run_dir, "vehicle_state.csv"), "w", newline="")
    state_w = csv.writer(state_f)
    state_w.writerow(["time_s", "ego_speed_mps", "ego_x", "ego_y", "ego_yaw",
                      "throttle", "brake", "steer", "action", "decision_reason", "ped_in_path"])

    fusion_f = open(os.path.join(run_dir, "sensor_fusion.csv"), "w", newline="")
    fusion_w = csv.writer(fusion_f)
    fusion_w.writerow(["time_s", "true_dist_m", "radar_dist_m", "lidar_dist_m", "fused_dist_m",
                       "relative_vel_mps", "ttc_s", "safety_level"])

    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    world = client.load_world(TOWN)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    bp = world.get_blueprint_library()
    actors = []
    files_open = True

    try:
        # Spawn ego on a straight driving lane with 70 m clear ahead
        spawn_points = world.get_map().get_spawn_points()
        ego_bp = bp.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("color", "255,255,255")
        ego = None
        for sp in spawn_points:
            ego_tmp = world.try_spawn_actor(ego_bp, sp)
            if not ego_tmp:
                continue
            wp = world.get_map().get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            dist = 0.0
            ok = True
            w = wp
            while dist < 70.0:
                nxt = w.next(5.0)
                if not nxt:
                    ok = False; break
                w = nxt[0]
                if w.lane_type != carla.LaneType.Driving or w.lane_id * wp.lane_id <= 0:
                    ok = False; break
                dist += 5.0
            if ok:
                ego = ego_tmp
                actors.append(ego)
                break
            ego_tmp.destroy()
        if ego is None:
            print("Failed to spawn ego")
            return
        ego.set_autopilot(False)

        # Pedestrian crossing exactly 50 m ahead (along ego lane)
        ego_wp = world.get_map().get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        ahead = ego_wp
        walked = 0.0
        step = 1.0
        while walked < 50.0:
            nxt = ahead.next(step)
            if not nxt:
                break
            ahead = nxt[0]
            walked += step
        right_vec = ahead.transform.get_right_vector()
        start_loc = ahead.transform.location + carla.Location(x=right_vec.x*3.0, y=right_vec.y*3.0, z=0.5)
        end_loc = ahead.transform.location + carla.Location(x=-right_vec.x*3.0, y=-right_vec.y*3.0, z=0.5)

        walker_bp = bp.filter('walker.pedestrian.*')[0]
        walker = world.try_spawn_actor(walker_bp, carla.Transform(start_loc))
        if walker is None:
            print("Failed to spawn pedestrian")
            return
        actors.append(walker)
        controller_bp = bp.find('controller.ai.walker')
        controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
        actors.append(controller)
        controller.start()
        controller.set_max_speed(1.4)
        controller.go_to_location(end_loc)

        # Sensors
        cam_bp = bp.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(CAM_W))
        cam_bp.set_attribute("image_size_y", str(CAM_H))
        cam_bp.set_attribute("fov", str(CAM_FOV))
        cam_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        cam = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=2.5, z=1.5)), attach_to=ego)
        actors.append(cam)

        lidar_bp = bp.find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", "120")
        lidar_bp.set_attribute("channels", "32")
        lidar_bp.set_attribute("points_per_second", "56000")
        lidar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=2.5)), attach_to=ego)
        actors.append(lidar)

        radar_bp = bp.find("sensor.other.radar")
        radar_bp.set_attribute("horizontal_fov", "30")
        radar_bp.set_attribute("range", "120")
        radar_bp.set_attribute("sensor_tick", str(SENSOR_TICK))
        radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=ego)
        actors.append(radar)

        sensor_data = {"radar_dist": SENTINEL_DIST, "radar_vel": 0.0, "lidar_dist": SENTINEL_DIST}

        def on_radar(meas):
            if not files_open:
                return
            for d in meas:
                radar_w.writerow([meas.timestamp, meas.frame, float(d.depth), float(d.velocity), float(d.azimuth), float(d.altitude)])
            front = [(float(d.depth), abs(float(d.velocity))) for d in meas if abs(float(d.azimuth)) < 20]
            if front:
                depth, vel = min(front, key=lambda x: x[0])
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
            pts = np.frombuffer(meas.raw_data, dtype=np.float32).reshape(-1, 4)
            lidar_w.writerow([meas.timestamp, meas.frame, dist, lateral, len(pts)])

        def on_camera(img):
            if files_open:
                img.save_to_disk(os.path.join(run_dir, "camera", f"{img.frame:06d}.jpg"))

        radar.listen(on_radar)
        lidar.listen(on_lidar)
        cam.listen(on_camera)

        print("\n======================================================================")
        print("SCENARIO START: Pedestrian Crossing with Sensors")
        print("======================================================================\n")

        start = time.time()
        frame = 0
        brake_hold = False
        ped_done = False
        steer_smooth = 0.0

        while time.time() - start < DURATION_SEC:
            world.tick()
            elapsed = time.time() - start

            ego_pos = ego.get_location()
            ego_tf = ego.get_transform()
            ego_speed = get_speed(ego)

            # Ped status
            if walker.is_alive:
                ped_loc = walker.get_location()
                to_ped = ped_loc - ego_pos
                fwd = ego_tf.get_forward_vector()
                right = ego_tf.get_right_vector()
                longitudinal = to_ped.x * fwd.x + to_ped.y * fwd.y
                lateral = to_ped.x * right.x + to_ped.y * right.y
                true_dist = math.sqrt(to_ped.x**2 + to_ped.y**2)
                ped_in_path = (longitudinal > -5 and 0 < longitudinal < 25 and abs(lateral) < 3.0)
                if ped_loc.distance(end_loc) < 1.0:
                    ped_done = True
            else:
                true_dist = SENTINEL_DIST
                ped_in_path = False
                ped_done = True

            # Sensors
            radar_dist = sensor_data["radar_dist"]
            radar_vel = sensor_data["radar_vel"]
            lidar_dist = sensor_data["lidar_dist"]

            # Fusion (gated by true distance if available)
            candidates = []
            if lidar_dist < SENTINEL_DIST and abs(lidar_dist - true_dist) < ASSOC_GATE_M:
                candidates.append(lidar_dist)
            if radar_dist < SENTINEL_DIST and abs(radar_dist - true_dist) < ASSOC_GATE_M:
                candidates.append(radar_dist)
            fused_dist = min(candidates) if candidates else SENTINEL_DIST
            approach_vel = radar_vel if radar_vel > 0.1 else ego_speed
            ttc = calculate_ttc(fused_dist, approach_vel) if fused_dist < SENTINEL_DIST else SENTINEL_DIST

            if (fused_dist < BRAKE_DISTANCE) or (fused_dist < SENTINEL_DIST and ttc < TTC_CRITICAL) or ped_in_path:
                safety = "CRITICAL"
            elif fused_dist < AVOID_DISTANCE:
                safety = "WARNING"
            else:
                safety = "SAFE"

            fusion_w.writerow([elapsed, true_dist, radar_dist, lidar_dist, fused_dist, approach_vel, ttc, safety])

            # Lane-follow steering (simple waypoint lookahead)
            current_wp = world.get_map().get_waypoint(ego_pos, project_to_road=True, lane_type=carla.LaneType.Driving)
            lookahead = 6.0 + ego_speed * 0.3
            next_wps = current_wp.next(lookahead)
            lane_steer = 0.0
            if next_wps:
                target_wp = next_wps[0]
                target_vec = target_wp.transform.location - ego_pos
                target_yaw = math.degrees(math.atan2(target_vec.y, target_vec.x))
                ego_yaw = ego_tf.rotation.yaw
                angle_diff = (target_yaw - ego_yaw + 180) % 360 - 180
                lane_steer = max(-1.0, min(1.0, angle_diff / 30.0))
            steer_smooth = 0.7 * steer_smooth + 0.3 * lane_steer

            # Control logic: stop if ped in path or critical distance
            ego_ctrl = carla.VehicleControl()
            ego_ctrl.hand_brake = False
            brake_trigger = ped_in_path or (fused_dist < BRAKE_DISTANCE) or (fused_dist < SENTINEL_DIST and ttc < TTC_CRITICAL)
            if brake_trigger and not ped_done:
                ego_ctrl.throttle = 0.0
                ego_ctrl.brake = 1.0
                ego_ctrl.steer = 0.0
                action = "STOP_FOR_PED"
                reason = "Pedestrian crossing" if ped_in_path else "Critical distance"
            else:
                ego_ctrl.throttle = 0.6 if ego_speed < 12 else 0.3
                ego_ctrl.brake = 0.0
                ego_ctrl.steer = steer_smooth
                action = "DRIVING"
                reason = "Path clear"

            ego.apply_control(ego_ctrl)

            state_w.writerow([elapsed, ego_speed, ego_pos.x, ego_pos.y, ego_tf.rotation.yaw,
                              ego_ctrl.throttle, ego_ctrl.brake, ego_ctrl.steer,
                              action, reason, ped_in_path])

            if frame % 40 == 0:
                print(f"{elapsed:.1f}s: v={ego_speed:.1f}m/s | fused={fused_dist:.1f}m | true={true_dist:.1f}m | TTC={ttc:.1f}s | {action}")

            frame += 1

        print("\n======================================================================")
        print("SCENARIO COMPLETE")
        print("======================================================================\n")

        files_open = False
        radar.stop(); lidar.stop(); cam.stop()
        radar_f.close(); lidar_f.close(); state_f.close(); fusion_f.close()
        create_video(run_dir, fps=20)
        print(f"\nLogs saved to: {run_dir}\n")

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
