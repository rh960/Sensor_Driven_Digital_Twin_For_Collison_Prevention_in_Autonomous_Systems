"""
data_logger.py  —  runs on Jetson
Logs obstacle detection and avoidance events to CSV.
Only saves rows when an obstacle is actually detected.

CSV columns:
    timestamp, fusion_level, lidar_fl, lidar_fc, lidar_fr,
    radar_range_m, radar_velocity_mps, radar_ttc_s,
    camera_class, camera_dist_level,
    motor_state, action_taken
"""

import csv
import os
import time
from datetime import datetime


class DataLogger:
    def __init__(self, log_dir="/home/digit/robot_env/logs"):
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(log_dir, f"avoidance_log_{timestamp}.csv")

        self._headers = [
            "timestamp",
            "fusion_level",
            "lidar_fl_m",
            "lidar_fc_m",
            "lidar_fr_m",
            "radar_range_m",
            "radar_velocity_mps",
            "radar_ttc_s",
            "camera_class",
            "camera_dist_level",
            "motor_state",
            "action_taken"
        ]

        with open(self._path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self._headers)

        self._last_state  = "NORMAL"
        self._last_level  = "SAFE"
        print(f"[LOGGER] Logging to {self._path}")

    def log(self, fusion_level, fl, fc, fr,
            radar_tracks, cam_dets, motor_state):
        """
        Call every fusion tick.
        Only writes to CSV when obstacle detected or motor state changes.
        """
        # Only log when something is happening
        is_obstacle   = fusion_level in ("CAUTION", "IMMINENT")
        state_changed = motor_state != self._last_state
        level_changed = fusion_level != self._last_level

        if not (is_obstacle or state_changed or level_changed):
            return

        self._last_state = motor_state
        self._last_level = fusion_level

        # Radar — closest track
        radar_range    = None
        radar_velocity = None
        radar_ttc      = None
        if radar_tracks:
            closest = min(radar_tracks, key=lambda t: t.range_m)
            radar_range    = round(closest.range_m, 2)
            radar_velocity = round(closest.vel_mps, 2)
            radar_ttc      = round(closest.ttc_s, 2) if closest.ttc_s else None

        # Camera — closest/most confident detection
        cam_class = "none"
        cam_dist  = "none"
        if cam_dets:
            best = max(cam_dets, key=lambda d: d.get("conf", 0))
            cam_class = best.get("label", "foreign")
            cam_dist  = best.get("dist_level", "none")
            if not cam_class:
                cam_class = "foreign"

        # Determine action taken
        action = _action_from_state(motor_state, fusion_level)

        row = [
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            fusion_level,
            round(fl, 2) if fl is not None else "none",
            round(fc, 2) if fc is not None else "none",
            round(fr, 2) if fr is not None else "none",
            radar_range    if radar_range    is not None else "none",
            radar_velocity if radar_velocity is not None else "none",
            radar_ttc      if radar_ttc      is not None else "none",
            cam_class,
            cam_dist,
            motor_state,
            action
        ]

        try:
            with open(self._path, 'a', newline='') as f:
                csv.writer(f).writerow(row)
        except Exception as e:
            print(f"[LOGGER] Write error: {e}")

    def get_path(self):
        return self._path


def _action_from_state(motor_state, fusion_level):
    mapping = {
        "NORMAL":        "driving_forward",
        "STOPPING":      "braking_to_stop",
        "REVERSING":     "reversing_straight",
        "TURNING":       "point_turn_escape",
        "ESCAPING":      "forward_on_new_heading",
        "STRAIGHTENING": "recentring_steering",
    }
    return mapping.get(motor_state, "unknown")