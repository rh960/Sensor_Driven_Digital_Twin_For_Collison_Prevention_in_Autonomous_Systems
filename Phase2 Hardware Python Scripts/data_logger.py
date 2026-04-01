"""
data_logger.py  —  runs on Jetson
Logs obstacle detection and avoidance events to CSV.
Only saves rows when an obstacle is detected or motor state changes.

Compatible with simple MotorController (retry_count, no locked flags).

CSV columns:
    timestamp, fusion_level, lidar_fl_m, lidar_fc_m, lidar_fr_m,
    radar_range_m, radar_velocity_mps, radar_ttc_s,
    camera_class, camera_dist_level,
    motor_state, retry_count, action_taken
"""

import csv
import os
from datetime import datetime


class DataLogger:
    def __init__(self, log_dir="/home/digit/robot_env/logs"):
        os.makedirs(log_dir, exist_ok=True)
        timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
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
            "retry_count",
            "action_taken"
        ]

        with open(self._path, 'w', newline='') as f:
            csv.writer(f).writerow(self._headers)

        self._last_state = "NORMAL"
        self._last_level = "SAFE"
        print(f"[LOGGER] Logging to {self._path}")

    def log(self, fusion_level, fl, fc, fr,
            radar_tracks, cam_dets, motor):
        """
        motor = MotorController instance (reads _state and _retry_count directly)
        Only writes when obstacle detected or state/level changes.
        """
        motor_state = motor._state       if motor else "UNKNOWN"
        retry_count = motor._retry_count if motor else 0

        is_obstacle   = fusion_level in ("CAUTION", "IMMINENT")
        state_changed = motor_state != self._last_state
        level_changed = fusion_level != self._last_level

        if not (is_obstacle or state_changed or level_changed):
            return

        self._last_state = motor_state
        self._last_level = fusion_level

        # Radar — closest track
        radar_range    = "none"
        radar_velocity = "none"
        radar_ttc      = "none"
        if radar_tracks:
            try:
                closest        = min(radar_tracks, key=lambda t: t.range_m)
                radar_range    = round(closest.range_m, 2)
                radar_velocity = round(closest.vel_mps, 2)
                radar_ttc      = round(closest.ttc_s, 2) if closest.ttc_s else "none"
            except Exception:
                pass

        # Camera — most confident detection
        cam_class = "none"
        cam_dist  = "none"
        if cam_dets:
            try:
                best      = max(cam_dets, key=lambda d: d.get("conf", 0))
                cam_class = best.get("label", "foreign") or "foreign"
                cam_dist  = best.get("dist_level", "none")
            except Exception:
                pass

        row = [
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            fusion_level,
            round(fl, 2) if fl is not None else "none",
            round(fc, 2) if fc is not None else "none",
            round(fr, 2) if fr is not None else "none",
            radar_range,
            radar_velocity,
            radar_ttc,
            cam_class,
            cam_dist,
            motor_state,
            retry_count,
            _action_from_state(motor_state)
        ]

        try:
            with open(self._path, 'a', newline='') as f:
                csv.writer(f).writerow(row)
        except Exception as e:
            print(f"[LOGGER] Write error: {e}")

    def get_path(self):
        return self._path


def _action_from_state(motor_state):
    return {
        "NORMAL":        "driving_forward",
        "STOPPING":      "braking_to_stop",
        "REVERSING":     "reversing_straight",
        "TURNING":       "turning_escape",
        "ESCAPING":      "forward_on_new_heading",
        "STRAIGHTENING": "recentring_steering",
    }.get(motor_state, "unknown")
