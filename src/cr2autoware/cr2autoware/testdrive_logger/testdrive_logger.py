import os
import json
import numpy as np
import threading

class TestDriveLogger:
    def __init__(self, save_path, save_id):
        self.log_dir = save_path
        self.log_file_path = os.path.join(self.log_dir, f"testdrive_logging_{save_id}.ndjson")
        self.data = []
        self.lock = threading.Lock()
        self.save_thread = None

        with open(self.log_file_path, "w") as log_file:
            json.dump([], log_file)

    def log_data(
        self,
        global_time,
        scenario_time,
        behavior_time,
        trajectory_time,
        total_cycle_time,
        current_velocity,
        velocity_profile,
        current_behavior_velocity,
        smoothed_velocity,
        current_smoothed_velocity,
        current_position,
        curvilinear_path,
    ):
        self.data.append(
            {
                "global_time": global_time,
                "scenario_update": scenario_time,
                "behavior_planning": behavior_time,
                "trajectory_planning": trajectory_time,
                "total_cycle_time": total_cycle_time,
                "current_velocity": current_velocity,
                "velocity_profile": (
                    velocity_profile.tolist()
                    if isinstance(velocity_profile, np.ndarray)
                    else velocity_profile
                ),
                "current_behavior_velocity": current_behavior_velocity,
                "smoothed_velocity": (
                    smoothed_velocity.tolist()
                    if isinstance(velocity_profile, np.ndarray)
                    else velocity_profile
                ),
                "current_smoothed_velocity": current_smoothed_velocity,
                "current_position": (
                    current_position.tolist()
                    if isinstance(velocity_profile, np.ndarray)
                    else velocity_profile
                ),
                "curvilinear_path": (
                    curvilinear_path.tolist()
                    if isinstance(velocity_profile, np.ndarray)
                    else velocity_profile
                ),
            }
        )

    def save_to_file(self):

        if self.save_thread is not None and self.save_thread.is_alive():
            return

        self.save_thread = threading.Thread(target=self._save_to_file)
        self.save_thread.start()

    def _save_to_file(self):
        with self.lock:

            with open(self.log_file_path, "a") as log_file:
                json.dump(self.data, log_file)

            self.data = []
