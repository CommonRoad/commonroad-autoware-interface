import os
import json
import numpy as np

class TestDriveLogger:
    def __init__(self, save_path, save_id):
        self.log_dir = save_path
        self.log_file_path = os.path.join(self.log_dir, f"testdrive_logging_{save_id}.ndjson")
        self.data = []

        with open(self.log_file_path, "w") as log_file:
            json.dump([], log_file)

    def log_data(self, scenario_time, behavior_time, trajectory_time, total_cycle_time, saving_time, current_velocity, velocity_profile, smoothed_velocity, current_position, curvilinear_path):
        self.data.append({
            "scenario_update": scenario_time,
            "behavior_planning": behavior_time,
            "trajectory_planning": trajectory_time,
            "total_cycle_time": total_cycle_time - saving_time,
            "saving_time": saving_time,
            "current_velocity": current_velocity,
            "velocity_profile": velocity_profile.tolist() if isinstance(velocity_profile, np.ndarray) else velocity_profile,
            "smoothed_velocity": smoothed_velocity.tolist() if isinstance(velocity_profile, np.ndarray) else velocity_profile,
            "current_position": current_position.tolist() if isinstance(velocity_profile, np.ndarray) else velocity_profile,
            "curvilinear_path": curvilinear_path.tolist() if isinstance(velocity_profile, np.ndarray) else velocity_profile,
        })

    def save_to_file(self):

        with open(self.log_file_path, "r") as log_file:
            existing_data = json.load(log_file)

        existing_data.extend(self.data)

        with open(self.log_file_path, "w") as log_file:
            json.dump(existing_data, log_file)

        self.data = []
