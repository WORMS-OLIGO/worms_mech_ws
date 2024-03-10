import numpy as np
import pandas as pd
import subprocess
import os

def get_mac_address():
    try:
        mac_address = subprocess.check_output("cat /sys/class/net/wlan0/address", shell=True).decode().strip()
        return mac_address
    except Exception as e:
        print(f"Error getting MAC address: {e}")
        return None

def find_robot_name(mac_address, spreadsheet_path):
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['MAC Address'] == mac_address, 'Species']
    if not match.empty:
        return match.iloc[0]
    else:
        print("Robot species not found. Please check the MAC address and spreadsheet.")
        return None

class JointCommandPublisher:
    def __init__(self):
        self.action = "lift"  # Options: "step", "lift", "prone"
        self.current_position = [50, 50, 50]  # Simulate current robot joint positions

        # Waypoints for actions
        self.action_waypoints = {
            "step": [[0, 145, 175], [15, 145, 175], [15, 135, 175]],
            "lift": [[15, -45, 35]],
            "prone": [[0, -120, 175]]
        }

        self.simulate_actions()

    def interpolate_waypoints(self, waypoints, interval_degrees=1):
        interpolated = [self.current_position]
        for waypoint in waypoints:
            distance = np.linalg.norm(np.array(waypoint) - np.array(interpolated[-1]))
            steps = int(distance / interval_degrees)
            for step in range(1, steps + 1):
                interpolated_point = interpolated[-1] + (np.array(waypoint) - np.array(interpolated[-1])) * (step / steps)
                interpolated.append(interpolated_point)
        return interpolated

    def simulate_actions(self):
        if self.action in self.action_waypoints:
            print(f"Simulating '{self.action}' action...")
            waypoints = self.action_waypoints[self.action]
            interpolated_positions = self.interpolate_waypoints(waypoints)
            for position in interpolated_positions:
                print(f"Simulating move to: {position}")
        else:
            print("Invalid action.")

def main():
    joint_command_publisher = JointCommandPublisher()

if __name__ == '__main__':
    main()