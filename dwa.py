# dwa.py

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def wrap_angle(ang):
    if isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
        ang[ang < -np.pi] += 2 * np.pi
    else:
        while ang > np.pi:
            ang -= 2 * np.pi
        while ang < -np.pi:
            ang += 2 * np.pi
    return ang

class DWA:
    def __init__(self, params):
        self.robot_pose = params['robot_pose']  # Initial robot pose (x, y, orientation)
        self.robot_speeds = params['robot_speeds']  # Initial speeds (linear, angular)
        self.min_v = params['min_v']  # Minimum linear velocity
        self.max_v = params['max_v']  # Maximum linear velocity
        self.min_w = params['min_w']  # Minimum angular velocity
        self.max_w = params['max_w']  # Maximum angular velocity
        self.max_v_acc = params['max_v_acc']  # Maximum linear acceleration
        self.max_w_acc = params['max_w_acc']  # Maximum angular acceleration
        self.dt = params['dt']  # Time step
        self.traj_time = params['traj_time']  # Trajectory prediction time
        self.v_steps = params['v_steps']  # Velocity increments
        self.w_steps = params['w_steps']  # Angular velocity increments
        self.distance_check = params['distance_check']  # Collision distance check
        self.all_trajectories = []  # Store all trajectories for analysis

    def predict_trajectory(self, v, w):
        temp_pose = self.robot_pose.copy()
        trajectory = []
        for _ in range(int(self.traj_time / self.dt)):
            temp_pose[0] += v * np.cos(temp_pose[2]) * self.dt
            temp_pose[1] += v * np.sin(temp_pose[2]) * self.dt
            temp_pose[2] += w * self.dt
            temp_pose[2] = wrap_angle(temp_pose[2])
            trajectory.append(temp_pose.copy())
        self.all_trajectories.append(np.array(trajectory))
        return np.array(trajectory)

    def is_valid(self, pose, obstacles):
        obstacles_array = np.array(obstacles)
        distances = np.linalg.norm(obstacles_array - pose[:2], axis=1)
        return np.all(distances >= self.distance_check)

    def calculate_cost(self, trajectory, goal, obstacles):
        alpha = 1.0  # Weight for the heading cost
        beta = 1.5  # Increased weight for the goal distance cost
        gamma = 0.5  # Weight for the velocity cost

        final_pose = trajectory[-1]
        goal_dist = np.linalg.norm(final_pose[:2] - goal)
        direction_to_goal = np.arctan2(goal[1] - final_pose[1], goal[0] - final_pose[0])
        heading_difference = np.abs(wrap_angle(final_pose[2] - direction_to_goal))
        heading_cost = 1 - np.cos(heading_difference)
        v = np.linalg.norm(trajectory[-1, 0:2] - trajectory[-2, 0:2]) / self.dt
        epsilon = 1e-6  # A small number to prevent division by zero
        velocity_cost = 1 / (v + epsilon)
        cost = alpha * heading_cost + beta * goal_dist + gamma * 1 / velocity_cost

        for pose in trajectory:
            if not self.is_valid(pose, obstacles):
                cost += 100  # Collision penalty

        return cost

    def find_optimal_velocity(self, goal, obstacles):
        best_v, best_w = self.robot_speeds
        best_cost = float('inf')
        best_trajectory = None

        for acc_v in np.arange(-self.max_v_acc, self.max_v_acc + self.v_steps, self.v_steps):
            for acc_w in np.arange(-self.max_w_acc, self.max_w_acc + self.w_steps, self.w_steps):
                new_v = np.clip(best_v + acc_v, self.min_v, self.max_v)
                new_w = np.clip(best_w + acc_w, self.min_w, self.max_w)
                trajectory = self.predict_trajectory(new_v, new_w)
                cost = self.calculate_cost(trajectory, goal, obstacles)

                if new_v == self.min_v or new_v == self.max_v or new_w == self.min_w or new_w == self.max_w:
                    print(f"Velocity Saturation: new_v = {new_v}, new_w = {new_w}, Cost = {cost}")

                if cost < best_cost:
                    best_cost = cost
                    best_v, best_w = new_v, new_w
                    best_trajectory = trajectory

        return best_v, best_w, best_trajectory, best_cost

    def command_wheel_speeds(self, v, w):
        self.robot_pose[0] += v * np.cos(self.robot_pose[2]) * self.dt
        self.robot_pose[1] += v * np.sin(self.robot_pose[2]) * self.dt
        self.robot_pose[2] += w * self.dt
        self.robot_pose[2] = wrap_angle(self.robot_pose[2])

    def simulate_robot(self, goal, obstacles, output_path='robot_motion.gif'):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(self.robot_pose[0], self.robot_pose[1], 'bo', label='Initial Pose')
        ax.plot(goal[0], goal[1], 'rx', label='Goal')
        ax.plot(obstacles[:, 0], obstacles[:, 1], 'ko', label='Obstacles')
        ax.legend()

        path_x, path_y = [self.robot_pose[0]], [self.robot_pose[1]]
        goal_tolerance = 0.2  # Tolerance radius to consider the goal reached

        def update(frame):
            nonlocal ax, path_x, path_y, ani
            self.all_trajectories = []
            ax.clear()

            ax.plot(goal[0], goal[1], 'rx', label='Goal')
            ax.plot(obstacles[:, 0], obstacles[:, 1], 'ko', label='Obstacles')

            v, w, best_trajectory, _ = self.find_optimal_velocity(goal, obstacles)
            self.command_wheel_speeds(v, w)
            path_x.append(self.robot_pose[0])
            path_y.append(self.robot_pose[1])

            ax.plot(self.robot_pose[0], self.robot_pose[1], 'g^', markersize=10, label='Robot')

            for trajectory in self.all_trajectories:
                ax.plot(trajectory[:, 0], trajectory[:, 1], 'y-', alpha=0.1)

            ax.plot(best_trajectory[:, 0], best_trajectory[:, 1], 'r-', label='Best Trajectory', alpha=0.5)

            ax.plot(path_x, path_y, 'g--', label='Path')

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('Robot Simulation with DWA')
            ax.axis('equal')
            ax.grid(True)
            ax.legend()

            # Check if the robot has reached the goal
            if np.linalg.norm(self.robot_pose[:2] - goal) <= goal_tolerance:
                print(f"Goal reached at frame {frame}. Stopping simulation.")
                ani.event_source.stop()

        ani = animation.FuncAnimation(fig, update, frames=range(500), repeat=False)
        plt.show()
