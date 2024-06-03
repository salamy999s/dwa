# run_simulation.py

from dwa import DWA
import numpy as np

def run_simulation(goal, obstacles, params):
    # Create an instance of the DWA class with the defined parameters
    dwa = DWA(params)

    # Run the simulation
    dwa.simulate_robot(goal, obstacles)

# Define common parameters for the DWA class
params = {
    'robot_pose': np.array([0.0, 0.0, np.pi / 2]),
    'robot_speeds': np.array([0.0, 0.0]),
    'min_v': 0.1,
    'max_v': 0.8,
    'min_w': -0.6,
    'max_w': 0.6,
    'max_v_acc': 0.2,
    'max_w_acc': 0.5,
    'dt': 0.1,
    'traj_time': 2.5,
    'v_steps': 0.1,
    'w_steps': 0.1,
    'distance_check': 0.5
}

# Define different environments
environments = [
    {
        'goal': np.array([8, 2]),
        'obstacles': np.array([[3, 2], [4, 4], [5, 6], [6, 3], [7, 5], [2, 7], [1, 5], [5, 3], [2, 4]])
    },
    {
        'goal': np.array([8, 8]),
        'obstacles': np.array([[3, 2], [4, 4], [5, 6], [6, 3], [7, 5], [2, 7], [1, 5], [5, 3], [2, 4]])
    },
    {
        'goal': np.array([5, 5]),
        'obstacles': np.array([[1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 7]])
    }
]

# Run simulations for each environment
for env in environments:
    run_simulation(env['goal'], env['obstacles'], params)

