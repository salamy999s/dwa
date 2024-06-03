
# Dynamic Window Approach (DWA) Simulation

This repository contains a simulation of the Dynamic Window Approach (DWA) for mobile robot path planning. The simulation is implemented in Python using the `matplotlib` library for visualization.

## GitHub Repository Link
https://github.com/salamy999s/dwa

## Overview

The Dynamic Window Approach is an online collision avoidance strategy for mobile robots. It considers the robot's dynamics and calculates the best velocity commands to reach a goal while avoiding obstacles.

## Files

- `dwa.py`: Contains the `DWA` class that implements the Dynamic Window Approach.
- `run_simulation.py`: Script to run the simulation with different environments.
- `README.md`: This file, explaining the contents and usage of the repository.

## DWA Class (`dwa.py`)

The `DWA` class implements the Dynamic Window Approach algorithm. The class has methods for predicting trajectories, calculating costs, and finding the optimal velocities. It also includes a method to simulate the robot's motion and visualize the results.

### Class Initialization

The class can be initialized with custom parameters or default values.

```python
class DWA:
    def __init__(self, 
                 robot_pose=np.array([1.0, 1.0, np.pi / 2]), 
                 robot_speeds=np.array([0.0, 0.0]), 
                 min_v=0.1, 
                 max_v=0.8, 
                 min_w=-0.6, 
                 max_w=0.6, 
                 max_v_acc=0.2, 
                 max_w_acc=0.5, 
                 dt=0.1, 
                 traj_time=2.5, 
                 v_steps=0.04, 
                 w_steps=0.04, 
                 distance_check=1.5):
```

### Methods

- `predict_trajectory(v, w)`: Predicts the robot's trajectory given linear and angular velocities.
- `is_valid(pose, obstacles)`: Checks if a given pose is valid (i.e., not in collision with obstacles).
- `calculate_cost(trajectory, goal, obstacles)`: Calculates the cost of a trajectory based on heading, distance to goal, and velocity.
- `find_optimal_velocity(goal, obstacles)`: Finds the optimal linear and angular velocities to reach the goal while avoiding 
obstacles.
- `command_wheel_speeds(v, w)`: Updates the robot's pose based on the given velocities.
- `simulate_robot(goal, obstacles, output_path='robot_motion.gif')`: Simulates the robot's motion and visualizes the results. The simulation stops when the robot reaches the goal.


###Using matplotlib.animation for Simulation

The matplotlib.animation module in the matplotlib library is used to create animations by repeatedly calling a function, updating the plot each time. This is particularly useful for visualizing dynamic systems like the robot's motion in our DWA simulation.
Key Functionality

    FuncAnimation: This function makes an animation by repeatedly calling a function (update in our case) at specified intervals. It updates the figure each time, creating an animation effec

## Running the Simulation (`run_simulation.py`)

The `run_simulation.py` script demonstrates how to use the `DWA` class to run simulations in different environments.

### Defining Parameters

First, define the parameters for the DWA algorithm.

```python
params = {
    'robot_pose': np.array([1.0, 1.0, np.pi / 2]),
    'robot_speeds': np.array([0.0, 0.0]),
    'min_v': 0.1,
    'max_v': 0.8,
    'min_w': -0.6,
    'max_w': 0.6,
    'max_v_acc': 0.2,
    'max_w_acc': 0.5,
    'dt': 0.1,
    'traj_time': 2.5,
    'v_steps': 0.04,
    'w_steps': 0.04,
    'distance_check': 1.5
}
```

### Defining Environments

Next, define different environments with various goals and obstacle configurations.

```python
environments = [
    {
        'goal': np.array([8, 8]),
        'obstacles': np.array([[3, 2], [4, 4], [5, 6], [6, 3], [7, 5], [2, 7], [1, 5], [5, 3], [2, 4]])
    },
    {
        'goal': np.array([10, 10]),
        'obstacles': np.array([[2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7]])
    },
    {
        'goal': np.array([5, 5]),
        'obstacles': np.array([[1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 7]])
    }
]
```

### Running the Simulations

Run the simulations for each environment.

```python
for env in environments:
    run_simulation(env['goal'], env['obstacles'], params)
```

The `run_simulation` function creates an instance of the `DWA` class with the given parameters and runs the simulation.

```python
def run_simulation(goal, obstacles, params):
    dwa = DWA(
        robot_pose=params['robot_pose'],
        robot_speeds=params['robot_speeds'],
        min_v=params['min_v'],
        max_v=params['max_v'],
        min_w=params['min_w'],
        max_w=params['max_w'],
        max_v_acc=params['max_v_acc'],
        max_w_acc=params['max_w_acc'],
        dt=params['dt'],
        traj_time=params['traj_time'],
        v_steps=params['v_steps'],
        w_steps=params['w_steps'],
        distance_check=params['distance_check']
    )
    dwa.simulate_robot(goal, obstacles)
```

## Usage

1. Clone the repository.
2. Install the required dependencies:

    pip install numpy matplotlib

3. Run the simulation:

    python3 run_simulation.py


