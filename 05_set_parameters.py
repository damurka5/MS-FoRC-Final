"""Example of setting dynamic parameters for the UR5e robot.

This script demonstrates how to modify dynamic parameters of the robot including:
- Joint damping coefficients
- Joint friction
- Link masses and inertias

The example uses a simple PD controller to show the effects of parameter changes.
"""

import numpy as np
from simulator import Simulator
from pathlib import Path
import os
from typing import Dict
import matplotlib.pyplot as plt

def joint_controller(q: np.ndarray, dq: np.ndarray, q0: np.array, t: float) -> np.ndarray:
    """Joint space PD controller.
    
    Args:
        q: Current joint positions [rad]
        dq: Current joint velocities [rad/s]
        t: Current simulation time [s]
        q0: Desired joint positions
        
    Returns:
        tau: Joint torques command [Nm]
    """
    # Control gains tuned for UR5e
    kp = np.array([200, 400, 160, 10, 10, 0.1])
    kd = np.array([20, 40, 40, 2, 2, 0.01])
    
    # Target joint configuration
    # q0 = np.array([-1.4, -1.3, 1., 0, 0, 0])
    
    # PD control law
    tau = kp * (q0 - q) - kd * dq
    return tau

def sliding_mode_controller(q: np.ndarray, dq: np.ndarray, q0: np.array, t: float) -> np.ndarray:
    """Sliding mode controller.
    
    Args:
        q: Current joint positions [rad]
        dq: Current joint velocities [rad/s]
        t: Current simulation time [s]
        q0: Desired joint positions
        
    Returns:
        tau: Joint torques command [Nm]
    """
    # Control gains tuned for UR5e
    kp = np.array([200, 400, 160, 10, 10, 0.1])
    kd = np.array([20, 40, 40, 2, 2, 0.01])
    
    # Target joint configuration
    # q0 = np.array([-1.4, -1.3, 1., 0, 0, 0])
    
    # PD control law
    tau = kp * (q0 - q) - kd * dq
    return tau

def plot_results(times: np.ndarray, positions: np.ndarray, velocities: np.ndarray, desired_positions: np.ndarray, fname: str):
    """Plot and save simulation results."""
    # Joint positions plot
    plt.figure(figsize=(10, 6))
    for i in range(positions.shape[1]):
        color = plt.cm.tab10(i / 10)  # Get a unique color for each joint
        plt.plot(times, positions[:, i], label=f'Joint {i+1}', color=color)
        plt.axhline(y=desired_positions[i], color=color, linestyle='--', label=f'Desired Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Positions [rad]')
    plt.title('Joint Positions over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'logs/plots/05_positions_{fname}.png')
    plt.close()
    
    # Joint velocities plot
    plt.figure(figsize=(10, 6))
    for i in range(velocities.shape[1]):
        plt.plot(times, velocities[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Velocities [rad/s]')
    plt.title('Joint Velocities over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'logs/plots/05_velocities_{fname}.png')
    plt.close()

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    Path("logs/plots").mkdir(parents=True, exist_ok=True)
    # Initialize simulator
    sim = Simulator(
        xml_path="robots/universal_robots_ur5e/scene.xml",
        enable_task_space=False,  # Using joint space control
        show_viewer=False,
        record_video=True,
        video_path="logs/videos/08_parameters.mp4",
        fps=30,
        width=1920,
        height=1080
    )
    
    # Set joint damping (example values, adjust as needed)
    damping = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])  # Nm/rad/s
    sim.set_joint_damping(damping)
    
    # Set joint friction (example values, adjust as needed)
    friction = np.array([1.5, 0.5, 0.5, 0.1, 0.1, 0.1])  # Nm
    sim.set_joint_friction(friction)
    
    # Get original properties
    ee_name = "end_effector"
    
    original_props = sim.get_body_properties(ee_name)
    print(f"\nOriginal end-effector properties:")
    print(f"Mass: {original_props['mass']:.3f} kg")
    print(f"Inertia:\n{original_props['inertia']}")
    
    # Add the end-effector mass and inertia
    sim.modify_body_properties(ee_name, mass=15)
    # Print modified properties
    props = sim.get_body_properties(ee_name)
    print(f"\nModified end-effector properties:")
    print(f"Mass: {props['mass']:.3f} kg")
    print(f"Inertia:\n{props['inertia']}")

    # Simulation parameters
    t = 0
    dt = sim.dt
    time_limit = 10.0
    target_q = np.array([-1.4, -1.3, 1., 0, 0, 0])
    
    # Data collection
    times = []
    positions = []
    velocities = []
    
    while t < time_limit:
        state = sim.get_state()
        times.append(t)
        positions.append(state['q'])
        velocities.append(state['dq'])
        
        tau = joint_controller(q=state['q'], dq=state['dq'], q0=target_q, t=t)
        # print(tau)
        sim.step(tau)
        
        if sim.record_video and len(sim.frames) < sim.fps * t:
            sim.frames.append(sim._capture_frame())
        t += dt
    
    # Process and save results
    times = np.array(times)
    positions = np.array(positions)
    velocities = np.array(velocities)
    
    print(f"Simulation completed: {len(times)} steps")
    print(f"Final joint positions: {positions[-1]}")
    
    sim._save_video()
    plot_results(times, positions, velocities, target_q, fname='simple_id_control')
    
if __name__ == "__main__":
    main() 