# Maze Navigation and Path Optimization for an Autonomous Robot

**Authors:** Kamren James and Mikayla Lewis  
**Advisor:** Dr. Mohammad Habibi  
**Institution:** Tennessee State University, College of Engineering

## Overview

This project focuses on the development of deep reinforcement learning (DRL) algorithms for autonomous mapping and path planning. The objective is to design a virtual robot that can autonomously explore a maze and plan the most efficient path to a destination while avoiding obstacles.

The system utilizes the **Twin Delayed Deep Deterministic Policy Gradient (TD3)** algorithm, an advanced Actor-Critic method, to train the robot in a simulated environment. The project leverages **ROS2 Foxy** and **Gazebo** for simulation and **PyTorch** for the neural network implementation.

<p align="center">
    <img width="70%" src="Test_example_env1.gif" alt="Simulation Example">
</p>

## Problem Statement

Navigating through complex environments efficiently is a fundamental challenge in robotics. This project addresses the need for advanced algorithms that enable robots to:
1.  **Navigate** through intricate mazes without human intervention.
2.  **Avoid Obstacles** using LIDAR sensor data.
3.  **Optimize Paths** to reach a target goal in the shortest time.

## System Architecture

### Software Stack
*   **Operating System:** Linux Mint / Ubuntu 20.04
*   **Robot Operating System:** ROS2 Foxy
*   **Simulation:** Gazebo 11
*   **Visualization:** Rviz
*   **Machine Learning:** PyTorch (Python)

### Neural Network: TD3 (Twin Delayed DDPG)
TD3 is an actor-critic algorithm that addresses the overestimation bias of DDPG.
*   **Actor Network:** Takes the environmental state as input and outputs the optimal action (velocity and angular velocity).
*   **Critic Networks:** Two critic networks estimate the Q-value of the state-action pair. The minimum of the two Q-values is used to prevent overestimation.

#### Network Architecture
The architecture consists of an Actor network for decision making and two Critic networks for value estimation.

<p align="center">
    <img width="45%" src="Actor.png" alt="Actor Network">
    <img width="45%" src="Critic.png" alt="Critic Network">
</p>

<p align="center">
    <img width="80%" src="Td3.png" alt="TD3 Architecture">
</p>

### Virtual Robot & Environment
*   **Sensors:** Simulated LIDAR (Light Detection and Ranging) for distance sensing (180-degree range).
*   **State Space:**
    *   LIDAR readings (distance to obstacles)
    *   Distance to goal
    *   Angle to goal relative to heading
    *   Previous action
*   **Action Space:**
    *   Translational velocity ($v$)
    *   Angular velocity ($\omega$)
*   **Reward Function:**
    *   **Goal Reached:** +100
    *   **Collision:** -100
    *   **Step Reward:** $v - |\omega| - penalty$. The penalty discourages being too close to obstacles.

<p align="center">
    <img width="60%" src="Training_env.png" alt="Training Environment">
</p>

## Hardware Prototype (Design Phase)
A physical prototype was designed to validate the concept, although the primary training and testing remained in the simulation.
*   **Microcomputer:** Raspberry Pi
*   **Sensors:** RPLIDAR A1
*   **Actuation:** 12V DC Motors with Encoders, H-Bridge Motor Drivers.
*   **Power:** Lithium-Ion Battery (6000mAh).

## Results
The model was trained for over 2000 iterations.
*   **Average Q-Value:** showed improvement after initial exploration, indicating the robot was learning better value estimations for its actions.
*   **Max Q-Value:** showed the peak expected rewards the robot anticipated.

<p align="center">
    <img width="70%" src="Tensorboard.PNG" alt="Tensorboard Results">
</p>

## Installation & Usage

### Prerequisites
*   [PyTorch](https://pytorch.org/get-started/locally/)
*   [Tensorboard](https://github.com/tensorflow/tensorboard)
*   [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

### Build Instructions
1.  **Install dependencies:**
    ```shell
    sudo apt install python3-colcon-common-extensions
    sudo apt install ros-foxy-gazebo-ros-pkgs
    sudo apt install ros-foxy-xacro
    ```
2.  **Clone the repository:**
    ```shell
    git clone https://github.com/Kjames3/capstone_bot.git
    cd capstone_bot
    ```
3.  **Build the workspace:**
    ```shell
    source /opt/ros/foxy/setup.bash
    colcon build
    source install/setup.bash
    ```

### Running the Simulation
**Training:**
```shell
ros2 launch Capstone_Project training_simulation.launch.py
```
**Monitoring:**
```shell
tensorboard dev upload --logdir './src/Capstone_Project/runs/train/tensorboard'
```

**Testing:**
```shell
ros2 launch Capstone_Project test_simulation.launch.py
```

## Future Recommendations
*   **Central Server:** Implement the network model on a central server to offload computation from the robot.
*   **Data Optimization:** Improve data storage and retrieval for saved evaluation metrics.


