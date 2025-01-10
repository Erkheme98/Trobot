# Trobot
## Simple wheeled robot in Isaac  Sim with ROS2 Humble
## Node: DrawCircleNode

The `DrawCircleNode` class is responsible for controlling the robot's movement. It publishes velocity commands to the `/trobot_controller/cmd_vel` topic.

### Parameters

- `distance`: Target distance to travel (meters).
- `radius`: Wheel radius (meters).
- `desired_time`: Desired time to travel the distance (seconds).
- `rpm`: Revolutions per minute (optional, overrides time calculation).

### Methods

- `calculate_travel_time()`: Calculates the travel time based on the desired time or RPM.
- `send_velocity_command()`: Sends velocity commands to the robot.
- `stop_robot()`: Stops the robot by publishing zero velocity.

## Installation

1. Clone the repository:
    ```sh
    git clone <repository-url>
    cd <repository-directory>
    ```

2. Build the project using colcon:
    ```sh
    colcon build
    ```

3. Source the setup script:
    ```sh
    source install/setup.bash
    ```

## Usage

Run the [DrawCircleNode](http://_vscodecontentref_/3):
```sh
ros2 run trobot_controller draw_circle_node

## System

![image](https://github.com/user-attachments/assets/4b5a17ea-d924-4a45-b6d2-3ff16d27198c)
