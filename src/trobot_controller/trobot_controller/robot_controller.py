#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("move_control")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/trobot_controller/cmd_vel", 10)

        # Parameters
        self.distance = 12.0  # Target distance (meters)
        self.radius = 0.5     # Wheel radius (meters)
        self.desired_time = 10.0  # Desired time to travel the distance (seconds)
        self.rpm = None       # If provided, overrides time calculation

        # Calculate travel time if needed
        self.travel_time = self.calculate_travel_time()
        self.start_time = None  # To track when motion starts

        # Timer to control the robot
        self.timer_ = self.create_timer(0.1, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def calculate_travel_time(self):
        if self.desired_time is not None:
            return self.desired_time
        elif self.rpm is not None:
            result = calculate_rpm_or_time(distance=self.distance, radius=self.radius, rpm=self.rpm)
            # Extract the RPM value from the dictionary
            #self.rpm = result["rpm"]  # Correctly assign the numeric RPM value
            return result["time"]
        else:
            raise ValueError("Either 'desired_time' or 'rpm' must be specified")

    def send_velocity_command(self):
        # Initialize start time
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info(f"Starting motion for {self.travel_time:.2f} seconds")

        # Check elapsed time
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= self.travel_time:
            # Stop the robot
            self.stop_robot()
            self.get_logger().info("Reached target distance, stopping the robot.")
            self.destroy_timer(self.timer_)  # Stop the timer
            return

        # Send velocity command
        msg = Twist()
        if self.rpm is not None:
            msg.linear.x = float(calculate_linear_velocity_from_rpm(self.rpm, self.radius)) # Set desired linear velocity
        else:
            # Calculate rpm if not provided
            result = calculate_rpm_or_time(distance=self.distance, radius=self.radius, time=self.desired_time)
            self.rpm = result["rpm"]  # Assign the numeric RPM value
            msg.linear.x = float(calculate_linear_velocity_from_rpm(self.rpm, self.radius))
        msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(msg)

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(stop_msg)

def calculate_rpm_or_time(distance, radius, rpm=None, time=None):
    # Convert distance to angular distance in radians
    angular_distance = distance / radius  # in radians

    if time is not None:  # If time is provided, calculate RPM
        angular_velocity_rad_per_s = angular_distance / time
        rpm_calculated = angular_velocity_rad_per_s * (60 / (2 * math.pi))
        return {"rpm": rpm_calculated}

    elif rpm is not None:  # If RPM is provided, calculate time
        angular_velocity_rad_per_s = rpm * (2 * math.pi / 60)
        time_calculated = angular_distance / angular_velocity_rad_per_s
        return {"time": time_calculated}

    else:
        raise ValueError("Either 'rpm' or 'time' must be provided.")
    
def calculate_linear_velocity_from_rpm(rpm, radius):
    angular_velocity = rpm * (2 * math.pi / 60)  # Convert RPM to rad/s
    linear_velocity = angular_velocity * radius  # Convert rad/s to m/s
    return linear_velocity


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
