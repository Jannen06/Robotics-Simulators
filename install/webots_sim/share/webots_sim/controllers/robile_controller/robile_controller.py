#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from controller import Robot
import math
import sys


class Robile4WDController(Node):
    def __init__(self):
        super().__init__('robile_controller')
        self.get_logger().info("Robile4WDController node starting...")

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.get_logger().info(f"Webots timestep: {self.timestep} ms")

        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_base', 0.20)
        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter(
            'wheel_base').get_parameter_value().double_value
        self.get_logger().info(
            f"Robot parameters: Wheel Radius = {self.wheel_radius} m, Wheel Base = {self.wheel_base} m")

        # TF topic
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF2 listener initialized.")

        self.get_logger().info("Attempting to get all four motors...")
        try:
            self.front_left_motor = self.robot.getDevice(
                'front left wheel motor')
            self.front_right_motor = self.robot.getDevice(
                'front right wheel motor')
            self.rear_left_motor = self.robot.getDevice(
                'rear left wheel motor')
            self.rear_right_motor = self.robot.getDevice(
                'rear right wheel motor')

            if None in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
                raise ValueError(
                    "One or more motors were not found. Check your Webots robot PROTO names.")

            self.motors = [self.front_left_motor, self.front_right_motor,
                           self.rear_left_motor, self.rear_right_motor]
            self.get_logger().info("All four motors found successfully.")

        except ValueError as e:
            self.get_logger().error(
                f"Error initializing motors: {e}. Exiting.")
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(
                f"An unexpected error occurred while getting motors: {e}. Exiting.")
            sys.exit(1)

        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        self.get_logger().info("All motors initialized to 0.0 velocity.")

        try:
            self.lidar = self.robot.getDevice('lidar')
            self.lidar.enable(self.timestep)

            self.lidar_resolution = self.lidar.getHorizontalResolution()
            self.fov = self.lidar.getFov()
            self.min_range = self.lidar.getMinRange()
            self.max_range = self.lidar.getMaxRange()
            self.get_logger().info(
                f"LiDAR initialized: resolution={self.lidar_resolution}, fov={math.degrees(self.fov):.2f}°, range=({self.min_range}, {self.max_range})")

            self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize LiDAR: {e}")
            sys.exit(1)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribed to /cmd_vel topic.")

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().debug(
            f"Received cmd_vel: linear_x={self.linear_velocity:.2f}, angular_z={self.angular_velocity:.2f}")

    def run(self):
        self.get_logger().info("Starting robot control loop...")
        while True:
            step_result = self.robot.step(self.timestep)

            if step_result == -1 or not rclpy.ok():
                self.get_logger().info("Simulation or ROS 2 context ended. Exiting control loop.")
                break

            v_l = (self.linear_velocity - self.angular_velocity *
                   self.wheel_base / 2.0) / self.wheel_radius
            v_r = (self.linear_velocity + self.angular_velocity *
                   self.wheel_base / 2.0) / self.wheel_radius

            self.front_left_motor.setVelocity(v_l)
            self.rear_left_motor.setVelocity(v_l)
            self.front_right_motor.setVelocity(v_r)
            self.rear_right_motor.setVelocity(v_r)

            ranges = self.lidar.getRangeImage()

            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser'

            scan_msg.angle_min = -self.fov / 2.0
            scan_msg.angle_max = self.fov / 2.0
            scan_msg.angle_increment = self.fov / self.lidar_resolution
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = self.timestep / 1000.0
            scan_msg.range_min = self.min_range
            scan_msg.range_max = self.max_range
            scan_msg.ranges = list(ranges)

            self.lidar_pub.publish(scan_msg)

            rclpy.spin_once(self, timeout_sec=0.0)

        self.get_logger().info("Robile4WDController loop finished cleanly.")


def main(args=None):
    rclpy.init(args=args)
    node = Robile4WDController()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Shutting down.")
    except Exception as e:
        node.get_logger().error(
            f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("Robile4WDController node shut down.")


if __name__ == '__main__':
    main()
