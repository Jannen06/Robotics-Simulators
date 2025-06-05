#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller import Robot
import sys 

class RobileController(Node):
    def __init__(self):
        super().__init__('robile_controller')
        self.get_logger().info("RobileController node starting...")

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.get_logger().info(f"Webots timestep: {self.timestep} ms")

        self.declare_parameter('wheel_radius', 0.03) 
        self.declare_parameter('wheel_base', 0.20)     
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.get_logger().info(f"Robot parameters: Wheel Radius = {self.wheel_radius} m, Wheel Base = {self.wheel_base} m")

        self.get_logger().info("Attempting to get motors...")
        self.left_motor = self.robot.getDevice('front left wheel motor')
        self.right_motor = self.robot.getDevice('front right wheel motor')

        
        if self.left_motor is None:
            self.get_logger().error("Left motor 'robile_1_drive_left_motor' was NOT found by getDevice(). Exiting.")
            sys.exit(1) 
        else:
            self.get_logger().info("Left motor 'robile_1_drive_left_motor' was found.")

        if self.right_motor is None:
            self.get_logger().error("Right motor 'robile_1_drive_right_motor' was NOT found by getDevice(). Exiting.")
            sys.exit(1) 
        else:
            self.get_logger().info("Right motor 'robile_1_drive_right_motor' was found.")

        self.left_motor.setPosition(float('inf')) 
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.get_logger().info("Motors initialized to 0.0 velocity.")

        
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
        """
        Callback function for /cmd_vel topic.
        Updates the desired linear and angular velocities.
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.get_logger().debug(f"Received cmd_vel: linear_x={self.linear_velocity:.2f}, angular_z={self.angular_velocity:.2f}")

    def run(self):
        """
        Main loop for the robot controller.
        Steps the Webots simulation and updates motor velocities based on received cmd_vel.
        """
        self.get_logger().info("Starting robot control loop...")
        loop_count = 0
        while True: 
            self.get_logger().info(f"--- Entering control loop iteration {loop_count + 1} ---")
            

            step_result = self.robot.step(self.timestep)
            
            if step_result == -1:
                self.get_logger().warn(f"Webots simulation step returned -1. Controller loop ending after {loop_count} iterations.")
                break 
            
            if not rclpy.ok():
                self.get_logger().warn(f"ROS 2 context (rclpy) is no longer OK. Controller loop ending after {loop_count} iterations.")
                break 

            loop_count += 1
            self.get_logger().debug(f"Robot control loop iteration: {loop_count}. Webots step result: {step_result}")

           
            v_l = (self.linear_velocity - self.angular_velocity * self.wheel_base / 2.0) / self.wheel_radius
            v_r = (self.linear_velocity + self.angular_velocity * self.wheel_base / 2.0) / self.wheel_radius

            self.left_motor.setVelocity(v_l)
            self.right_motor.setVelocity(v_r)

            
            rclpy.spin_once(self, timeout_sec=0.0)
        
        self.get_logger().info(f"Robot control loop finished cleanly after {loop_count} iterations.")
def main(args=None):
    rclpy.init(args=args)
    node = RobileController()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Shutting down.")
    except Exception as e:
        node.get_logger().error(f"An unexpected error occurred: {e}", exc_info=True) 
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("RobileController node shut down.")

if __name__ == '__main__':
    main()