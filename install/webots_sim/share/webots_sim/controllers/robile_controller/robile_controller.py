from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import sys
import math
from controller import Robot
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.node import Node
import rclpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

#!/usr/bin/env python3


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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Lidar
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'base_link'
        static_t.child_frame_id = 'laser'
        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.031  # Match LIDAR's vertical offset
        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.0
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 1.0  # No rotation

        self.static_tf_broadcaster.sendTransform(static_t)
        self.get_logger().info("Static transform from base_link to laser published.")

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info("Attempting to get all four motors...")
        try:
            self.front_left_motor_1 = self.robot.getDevice(
                'robile_1_drive_left_motor')
            self.front_left_motor_2 = self.robot.getDevice(
                'robile_2_drive_left_motor')
            self.front_right_motor_1 = self.robot.getDevice(
                'robile_1_drive_right_motor')
            self.front_right_motor_2 = self.robot.getDevice(
                'robile_2_drive_right_motor')
            self.rear_left_motor_5 = self.robot.getDevice(
                'robile_5_drive_left_motor')
            self.rear_left_motor_6 = self.robot.getDevice(
                'robile_6_drive_left_motor')
            self.rear_right_motor_5 = self.robot.getDevice(
                'robile_5_drive_right_motor')
            self.rear_right_motor_6 = self.robot.getDevice(
                'robile_6_drive_right_motor')

            if None in [self.front_left_motor_1, self.front_left_motor_2, self.front_right_motor_1, self.front_right_motor_2,
                        self.rear_left_motor_5, self.rear_left_motor_6, self.rear_right_motor_5, self.rear_right_motor_6]:
                raise ValueError(
                    "One or more motors were not found. Check your Webots robot PROTO names.")

            self.motors = [self.front_left_motor_1, self.front_left_motor_2, self.front_right_motor_1, self.front_right_motor_2,
                           self.rear_left_motor_5, self.rear_left_motor_6, self.rear_right_motor_5, self.rear_right_motor_6]
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
            self.lidar = self.robot.getDevice('base_laser_front')
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
        self.get_logger().info(
            f"Received cmd_vel: linear_x={self.linear_velocity:.2f}, angular_z={self.angular_velocity:.2f}")

    def run(self):
        self.get_logger().info("Starting robot control loop...")
        while True:
            # Step Webots simulation
            step_result = self.robot.step(self.timestep)

            if step_result == -1 or not rclpy.ok():
                self.get_logger().info("Simulation or ROS 2 context ended. Exiting control loop.")
                break

            # --- PROCESS ROS 2 MESSAGES FIRST ---
            # This allows cmd_vel_callback to update self.linear_velocity/angular_velocity
            # BEFORE we calculate odometry and motor commands for this timestep.
            rclpy.spin_once(self, timeout_sec=0.0)

            current_time = self.get_clock().now()
            # Calculate dt based on time since last iteration
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time  # Update last_time for the next iteration

            # --- NEW LOG PLACEMENT (after cmd_vel update, before calculation) ---
            self.get_logger().info(
                f"Controller State: "
                f"cmd_vel_input: linear_x={self.linear_velocity:.3f}, angular_z={self.angular_velocity:.3f} | "
                f"Current Odom: x={self.x:.3f}, y={self.y:.3f}, theta_rad={self.theta:.3f} (deg={math.degrees(self.theta):.1f})"
            )
            self.get_logger().info(
                f"Motors receiving: linear_vel={self.linear_velocity:.3f}, angular_vel={self.angular_velocity:.3f}"
            )
            # --- END NEW LOG PLACEMENT ---

            # Calculate wheel velocities for Webots motors
            v_l = (self.linear_velocity - self.angular_velocity *
                   self.wheel_base / 2.0) / self.wheel_radius
            v_r = (self.linear_velocity + self.angular_velocity *
                   self.wheel_base / 2.0) / self.wheel_radius

            self.front_left_motor_1.setVelocity(v_l)
            self.front_left_motor_2.setVelocity(v_l)

            self.rear_left_motor_5.setVelocity(v_l)
            self.rear_left_motor_6.setVelocity(v_l)

            self.front_right_motor_1.setVelocity(v_r)
            self.front_right_motor_2.setVelocity(v_r)

            self.rear_right_motor_5.setVelocity(v_r)
            self.rear_right_motor_6.setVelocity(v_r)

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

            # --- Odometry calculation based on current velocities and dt ---
            delta_s = self.linear_velocity * dt  # Distance moved in this time step

            # Update orientation first (since delta_theta depends only on angular_velocity)
            # Store initial theta for this step's calculation
            current_theta_for_odom = self.theta

            self.theta += self.angular_velocity * dt  # Update theta

            # Handle linear movement for x, y
            # Only update x, y if there's significant linear movement
            if abs(self.linear_velocity) > 1e-6:
                # Use the average orientation during the step for linear displacement
                # This makes the odometry more accurate for curved paths
                avg_theta = current_theta_for_odom + \
                    (self.angular_velocity * dt / 2.0)

                delta_x = delta_s * math.cos(avg_theta)
                delta_y = delta_s * math.sin(avg_theta)

                self.x += delta_x
                self.y += delta_y
            # If linear_velocity is 0, delta_x and delta_y remain 0, and x,y don't change, which is correct for pure rotation.

            # Normalize theta to [-pi, pi] - IMPORTANT to do after adding delta_theta
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
            if self.theta > math.pi:
                self.theta -= 2 * math.pi
            elif self.theta < -math.pi:  # Also handle negative wrap-around
                self.theta += 2 * math.pi

            # Publish odom -> base_link TF
            odom_transform = TransformStamped()
            odom_transform.header.stamp = current_time.to_msg()
            odom_transform.header.frame_id = 'odom'
            odom_transform.child_frame_id = 'base_link'
            odom_transform.transform.translation.x = self.x
            odom_transform.transform.translation.y = self.y
            odom_transform.transform.translation.z = 0.0

            # Convert yaw (theta) to quaternion
            # For yaw only, qz = sin(yaw/2), qw = cos(yaw/2)
            qz = math.sin(self.theta / 2.0)
            qw = math.cos(self.theta / 2.0)
            odom_transform.transform.rotation.x = 0.0
            odom_transform.transform.rotation.y = 0.0
            odom_transform.transform.rotation.z = qz
            odom_transform.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(odom_transform)

            # Publish base_link -> laser TF (static transform, but published repeatedly)
            # This can also be published once as a static transform if it never changes
            # but repeatedly publishing it within the loop is fine too.

            ###############################################################
            # t2 = TransformStamped()
            # t2.header.stamp = current_time.to_msg()
            # t2.header.frame_id = 'base_link'
            # t2.child_frame_id = 'laser'
            # t2.transform.translation.x = 0.0
            # t2.transform.translation.y = 0.0
            # t2.transform.translation.z = 0.031
            # t2.transform.rotation.x = 0.0
            # t2.transform.rotation.y = 0.0
            # t2.transform.rotation.z = 0.0
            # t2.transform.rotation.w = 1.0
            # self.tf_broadcaster.sendTransform(t2)
            ###############################################################
            # Publish Odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            # Set the twist values based on the commanded velocities
            odom.twist.twist.linear.x = self.linear_velocity
            odom.twist.twist.angular.z = self.angular_velocity
            self.odom_pub.publish(odom)

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
