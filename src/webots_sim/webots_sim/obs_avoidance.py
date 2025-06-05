import rclpy  # Ros client library for python code.
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

from tf_transformations import euler_from_quaternion
import time
import math
import numpy as np
from rcl_interfaces.msg import SetParametersResult


class Planner(Node):
    def __init__(self):
        super().__init__("planner_node")

        # initialize parameters
        # Attractive force gain
        self.ka = 0.75

        # Repulsive force gain
        self.kr = 1.5

        # Obstacle influence distance
        self.p_0 = 1.0

        self.latest_scan = None

        # Populate the /tf and /tf_static into buffer and store the transormations.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # define states
        self.state = 'rotate'
        # 'rotate'
        # 'move'
        # 'orient'

        # We initialize the current pose of the robot here
        self.robot_pose = {
            "x": 0,
            "y": 0,
            "theta": 0
        }

        # Initialize the goal pose ( desired position)
        self.goal_pose = {
            "x": 4.0,
            "y": 10.0,
            "theta": -1.0
        }

        # Subscribe to topic /scan for laser scanner to get obstacle data
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.planner_loop)

    def get_robot_pose(self):

        timeout = Duration(seconds=0.5)

        try:
            # Get transform from odom to base_footprint (same as base_link)
            transformation = self.tf_buffer.lookup_transform(
                target_frame='odom',
                source_frame='base_footprint',
                # Latest available
                time=rclpy.time.Time(),
                timeout=timeout
            )

            # Extract translation
            t = transformation.transform.translation
            self.robot_pose['x'] = t.x
            self.robot_pose['y'] = t.y

            # Extract rotation
            r = transformation.transform.rotation
            quat = [r.x, r.y, r.z, r.w]
            _, _, yaw = euler_from_quaternion(quat)

            # Assign it to the current angle of robot
            self.robot_pose['theta'] = yaw

        except Exception as e:
            self.get_logger().warn(
                f"Could not get robile's current pose Unfortunately!!!! :'( : {e}")

    # Scan call back function

    def scan_callback(self, msg):

        # Store the latest scan for use in potential field calculations.
        self.latest_scan = msg

    def planner_loop(self):

        twist = Twist()
        # update the robot pose periodically.
        self.get_robot_pose()

        # checker for laser scan data.
        if self.latest_scan is None:
            self.get_logger().warn("No laser scan data available yet.")
            return

        # Extract robot pose from dict
        x = self.robot_pose["x"]
        y = self.robot_pose["y"]
        theta = self.robot_pose["theta"]

        # Extract goal pose
        goal_x = self.goal_pose["x"]
        goal_y = self.goal_pose["y"]
        goal_theta = self.goal_pose['theta']

        # Calculate attractive velocity vector towards the goal

        difference_x = goal_x - x
        difference_y = goal_y - y
        # Euclidean distance
        distance_to_goal = math.sqrt(difference_x**2 + difference_y**2)

        # Initialize repulsive velocity vector from obstacles, local variable
        v_repulsive_x = 0.0
        v_repulsive_y = 0.0

        # Extract data from LaserScan data
        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment
        ranges = self.latest_scan.ranges

        # Check for proximity to any obstacle
        too_close_to_obstacle = any(
            r < 1.0 and r > 0.05 for r in ranges
        )

        # Reduce attraction if too close to obstacle
        if too_close_to_obstacle:
            scale = 0.3
            self.get_logger().info("Obstacle nearby! Reducing attraction.")
        else:
            scale = 1.0

        # attractive velocity vector towards the goal in x and y
        # attractive velocity: v_attractive = -ka * (q - q_goal) / ||q - q_goal||; taken from the slides
        # Scaled attractive velocity
        v_attractive_x = scale * self.ka * (difference_x / distance_to_goal)
        v_attractive_y = scale * self.ka * (difference_y / distance_to_goal)

        # Compute repulsive velocity vector depending on the scan data angle and increments
        for i, r in enumerate(ranges):
            if r == float('inf') or r == 0.0:
                # ignore invalid or no obstacle
                continue

            # If obstacle within influence range
            if r < self.p_0 and r > 0.01:

                # Calculate angle of obstacle relative to robot frame
                # angle = starting_angle + index * step_size
                angle = angle_min + i * angle_increment

                # Repulsive velocity: v_repulsive = kr * (1/r - 1/p_0) * (1/r^2) * (q - q_o) / ||q - q_o||, if r < p_0
                # q_o : obstacle point
                # r = ||q - q_o||
                # Repulsive veliociy vector components x and y
                force_mag = self.kr * (1.0/r - 1.0/self.p_0) / (r**2)
                force_mag = min(force_mag, 5.0)  # clamp force to avoid spikes

                # Reverse the repulsion direction
                v_repulsive_x += -force_mag * math.cos(angle)
                v_repulsive_y += -force_mag * math.sin(angle)

        # concatinate attractive and repulsive vectors
        velocity_x = v_attractive_x + v_repulsive_x
        velocity_y = v_attractive_y + v_repulsive_y

        # Convert the velocities to base_link / base_footprint frame.

        velocity_x_robot, velocity_y_robot = self.compute_transformation(
            velocity_x, velocity_y)

        # We can use the  velocity_x_robot as the linear velocity in to move the robot in x direction of base_link
        twist.linear.x = max(min(velocity_x_robot, 0.5), -0.5) * 2.0

        # Angle difference between goal and current angle of the robot can used for the desired angular rotation.
        desired_heading = math.atan2(velocity_y, velocity_x)
        angle_diff = (desired_heading - theta +
                      math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = 2.0 * angle_diff

        # Finally publish the velocity with the planned direction and speed.
        self.cmd_pub.publish(twist)

        # Check for distance with threshold
        if distance_to_goal < 0.1:
            # Compute desired orientation
            # desired_orientation = math.atan2(difference_y, difference_x)
            # pose_error = desired_orientation - self.robot_pose["theta"]

            angle_diff = self.goal_pose['theta'] - self.robot_pose['theta']
            pose_error = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

            if abs(pose_error) > 0.1:
                self.get_logger().info("Goal reached! Now aligning to goal pose...")
                # always aligns with the anglular difference and direction.
                twist.angular.z = 3.0 * np.sin(pose_error)
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.cmd_pub.publish(twist)

            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                self.align_to_goal(difference_x, difference_y)
                self.cmd_pub.publish(twist)
                self.get_logger().info("Goal reached and aligned! All done! yipieeee!!!")
                return

    def compute_transformation(self, v_x, v_y):

        # Convert the velocities to base_link / base_footprint frame.
        # Original velocity in world/odom frame
        v_world = np.array([v_x, v_y])

        # Robot's orientation in radians
        theta = self.robot_pose["theta"]

        # 2D rotation matrix for transforming odom frame to robot frame
        rotation_matrix = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        # Rotate the velocity vector
        v_robot = rotation_matrix @ v_world

        # Extract components
        v_x_robot, v_y_robot = v_robot

        return v_x_robot, v_y_robot

    def align_to_goal(self, dx, dy):

        twist = Twist()
        # Compute desired orientation
        desired_orientation = math.atan2(dy, dx)
        pose_error = desired_orientation - self.robot_pose["theta"]

        # Normalize angle to (-pi, pi)
        pose_error = (pose_error + math.pi) % (2 * math.pi) - math.pi

        # always aligns with the anglular difference and direction.
        twist.angular.z = 2.0 * pose_error
        self.cmd_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = Planner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
