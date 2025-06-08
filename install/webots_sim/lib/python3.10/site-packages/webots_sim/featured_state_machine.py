import rclpy
import smach

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml
import time

from std_msgs.msg import Int64

collision_threshold = 0.8
battery_threshold = 80


class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions
    """

    def __init__(self, node):
        smach.State.__init__(
            self, outcomes=['low_battery', 'collision_detected', 'all_good'])

        self.node = node
        self.battery_level = 100
        self.collision_detected = False

        self.node.create_subscription(
            Int64, 'battery_level', self.battery_callback, 10)
        self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # addional stuff
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def battery_callback(self, msg):
        print("Callback called!")
        self.battery_level = msg.data
        self.node.get_logger().info(
            f"Received battery level: {self.battery_level}")

    def scan_callback(self, msg):
        # Check if there are any readings
        if not msg.ranges:
            return

        # Total number of laser readings
        num_readings = len(msg.ranges)

        # Find the index for the center (front of robot)
        center_index = num_readings // 2

        # Define how wide we want to check in front (+/- 15 degrees)
        spread = 15  # About 30 degrees total

        # Get the front laser readings
        front_ranges = msg.ranges[center_index - spread: center_index + spread]

        # Remove any readings that are 'inf'
        cleaned_ranges = []
        for r in front_ranges:
            if r < float('inf'):
                cleaned_ranges.append(r)

        # If any of the cleaned ranges is less than threshold, collision is likely
        for distance in cleaned_ranges:
            if distance < collision_threshold:
                self.collision_detected = True
                return

        # If no close object found, set collision to False
        self.collision_detected = False

    def execute(self, userdata):
        # rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.get_logger().info(
            f"Battery: {self.battery_level}, Collision detection: {self.collision_detected}"
        )
        # time.sleep(1.0)
        # rclpy.spin_once(self.node, timeout_sec=0.1)
        twist = Twist()
        twist.linear.x = 0.5

        if self.collision_detected:
            return 'collision_detected'
        elif self.battery_level < battery_threshold+1:
            return 'low_battery'
        else:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.5)
            return 'all_good'


class RotateBase(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['battery_charged'])
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_level = 100
        self.node.create_subscription(
            Int64, 'battery_level', self.battery_callback, 10)

    def battery_callback(self, msg):
        self.battery_level = msg.data
        self.node.get_logger().info(
            f"Received battery level: {self.battery_level}")

    def execute(self, userdata):
        # rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.get_logger().info("Battery low so rotating in place!!!")
        twist = Twist()
        twist.linear.z = 0.0
        twist.angular.z = 0.5

        while not self.battery_level > battery_threshold+1:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.5)
            self.node.get_logger().info(
                f"Battery level: {self.battery_level}%")
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.node.get_logger().info("Battery charged and resuming monitoring.")
        return 'battery_charged'


class StopMotion(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['resume'])
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_data = None
        # adding this subscription so that we get the data from scan and check if we are near obstacle or not repeatatively even after rotating
        self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        self.scan_data = msg

    def is_path_clear(self):
        # If there is no scan data available yet, assume path is not clear
        if not self.scan_data or not self.scan_data.ranges:
            return False

        # Find the index corresponding to the front of the robot
        center_index = len(self.scan_data.ranges) // 2

        # Define a spread around the center to check for obstacles (i.e., +-15 readings)
        spread = 15

        # Extract the front-facing laser scan readings
        front_ranges = self.scan_data.ranges[center_index -
                                             spread:center_index + spread]

        # Check each reading in the front range
        for r in front_ranges:
            # If any distance is less than the collision threshold, obstacle is too close
            if r < collision_threshold:
                return False  # Path is not clear

        # If all distances are safe, path is clear
        return True

    def rotate_90_degrees(self):
        twist = Twist()
        twist.angular.z = 0.5  # Set a moderate rotation speed

        # Rotate for a duration that approximates 90 degrees
        # ~90 degrees in radians / angular speed
        rotate_duration = 1.57 / twist.angular.z
        start_time = time.time()

        while time.time() - start_time < rotate_duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # Allow sensor data to settle
        time.sleep(0.5)
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def execute(self, userdata):
        self.node.get_logger().info(
            "Collision detected! Attempting to auto-rotate to find clear path.")

        # First stop
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        rclpy.spin_once(self.node, timeout_sec=0.5)

        for i in range(4):  # Try 0, 90, 180, 270 degrees
            if self.is_path_clear():
                self.node.get_logger().info("Path is clear. Resuming motion.")
                return 'resume'

            self.node.get_logger().info(
                f"Path blocked. Rotating 90 degrees (attempt {i+1}/4)...")
            self.rotate_90_degrees()

        self.node.get_logger().info("All directions blocked. Waiting for manual clearance.")

        while True:
            user_input = input(
                "Please move the robot to a safe location and type 'clear' 3 TIMES to continue: ")
            if user_input.strip() == "clear":
                self.node.get_logger().info("Robot cleared and safe. Resuming monitoring.")
                break
            else:
                self.node.get_logger().info(
                    "Invalid input. Please type 'clear' to confirm the robot is safe.")
        return 'resume'


def main(args=None):
    """Main function to initialise and execute the state machine
    """

    rclpy.init(args=args)
    node = rclpy.create_node('robile_state_machine')

    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('MONITOR', MonitorBatteryAndCollision(node),
                               transitions={
                                   'low_battery': 'ROTATE',
                                   'collision_detected': 'STOP',
                                   'all_good': 'MONITOR'
        })

        smach.StateMachine.add('ROTATE', RotateBase(node),
                               transitions={'battery_charged': 'MONITOR'})

        smach.StateMachine.add('STOP', StopMotion(node),
                               transitions={'resume': 'MONITOR'})

    node.get_logger().info("Initiate the robile state machine...")
    sm.execute()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
