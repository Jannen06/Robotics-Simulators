#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
# from launch_ros.actions import Node # This is no longer needed
import os

def generate_launch_description():
    # Get the path to your Webots world file
    webots_world_path = os.path.join(
        os.environ['HOME'],
        'ros2_ws/src/robile_gazebo/worlds/null_world.wbt'
    )

    return LaunchDescription([
        # Start Webots with your world
        ExecuteProcess(
            cmd=[
                'webots',
                webots_world_path
            ],
            output='screen',
            # Set the name of the process for cleaner logging if you have multiple
            name='webots_simulator',
            # Optionally, if you want launch to wait for Webots to gracefully exit,
            # you might need to handle signals or a more complex shutdown.
            # For now, it's fine as is.
        ),

        # IMPORTANT:
        # The 'robile_controller' Python script is configured in your .wbt file
        # to be launched directly by Webots. Therefore, you do NOT need to
        # launch it again as a separate ROS 2 Node process here.
        # Removing the 'Node' action below resolves the "finished cleanly" message
        # because the original process monitored by ros2 launch will now be Webots itself.

        # Node(
        #     package='robile_gazebo',
        #     executable='robile_controller',
        #     name='ros2_webots_controller',
        #     output='screen'
        # )
        # The above 'Node' action is commented out/removed as it causes the issue.
    ])