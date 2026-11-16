import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Locate the simulation package share directory
    sim_pkg_share = get_package_share_directory("reactive_robot_sim")

    # Include the ITU Rover simulation world (WSL2 compatible settings)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_share, "launch", "reactive_robot_sim.launch.py")
        ),
        launch_arguments={
            "gui": "true",
            
        }.items(),
    )

    # Configure the reactive obstacle avoidance node with tunable parameters
    avoid_node = Node(
        package="reactive_robot",
        executable="avoid_node",
        name="reactive_obstacle_avoider",
        output="screen",
        parameters=[{
            "max_linear_speed": 0.22,
            "min_linear_speed": 0.06,
            "max_angular_speed": 1.2,
            "safe_distance": 1.25,
            "danger_distance": 0.45,
            "run_duration": 60.0,
        }],
    )

    # Wait 3 seconds so the robot finishes spawning before movement commands start
    delayed_avoid_node = TimerAction(
        period=3.0,
        actions=[avoid_node],
    )

    return LaunchDescription([
        sim_launch,
        delayed_avoid_node,
    ])
