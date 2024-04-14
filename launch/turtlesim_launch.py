from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
     # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add nodes to the LaunchDescription object
    ld.add_action(Node(
        package='ros2_course',
        executable='turtlesim_controller',
        name='turtlesim_controller'
    ))

    return ld