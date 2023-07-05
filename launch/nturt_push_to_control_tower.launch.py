from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "ip",
            default_value="140.112.14.14",
            description="The ip of the control tower.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="'21543'",
            description="The port of the control tower.",
        )
    )
    # initialize arguments
    ip = LaunchConfiguration("ip")
    port = LaunchConfiguration("port")

    # declare nodes
    # node for receiving gps signal 
    push_to_control_tower_node = Node(
        package="nturt_push_to_control_tower",
        executable="nturt_push_to_control_tower_node",
        output="both",
        parameters=[{
            "ip": ip,
            "port": port,
        }]
    )

    return LaunchDescription(arguments + [push_to_control_tower_node])
