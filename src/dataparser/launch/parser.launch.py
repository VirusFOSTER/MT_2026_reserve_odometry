import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    input_nav_sat_fix = DeclareLaunchArgument('input_nav_sat_fix', default_value='/sensing/gnss/master/fix')
    input_gnss_master_vel = DeclareLaunchArgument('input_gnss_master_vel', default_value='/sensing/gnss/master/vel')
    input_gnss_rover_fix = DeclareLaunchArgument('input_gnss_rover_fix', default_value='/sensing/gnss/rover/fix')
    input_gnss_rover_vel = DeclareLaunchArgument('input_gnss_rover_vel', default_value='/sensing/gnss/rover/vel')
    input_rear_bogie_vel = DeclareLaunchArgument('input_rear_bogie_vel', default_value='/vehicle/rear_bogie_velocity')
    input_driver_position_cmd = DeclareLaunchArgument('input_driver_position_cmd', default_value='/vehicle/driver_position_cmd')
    input_front_bogie_vel = DeclareLaunchArgument('input_front_bogie_vel', default_value='/vehicle/front_bogie_velocity')

    data_parser_node = Node(
        package='data_parser',
        executable='data_parser_node',
        name='data_parser',
        remappings=[
            ("input_nav_sat_fix", LaunchConfiguration("input_nav_sat_fix")),
            ("input_gnss_master_vel", LaunchConfiguration("input_gnss_master_vel")),
            ("input_gnss_rover_fix", LaunchConfiguration("input_gnss_rover_fix")),
            ("input_gnss_rover_vel", LaunchConfiguration("input_gnss_rover_vel")),
            ("input_rear_bogie_vel", LaunchConfiguration("input_rear_bogie_vel")),
            ("input_driver_position_cmd", LaunchConfiguration("input_driver_position_cmd")),
            ("input_front_bogie_vel", LaunchConfiguration("input_front_bogie_vel"))
        ],
        parameters=[]
    )

    return launch.LaunchDescription(
        [
            input_nav_sat_fix,
            input_gnss_master_vel,
            input_gnss_rover_fix,
            input_gnss_rover_vel,
            input_rear_bogie_vel,
            input_driver_position_cmd,
            input_front_bogie_vel,
            data_parser_node
        ]
    )
