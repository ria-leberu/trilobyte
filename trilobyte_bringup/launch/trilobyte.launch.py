import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='debug',  # Default logging level
        description='Logging level for all nodes (debug, info, warn, error, fatal)'
    )

    log_level = LaunchConfiguration('log_level')

    
    trilobyte_hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('trilobyte_hardware_interface'), 'launch'),
                '/trilobyte_hardware_interface.launch.py'])
    )
    

    return LaunchDescription([
        declare_log_level_arg,
        trilobyte_hardware_interface_launch
    ])


