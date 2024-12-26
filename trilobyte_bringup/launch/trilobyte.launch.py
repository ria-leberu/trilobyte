import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('trilobyte_hardware_interface'),
                'launch',
                'trilobyte_hardware_interface.launch.py'
            )
        )
    )
    return LaunchDescription([included_launch])