import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare arguments

    
    use_mock_hardware_arg = DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    log_level_arg = DeclareLaunchArgument(
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


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("trilobyte_description"), "urdf", "trilobyte.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("trilobyte_hardware_interface"),
            "config",
            "controller_configuration.yaml",
        ]
    )
    robot_localization_config = PathJoinSubstitution(
    [
        FindPackageShare("trilobyte_description"),
        "config",
        "ekf.yaml",
    ]
    )

    amcl_config = PathJoinSubstitution(
    [
        FindPackageShare("trilobyte_description"),
        "config",
        "amcl.yaml",
    ]
    )


      # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
    )

    amcl_node = Node(
         package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config],
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[robot_localization_config]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        ldlidar_node,
        # amcl_node,
        control_node,
        robot_state_pub_node,
        servo_controller_spawner,
        robot_controller_spawner,
        # robot_localization_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]
    

    return LaunchDescription([
        use_mock_hardware_arg,
        log_level_arg,
        GroupAction(actions=nodes)
    ])

# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



