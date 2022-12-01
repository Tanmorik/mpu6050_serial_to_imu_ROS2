import os

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import launch
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    imu_launch_arg = DeclareLaunchArgument(
        "imu", default_value=TextSubstitution(text="imu")
    )

    imu_config = os.path.join( 
        get_package_share_directory('mpu6050_serial_to_imu'),
        'params',
        'imu_config.yaml'
    )

    imu_node = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('imu')),
            Node(
                package='mpu6050_serial_to_imu',
                executable='serial_to_imu_node',
                name='serial_to_imu_node',
                parameters=[imu_config]
            ),
        ]
    )
    return launch.LaunchDescription([
        imu_launch_arg,
        imu_node])