import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mb_1r2t_lidar_driver'
    #launch_file_dir = os.path.join(get_package_share_directory(pkg_name), 'launch')
    #pkg_dir = get_package_share_directory(pkg_name)
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name)
        , 'rviz'
        , 'china_lidar_rviz_config.rviz'
    )

    baud = LaunchConfiguration('baud_rate', default='153600')
    serial = LaunchConfiguration('device', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='laser')

    # Declare the launch arguments
    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate'
        , default_value='153600'
        , description='Baud rate for the serial port')

    declare_device_cmd = DeclareLaunchArgument(
        'device'
        , default_value='/dev/ttyUSB0'
        , description='Serial port to use for communication')
    
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id'
        , default_value='laser'
        , description='Frame identifier under which the lidar data is published')

    data_reader = Node(
        package = pkg_name,
        executable = 'data_reader',
        output = 'screen',
        parameters = [
            'baud_rate', baud,
            'device', serial,
            'frame_id', frame_id
        ]
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_baud_rate_cmd)
    ld.add_action(declare_device_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(data_reader)
    ld.add_action(rviz)

    return ld