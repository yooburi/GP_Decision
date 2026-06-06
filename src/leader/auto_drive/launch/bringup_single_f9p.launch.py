import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_drive_share_dir = get_package_share_directory('auto_drive')
    gps_to_utm_share_dir = get_package_share_directory('gps_to_utm')
    default_csv_path = os.path.join(
        gps_to_utm_share_dir,
        'config',
        'path_csv',
        'gongD_back.csv',
    )
    default_rviz_config = os.path.join(
        auto_drive_share_dir, 'config', 'single_f9p_debug.rviz'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path', default_value=default_csv_path
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config
    )
    use_serial_bridge_arg = DeclareLaunchArgument(
        'use_serial_bridge', default_value='true'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM1'
    )
    publish_velodyne_tf_arg = DeclareLaunchArgument(
        'publish_velodyne_tf',
        default_value='true',
        description='Publish static TF vehicle_frame -> velodyne_frame.',
    )
    vehicle_frame_arg = DeclareLaunchArgument(
        'vehicle_frame',
        default_value='leader/base_link',
        description='Vehicle planning frame used as static TF parent.',
    )
    velodyne_frame_arg = DeclareLaunchArgument(
        'velodyne_frame',
        default_value='velodyne',
        description='LiDAR frame used by detected object markers.',
    )
    velodyne_x_arg = DeclareLaunchArgument(
        'velodyne_x',
        default_value='0.0',
        description='LiDAR x offset from vehicle_frame in meters.',
    )
    velodyne_y_arg = DeclareLaunchArgument(
        'velodyne_y',
        default_value='0.0',
        description='LiDAR y offset from vehicle_frame in meters.',
    )
    velodyne_z_arg = DeclareLaunchArgument(
        'velodyne_z',
        default_value='0.0',
        description='LiDAR z offset from vehicle_frame in meters.',
    )
    velodyne_yaw_arg = DeclareLaunchArgument(
        'velodyne_yaw',
        default_value='0.0',
        description='LiDAR yaw offset from vehicle_frame in radians.',
    )
    velodyne_pitch_arg = DeclareLaunchArgument(
        'velodyne_pitch',
        default_value='0.0',
        description='LiDAR pitch offset from vehicle_frame in radians.',
    )
    velodyne_roll_arg = DeclareLaunchArgument(
        'velodyne_roll',
        default_value='0.0',
        description='LiDAR roll offset from vehicle_frame in radians.',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gps_to_utm_share_dir,
                'launch',
                'localization_single_f9p.launch.py',
            )
        ),
        launch_arguments={
            'csv_file_path': LaunchConfiguration('csv_file_path'),
        }.items(),
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                auto_drive_share_dir, 'launch', 'planning_single_f9p.launch.py'
            )
        ),
        launch_arguments={
            'csv_file_path': LaunchConfiguration('csv_file_path'),
            'publish_velodyne_tf': LaunchConfiguration('publish_velodyne_tf'),
            'vehicle_frame': LaunchConfiguration('vehicle_frame'),
            'velodyne_frame': LaunchConfiguration('velodyne_frame'),
            'velodyne_x': LaunchConfiguration('velodyne_x'),
            'velodyne_y': LaunchConfiguration('velodyne_y'),
            'velodyne_z': LaunchConfiguration('velodyne_z'),
            'velodyne_yaw': LaunchConfiguration('velodyne_yaw'),
            'velodyne_pitch': LaunchConfiguration('velodyne_pitch'),
            'velodyne_roll': LaunchConfiguration('velodyne_roll'),
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                auto_drive_share_dir, 'launch', 'control_single_f9p.launch.py'
            )
        )
    )

    serial_bridge_node = Node(
        package='serial_bridge',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[
            {
                'input_mode': 'direct',
                'port': LaunchConfiguration('serial_port'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('use_serial_bridge')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [
            csv_file_arg,
            use_rviz_arg,
            rviz_config_arg,
            use_serial_bridge_arg,
            serial_port_arg,
            publish_velodyne_tf_arg,
            vehicle_frame_arg,
            velodyne_frame_arg,
            velodyne_x_arg,
            velodyne_y_arg,
            velodyne_z_arg,
            velodyne_yaw_arg,
            velodyne_pitch_arg,
            velodyne_roll_arg,
            localization_launch,
            planning_launch,
            control_launch,
            serial_bridge_node,
            rviz_node,
        ]
    )
