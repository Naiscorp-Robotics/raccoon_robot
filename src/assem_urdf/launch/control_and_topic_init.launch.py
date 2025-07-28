from launch import LaunchDescription  
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parameters for all nodes
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
        
    # SLAM Toolbox params file
    slam_params_file = LaunchConfiguration('slam_params_file')

    # EKF params file
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'ekf.yaml'),
        description='Full path to the EKF parameters file'
    )

    # Nav2 params file
    nav_params_file = LaunchConfiguration('nav_params_file')
    declare_nav_params_file = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    # Load controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diffbot_base_controller'],
        output='screen'
    )
    
    # Node EKF để fusion dữ liệu từ odometry và IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom')]  # Đổi tên output topic thành /odom để SLAM sử dụng
    )
    
    # Relay cmd_vel
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diffbot_base_controller/cmd_vel_unstamped",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    # Connection established
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/diffbot_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )
    bridge_imu = Node (
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
        output='screen'
    )
    
    # Cấu hình SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Cấu hình RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
    # Cấu hình Nav2
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    # Sử dụng bringup_launch.py thay vì navigation_launch.py
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'map': '',  # Không cần load map trước, SLAM sẽ tạo map
            'use_composition': 'False',
            'autostart': 'True'
        }.items()
    )
    
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                'rvizconfig',
                default_value=os.path.join(
                    get_package_share_directory('assem_urdf'),
                    'rviz',
                    'navigation_config.rviz'),
            ),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'slam_toolbox_params.yaml'),
                description='Full path to the SLAM Toolbox parameters file to use'
            ),
            declare_ekf_params_file,
            declare_nav_params_file,
            declare_use_sim_time,
            bridge_clock,
            load_joint_state_controller,
            load_diff_drive_controller,
            bridge_cmd_vel,
            bridge_lidar,
            bridge_imu,
            relay_cmd_vel,
            # ekf_node,  # EKF trước SLAM và Navigation
            slam_toolbox_node,
            nav2_launch,  # Nav2 bringup launch
            rviz_node,
        ] )
    return ld
   
