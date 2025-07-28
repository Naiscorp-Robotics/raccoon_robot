import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package configuration
    package_description = "assem_urdf"
    package_directory = get_package_share_directory(package_description)
    orbbec_camera_directory = get_package_share_directory("orbbec_camera")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Set to false for real hardware
        description='Whether to use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF Configuration with use_sim=false for real hardware
    urdf_file = 'assem_urdf.urdf.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")

    print(f"Loading URDF/XACRO from: {robot_desc_path}")
    robot_description_content = Command(['xacro ', robot_desc_path, ' use_sim:=false'])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'), '--ros-args', '--log-level', 'info']
    )
    
    declare_rvizconfig = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(
            get_package_share_directory('assem_urdf'),
            'rviz',
            'navigation_with_camera.rviz'),
    )

    # SLLIDAR A1 configuration parameters
    declare_sllidar_params_file = DeclareLaunchArgument(
        'sllidar_params_file',
        default_value=os.path.join(get_package_share_directory('assem_urdf'), 'config', 'sllidar_params.yaml'),
        description='Full path to the SLLIDAR parameters file'
    )
    sllidar_params_file = LaunchConfiguration('sllidar_params_file')

    # SLLIDAR A1 Node
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            sllidar_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Orbbec Gemini2 Camera Launch
    gemini2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_directory, 'launch', 'gemini2.launch.py')
        ),
        launch_arguments={
            'camera_name': 'gemini2',
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'true',
            'depth_width': '640',
            'depth_height': '400',
            'depth_fps': '15',
            'color_width': '640',
            'color_height': '360',
            'color_fps': '15',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_ir': 'true',
            'publish_tf': 'true',
            'tf_publish_rate': '10.0',
            'use_sim_time': use_sim_time
        }.items()
    )

    # TF static transform for camera to base_link
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0.12795', '-0.005377', '0.075802', '0', '0', '0', 'chassis_link', 'camera_link'],
        output='screen'
    )

    # TF static transform for Orbbec Gemini2 camera to camera_link
    gemini2_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gemini2_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'gemini2_link'],
        output='screen'
    )

    # Joint State Publisher for visualization (since we're not running actual controllers)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Combine all launch components
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_rvizconfig,
        declare_sllidar_params_file,
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        sllidar_node,
        rviz_node,
        # Orbbec camera components
        gemini2_camera_launch,
        camera_tf_node,
        gemini2_tf_node,
    ])

    return ld 