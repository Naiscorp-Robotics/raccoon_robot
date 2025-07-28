# ASSEM_URDF - ROS2 Navigation Robot Package

## Tổng quan

`assem_urdf` là package chính của hệ thống robot tự hành sử dụng ROS2, được thiết kế để điều khiển robot di chuyển, định vị và navigate trong môi trường thực tế và mô phỏng. Package này tích hợp đầy đủ các tính năng SLAM, Navigation, điều khiển robot và xử lý dữ liệu cảm biến.

## Đặc điểm chính

### 🤖 Robot Hardware Support

- **Differential Drive Robot**: Hỗ trợ robot bánh xe vi sai
- **LIDAR Integration**: Tích hợp SLLIDAR cho navigation và mapping
- **Camera Support**: Hỗ trợ camera Orbbec cho computer vision
- **IMU Sensor**: Fusion dữ liệu IMU với odometry

### 🗺️ SLAM & Navigation

- **SLAM Toolbox**: Tạo bản đồ tự động trong thời gian thực
- **Nav2 Stack**: Navigation stack đầy đủ với path planning
- **EKF Localization**: Fusion cảm biến cho định vị chính xác
- **Waypoint Navigation**: Điều hướng theo điểm đích được định trước

### 🎮 Control Methods

- **Joystick/PS4 Controller**: Điều khiển thủ công
- **Autonomous Navigation**: Tự động điều hướng đến mục tiêu
- **Topic-based Control**: Điều khiển qua ROS topics

## Cấu trúc Package

```
assem_urdf/
├── config/                 # File cấu hình
│   ├── hardware_params.yaml    # Thông số phần cứng
│   ├── nav2_params.yaml        # Cấu hình Navigation2
│   ├── ekf.yaml               # Extended Kalman Filter
│   ├── slam_toolbox_params.yaml # SLAM configuration
│   ├── control.yaml           # Cấu hình điều khiển
│   ├── ps4_joy_config.yaml    # Cấu hình tay cầm PS4
│   └── sllidar_params.yaml    # Thông số LIDAR
├── launch/                 # Launch files
│   ├── bringup.launch.py       # Launch chính cho robot thực
│   ├── sim_robot.launch.py     # Launch cho simulation
│   ├── real_robot.launch.py    # Launch robot thực (cơ bản)
│   ├── real_robot_with_camera.launch.py # Robot + camera
│   ├── gazebo.launch.py        # Mô phỏng Gazebo
│   ├── slam_toolbox.launch.py  # SLAM mapping
│   ├── waypoint_navigation.launch.py # Navigation waypoint
│   └── joy_teleop.launch.py    # Điều khiển joystick
├── scripts/                # Python scripts
│   ├── waypoint_navigation.py  # Navigation node
│   └── simple_waypoint_test.py # Test waypoint
├── urdf/                   # Robot description
│   ├── assem_urdf.urdf.xacro  # Robot URDF model
│   └── orbbec_camera.xacro    # Camera model
├── rviz/                   # RViz configurations
│   ├── navigation_config.rviz  # Cấu hình navigation
│   └── navigation_with_camera.rviz # Navigation + camera
├── world/                  # Gazebo worlds
│   ├── bookstore.sdf          # Bookstore simulation
│   ├── robocon2025_map.sdf    # Robocon map
│   └── empty.sdf              # Empty world
├── meshes/                 # 3D meshes cho robot
└── models/                 # Gazebo models
```

## Cài đặt và Dependencies

### Yêu cầu hệ thống

- **ROS2 Humble** hoặc mới hơn
- **Ubuntu 22.04** (khuyến nghị)
- **Python 3.8+**

### Dependencies chính

```xml
- joint_state_publisher      # Joint state publishing
- robot_state_publisher      # Robot state publishing
- gazebo_ros                # Gazebo simulation
- robot_localization        # EKF localization
- nav2_bringup             # Navigation2 stack
- orbbec_camera            # Orbbec camera driver
- slam_toolbox             # SLAM functionality
```

### Build package

```bash
cd naiscorp_ws
colcon build --packages-select assem_urdf
source install/setup.bash
```

## Sử dụng

### 1. Robot thực tế - Chế độ cơ bản

```bash
ros2 launch assem_urdf real_robot.launch.py
```

### 2. Robot thực tế - Với camera

```bash
ros2 launch assem_urdf real_robot_with_camera.launch.py
```

### 3. Robot thực tế - Full bringup

```bash
ros2 launch assem_urdf bringup.launch.py
```

### 4. Simulation - Gazebo

```bash
ros2 launch assem_urdf gazebo.launch.py
```

### 5. Simulation - Robot model

```bash
ros2 launch assem_urdf sim_robot.launch.py
```

### 6. SLAM Mapping

```bash
ros2 launch assem_urdf slam_toolbox.launch.py
```

### 7. Waypoint Navigation

```bash
ros2 launch assem_urdf waypoint_navigation.launch.py
```

### 8. Điều khiển PS4/Joystick

```bash
ros2 launch assem_urdf joy_teleop.launch.py
```

## Chế độ hoạt động

### 🎯 Mapping Mode

- Sử dụng SLAM Toolbox để tạo bản đồ
- Robot di chuyển và scan môi trường
- Lưu map để sử dụng cho navigation

### 🚀 Navigation Mode

- Load bản đồ đã tạo
- Set goal position trong RViz
- Robot tự động navigate đến đích

### 🎮 Teleoperation Mode

- Điều khiển thủ công bằng PS4 controller
- Real-time control cho testing

### 📍 Waypoint Mode

- Định nghĩa các điểm waypoint
- Robot tự động di chuyển theo route

## Topics quan trọng

### Input Topics

- `/cmd_vel` - Velocity commands
- `/scan` - LIDAR data
- `/imu/data` - IMU sensor data
- `/joy` - Joystick input

### Output Topics

- `/odom` - Odometry data
- `/tf` - Transform tree
- `/map` - Occupancy grid map
- `/path` - Planned path

## Parameters chính

### Hardware Parameters (`hardware_params.yaml`)

- Wheel radius, separation
- Motor specifications
- Sensor configurations

### Navigation Parameters (`nav2_params.yaml`)

- Local/Global planner settings
- Costmap configurations
- Recovery behaviors

### EKF Parameters (`ekf.yaml`)

- Sensor fusion settings
- Process/measurement noise
- Covariance matrices

## Troubleshooting

### Lỗi thường gặp:

1. **LIDAR không kết nối**

   ```bash
   sudo chmod 666 /dev/ttyUSB*
   ```

2. **Controller không spawn**

   - Kiểm tra hardware_params.yaml
   - Verify robot_description

3. **Navigation không hoạt động**

   - Ensure map được load
   - Check tf tree integrity

4. **Camera không hiển thị**
   - Verify camera permissions
   - Check USB connection

## Phát triển và Tùy chỉnh

### Thêm sensor mới

1. Update URDF model
2. Thêm driver trong launch file
3. Cập nhật EKF configuration

### Tùy chỉnh navigation

1. Adjust nav2_params.yaml
2. Modify costmap layers
3. Tune planner parameters

## Liên hệ & Hỗ trợ

- **Maintainer**: Thinh
- **Email**: thinh@todo.todo
- **Version**: 0.0.0
- **License**: TODO

## Changelog

### Version 0.0.0

- Initial release
- Basic navigation functionality
- SLAM integration
- Multi-sensor fusion
- Simulation support

---

_Để biết thêm chi tiết, vui lòng tham khảo các file launch và config tương ứng._
