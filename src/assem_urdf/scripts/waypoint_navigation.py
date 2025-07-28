#!/usr/bin/env python3
"""
Script điều hướng waypoint cho robot đi một vòng qua các điểm trên map
Dựa trên Nav2 Simple Commander - follow_waypoints.py
"""

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import math

# Định nghĩa waypoints bằng YAML (dễ chỉnh sửa)
waypoints_yaml = '''
waypoints:
  - position:
      x: 1.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - position:
      x: 1.0
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071068
      w: 0.7071068
  - position:
      x: 0.0
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  - position:
      x: -1.0
      y: 1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  - position:
      x: -1.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7071068
      w: 0.7071068
  - position:
      x: -1.0
      y: -1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7071068
      w: 0.7071068
  - position:
      x: 0.0
      y: -1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - position:
      x: 1.0
      y: -1.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - position:
      x: 1.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071068
      w: 0.7071068
  - position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
'''

# Waypoints nhỏ hơn để test an toàn
small_waypoints_yaml = '''
waypoints:
  - position:
      x: 0.5
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  - position:
      x: 0.5
      y: 0.5
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071068
      w: 0.7071068
  - position:
      x: 0.0
      y: 0.5
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0
  - position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
'''

def create_pose_from_yaml(navigator, transform):
    """Tạo PoseStamped từ YAML transform (giống follow_waypoints.py)"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = transform["position"]["x"]
    pose.pose.position.y = transform["position"]["y"]
    pose.pose.position.z = transform["position"]["z"]
    pose.pose.orientation.x = transform["orientation"]["x"]
    pose.pose.orientation.y = transform["orientation"]["y"]
    pose.pose.orientation.z = transform["orientation"]["z"]
    pose.pose.orientation.w = transform["orientation"]["w"]
    return pose

def create_pose_from_coords(navigator, x, y, yaw_degrees):
    """Tạo PoseStamped từ tọa độ x, y, yaw (tiện lợi hơn)"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Vị trí
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    # Chuyển đổi góc từ degrees sang quaternion
    yaw_radians = math.radians(yaw_degrees)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_radians / 2.0)
    pose.pose.orientation.w = math.cos(yaw_radians / 2.0)
    
    return pose

def navigate_waypoints(waypoints_data, timeout_seconds=600, loop_count=1):
    """
    Hàm chính để điều hướng waypoints
    Args:
        waypoints_data: YAML string chứa waypoints
        timeout_seconds: Timeout cho navigation (giây)
        loop_count: Số vòng lặp
    """
    rclpy.init()
    navigator = BasicNavigator()
    
    # Parse waypoints từ YAML
    waypoints = yaml.safe_load(waypoints_data)
    
    # Tạo danh sách poses
    goal_poses = list(map(lambda transform: create_pose_from_yaml(navigator, transform), 
                         waypoints["waypoints"]))
    
    print(f'Đã tạo {len(goal_poses)} waypoints')
    for i, wp in enumerate(waypoints["waypoints"]):
        pos = wp["position"]
        print(f'Waypoint {i+1}: x={pos["x"]:.2f}, y={pos["y"]:.2f}')
    
    # Đợi Nav2 khởi động hoàn toàn
    navigator.waitUntilNav2Active()
    print('🚀 Nav2 đã sẵn sàng!')
    
    # Thực hiện navigation theo số vòng lặp
    for loop in range(loop_count):
        if loop_count > 1:
            print(f'\n--- Vòng {loop + 1}/{loop_count} ---')
        
        # Bắt đầu navigation
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)
        
        # Theo dõi tiến trình
        i = 0
        while not navigator.isTaskComplete():
            # Cập nhật feedback
            i = i + 1
            feedback = navigator.getFeedback()
            
            if feedback and i % 5 == 0:  # In log mỗi 5 giây
                print(f'Đang thực hiện waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}')
                now = navigator.get_clock().now()
                
                # Timeout để tránh kẹt vô hạn
                if now - nav_start > Duration(seconds=timeout_seconds):
                    print(f'⏰ Timeout sau {timeout_seconds} giây, hủy nhiệm vụ...')
                    navigator.cancelTask()
                    break
        
        # Kiểm tra kết quả
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('🎉 Hoàn thành waypoints thành công!')
        elif result == TaskResult.CANCELED:
            print('⚠️ Nhiệm vụ bị hủy!')
            break
        elif result == TaskResult.FAILED:
            print('❌ Nhiệm vụ thất bại!')
            break
        else:
            print(f'❓ Kết quả không xác định: {result}')
            break
        
        # Nghỉ giữa các vòng lặp
        if loop < loop_count - 1:
            print('Nghỉ 3 giây trước vòng tiếp theo...')
            rclpy.spin_once(navigator, timeout_sec=3.0)
    
    print('🏁 Kết thúc navigation!')
    rclpy.shutdown()

def main():
    print("\n=== WAYPOINT NAVIGATION ===")
    print("1. Test nhỏ (hình vuông 0.5x0.5m)")
    print("2. Vòng lớn (hình vuông 2x2m)")
    print("3. Nhiều vòng nhỏ (3 lần)")
    print("4. Nhiều vòng lớn (3 lần)")
    print("5. Tùy chỉnh")
    
    try:
        choice = input("Chọn chế độ (1-5): ").strip()
        
        if choice == "1":
            print("🔸 Chạy test nhỏ...")
            navigate_waypoints(small_waypoints_yaml, timeout_seconds=300, loop_count=1)
            
        elif choice == "2":
            print("🔹 Chạy vòng lớn...")
            navigate_waypoints(waypoints_yaml, timeout_seconds=600, loop_count=1)
            
        elif choice == "3":
            print("🔸 Chạy 3 vòng nhỏ...")
            navigate_waypoints(small_waypoints_yaml, timeout_seconds=300, loop_count=3)
            
        elif choice == "4":
            print("🔹 Chạy 3 vòng lớn...")
            navigate_waypoints(waypoints_yaml, timeout_seconds=600, loop_count=3)
            
        elif choice == "5":
            try:
                size = input("Kích thước (small/large): ").strip().lower()
                loops = int(input("Số vòng: "))
                timeout = int(input("Timeout (giây, mặc định 600): ") or "600")
                
                if size == "small":
                    waypoints_data = small_waypoints_yaml
                else:
                    waypoints_data = waypoints_yaml
                    
                navigate_waypoints(waypoints_data, timeout_seconds=timeout, loop_count=loops)
                
            except ValueError:
                print("❌ Giá trị không hợp lệ!")
                
        else:
            print("❌ Lựa chọn không hợp lệ! Chạy test nhỏ...")
            navigate_waypoints(small_waypoints_yaml, timeout_seconds=300, loop_count=1)
            
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình bởi người dùng")
    except Exception as e:
        print(f"❌ Lỗi: {e}")

if __name__ == '__main__':
    main() 