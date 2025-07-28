#!/usr/bin/env python3
"""
Script test đơn giản cho waypoint navigation
Đi qua 4 điểm tạo thành hình vuông
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

def create_pose(x, y, yaw_deg):
    """Tạo PoseStamped từ tọa độ x, y, yaw"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.time.Time().to_msg()
    
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    # Chuyển đổi yaw từ degrees sang quaternion
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    
    return pose

def main():
    rclpy.init()
    
    # Khởi tạo navigator
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    print("Nav2 đã sẵn sàng!")
    
    # Định nghĩa waypoints tạo hình vuông 1x1m (nhỏ hơn để an toàn)
    waypoints = [
        create_pose(0.5, 0.0, 0.0),    # Điểm 1: Đi về phía trước
        create_pose(0.5, 0.5, 90.0),   # Điểm 2: Rẽ trái
        create_pose(0.0, 0.5, 180.0),  # Điểm 3: Đi về phía sau
        create_pose(0.0, 0.0, 270.0),  # Điểm 4: Về điểm xuất phát
    ]
    
    print(f"Bắt đầu điều hướng qua {len(waypoints)} waypoints...")
    
    # Gửi waypoints
    navigator.followWaypoints(waypoints)
    
    # Đợi hoàn thành
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Đang thực hiện waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}")
        rclpy.spin_once(navigator, timeout_sec=1.0)
    
    # Kiểm tra kết quả
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("🎉 Hoàn thành thành công!")
    else:
        print(f"❌ Thất bại: {result}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 