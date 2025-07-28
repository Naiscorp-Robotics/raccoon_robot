#!/bin/bash

echo "===== Setting up device permissions for ASSEM robot ====="

# Kiểm tra và cấp quyền cho LIDAR
if [ -e /dev/ttyUSB0 ]; then
    echo "LIDAR device found at /dev/ttyUSB0"
    sudo chmod 666 /dev/ttyUSB0
    echo "Set permissions for /dev/ttyUSB0"
else
    echo "WARNING: LIDAR device not found at /dev/ttyUSB0"
    
    # Kiểm tra xem ttyUSB? có tồn tại không
    USB_DEVICES=$(ls /dev/ttyUSB* 2>/dev/null)
    if [ -n "$USB_DEVICES" ]; then
        echo "Available USB devices:"
        for device in $USB_DEVICES; do
            echo "  $device"
            sudo chmod 666 $device
            echo "  Set permissions for $device"
        done
        
        # Cập nhật tham số nếu không tìm thấy /dev/ttyUSB0
        FIRST_USB=$(ls /dev/ttyUSB* 2>/dev/null | head -n 1)
        if [ -n "$FIRST_USB" ]; then
            echo "Consider updating your sllidar_params.yaml to use: $FIRST_USB"
        fi
    else
        echo "No USB devices found. Make sure your LIDAR is connected."
    fi
fi

# Kiểm tra và cấp quyền cho Motor Controller
if [ -e /dev/ttyUSB1 ]; then
    echo "Motor controller device found at /dev/ttyUSB1"
    sudo chmod 666 /dev/ttyUSB1
    echo "Set permissions for /dev/ttyUSB1"
fi

# Kiểm tra thiết bị CH341
CH341_DEVICES=$(ls /dev/ttyACM* 2>/dev/null)
if [ -n "$CH341_DEVICES" ]; then
    echo "Available ACM devices (possibly CH341):"
    for device in $CH341_DEVICES; do
        echo "  $device"
        sudo chmod 666 $device
        echo "  Set permissions for $device"
    done
fi

echo "===== Device setup complete ====="
echo "You can now launch the robot with: ros2 launch assem_urdf real_robot.launch.py" 